#include "tracking/target_filter.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <kalman/tracker.hpp>
#include <utility>

namespace patronus::tracking {
namespace {

  /// 2.56 s at a 10 ms tick: always spans the pipeline latency.
  constexpr int k_angle_history = 256;

  /// So a stalled pipeline cannot blow up the covariance.
  constexpr double k_max_step_s = 0.25;

  /// Averages encoder quantisation: one count per 10 ms is hundreds of px/s
  /// on the telephoto.
  constexpr double k_rate_window_s = 0.05;

  constexpr double k_max_pose_extrapolation_s = 0.1;

  constexpr float k_two_pi = 6.28318530717958647692F;

  struct GimbalSample {
    double t_s;
    float pan_rad;
    float tilt_rad;
  };

} // namespace

struct TargetFilter::Impl {
  float meas_sigma_px{8.0F};
  float conf_noise_scale{10.0F};
  double latency_s{0.0};
  double max_lead_s{0.25};
  double actuation_s{0.0};
  double max_coast_s{1.0};

  float det_w_px{1.0F};
  float det_h_px{1.0F};
  // Negative: panning right moves the image left.
  float pan_px_per_rad{0.0F};
  float tilt_px_per_rad{0.0F};
  bool ego_active{false};

  kalman::Tracker tracker;
  bool initialized{false};
  double last_time_s{0.0};
  double last_capture_s{0.0};
  int coast{0};
  double coast_s{0.0};

  bool have_frame_pose{false};
  float frame_pan_rad{0.0F};
  float frame_tilt_rad{0.0F};
  float camera_vu{0.0F};
  float camera_vv{0.0F};
  bool camera_compensated{false};

  float box_w_px{0.0F};
  float box_h_px{0.0F};
  core::Point detection_error_px{0.0F, 0.0F};
  uint64_t detection_frame_id{0};

  double frame_capture_s{0.0};

  // Reused for a frame no newer than the last.
  AimPoint lead{};

  // Pan is stored as effective pan, see push_angle().
  std::array<GimbalSample, k_angle_history> history{};
  float level_tilt_rad{0.0F};
  float last_raw_pan_rad{0.0F};
  float effective_pan_rad{0.0F};
  int history_head{0}; // next write
  int history_count{0};

  explicit Impl(const kalman::Config &kcfg) : tracker(kcfg) {
  }

  /// Keeps the encoder history, so the next detection re-seeds at once.
  void drop_track() noexcept {
    initialized = false;
    coast = 0;
    coast_s = 0.0;
    box_w_px = 0.0F;
    box_h_px = 0.0F;
    detection_error_px = core::Point{0.0F, 0.0F};
    detection_frame_id = 0;
  }

  void reset() noexcept {
    drop_track();
    last_time_s = 0.0;
    last_capture_s = 0.0;
    have_frame_pose = false;
    history_head = 0;
    history_count = 0;
  }

  /// Pan is stored scaled by cos(elevation), which is how far it actually
  /// sweeps the view; the stabiliser is linear in the angle.
  void push_angle(double t_s, float pan_rad, float tilt_rad) {
    if (history_count == 0) {
      effective_pan_rad = pan_rad;
    } else {
      const float d_pan = std::remainder(pan_rad - last_raw_pan_rad, k_two_pi);
      effective_pan_rad += d_pan * std::cos(tilt_rad - level_tilt_rad);
    }
    last_raw_pan_rad = pan_rad;
    history[history_head] = GimbalSample{t_s, effective_pan_rad, tilt_rad};
    history_head = (history_head + 1) % k_angle_history;
    if (history_count < k_angle_history)
      ++history_count;
  }

  void rate_at_newest(int newest, float &pan_rate, float &tilt_rate) const {
    pan_rate = 0.0F;
    tilt_rate = 0.0F;
    if (history_count < 2)
      return;
    const GimbalSample &sn = history[newest];
    int back = 1;
    while (back + 1 < history_count &&
           sn.t_s - history[(newest + k_angle_history - back - 1) % k_angle_history].t_s <=
             k_rate_window_s)
      ++back;
    const GimbalSample &so = history[(newest + k_angle_history - back) % k_angle_history];
    const double span = sn.t_s - so.t_s;
    if (span > 1.0e-9) {
      pan_rate = static_cast<float>((sn.pan_rad - so.pan_rad) / span);
      tilt_rate = static_cast<float>((sn.tilt_rad - so.tilt_rad) / span);
    }
  }

  bool angle_at(double t_query, float &pan_rad, float &tilt_rad) const {
    if (history_count == 0)
      return false;

    const int newest = (history_head + k_angle_history - 1) % k_angle_history;
    const GimbalSample &sn = history[newest];

    if (t_query >= sn.t_s) {
      float pan_rate = 0.0F;
      float tilt_rate = 0.0F;
      rate_at_newest(newest, pan_rate, tilt_rate);
      const auto ahead = static_cast<float>(std::min(t_query - sn.t_s, k_max_pose_extrapolation_s));
      pan_rad = sn.pan_rad + (pan_rate * ahead);
      tilt_rad = sn.tilt_rad + (tilt_rate * ahead);
      return true;
    }

    if (history_count == 1) {
      pan_rad = sn.pan_rad;
      tilt_rad = sn.tilt_rad;
      return true;
    }

    for (int back = 1; back < history_count; ++back) {
      const GimbalSample &older = history[(newest + k_angle_history - back) % k_angle_history];
      if (older.t_s <= t_query) {
        const GimbalSample &newer =
          history[(newest + k_angle_history - (back - 1)) % k_angle_history];
        const double span = newer.t_s - older.t_s;
        const double w = (span > 1.0e-9) ? ((t_query - older.t_s) / span) : 0.0;
        pan_rad = static_cast<float>(older.pan_rad + (w * (newer.pan_rad - older.pan_rad)));
        tilt_rad = static_cast<float>(older.tilt_rad + (w * (newer.tilt_rad - older.tilt_rad)));
        return true;
      }
    }

    const GimbalSample &oldest =
      history[(newest + k_angle_history - (history_count - 1)) % k_angle_history];
    pan_rad = oldest.pan_rad;
    tilt_rad = oldest.tilt_rad;
    return true;
  }

  [[nodiscard]] double capture_time(const core::FrameObservation &obs) const noexcept {
    return (obs.captured_at_s_ > 0.0) ? obs.captured_at_s_ : (obs.timestamp_s_ - latency_s);
  }

  /// Without encoders the pose stays 0, which the tracker reads as a still camera.
  void update_frame_pose(const core::FrameObservation &obs, double dt) {
    float pan = 0.0F;
    float tilt = 0.0F;
    frame_capture_s = capture_time(obs);
    const bool have_pose = ego_active && angle_at(frame_capture_s, pan, tilt);

    camera_compensated = have_pose && have_frame_pose && dt > 0.0;
    camera_vu = 0.0F;
    camera_vv = 0.0F;
    if (camera_compensated) {
      // Shortest arc, as the tracker unwraps.
      const float d_pan = std::remainder(pan - frame_pan_rad, k_two_pi);
      const float d_tilt = std::remainder(tilt - frame_tilt_rad, k_two_pi);
      camera_vu = pan_px_per_rad * d_pan / static_cast<float>(dt);
      camera_vv = tilt_px_per_rad * d_tilt / static_cast<float>(dt);
    }

    have_frame_pose = have_pose;
    frame_pan_rad = pan;
    frame_tilt_rad = tilt;
  }

  /// Only the target's motion is capped at max_lead_s. The gimbal's rotation
  /// is measured, and capping it would make the control law correct again for
  /// rotation that already happened. Likewise the pose is extrapolated past
  /// now_s by the actuation latency, so the command eases off before arriving.
  [[nodiscard]] AimPoint aim_at(double now_s) const {
    const double t_aim = now_s + actuation_s;
    const double age = std::max(0.0, t_aim - frame_capture_s);
    const auto h = static_cast<float>(std::min(age, max_lead_s));

    float pan = frame_pan_rad;
    float tilt = frame_tilt_rad;
    if (ego_active)
      angle_at(t_aim, pan, tilt);

    const kalman::Rect at = tracker.roi(pan, tilt, h);
    const kalman::PredictedState p = tracker.predict(h);
    AimPoint aim{h, core::Point{at.cu, at.cv}, core::Point{p.velocity.x(), p.velocity.y()},
                 core::Point{at.cu, at.cv}};
    if (actuation_s > 0.0) {
      float pan_now = frame_pan_rad;
      float tilt_now = frame_tilt_rad;
      if (ego_active)
        angle_at(now_s, pan_now, tilt_now);
      const auto h_now =
        static_cast<float>(std::min(std::max(0.0, now_s - frame_capture_s), max_lead_s));
      const kalman::Rect now = tracker.roi(pan_now, tilt_now, h_now);
      aim.error_now_px = core::Point{now.cu, now.cv};
    }
    return aim;
  }

  void update_lead(double now_s) {
    lead = aim_at(now_s);
  }

  [[nodiscard]] Estimate build_estimate(bool measured) const {
    // Zero horizon: the estimate mapped back into this frame's image.
    const kalman::Rect at = tracker.roi(frame_pan_rad, frame_tilt_rad, 0.0F);
    const Eigen::Vector2f velocity = tracker.velocity();
    const auto cov = tracker.covariance();

    Estimate::Values v;
    v.time_s = last_time_s;
    v.error_u_px = at.cu;
    v.error_v_px = at.cv;
    v.vu_px_s = velocity.x();
    v.vv_px_s = velocity.y();
    v.camera_vu_px_s = camera_vu;
    v.camera_vv_px_s = camera_vv;
    v.sigma_u_px = std::sqrt(std::max(cov(0, 0), 0.0F));
    v.sigma_v_px = std::sqrt(std::max(cov(1, 1), 0.0F));
    v.box_w_px = box_w_px;
    v.box_h_px = box_h_px;
    v.detection_error_px = detection_error_px;
    v.detection_frame_id = detection_frame_id;
    v.coast_frames = coast;
    v.measured = measured;
    v.camera_compensated = camera_compensated;
    v.lead_horizon_s = lead.horizon_s;
    v.lead_error_u_px = lead.error_px.cx_;
    v.lead_error_v_px = lead.error_px.cy_;
    v.lead_vu_px_s = lead.velocity_px_s.cx_;
    v.lead_vv_px_s = lead.velocity_px_s.cy_;
    v.au_px_s2 = tracker.state()(4);
    v.av_px_s2 = tracker.state()(5);
    v.capture_s = frame_capture_s;
    v.frame_pan_rad = frame_pan_rad;
    v.frame_tilt_rad = frame_tilt_rad;
    v.nis = tracker.lastNis();
    return Estimate(v);
  }

  [[nodiscard]] core::Point detection_error(const core::Detection &det) const {
    return core::Point{(det.left_ + (det.width_ * 0.5F)) - (det_w_px * 0.5F),
                       (det.top_ + (det.height_ * 0.5F)) - (det_h_px * 0.5F)};
  }

  [[nodiscard]] kalman::Detection measurement(const core::Detection &det,
                                              const core::Point &error) const {
    const float confidence = std::clamp(det.confidence_, 0.0F, 1.0F);
    const float sigma = meas_sigma_px * std::sqrt(1.0F + (conf_noise_scale * (1.0F - confidence)));
    return kalman::Detection{error.cx_, error.cy_, sigma};
  }

  void accept(const core::Detection &det, const core::Point &error, uint64_t frame_id) {
    box_w_px = det.width_;
    box_h_px = det.height_;
    detection_error_px = error;
    detection_frame_id = frame_id;
    coast = 0;
    coast_s = 0.0;
  }

  bool seed(const core::FrameObservation &obs) {
    const core::Detection &det = *obs.detection_;
    const core::Point error = detection_error(det);
    if (!tracker.reset(measurement(det, error), frame_pan_rad, frame_tilt_rad))
      return false;
    initialized = true;
    accept(det, error, obs.frame_id_);
    return true;
  }
};

namespace {

  kalman::Config make_tracker_config(const config::TrackingConfig &cfg, float pan_px_per_rad,
                                     float tilt_px_per_rad) {
    kalman::Config k;
    k.q_jerk = cfg.filter_q_jerk_;
    k.detection_sigma = cfg.filter_meas_sigma_px_;
    k.gate_chi2 = cfg.filter_gate_;
    k.init_pos_sigma = cfg.filter_meas_sigma_px_;
    k.pan_px_per_rad = pan_px_per_rad;
    k.tilt_px_per_rad = tilt_px_per_rad;
    // Track loss is decided by time instead, so it means the same on every camera.
    k.max_coast_frames = 0;
    return k;
  }

} // namespace

TargetFilter::TargetFilter(const config::TrackingConfig &cfg, uint32_t det_width,
                           uint32_t det_height, float focal_x_px, float focal_y_px) {
  // Stabilising only one axis would be worse than neither.
  const bool ego_active = focal_x_px > 0.0F && focal_y_px > 0.0F;
  const float pan_gain = ego_active ? -focal_x_px : 0.0F;
  const float tilt_gain = ego_active ? -focal_y_px : 0.0F;

  impl_ = std::make_unique<Impl>(make_tracker_config(cfg, pan_gain, tilt_gain));
  impl_->det_w_px = (det_width > 0U) ? static_cast<float>(det_width) : 1.0F;
  impl_->det_h_px = (det_height > 0U) ? static_cast<float>(det_height) : 1.0F;
  impl_->ego_active = ego_active;
  impl_->pan_px_per_rad = pan_gain;
  impl_->tilt_px_per_rad = tilt_gain;
  impl_->meas_sigma_px = cfg.filter_meas_sigma_px_;
  impl_->conf_noise_scale = cfg.filter_conf_noise_scale_;
  impl_->latency_s = static_cast<double>(cfg.filter_pipeline_latency_ms_) * 1.0e-3;
  impl_->max_coast_s = static_cast<double>(cfg.filter_max_coast_ms_) * 1.0e-3;
  impl_->max_lead_s = static_cast<double>(cfg.filter_max_lead_ms_) * 1.0e-3;
  impl_->actuation_s = static_cast<double>(cfg.filter_actuation_latency_ms_) * 1.0e-3;
}

TargetFilter::~TargetFilter() = default;

TargetFilter::TargetFilter(TargetFilter &&) noexcept = default;

TargetFilter &TargetFilter::operator=(TargetFilter &&) noexcept = default;

bool TargetFilter::ego_compensation_active() const noexcept {
  return impl_->ego_active;
}

bool TargetFilter::initialized() const noexcept {
  return impl_->initialized;
}

int TargetFilter::coast_frames() const noexcept {
  return impl_->coast;
}

void TargetFilter::set_level_tilt(float tilt_rad) noexcept {
  impl_->level_tilt_rad = tilt_rad;
}

void TargetFilter::push_gimbal_sample(double t_s, float pan_rad, float tilt_rad) {
  impl_->push_angle(t_s, pan_rad, tilt_rad);
}

std::optional<Estimate> TargetFilter::step(const core::FrameObservation &observation,
                                           double now_s) {
  Impl &f = *impl_;
  const core::FrameObservation &obs = observation;

  // Prefer the capture cadence: arrival jitter divided by dt becomes velocity noise.
  const bool first_frame = f.last_time_s <= 0.0;
  double dt = obs.timestamp_s_ - f.last_time_s;
  if (obs.capture_s_ > 0.0 && f.last_capture_s > 0.0)
    dt = obs.capture_s_ - f.last_capture_s;

  if (!first_frame && !(dt > 0.0)) {
    if (!f.initialized)
      return std::nullopt;
    return f.build_estimate(false);
  }
  if (first_frame)
    dt = 0.0;
  dt = std::min(dt, k_max_step_s);

  f.last_time_s = obs.timestamp_s_;
  f.last_capture_s = obs.capture_s_;
  f.update_frame_pose(obs, dt);

  if (!f.initialized) {
    if (!obs.detection_.has_value() || !f.seed(obs))
      return std::nullopt;
    f.update_lead(now_s);
    return f.build_estimate(true);
  }

  std::optional<kalman::Detection> z;
  core::Point error{0.0F, 0.0F};
  if (obs.detection_.has_value()) {
    error = f.detection_error(*obs.detection_);
    z = f.measurement(*obs.detection_, error);
  }

  const bool accepted =
    f.tracker.step(f.frame_pan_rad, f.frame_tilt_rad, z, static_cast<float>(dt));
  if (accepted) {
    f.accept(*obs.detection_, error, obs.frame_id_);
  } else {
    ++f.coast;
    f.coast_s += dt;
    if (f.max_coast_s > 0.0 && f.coast_s > f.max_coast_s) {
      f.drop_track();
      return std::nullopt;
    }
  }

  f.update_lead(now_s);
  return f.build_estimate(accepted);
}

std::optional<AimPoint> TargetFilter::aim_at(double now_s) const {
  if (!impl_->initialized)
    return std::nullopt;
  return impl_->aim_at(now_s);
}

void TargetFilter::reset() noexcept {
  (*impl_).reset();
}

} // namespace patronus::tracking
