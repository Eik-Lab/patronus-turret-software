#include "tracking/target_filter.h"

#include <algorithm>
#include <cmath>

namespace patronus::tracking {
namespace {

  constexpr int k_angle_history = PoseHistory::capacity;
  /// So a stalled pipeline cannot blow up the covariance.
  constexpr double k_max_step_s = 0.25;
  /// Average encoder quantisation over 50 ms before extrapolating a pose.
  constexpr double k_rate_window_s = 0.05;
  constexpr double k_max_pose_extrapolation_s = 0.1;
  constexpr float k_two_pi = 6.28318530717958647692F;

  /// Keeps encoder history, so the next detection re-seeds at once.
  void drop_track(FilterState &f) noexcept {
    f.initialized = false;
    f.coast = 0;
    f.coast_s = 0.0;
    f.box_w_px = 0.0F;
    f.box_h_px = 0.0F;
    f.detection_error_px = core::Point{0.0F, 0.0F};
    f.detection_frame_id = 0;
  }

  void rate_at_newest(const PoseHistory &p, int newest, float &pan_rate, float &tilt_rate) {
    pan_rate = 0.0F;
    tilt_rate = 0.0F;
    if (p.count < 2)
      return;
    const auto &sn = p.samples[newest];
    int back = 1;
    while (back + 1 < p.count &&
           sn.t_s - p.samples[(newest + k_angle_history - back - 1) % k_angle_history].t_s <=
             k_rate_window_s)
      ++back;
    const auto &so = p.samples[(newest + k_angle_history - back) % k_angle_history];
    const double span = sn.t_s - so.t_s;
    if (span > 1.0e-9) {
      pan_rate = static_cast<float>((sn.pan_rad - so.pan_rad) / span);
      tilt_rate = static_cast<float>((sn.tilt_rad - so.tilt_rad) / span);
    }
  }

  bool angle_at(const PoseHistory &p, double t_query, float &pan_rad, float &tilt_rad) {
    if (p.count == 0)
      return false;
    const int newest = (p.head + k_angle_history - 1) % k_angle_history;
    const auto &sn = p.samples[newest];
    if (t_query >= sn.t_s) {
      float pan_rate = 0.0F;
      float tilt_rate = 0.0F;
      rate_at_newest(p, newest, pan_rate, tilt_rate);
      const auto ahead = static_cast<float>(std::min(t_query - sn.t_s, k_max_pose_extrapolation_s));
      pan_rad = sn.pan_rad + (pan_rate * ahead);
      tilt_rad = sn.tilt_rad + (tilt_rate * ahead);
      return true;
    }
    if (p.count == 1) {
      pan_rad = sn.pan_rad;
      tilt_rad = sn.tilt_rad;
      return true;
    }
    for (int back = 1; back < p.count; ++back) {
      const auto &older = p.samples[(newest + k_angle_history - back) % k_angle_history];
      if (older.t_s <= t_query) {
        const auto &newer = p.samples[(newest + k_angle_history - (back - 1)) % k_angle_history];
        const double span = newer.t_s - older.t_s;
        const double w = (span > 1.0e-9) ? ((t_query - older.t_s) / span) : 0.0;
        pan_rad = static_cast<float>(older.pan_rad + (w * (newer.pan_rad - older.pan_rad)));
        tilt_rad = static_cast<float>(older.tilt_rad + (w * (newer.tilt_rad - older.tilt_rad)));
        return true;
      }
    }
    const auto &oldest = p.samples[(newest + k_angle_history - (p.count - 1)) % k_angle_history];
    pan_rad = oldest.pan_rad;
    tilt_rad = oldest.tilt_rad;
    return true;
  }

  /// Without encoders the pose stays 0, which the tracker reads as a still camera.
  void update_frame_pose(FilterState &f, const FilterParameters &p,
                         const core::FrameObservation &obs, double dt) {
    float pan = 0.0F;
    float tilt = 0.0F;
    f.frame_capture_s =
      (obs.captured_at_s_ > 0.0) ? obs.captured_at_s_ : (obs.timestamp_s_ - p.latency_s);
    const bool have_pose = p.ego_active && angle_at(f.poses, f.frame_capture_s, pan, tilt);
    f.camera_compensated = have_pose && f.have_frame_pose && dt > 0.0;
    f.camera_vu = 0.0F;
    f.camera_vv = 0.0F;
    if (f.camera_compensated) {
      // Shortest arc, as the tracker unwraps.
      const float d_pan = std::remainder(pan - f.frame_pan_rad, k_two_pi);
      const float d_tilt = std::remainder(tilt - f.frame_tilt_rad, k_two_pi);
      f.camera_vu = p.pan_px_per_rad * d_pan / static_cast<float>(dt);
      f.camera_vv = p.tilt_px_per_rad * d_tilt / static_cast<float>(dt);
    }
    f.have_frame_pose = have_pose;
    f.frame_pan_rad = pan;
    f.frame_tilt_rad = tilt;
  }

  /// Cap target extrapolation, not measured camera rotation. Extrapolate the
  /// pose through actuation latency so commands ease off before arrival.
  AimPoint project_aim(const FilterState &f, const FilterParameters &params, double now_s) {
    const double t_aim = now_s + params.actuation_s;
    const double age = std::max(0.0, t_aim - f.frame_capture_s);
    const auto h = static_cast<float>(std::min(age, params.max_lead_s));
    float pan = f.frame_pan_rad;
    float tilt = f.frame_tilt_rad;
    if (params.ego_active)
      angle_at(f.poses, t_aim, pan, tilt);
    const kalman::Rect at = f.tracker.roi(pan, tilt, h);
    const kalman::PredictedState p = f.tracker.predict(h);
    AimPoint aim{h, core::Point{at.cu, at.cv}, core::Point{p.velocity.x(), p.velocity.y()},
                 core::Point{at.cu, at.cv}};
    if (params.actuation_s > 0.0) {
      float pan_now = f.frame_pan_rad;
      float tilt_now = f.frame_tilt_rad;
      if (params.ego_active)
        angle_at(f.poses, now_s, pan_now, tilt_now);
      const auto h_now =
        static_cast<float>(std::min(std::max(0.0, now_s - f.frame_capture_s), params.max_lead_s));
      const kalman::Rect now = f.tracker.roi(pan_now, tilt_now, h_now);
      aim.error_now_px = core::Point{now.cu, now.cv};
    }
    return aim;
  }

  Estimate build_estimate(const FilterState &f, bool measured) {
    // Zero horizon: the estimate mapped back into this frame's image.
    const kalman::Rect at = f.tracker.roi(f.frame_pan_rad, f.frame_tilt_rad, 0.0F);
    const Eigen::Vector2f velocity = f.tracker.velocity();
    const auto cov = f.tracker.covariance();
    Estimate::Values v;
    v.time_s = f.last_time_s;
    v.error_u_px = at.cu;
    v.error_v_px = at.cv;
    v.vu_px_s = velocity.x();
    v.vv_px_s = velocity.y();
    v.camera_vu_px_s = f.camera_vu;
    v.camera_vv_px_s = f.camera_vv;
    v.sigma_u_px = std::sqrt(std::max(cov(0, 0), 0.0F));
    v.sigma_v_px = std::sqrt(std::max(cov(1, 1), 0.0F));
    v.box_w_px = f.box_w_px;
    v.box_h_px = f.box_h_px;
    v.detection_error_px = f.detection_error_px;
    v.detection_frame_id = f.detection_frame_id;
    v.coast_frames = f.coast;
    v.measured = measured;
    v.camera_compensated = f.camera_compensated;
    v.lead_horizon_s = f.lead.horizon_s;
    v.lead_error_u_px = f.lead.error_px.cx_;
    v.lead_error_v_px = f.lead.error_px.cy_;
    v.lead_vu_px_s = f.lead.velocity_px_s.cx_;
    v.lead_vv_px_s = f.lead.velocity_px_s.cy_;
    v.au_px_s2 = f.tracker.state()(4);
    v.av_px_s2 = f.tracker.state()(5);
    v.capture_s = f.frame_capture_s;
    v.frame_pan_rad = f.frame_pan_rad;
    v.frame_tilt_rad = f.frame_tilt_rad;
    v.nis = f.tracker.lastNis();
    return Estimate(v);
  }

  core::Point detection_error(const FilterParameters &p, const core::Detection &det) {
    return core::Point{(det.left_ + (det.width_ * 0.5F)) - (p.det_w_px * 0.5F),
                       (det.top_ + (det.height_ * 0.5F)) - (p.det_h_px * 0.5F)};
  }

  kalman::Detection measurement(const FilterParameters &p, const core::Detection &det,
                                const core::Point &error) {
    const float confidence = std::clamp(det.confidence_, 0.0F, 1.0F);
    const float sigma =
      p.meas_sigma_px * std::sqrt(1.0F + (p.conf_noise_scale * (1.0F - confidence)));
    return kalman::Detection{error.cx_, error.cy_, sigma};
  }

  void accept(FilterState &f, const core::Detection &det, const core::Point &error,
              uint64_t frame_id) {
    f.box_w_px = det.width_;
    f.box_h_px = det.height_;
    f.detection_error_px = error;
    f.detection_frame_id = frame_id;
    f.coast = 0;
    f.coast_s = 0.0;
  }

  bool seed(FilterState &f, const FilterParameters &p, const core::FrameObservation &obs) {
    const core::Detection &det = *obs.detection_;
    const core::Point error = detection_error(p, det);
    if (!f.tracker.reset(measurement(p, det, error), f.frame_pan_rad, f.frame_tilt_rad))
      return false;
    f.initialized = true;
    accept(f, det, error, obs.frame_id_);
    return true;
  }

} // namespace

FilterParameters make_filter_parameters(const config::TrackingConfig &cfg, uint32_t det_width,
                                        uint32_t det_height, float focal_x_px, float focal_y_px,
                                        float level_tilt_rad) {
  FilterParameters p;
  // Stabilising only one axis would be worse than neither.
  p.ego_active = focal_x_px > 0.0F && focal_y_px > 0.0F;
  p.pan_px_per_rad = p.ego_active ? -focal_x_px : 0.0F;
  p.tilt_px_per_rad = p.ego_active ? -focal_y_px : 0.0F;
  p.level_tilt_rad = level_tilt_rad;
  p.det_w_px = (det_width > 0U) ? static_cast<float>(det_width) : 1.0F;
  p.det_h_px = (det_height > 0U) ? static_cast<float>(det_height) : 1.0F;
  p.meas_sigma_px = cfg.filter_meas_sigma_px_;
  p.conf_noise_scale = cfg.filter_conf_noise_scale_;
  p.latency_s = static_cast<double>(cfg.filter_pipeline_latency_ms_) * 1.0e-3;
  p.max_coast_s = static_cast<double>(cfg.filter_max_coast_ms_) * 1.0e-3;
  p.max_lead_s = static_cast<double>(cfg.filter_max_lead_ms_) * 1.0e-3;
  p.actuation_s = static_cast<double>(cfg.filter_actuation_latency_ms_) * 1.0e-3;
  p.model.q_jerk = cfg.filter_q_jerk_;
  p.model.detection_sigma = cfg.filter_meas_sigma_px_;
  p.model.gate_chi2 = cfg.filter_gate_;
  p.model.init_pos_sigma = cfg.filter_meas_sigma_px_;
  p.model.pan_px_per_rad = p.pan_px_per_rad;
  p.model.tilt_px_per_rad = p.tilt_px_per_rad;
  // Track loss is decided by time instead, so it means the same on every camera.
  p.model.max_coast_frames = 0;
  return p;
}

void push_gimbal_sample(PoseHistory &p, double t_s, float pan_rad, float tilt_rad,
                        float level_tilt_rad) {
  if (p.count == 0) {
    p.effective_pan_rad = pan_rad;
  } else {
    const float d_pan = std::remainder(pan_rad - p.last_raw_pan_rad, k_two_pi);
    p.effective_pan_rad += d_pan * std::cos(tilt_rad - level_tilt_rad);
  }
  p.last_raw_pan_rad = pan_rad;
  p.samples[p.head] = PoseHistory::Sample{t_s, p.effective_pan_rad, tilt_rad};
  p.head = (p.head + 1) % k_angle_history;
  if (p.count < k_angle_history)
    ++p.count;
}

std::optional<Estimate> step_filter(FilterState &f, const FilterParameters &p,
                                    const core::FrameObservation &obs, double now_s) {
  // Prefer capture cadence: arrival jitter divided by dt becomes velocity noise.
  const bool first_frame = f.last_time_s <= 0.0;
  double dt = obs.timestamp_s_ - f.last_time_s;
  if (obs.capture_s_ > 0.0 && f.last_capture_s > 0.0)
    dt = obs.capture_s_ - f.last_capture_s;
  if (!first_frame && !(dt > 0.0)) {
    if (!f.initialized)
      return std::nullopt;
    return build_estimate(f, false);
  }
  if (first_frame)
    dt = 0.0;
  dt = std::min(dt, k_max_step_s);
  f.last_time_s = obs.timestamp_s_;
  f.last_capture_s = obs.capture_s_;
  update_frame_pose(f, p, obs, dt);
  if (!f.initialized) {
    if (!obs.detection_.has_value() || !seed(f, p, obs))
      return std::nullopt;
    f.lead = project_aim(f, p, now_s);
    return build_estimate(f, true);
  }
  std::optional<kalman::Detection> z;
  core::Point error{0.0F, 0.0F};
  if (obs.detection_.has_value()) {
    error = detection_error(p, *obs.detection_);
    z = measurement(p, *obs.detection_, error);
  }
  const bool accepted =
    f.tracker.step(f.frame_pan_rad, f.frame_tilt_rad, z, static_cast<float>(dt));
  if (accepted) {
    accept(f, *obs.detection_, error, obs.frame_id_);
  } else {
    ++f.coast;
    f.coast_s += dt;
    if (p.max_coast_s > 0.0 && f.coast_s > p.max_coast_s) {
      drop_track(f);
      return std::nullopt;
    }
  }
  f.lead = project_aim(f, p, now_s);
  return build_estimate(f, accepted);
}

std::optional<AimPoint> aim_at(const FilterState &f, const FilterParameters &p, double now_s) {
  if (!f.initialized)
    return std::nullopt;
  return project_aim(f, p, now_s);
}

void reset_filter(FilterState &f) noexcept {
  drop_track(f);
  f.last_time_s = 0.0;
  f.last_capture_s = 0.0;
  f.have_frame_pose = false;
  f.poses.head = 0;
  f.poses.count = 0;
}

} // namespace patronus::tracking
