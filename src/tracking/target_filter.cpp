#include "patronus/tracking/target_filter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <kalman/core/imm.hpp>
#include <kalman/models/pixel.hpp>
#include <utility>

namespace patronus::tracking {
namespace {

  // Constant velocity + constant acceleration. Deliberately excludes the CT
  // (coordinated turn) model: its turn rate is unobservable from a linear
  // position update, so `ImmFilter` would have to be handed a prior we have no
  // way to measure here (no optical flow in this pipeline), leaving it pinned at
  // zero and duplicating CV at three times the per-tick cost.
  using ModelCv = kalman::ImmPixelModel<double, void>;
  using ModelCa = kalman::ImmPixelModel<double, kalman::PixelCATag>;
  using Imm = kalman::ImmFilter<double, ModelCv, ModelCa>;

  /// Encoder-angle history length. At the 10 ms default tick this spans 2.56 s,
  /// far beyond any plausible pipeline latency, so a lookup never runs out of
  /// bracketing samples. Fixed capacity: push_gimbal_sample() must not allocate.
  constexpr int k_angle_history = 256;

  /// Fallback initial velocity variance, px^2/s^2. A target seen for the first
  /// time may be moving at any speed; starting with a large velocity covariance
  /// lets the first few updates converge in a tick or two instead of creeping.
  constexpr double k_initial_velocity_var = 1.0e4;

  struct GimbalSample {
    double t_s;
    float pan_rad;
    float tilt_rad;
  };

} // namespace

// ---------------------------------------------------------------------------
//  Impl
// ---------------------------------------------------------------------------

struct TargetFilter::Impl {
  // Configuration.
  float meas_sigma_px{4.0F};
  double qc{2000.0};
  double gate{9.21};
  int adapt_window{30};
  double latency_s{0.0};

  // Geometry.
  float det_w_px{1.0F};
  float det_h_px{1.0F};
  float focal_x{0.0F};
  float focal_y{0.0F};
  bool ego_active{false};

  // Filter.
  Imm imm;
  bool initialized{false};
  double last_time_s{0.0};
  int coast{0};

  // Ego-stabilised reference pose, captured when the filter is seeded. Chosen
  // at seed time so the stabilised coordinate equals the raw aim error there.
  float pan_ref_rad{0.0F};
  float tilt_ref_rad{0.0F};

  // Most recent accepted detection box, for overlay sizing.
  float box_w_px{0.0F};
  float box_h_px{0.0F};

  // Encoder-angle history ring buffer.
  std::array<GimbalSample, k_angle_history> history{};
  int history_head{0}; // next write index
  int history_count{0};

  void reset() noexcept {
    initialized = false;
    coast = 0;
    last_time_s = 0.0;
    history_head = 0;
    history_count = 0;
    box_w_px = 0.0F;
    box_h_px = 0.0F;
  }

  void push_angle(double t_s, float pan_rad, float tilt_rad) {
    history[history_head] = GimbalSample{t_s, pan_rad, tilt_rad};
    history_head = (history_head + 1) % k_angle_history;
    if (history_count < k_angle_history)
      ++history_count;
  }

  /// Look up the pose at an arbitrary past or future instant. Extrapolates
  /// linearly beyond the newest sample (the common case: the tracking loop
  /// pushes its sample before asking), interpolates inside the buffer, and
  /// clamps to the oldest sample if the query predates all history.
  bool angle_at(double t_query, float &pan_rad, float &tilt_rad) const {
    if (history_count == 0)
      return false;

    const int newest = (history_head + k_angle_history - 1) % k_angle_history;
    const GimbalSample &sn = history[newest];

    if (history_count == 1) {
      pan_rad = sn.pan_rad;
      tilt_rad = sn.tilt_rad;
      return true;
    }

    if (t_query >= sn.t_s) {
      const GimbalSample &sp = history[(newest + k_angle_history - 1) % k_angle_history];
      const double span = sn.t_s - sp.t_s;
      if (span > 1.0e-9) {
        const double k = (t_query - sn.t_s) / span;
        pan_rad = static_cast<float>(sn.pan_rad + ((sn.pan_rad - sp.pan_rad) * k));
        tilt_rad = static_cast<float>(sn.tilt_rad + ((sn.tilt_rad - sp.tilt_rad) * k));
      } else {
        pan_rad = sn.pan_rad;
        tilt_rad = sn.tilt_rad;
      }
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

    // Query is older than everything we have: clamp rather than extrapolate
    // backwards through a pose we never measured.
    const GimbalSample &oldest =
      history[(newest + k_angle_history - (history_count - 1)) % k_angle_history];
    pan_rad = oldest.pan_rad;
    tilt_rad = oldest.tilt_rad;
    return true;
  }

  /// Detection-pixel aim error -> ego-stabilised pixel coordinate.
  void to_stable(float error_u, float error_v, float pan_rad, float tilt_rad, double &u,
                 double &v) const {
    u = static_cast<double>(error_u) + (static_cast<double>(focal_x) * (pan_rad - pan_ref_rad));
    v = static_cast<double>(error_v) + (static_cast<double>(focal_y) * (tilt_rad - tilt_ref_rad));
  }

  /// Ego-stabilised pixel coordinate -> aim error at the current pose.
  void to_error(double u, double v, float pan_rad, float tilt_rad, float &error_u,
                float &error_v) const {
    error_u = static_cast<float>(u - (static_cast<double>(focal_x) * (pan_rad - pan_ref_rad)));
    error_v = static_cast<float>(v - (static_cast<double>(focal_y) * (tilt_rad - tilt_ref_rad)));
  }

  void configure(const config::TrackingConfig &cfg) {
    meas_sigma_px = cfg.filter_meas_sigma_px_;
    qc = static_cast<double>(cfg.filter_qc_);
    gate = static_cast<double>(cfg.filter_gate_);
    adapt_window = cfg.filter_adapt_window_;
    latency_s = static_cast<double>(cfg.filter_pipeline_latency_ms_) * 1.0e-3;

    imm.setTransitionProbabilities(static_cast<double>(cfg.filter_imm_transition_p_));
    imm.setProcessNoiseSpectralDensities({qc, qc});
    // The expected normalised innovation of a well-tuned 2-D position filter is
    // the measurement dimension, which is exactly what the adaptive-noise
    // controller wants to drive towards.
    imm.setAdaptiveNoise(true, adapt_window, 2.0);
  }

  Estimate build_estimate(double t_s, bool measured) const {
    const auto state = imm.getState();
    const auto cov = imm.getCovariance();

    float pan = pan_ref_rad;
    float tilt = tilt_ref_rad;
    angle_at(t_s, pan, tilt);

    float error_u = 0.0F;
    float error_v = 0.0F;
    to_error(state(0), state(1), pan, tilt, error_u, error_v);

    return Estimate(error_u, error_v, static_cast<float>(state(2)), static_cast<float>(state(3)),
                    static_cast<float>(std::sqrt(std::max(cov(0, 0), 0.0))),
                    static_cast<float>(std::sqrt(std::max(cov(1, 1), 0.0))), box_w_px, box_h_px,
                    coast, measured);
  }

  void seed(const core::Detection &det, double t_capture) {
    float pan = 0.0F;
    float tilt = 0.0F;
    angle_at(t_capture, pan, tilt);

    // Anchor the stabilised frame at the seeding pose, so u_stable == the raw
    // aim error there and the two coordinate systems agree at t = seed.
    pan_ref_rad = pan;
    tilt_ref_rad = tilt;

    const float center_u = det.left_ + (det.width_ * 0.5F);
    const float center_v = det.top_ + (det.height_ * 0.5F);
    const float error_u = center_u - (det_w_px * 0.5F);
    const float error_v = center_v - (det_h_px * 0.5F);

    double u = 0.0;
    double v = 0.0;
    to_stable(error_u, error_v, pan, tilt, u, v);

    Eigen::Matrix<double, 4, 1> x4;
    x4 << u, v, 0.0, 0.0;
    Eigen::Matrix<double, 4, 4> p4 = Eigen::Matrix<double, 4, 4>::Zero();
    const double pos_var = static_cast<double>(meas_sigma_px) * static_cast<double>(meas_sigma_px);
    p4(0, 0) = pos_var;
    p4(1, 1) = pos_var;
    p4(2, 2) = k_initial_velocity_var;
    p4(3, 3) = k_initial_velocity_var;

    imm.initialize(x4, p4);
    initialized = true;
    coast = 0;
    box_w_px = det.width_;
    box_h_px = det.height_;
  }
};

// ---------------------------------------------------------------------------
//  TargetFilter
// ---------------------------------------------------------------------------

TargetFilter::TargetFilter(const config::TrackingConfig &cfg, uint32_t det_width,
                           uint32_t det_height, float focal_x_px, float focal_y_px)
  : impl_(std::make_unique<Impl>()) {
  impl_->det_w_px = (det_width > 0U) ? static_cast<float>(det_width) : 1.0F;
  impl_->det_h_px = (det_height > 0U) ? static_cast<float>(det_height) : 1.0F;
  // Both focal lengths are required: a partial pair would stabilise one axis and
  // leave the other to fight the gimbal, which is worse than neither.
  impl_->ego_active = focal_x_px > 0.0F && focal_y_px > 0.0F;
  impl_->focal_x = impl_->ego_active ? focal_x_px : 0.0F;
  impl_->focal_y = impl_->ego_active ? focal_y_px : 0.0F;
  impl_->configure(cfg);
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

int TargetFilter::coast_ticks() const noexcept {
  return impl_->coast;
}

void TargetFilter::push_gimbal_sample(double t_s, float pan_rad, float tilt_rad) {
  impl_->push_angle(t_s, pan_rad, tilt_rad);
}

std::optional<Estimate> TargetFilter::step(float dt_s,
                                           const std::optional<core::Detection> &measurement,
                                           double t_s) {
  Impl &f = *impl_;

  // Decide when the measurement was actually taken. The detection carries the
  // buffer PTS; the declared pipeline latency covers the part of that age the
  // filter cannot observe (exposure, ISP, inference). Clamp into the past: a
  // stamp ahead of now means a clock-domain mix-up, and trusting it would make
  // the filter run its update backwards in time.
  double t_capture = t_s;
  bool have_measurement = measurement.has_value();
  if (have_measurement && measurement->valid_timestamp()) {
    t_capture = measurement->timestamp_s_ - f.latency_s;
    if (t_capture > t_s)
      t_capture = t_s;
  }

  if (!f.initialized) {
    if (!have_measurement)
      return std::nullopt;
    f.seed(*measurement, t_capture);
    f.last_time_s = t_s;
    return f.build_estimate(t_s, true);
  }

  // Bound the advance so a scheduling stall cannot blow the covariance up
  // without limit; the filter is fed ~10 ms steps, not multi-second ones.
  const double max_step = (static_cast<double>(std::max(dt_s, 0.0F)) * 8.0) + 0.05;

  // Advance the state clock in two hops, capture-time then now, and never
  // rewind it. The measurement is deliberately older than the tick (that is what
  // the latency compensation is for), so rewinding `last_time_s` to the capture
  // time and then predicting to `t_s` would advance the state by
  // (age + tick) every tick instead of (tick). That over-prediction is
  // systematic, and it biases the velocity estimate low by roughly
  // latency/tick -- measured at ~20% low with a 20 ms latency on a 33 ms camera.
  double t_state = f.last_time_s;

  const double to_capture = std::clamp(t_capture - t_state, 0.0, max_step);
  if (to_capture > 0.0) {
    f.imm.predict(to_capture);
    t_state = t_capture;
  }

  bool accepted = false;
  if (have_measurement) {
    const core::Detection &det = *measurement;
    const float center_u = det.left_ + (det.width_ * 0.5F);
    const float center_v = det.top_ + (det.height_ * 0.5F);

    float pan = f.pan_ref_rad;
    float tilt = f.tilt_ref_rad;
    f.angle_at(t_capture, pan, tilt);

    double u = 0.0;
    double v = 0.0;
    f.to_stable(center_u - (f.det_w_px * 0.5F), center_v - (f.det_h_px * 0.5F), pan, tilt, u, v);

    Eigen::Matrix<double, 2, 1> z;
    z << u, v;
    Eigen::Matrix<double, 2, 2> r =
      Eigen::Matrix<double, 2, 2>::Identity() *
      (static_cast<double>(f.meas_sigma_px) * static_cast<double>(f.meas_sigma_px));

    accepted = f.imm.update(z, r, f.gate);
    if (accepted) {
      f.box_w_px = det.width_;
      f.box_h_px = det.height_;
      f.coast = 0;
    }
  }
  if (!accepted)
    ++f.coast;

  // Carry the corrected state forward to now, so the estimate handed to the
  // control law is current even though the fix it corrects with is older.
  // Measured from t_state, not from the previous tick, so the total advance this
  // tick is exactly the elapsed time no matter where the capture time fell.
  const double to_now = std::clamp(t_s - t_state, 0.0, max_step);
  if (to_now > 0.0) {
    f.imm.predict(to_now);
    t_state = t_s;
  }
  f.last_time_s = t_state;

  return f.build_estimate(t_s, accepted);
}

void TargetFilter::reset() noexcept {
  impl_->reset();
}

} // namespace patronus::tracking