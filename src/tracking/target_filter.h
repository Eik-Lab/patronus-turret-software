#pragma once

#include "core/config.h"
#include "core/state.h"
#include "core/types.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <kalman/tracker.hpp>
#include <optional>

namespace patronus::tracking {

/// Aim error at the moment a command sent now takes effect.
struct AimPoint {
  float horizon_s{0.0F};
  core::Point error_px{};
  core::Point velocity_px_s{};
  /// Without the actuation lead; pairs with the current encoder pose.
  core::Point error_now_px{};
};

/// Filter output for one frame. Velocities exclude camera motion.
class Estimate {
public:
  /// Detection pixels; errors from frame centre, positive right and down.
  struct Values {
    double time_s{0.0};
    float error_u_px{0.0F};
    float error_v_px{0.0F};
    float vu_px_s{0.0F};
    float vv_px_s{0.0F};
    float camera_vu_px_s{0.0F};
    float camera_vv_px_s{0.0F};
    float sigma_u_px{0.0F};
    float sigma_v_px{0.0F};
    float box_w_px{0.0F};
    float box_h_px{0.0F};
    core::Point detection_error_px{};
    uint64_t detection_frame_id{0};
    int coast_frames{0};
    bool measured{false};
    bool camera_compensated{false};
    float lead_horizon_s{0.0F};
    float lead_error_u_px{0.0F};
    float lead_error_v_px{0.0F};
    float lead_vu_px_s{0.0F};
    float lead_vv_px_s{0.0F};
    float au_px_s2{0.0F};
    float av_px_s2{0.0F};
    double capture_s{0.0};
    float frame_pan_rad{0.0F}; ///< Scaled by cos(elevation) in the pose history.
    float frame_tilt_rad{0.0F};
    float nis{0.0F}; ///< Normalised innovation squared, last update.
  };

  explicit Estimate(const Values &values) : v_(values) {
  }

  [[nodiscard]] double time_s() const noexcept {
    return v_.time_s;
  }
  [[nodiscard]] float error_u_px() const noexcept {
    return v_.error_u_px;
  }
  [[nodiscard]] float error_v_px() const noexcept {
    return v_.error_v_px;
  }
  [[nodiscard]] float vu() const noexcept {
    return v_.vu_px_s;
  }
  [[nodiscard]] float vv() const noexcept {
    return v_.vv_px_s;
  }
  /// Overlay only: the control law must not see camera motion.
  [[nodiscard]] float camera_vu() const noexcept {
    return v_.camera_vu_px_s;
  }
  [[nodiscard]] float camera_vv() const noexcept {
    return v_.camera_vv_px_s;
  }
  [[nodiscard]] float sigma_u_px() const noexcept {
    return v_.sigma_u_px;
  }
  [[nodiscard]] float sigma_v_px() const noexcept {
    return v_.sigma_v_px;
  }
  [[nodiscard]] float box_w_px() const noexcept {
    return v_.box_w_px;
  }
  [[nodiscard]] float box_h_px() const noexcept {
    return v_.box_h_px;
  }
  [[nodiscard]] core::Point detection_error_px() const noexcept {
    return v_.detection_error_px;
  }
  [[nodiscard]] uint64_t detection_frame_id() const noexcept {
    return v_.detection_frame_id;
  }
  [[nodiscard]] int coast_frames() const noexcept {
    return v_.coast_frames;
  }
  [[nodiscard]] bool measured() const noexcept {
    return v_.measured;
  }
  [[nodiscard]] bool camera_compensated() const noexcept {
    return v_.camera_compensated;
  }

  /// Target extrapolation behind lead_error(), capped at filter_max_lead_ms.
  [[nodiscard]] float lead_horizon_s() const noexcept {
    return v_.lead_horizon_s;
  }
  /// What a command sent now should act on: the target extrapolated, and the
  /// gimbal pose taken where it will be once the command takes effect.
  [[nodiscard]] core::Point lead_error() const noexcept {
    return core::Point{v_.lead_error_u_px, v_.lead_error_v_px};
  }
  [[nodiscard]] core::Point lead_velocity() const noexcept {
    return core::Point{v_.lead_vu_px_s, v_.lead_vv_px_s};
  }
  [[nodiscard]] core::Point acceleration() const noexcept {
    return core::Point{v_.au_px_s2, v_.av_px_s2};
  }
  [[nodiscard]] double capture_s() const noexcept {
    return v_.capture_s;
  }
  [[nodiscard]] float frame_pan_rad() const noexcept {
    return v_.frame_pan_rad;
  }
  [[nodiscard]] float frame_tilt_rad() const noexcept {
    return v_.frame_tilt_rad;
  }
  /// Averages 2 when the noise model fits.
  [[nodiscard]] float nis() const noexcept {
    return v_.nis;
  }

  [[nodiscard]] core::Point predicted_error(float horizon_s) const noexcept {
    return core::Point{v_.error_u_px + (v_.vu_px_s * horizon_s),
                       v_.error_v_px + (v_.vv_px_s * horizon_s)};
  }

  [[nodiscard]] float speed_px_s() const noexcept {
    return std::sqrt((v_.vu_px_s * v_.vu_px_s) + (v_.vv_px_s * v_.vv_px_s));
  }

private:
  Values v_;
};

/// Resolved once at startup; keep constant for the lifetime of a FilterState.
/// The Kalman model is copied into the tracker; the remaining parameters are
/// read directly by the frame and tick transformations.
struct FilterParameters {
  kalman::Config model;
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
  float level_tilt_rad{0.0F};
  bool ego_active{false};
};

/// Encoder samples are consumed together during interpolation, so keep AoS.
struct PoseHistory {
  static constexpr int capacity = 256; ///< 2.56 s at a 10 ms tick.
  struct Sample {
    double t_s;
    float pan_rad; ///< Effective pan, scaled by cos(elevation).
    float tilt_rad;
  };
  std::array<Sample, capacity> samples{};
  float last_raw_pan_rad{0.0F};
  float effective_pan_rad{0.0F};
  int head{0}; ///< Next write.
  int count{0};
};

/// Single-owner mutable data. Inline storage: no allocation or opaque pointer.
/// The small frame state precedes the larger, tick-written pose history.
struct FilterState {
  explicit FilterState(const FilterParameters &parameters) : tracker(parameters.model) {
  }

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
  AimPoint lead{}; ///< Reused for frames no newer than the last.
  PoseHistory poses;
};

[[nodiscard]] FilterParameters make_filter_parameters(const config::TrackingConfig &cfg,
                                                      uint32_t det_width, uint32_t det_height,
                                                      float focal_x_px, float focal_y_px,
                                                      float level_tilt_rad);

/// Every tick. A pose 50 ms off at 1 rad/s introduces 0.05 * focal px of error.
void push_gimbal_sample(PoseHistory &poses, double t_s, float pan_rad, float tilt_rad,
                        float level_tilt_rad);

/// Once per camera frame; only state is mutated. Empty while there is no track.
[[nodiscard]] std::optional<Estimate> step_filter(FilterState &f,
                                                  const FilterParameters &p,
                                                  const core::FrameObservation &obs,
                                                  double now_s);

/// Every tick, not per frame: place the error at the newest encoder pose.
[[nodiscard]] std::optional<AimPoint> aim_at(const FilterState &f,
                                             const FilterParameters &p, double now_s);

void reset_filter(FilterState &f) noexcept;

} // namespace patronus::tracking
