#pragma once

#include "core/config.h"
#include "core/state.h"
#include "core/types.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
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
    float frame_pan_rad{0.0F}; ///< Scaled by cos(elevation): see set_level_tilt().
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

/// Constant-acceleration Kalman filter (kalman-cpp) on the detection centre,
/// stepped once per camera frame.
///
/// It works in a camera-stabilised frame: each measurement is shifted by
/// -focal * encoder change at the frame's capture, or the gimbal's own slew
/// would read as target velocity. The capture time matters: a pose 50 ms off
/// at 1 rad/s puts 0.05 * focal px of error into the target.
///
/// Not thread-safe.
class TargetFilter {
public:
  /// Allocates. A 0 focal length disables camera-motion compensation.
  TargetFilter(const config::TrackingConfig &cfg, uint32_t det_width, uint32_t det_height,
               float focal_x_px, float focal_y_px);

  ~TargetFilter();

  TargetFilter(const TargetFilter &) = delete;
  TargetFilter &operator=(const TargetFilter &) = delete;
  TargetFilter(TargetFilter &&) noexcept;
  TargetFilter &operator=(TargetFilter &&) noexcept;

  [[nodiscard]] bool ego_compensation_active() const noexcept;
  [[nodiscard]] bool initialized() const noexcept;
  [[nodiscard]] int coast_frames() const noexcept;

  /// Tilt at which the camera looks level; pan is scaled by cos(elevation).
  void set_level_tilt(float tilt_rad) noexcept;

  /// Call every tick.
  void push_gimbal_sample(double t_s, float pan_rad, float tilt_rad);

  /// Once per camera frame. Empty while there is no track. Hot path.
  [[nodiscard]] std::optional<Estimate> step(const core::FrameObservation &observation,
                                             double now_s);

  /// Call every tick, not per frame: re-placing the error at the newest
  /// encoder pose is what stops the gimbal overshooting.
  [[nodiscard]] std::optional<AimPoint> aim_at(double now_s) const;

  void reset() noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace patronus::tracking
