#pragma once

#include "patronus/core/config.hpp"
#include "patronus/core/state.hpp"
#include "patronus/core/types.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

namespace patronus::tracking {

/// @brief One tick of filter output: where the target is, and where it is going.
///
/// All coordinates are in *detection* pixels, matching the pixel space the
/// detector reports boxes in (`PipelineConfig::detection_width_`), not the
/// network input and not necessarily the muxed stream size.
///
/// The position is expressed as an aim error (target centre minus frame
/// centre), which is the same quantity `detection_error()` produces, so the
/// control law and the filter can never disagree about the frame geometry.
///
/// The velocity is the target's *own* motion: the gimbal's slewing has already
/// been removed using the encoder angles. Feeding raw image velocity into the
/// D and lead terms makes them fight the gimbal and causes overshoot.
class Estimate {
public:
  /// @brief Construct an estimate. Used by TargetFilter; not intended for
  ///        direct construction.
  Estimate(float error_u_px, float error_v_px, float vu_px_s, float vv_px_s, float sigma_u_px,
           float sigma_v_px, float box_w_px, float box_h_px, int coast_ticks, bool measured)
    : error_u_px_(error_u_px), error_v_px_(error_v_px), vu_px_s_(vu_px_s), vv_px_s_(vv_px_s),
      sigma_u_px_(sigma_u_px), sigma_v_px_(sigma_v_px), box_w_px_(box_w_px), box_h_px_(box_h_px),
      coast_ticks_(coast_ticks), measured_(measured) {
  }

  /// @brief Aim error from the frame centre, pixels. Positive x right, y down.
  [[nodiscard]] float error_u_px() const noexcept {
    return error_u_px_;
  }
  /// @brief Aim error from the frame centre, pixels. Positive x right, y down.
  [[nodiscard]] float error_v_px() const noexcept {
    return error_v_px_;
  }
  /// @brief Target velocity, ego-motion removed, pixels/s.
  [[nodiscard]] float vu() const noexcept {
    return vu_px_s_;
  }
  /// @brief Target velocity, ego-motion removed, pixels/s.
  [[nodiscard]] float vv() const noexcept {
    return vv_px_s_;
  }
  /// @brief 1-sigma position uncertainty, pixels. Grows while coasting.
  [[nodiscard]] float sigma_u_px() const noexcept {
    return sigma_u_px_;
  }
  /// @brief 1-sigma position uncertainty, pixels. Grows while coasting.
  [[nodiscard]] float sigma_v_px() const noexcept {
    return sigma_v_px_;
  }
  /// @brief Width of the most recent accepted detection box, pixels.
  [[nodiscard]] float box_w_px() const noexcept {
    return box_w_px_;
  }
  /// @brief Height of the most recent accepted detection box, pixels.
  [[nodiscard]] float box_h_px() const noexcept {
    return box_h_px_;
  }
  /// @brief Ticks since the last accepted measurement. Rising means the gate
  ///        is rejecting everything and the estimate is extrapolating.
  [[nodiscard]] int coast_ticks() const noexcept {
    return coast_ticks_;
  }
  /// @brief True when this tick accepted a detection.
  [[nodiscard]] bool measured() const noexcept {
    return measured_;
  }

  /// @brief Aim error `horizon_s` seconds into the future.
  /// @param horizon_s Look-ahead time. Negative values extrapolate backwards.
  /// @return Predicted aim error, pixels.
  ///
  /// @note Constant-velocity extrapolation. This is deliberately the same
  ///       prediction the lead term of the control law performs, so what the
  ///       overlay draws is what the gimbal is actually aiming at.
  [[nodiscard]] core::Point predicted_error(float horizon_s) const noexcept {
    return core::Point{error_u_px_ + (vu_px_s_ * horizon_s), error_v_px_ + (vv_px_s_ * horizon_s)};
  }

  /// @brief Target speed, pixels/s.
  [[nodiscard]] float speed_px_s() const noexcept {
    return std::sqrt((vu_px_s_ * vu_px_s_) + (vv_px_s_ * vv_px_s_));
  }

private:
  float error_u_px_;
  float error_v_px_;
  float vu_px_s_;
  float vv_px_s_;
  float sigma_u_px_;
  float sigma_v_px_;
  float box_w_px_;
  float box_h_px_;
  int coast_ticks_;
  bool measured_;
};

/// @brief IMM target filter for a single gimbal.
///
/// Wraps `kalman::ImmFilter` over a two-model set — constant velocity and
/// constant acceleration — both in pixel space. The mix lets the filter follow
/// a drone that is cruising, accelerating into a turn, or hovering, without
/// being re-tuned for each.
///
/// ## Ego-motion compensation
///
/// A gimbal slewing at `omega` rad/s drags every target across the frame at
/// `focal_x * omega` px/s. Left in the measurement, that dominates the velocity
/// estimate and the filter concludes the *target* is manoeuvring as hard as the
/// gimbal slews. This filter therefore tracks the target in an ego-stabilised
/// pixel space:
///
/// @verbatim
///   u_stable = u_error + focal_x * (pan(t)  - pan_ref)
///   v_stable = v_error + focal_y * (tilt(t) - tilt_ref)
/// @endverbatim
///
/// A target that holds still in the world keeps a constant `u_stable` no matter
/// how the gimbal moves, and the estimated velocity is the target's own motion.
/// Positions are converted back to image space on output.
///
/// This requires the focal lengths. With `focal_x_px_`/`focal_y_px_` left at
/// zero the transform degenerates to the identity and the filter degrades
/// gracefully — see `ego_compensation_active()`.
///
/// ## Time base
///
/// Timestamps must be `std::chrono::steady_clock` seconds. On Linux that clock
/// is CLOCK_MONOTONIC, which is process-wide, so the tracking loop and the
/// pipeline thread can compare their stamps directly. `step()` consumes
/// `Detection::timestamp_s_` rather than counting ticks, because the loop is
/// event-driven: cameras run at 30/40 fps while the loop ticks faster, so one
/// tick is not a meaningful time step.
///
/// @warning Not thread-safe. One instance per gimbal, owned by that gimbal's
///          tracking thread. Publication to other threads goes through
///          `patronus::tracking::PredictionChannel`.
class TargetFilter {
public:
  /// @brief Construct a filter for one gimbal.
  /// @param cfg       Tracking configuration (filter gains, gate, tick period).
  /// @param det_width Detection-space width in pixels.
  /// @param det_height Detection-space height in pixels.
  /// @param focal_x_px Horizontal focal length in detection pixels per radian.
  /// @param focal_y_px Vertical focal length in detection pixels per radian.
  ///
  /// @note This performs a single heap allocation (the pimpl body), so construct
  ///       it once before the tracking loop starts, not per tick.
  TargetFilter(const config::TrackingConfig &cfg, uint32_t det_width, uint32_t det_height,
               float focal_x_px, float focal_y_px);

  ~TargetFilter();

  TargetFilter(const TargetFilter &) = delete;
  TargetFilter &operator=(const TargetFilter &) = delete;
  TargetFilter(TargetFilter &&) noexcept;
  TargetFilter &operator=(TargetFilter &&) noexcept;

  /// @brief True when focal lengths are set and self-motion is being removed.
  ///
  /// When false the filter still works, but the velocity it reports includes
  /// the gimbal's own slewing, which makes the D and lead terms fight the
  /// gimbal. Run `--calibrate-focal` to measure the focal lengths.
  [[nodiscard]] bool ego_compensation_active() const noexcept;

  /// @brief True once a detection has seeded the filter.
  [[nodiscard]] bool initialized() const noexcept;

  /// @brief Ticks since the last accepted measurement.
  [[nodiscard]] int coast_ticks() const noexcept;

  /// @brief Record the gimbal's encoder angles for the current instant.
  /// @param t_s      Time on the shared steady-clock base, seconds.
  /// @param pan_rad  Pan encoder angle, radians.
  /// @param tilt_rad Tilt encoder angle, radians.
  ///
  /// Call once per tick, *before* `step()`, so the history covers the pose that
  /// was current when a frame was captured. The history is a fixed-capacity
  /// ring buffer: it never allocates, and lookups walk it backwards from the
  /// newest sample.
  void push_gimbal_sample(double t_s, float pan_rad, float tilt_rad);

  /// @brief Advance the filter one tick and optionally correct it.
  ///
  /// HOT PATH: called every tick (~100 Hz) from one thread. No heap allocation,
  /// no I/O, no logging. Block-free.
  ///
  /// @param dt_s  Nominal tick interval in seconds. Used only to bound how far
  ///              the filter may advance in one call, so a scheduling stall
  ///              cannot inflate the covariance without limit.
  /// @param measurement Latest detection, if one arrived this tick. Consumed by
  ///                    value; the detection's own timestamp decides *when* it
  ///                    is applied, not the tick time.
  /// @param t_s  Current time on the shared steady-clock base, seconds.
  /// @return The current estimate, or `std::nullopt` before the filter has been
  ///         seeded by a first detection.
  ///
  /// @note A measurement whose timestamp is unusable, or whose innovation fails
  ///       the chi-square gate, is treated as absent: the filter predicts and
  ///       the coast counter advances.
  [[nodiscard]] std::optional<Estimate> step(float dt_s,
                                             const std::optional<core::Detection> &measurement,
                                             double t_s);

  /// @brief Drop the estimate and require a fresh detection to re-seed.
  void reset() noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace patronus::tracking