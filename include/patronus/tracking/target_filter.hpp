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

/// @brief Filter output for one camera frame: where the target is, and where it
///        is going.
///
/// All coordinates are in *detection* pixels, matching the pixel space the
/// detector reports boxes in (`PipelineConfig::detection_width_`), not the
/// network input and not necessarily the muxed stream size.
///
/// The position is expressed as an aim error (target centre minus frame
/// centre), which is the same quantity `detection_error()` produces, so the
/// control law and the filter can never disagree about the frame geometry.
///
/// The velocity is the target's *own* motion: the camera's motion has already
/// been removed. Feeding raw image velocity into the D and lead terms makes
/// them fight the gimbal and causes overshoot.
class Estimate {
public:
  /// @brief Plain values an Estimate is built from. Used by TargetFilter.
  struct Values {
    double time_s{0.0};               ///< Frame the estimate is valid at, steady seconds.
    float error_u_px{0.0F};           ///< Aim error from the frame centre.
    float error_v_px{0.0F};           ///< Aim error from the frame centre.
    float vu_px_s{0.0F};              ///< Target velocity, camera motion removed.
    float vv_px_s{0.0F};              ///< Target velocity, camera motion removed.
    float camera_vu_px_s{0.0F};       ///< Image velocity induced by camera motion.
    float camera_vv_px_s{0.0F};       ///< Image velocity induced by camera motion.
    float sigma_u_px{0.0F};           ///< 1-sigma position uncertainty.
    float sigma_v_px{0.0F};           ///< 1-sigma position uncertainty.
    float box_w_px{0.0F};             ///< Most recent accepted detection box.
    float box_h_px{0.0F};             ///< Most recent accepted detection box.
    core::Point detection_error_px{}; ///< Most recent accepted detection centre.
    uint64_t detection_frame_id{0};   ///< Frame that detection came from.
    int coast_frames{0};              ///< Frames since the last accepted detection.
    bool measured{false};             ///< This frame's detection was accepted.
    bool camera_compensated{false};   ///< Camera motion was measured this frame.
    float lead_horizon_s{0.0F};       ///< Capture-to-control interval, seconds.
    float lead_error_u_px{0.0F};      ///< Aim error at the control instant.
    float lead_error_v_px{0.0F};      ///< Aim error at the control instant.
    float lead_vu_px_s{0.0F};         ///< Target velocity at the control instant.
    float lead_vv_px_s{0.0F};         ///< Target velocity at the control instant.
  };

  /// @brief Construct an estimate. Used by TargetFilter; not intended for
  ///        direct construction.
  explicit Estimate(const Values &values) : v_(values) {
  }

  /// @brief Time of the frame this estimate is valid at, `steady_now_s()` seconds.
  [[nodiscard]] double time_s() const noexcept {
    return v_.time_s;
  }
  /// @brief Aim error from the frame centre, pixels. Positive x right, y down.
  [[nodiscard]] float error_u_px() const noexcept {
    return v_.error_u_px;
  }
  /// @brief Aim error from the frame centre, pixels. Positive x right, y down.
  [[nodiscard]] float error_v_px() const noexcept {
    return v_.error_v_px;
  }
  /// @brief Target velocity, camera motion removed, pixels/s.
  [[nodiscard]] float vu() const noexcept {
    return v_.vu_px_s;
  }
  /// @brief Target velocity, camera motion removed, pixels/s.
  [[nodiscard]] float vv() const noexcept {
    return v_.vv_px_s;
  }
  /// @brief Image velocity the camera's own motion gives a world-static point,
  ///        pixels/s. Zero when the camera motion could not be measured.
  ///
  /// The target's apparent velocity in the image is `vu() + camera_vu()`. The
  /// overlay needs that sum to draw the estimate on a later frame; the control
  /// law must not use it.
  [[nodiscard]] float camera_vu() const noexcept {
    return v_.camera_vu_px_s;
  }
  /// @brief See `camera_vu()`.
  [[nodiscard]] float camera_vv() const noexcept {
    return v_.camera_vv_px_s;
  }
  /// @brief 1-sigma position uncertainty, pixels. Grows while coasting.
  [[nodiscard]] float sigma_u_px() const noexcept {
    return v_.sigma_u_px;
  }
  /// @brief 1-sigma position uncertainty, pixels. Grows while coasting.
  [[nodiscard]] float sigma_v_px() const noexcept {
    return v_.sigma_v_px;
  }
  /// @brief Width of the most recent accepted detection box, pixels.
  [[nodiscard]] float box_w_px() const noexcept {
    return v_.box_w_px;
  }
  /// @brief Height of the most recent accepted detection box, pixels.
  [[nodiscard]] float box_h_px() const noexcept {
    return v_.box_h_px;
  }
  /// @brief Centre of the most recent accepted detection, as an aim error from
  ///        the frame centre, pixels.
  ///
  /// @note This is the raw measurement as the detector reported it, not a
  ///       filter output. It only changes on frames where `measured()` is true.
  [[nodiscard]] core::Point detection_error_px() const noexcept {
    return v_.detection_error_px;
  }
  /// @brief `FrameObservation::frame_id_` of the most recent accepted detection.
  [[nodiscard]] uint64_t detection_frame_id() const noexcept {
    return v_.detection_frame_id;
  }
  /// @brief Frames since the last accepted detection. Rising means the gate is
  ///        rejecting everything, or nothing is detected, and the estimate is
  ///        extrapolating.
  [[nodiscard]] int coast_frames() const noexcept {
    return v_.coast_frames;
  }
  /// @brief True when this frame's detection was accepted.
  [[nodiscard]] bool measured() const noexcept {
    return v_.measured;
  }
  /// @brief True when camera motion was removed for this frame using the gimbal
  ///        encoders.
  [[nodiscard]] bool camera_compensated() const noexcept {
    return v_.camera_compensated;
  }

  /// @brief How far past the frame's capture `lead_error()` and
  ///        `lead_velocity()` are extrapolated, seconds: the pipeline latency
  ///        plus the time the observation waited for the tracking loop.
  [[nodiscard]] float lead_horizon_s() const noexcept {
    return v_.lead_horizon_s;
  }
  /// @brief Aim error at the instant the control law runs, pixels.
  ///
  /// The filter's state extrapolated over `lead_horizon_s()` along its
  /// constant-acceleration model, and placed in the image using the newest
  /// gimbal encoder sample rather than the pose at capture. This is what the
  /// motors should act on: by the time a detection reaches the tracking loop,
  /// both the target and the gimbal have moved on.
  [[nodiscard]] core::Point lead_error() const noexcept {
    return core::Point{v_.lead_error_u_px, v_.lead_error_v_px};
  }
  /// @brief Target velocity at the instant the control law runs, camera motion
  ///        removed, pixels/s.
  [[nodiscard]] core::Point lead_velocity() const noexcept {
    return core::Point{v_.lead_vu_px_s, v_.lead_vv_px_s};
  }

  /// @brief Aim error `horizon_s` seconds into the future.
  /// @param horizon_s Look-ahead time. Negative values extrapolate backwards.
  /// @return Predicted aim error, pixels.
  ///
  /// @note Constant-velocity extrapolation of the target's own motion, the same
  ///       one the control law's `lead_gain` term performs. The control law's
  ///       latency compensation is separate: see `lead_error()`.
  [[nodiscard]] core::Point predicted_error(float horizon_s) const noexcept {
    return core::Point{v_.error_u_px + (v_.vu_px_s * horizon_s),
                       v_.error_v_px + (v_.vv_px_s * horizon_s)};
  }

  /// @brief Target speed, pixels/s.
  [[nodiscard]] float speed_px_s() const noexcept {
    return std::sqrt((v_.vu_px_s * v_.vu_px_s) + (v_.vv_px_s * v_.vv_px_s));
  }

private:
  Values v_;
};

/// @brief Target filter for a single gimbal.
///
/// Wraps `kalman::Tracker` from kalman-cpp: a linear Kalman filter over a
/// constant-acceleration pixel-space state `[u, v, vu, vv, au, av]`, corrected by
/// the detector's box centre as a gated position measurement whose noise grows
/// as the detection confidence falls.
///
/// ## One step per camera frame
///
/// `step()` is called once per `FrameObservation`, i.e. at the camera's frame
/// rate, and advances the model by the interval between frames on the camera's
/// capture clock. The tracking loop ticks faster than that to service the
/// motors; it must not step the filter on those extra ticks.
///
/// ## Camera motion
///
/// A gimbal slewing at `omega` rad/s drags every target across the frame at
/// `focal_x * omega` px/s. Left in the measurements, that dominates the velocity
/// estimate and the filter concludes the *target* is manoeuvring as hard as the
/// gimbal slews. `kalman::Tracker` therefore tracks in a camera-stabilised pixel
/// frame: every measurement is shifted by `-focal * delta_angle`, using the
/// gimbal encoder angle at the frame's capture, so a target that holds still in
/// the world holds still in the filter. Positions are converted back to image
/// space on output.
///
/// This needs `focal_x_px_`/`focal_y_px_` (see `--calibrate-focal`) and a motor
/// to read encoders from. Without either, the filter tracks in image space and
/// `Estimate::camera_compensated()` reports false.
///
/// ## Track loss
///
/// A track that has gone `filter_max_coast_ms` without an accepted detection is
/// dropped, and the next detection seeds a fresh one. Measured in time rather
/// than frames so the limit means the same thing on the 30 and 40 fps cameras.
///
/// @warning Not thread-safe. One instance per gimbal, owned by that gimbal's
///          tracking thread. Publication to other threads goes through
///          `patronus::tracking::PredictionChannel`.
class TargetFilter {
public:
  /// @brief Construct a filter for one gimbal.
  /// @param cfg       Tracking configuration (filter noise, gate, coast limit).
  /// @param det_width Detection-space width in pixels.
  /// @param det_height Detection-space height in pixels.
  /// @param focal_x_px Horizontal focal length in detection pixels per radian.
  /// @param focal_y_px Vertical focal length in detection pixels per radian.
  ///
  /// @note This performs a single heap allocation (the pimpl body), so construct
  ///       it once before the tracking loop starts, not per frame.
  TargetFilter(const config::TrackingConfig &cfg, uint32_t det_width, uint32_t det_height,
               float focal_x_px, float focal_y_px);

  ~TargetFilter();

  TargetFilter(const TargetFilter &) = delete;
  TargetFilter &operator=(const TargetFilter &) = delete;
  TargetFilter(TargetFilter &&) noexcept;
  TargetFilter &operator=(TargetFilter &&) noexcept;

  /// @brief True when focal lengths are set, so the gimbal encoders can remove
  ///        camera motion from the measurements.
  [[nodiscard]] bool ego_compensation_active() const noexcept;

  /// @brief True once a detection has seeded the filter.
  [[nodiscard]] bool initialized() const noexcept;

  /// @brief Frames since the last accepted detection.
  [[nodiscard]] int coast_frames() const noexcept;

  /// @brief Record the gimbal's encoder angles for the current instant.
  /// @param t_s      Time on the shared steady-clock base, seconds.
  /// @param pan_rad  Pan encoder angle, radians.
  /// @param tilt_rad Tilt encoder angle, radians.
  ///
  /// Call once per tracking-loop tick. The history is a fixed-capacity ring
  /// buffer: it never allocates, and lookups walk it backwards from the newest
  /// sample.
  void push_gimbal_sample(double t_s, float pan_rad, float tilt_rad);

  /// @brief Advance the filter by one camera frame.
  ///
  /// HOT PATH: called from the tracking loop. No heap allocation, no I/O, no
  /// logging. Block-free.
  ///
  /// @param observation What the pipeline measured on this frame. A frame with
  ///                    no detection is still a step: the filter predicts across
  ///                    it.
  /// @param now_s       Current time on the shared steady-clock base, seconds.
  ///                    Sets how far `Estimate::lead_error()` is extrapolated;
  ///                    values older than the observation are treated as the
  ///                    observation's own timestamp.
  /// @return The estimate at that frame, or `std::nullopt` while there is no
  ///         track: before a first detection, and again after the track has been
  ///         dropped for coasting past `filter_max_coast_ms`.
  ///
  /// @note A detection whose innovation fails the chi-square gate is treated as
  ///       absent. An observation that is not newer than the previous one is
  ///       ignored.
  [[nodiscard]] std::optional<Estimate> step(const core::FrameObservation &observation,
                                             double now_s);

  /// @brief Drop the estimate and require a fresh detection to re-seed.
  void reset() noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace patronus::tracking
