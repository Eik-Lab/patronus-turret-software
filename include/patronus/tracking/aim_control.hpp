#pragma once

#include "patronus/core/types.hpp"

#include <cstdint>

namespace patronus::tracking {

/// @brief Gains for the sensor-frame tracking law.
///
/// Output is normalised to [-1, 1] and scaled by the gimbal's configured
/// `max_velocity_rad_s_` by the caller, so the gains are dimensionless and do
/// not need retuning when the max velocity changes.
struct SensorControlGains {
  /// Proportional gain, normalised command per pixel of aim error.
  float kp_{0.006F};
  /// Derivative gain, normalised command per pixel/s of target velocity.
  float kd_{0.0F};
  /// Lead time [s] applied to the filtered velocity. Equivalent to a
  /// proportional term on `velocity * lead_gain`, i.e. predicting where the
  /// target will be rather than where it is. This is what removes the
  /// systematic lag of a pure P controller.
  float lead_gain_{0.0F};
  /// Output deadband, normalised units. Suppresses the dither that would
  /// otherwise occur when the aim error is within sensor noise of centre.
  float deadband_{0.01F};
};

/// @brief Convert a detection centre in detection-space pixels to an aim error.
/// @param center  Detection centre, in the same pixel space the detector emits.
/// @param frame_w Width of that pixel space.
/// @param frame_h Height of that pixel space.
/// @return Error from frame centre in pixels. Positive x is right, positive y
///         is down, matching the tilt sign convention.
///
/// @note This is the single place where detection pixels are mapped to an aim
///       error. Keeping the frame geometry out of the control law means the
///       centre cannot silently disagree with the geometry the filter was
///       configured for.
[[nodiscard]] core::Point detection_error(core::Point center, uint32_t frame_w, uint32_t frame_h);

/// @brief PD + lead controller on a pixel-space aim error.
///
/// @param error_px      Aim error from frame centre, pixels.
/// @param velocity_px_s Filtered target velocity, pixels/s. Must come from a
///                      filter that has already removed ego-motion; feeding
///                      raw image velocity here makes the derivative term
///                      fight the gimbal's own slewing and causes overshoot.
/// @param gains         Controller gains.
/// @return Normalised command in [-1, 1]. Multiply by `max_velocity_rad_s_`.
///
/// HOT PATH: called every tick (~100 Hz). Branch-free, no allocations, no I/O.
[[nodiscard]] core::AimAngles compute_control_sensor(core::Point error_px,
                                                     core::Point velocity_px_s,
                                                     const SensorControlGains &gains);

/// @brief Convert sensor-frame aim angles to weapon-frame angles,
///        compensating for the physical lever-arm offset between the
///        camera sensor and the weapon bore.
/// @param sensor_pan      Pan angle from the sensor (degrees).
/// @param sensor_tilt     Tilt angle from the sensor (degrees).
/// @param target_distance Slant range to the target (millimetres).
/// @return Compensated AimAngles for the weapon axes (degrees).
core::AimAngles compute_control_weapon(float sensor_pan, float sensor_tilt, float target_distance);

} // namespace patronus::tracking
