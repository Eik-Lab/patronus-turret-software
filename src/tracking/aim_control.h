#pragma once

#include "core/types.h"

#include <cstdint>

namespace patronus::tracking {

/// Output is normalised to [-1, 1] and scaled by max_velocity_rad_s_, so the
/// gains are dimensionless.
struct SensorControlGains {
  float kp_{0.006F};
  float kd_{0.0F};
  /// Seconds of lead: aim where the target will be, removing P-only lag.
  float lead_gain_{0.0F};
  /// Stops dither from sensor noise near centre.
  float deadband_{0.01F};
};

[[nodiscard]] core::Point detection_error(core::Point center, uint32_t frame_w, uint32_t frame_h);

/// velocity_px_s must exclude camera motion, or the D and lead terms fight
/// the gimbal's own slew.
[[nodiscard]] core::AimAngles compute_control_sensor(core::Point error_px,
                                                     core::Point velocity_px_s,
                                                     const SensorControlGains &gains);

/// Gimbal-camera-to-image-camera lever-arm correction, in degrees and mm. Unused.
core::AimAngles compute_control_image_camera(float sensor_pan, float sensor_tilt,
                                             float target_distance);

} // namespace patronus::tracking
