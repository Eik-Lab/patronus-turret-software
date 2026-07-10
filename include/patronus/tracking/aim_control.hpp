#pragma once

#include "patronus/core/types.hpp"

namespace patronus::tracking {

/// @brief Compute a P-controller correction from sensor-frame coordinates.
/// @param center Detection centre point in sensor pixel coordinates.
/// @param Kp     Proportional gain (rad/s per pixel of error).
/// @return AimAngles with normalised pan and tilt corrections.
core::AimAngles compute_control_sensor(core::Point center, float Kp);

/// @brief Convert sensor-frame aim angles to weapon-frame angles,
///        compensating for the physical lever-arm offset.
/// @param sensor_pan  Pan angle from the sensor (degrees).
/// @param sensor_tilt Tilt angle from the sensor (degrees).
/// @return Compensated AimAngles for the weapon axes (degrees).
core::AimAngles compute_control_weapon(float sensor_pan, float sensor_tilt);

} // namespace patronus::tracking
