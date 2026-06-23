#pragma once

#include "patronus/core/types.hpp"

AimAngles compute_control_sensor(Point center, float Kp);

AimAngles compute_control_weapon(float sensor_pan, float sensor_tilt);
