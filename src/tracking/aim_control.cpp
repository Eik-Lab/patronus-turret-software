#include "patronus/tracking/aim_control.hpp"

#include <algorithm>
#include <cmath>

namespace patronus::tracking {

// Lever-arm offsets from camera sensor origin to weapon bore (mm).
static constexpr float k_weapon_dx = 67.0F;
static constexpr float k_weapon_dy = -34.0F;
static constexpr float k_weapon_dz = 0.0F;

static constexpr float k_target_distance_mm = 100000.0F;

// Sensor-frame pixel centre (half of 1920x1080).
static constexpr float k_frame_cx = 1920.0F / 2.0F;
static constexpr float k_frame_cy = 1080.0F / 2.0F;

// HOT PATH: called every frame (~100 Hz). Keep branch-free, no allocs.
// Normalise pixel error to [-1, 1] range; centre = zero.
core::AimAngles compute_control_sensor(core::Point center, float Kp) {
  float pan = Kp * (center.cx_ - k_frame_cx);
  float tilt = Kp * (center.cy_ - k_frame_cy);

  pan = std::clamp(pan, -1.0F, 1.0F);
  tilt = std::clamp(tilt, -1.0F, 1.0F);

  if (std::abs(pan) < 0.01F)
    pan = 0.0F;
  if (std::abs(tilt) < 0.01F)
    tilt = 0.0F;

  return {.pan_ = pan, .tilt_ = tilt};
}

// Project sensor aim vector onto the weapon bore axis, compensating for
// the physical lever-arm offset between camera and weapon.
core::AimAngles compute_control_weapon(float sensor_pan, float sensor_tilt) {
  float pan_rad = sensor_pan * (static_cast<float>(M_PI) / 180.0F);
  float tilt_rad = sensor_tilt * (static_cast<float>(M_PI) / 180.0F);

  float tx = k_target_distance_mm * std::cos(tilt_rad) * std::cos(pan_rad);
  float ty = k_target_distance_mm * std::cos(tilt_rad) * std::sin(pan_rad);
  float tz = k_target_distance_mm * std::sin(tilt_rad);

  float vx = tx - k_weapon_dx;
  float vy = ty - k_weapon_dy;
  float vz = tz - k_weapon_dz;

  float pan = std::atan2(vy, vx) * (180.0F / static_cast<float>(M_PI));
  float tilt = std::atan2(vz, std::sqrt(vx * vx + vy * vy)) * (180.0F / static_cast<float>(M_PI));

  return {.pan_ = pan, .tilt_ = tilt};
}

} // namespace patronus::tracking
