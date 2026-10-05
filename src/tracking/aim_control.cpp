#include "patronus/tracking/aim_control.hpp"

#include <algorithm>
#include <cmath>

namespace patronus::tracking {

// Lever-arm offsets from camera sensor origin to weapon bore (mm).
static constexpr float k_weapon_dx = 67.0F;
static constexpr float k_weapon_dy = -34.0F;
static constexpr float k_weapon_dz = 0.0F;

// Sensor-frame pixel centre. There is deliberately no hardcoded frame geometry
// here: the detection pixel space and the aim error space are both supplied by
// the caller (see detection_error), so the two cannot drift apart.
core::Point detection_error(core::Point center, uint32_t frame_w, uint32_t frame_h) {
  const float half_w = static_cast<float>(frame_w) * 0.5F;
  const float half_h = static_cast<float>(frame_h) * 0.5F;
  return core::Point{center.cx_ - half_w, center.cy_ - half_h};
}

// HOT PATH: called every tick (~100 Hz). Keep branch-free, no allocs, no logging.
// The P term tracks position, the D term damps, and the lead term predicts
// where the target will be when the gimbal finishes slewing. All three are
// normalised to [-1, 1] here and scaled to rad/s by the caller.
core::AimAngles compute_control_sensor(core::Point error_px, core::Point velocity_px_s,
                                       const SensorControlGains &gains) {
  const float pan = (gains.kp_ * error_px.cx_) + (gains.kd_ * velocity_px_s.cx_) +
                    (gains.lead_gain_ * (gains.kp_ * velocity_px_s.cx_));

  const float tilt = (gains.kp_ * error_px.cy_) + (gains.kd_ * velocity_px_s.cy_) +
                     (gains.lead_gain_ * (gains.kp_ * velocity_px_s.cy_));

  core::AimAngles cmd{};
  cmd.pan_ = std::clamp(pan, -1.0F, 1.0F);
  cmd.tilt_ = std::clamp(tilt, -1.0F, 1.0F);

  if (std::abs(cmd.pan_) < gains.deadband_)
    cmd.pan_ = 0.0F;
  if (std::abs(cmd.tilt_) < gains.deadband_)
    cmd.tilt_ = 0.0F;

  return cmd;
}

// Project sensor aim vector onto the weapon bore axis, compensating for
// the physical lever-arm offset between camera and weapon.
core::AimAngles compute_control_weapon(float sensor_pan, float sensor_tilt, float target_distance) {
  float pan_rad = sensor_pan * (static_cast<float>(M_PI) / 180.0F);
  float tilt_rad = sensor_tilt * (static_cast<float>(M_PI) / 180.0F);

  float tx = target_distance * std::cos(tilt_rad) * std::cos(pan_rad);
  float ty = target_distance * std::cos(tilt_rad) * std::sin(pan_rad);
  float tz = target_distance * std::sin(tilt_rad);

  float vx = tx - k_weapon_dx;
  float vy = ty - k_weapon_dy;
  float vz = tz - k_weapon_dz;

  const float rad_to_deg = 180.0F / static_cast<float>(M_PI);
  float pan = std::atan2(vy, vx) * rad_to_deg;
  float tilt = std::atan2(vz, std::sqrt((vx * vx) + (vy * vy))) * rad_to_deg;

  return {.pan_ = pan, .tilt_ = tilt};
}

} // namespace patronus::tracking
