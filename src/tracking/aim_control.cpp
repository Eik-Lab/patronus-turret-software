#include "tracking/aim_control.h"

#include <algorithm>
#include <cmath>

namespace patronus::tracking {

// Camera to bore, mm.
static constexpr float k_weapon_dx = 67.0F;
static constexpr float k_weapon_dy = -34.0F;
static constexpr float k_weapon_dz = 0.0F;

core::Point detection_error(core::Point center, uint32_t frame_w, uint32_t frame_h) {
  const float half_w = static_cast<float>(frame_w) * 0.5F;
  const float half_h = static_cast<float>(frame_h) * 0.5F;
  return core::Point{center.cx_ - half_w, center.cy_ - half_h};
}

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
