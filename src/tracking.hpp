#include "state.hpp"
#include <iostream>
#include <cmath>

static constexpr float WEAPON_DX = 67.0f;
static constexpr float WEAPON_DY = -34.0f;
static constexpr float WEAPON_DZ = 0.0f;

struct AimAngles
{
  float pan_deg;
  float tilt_deg;
};

AimAngles compute_aim_angles_3d(float tx, float ty, float tz)
{
  float vx = tx - WEAPON_DX;
  float vy = ty - WEAPON_DY;
  float vz = tz - WEAPON_DZ;
  float pan_deg = std::atan2(vy, vx) * (180.0f / M_PI);
  float tilt_deg = std::atan2(vz, std::sqrt(vx * vx + vy * vy)) * (180.0f / M_PI);
  return {pan_deg, tilt_deg};
}

// TODO: on a later point we will implements a more complex kalman filter
void compute_control(Point center, float Kp, float &x, float &y)
{
  const float frame_cx = 1920.0f / 2.0f;
  const float frame_cy = 1080.0f / 2.0f;

  float ex = center.x - frame_cx;
  float ey = center.y - frame_cy;

  x = Kp * ex;
  y = Kp * ey;

  x = std::clamp(x, -1.0f, 1.0f);
  y = std::clamp(y, -1.0f, 1.0f);

  if (std::abs(x) < 0.02f)
    x = 0.0f;
  if (std::abs(y) < 0.02f)
    y = 0.0f;
}
