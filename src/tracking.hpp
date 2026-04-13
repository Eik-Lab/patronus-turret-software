#include "state.hpp"
#include <iostream>
#include <cmath>



//TODO: on a later point we will implements a more complex kalman filter
void compute_control(Point center, float Kp, float& x, float& y) {
  const float frame_cx = 1920.0f / 2.0f;
  const float frame_cy = 1080.0f / 2.0f;


  float ex =  center.x- frame_cx;
  float ey = center.y - frame_cy;

  x = Kp * ex;
  y = Kp * ey;

  x = std::clamp(ctrl_x, -1.0f, 1.0f);
  y = std::clamp(ctrl_y, -1.0f, 1.0f);

  if (std::abs(ctrl_x) < 0.02f) ctrl_x = 0.0f;
  if (std::abs(ctrl_y) < 0.02f) ctrl_y = 0.0f;
}

