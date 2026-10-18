#pragma once

namespace patronus::core {

/// @brief 2D point in normalised sensor-frame coordinates.
struct Point {
  float cx_; ///< Horizontal centre [0, sensor_width].
  float cy_; ///< Vertical centre [0, sensor_height].
};

/// @brief Pan/tilt aim angles in radians.
struct AimAngles {
  float pan_;  ///< Horizontal angle (positive = right).
  float tilt_; ///< Vertical angle (positive = down).
};

} // namespace patronus::core
