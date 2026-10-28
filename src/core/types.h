#pragma once

namespace patronus::core {

/// A pixel-space position or vector: detection centres, aim errors and
/// velocities all use it.
struct Point {
  float cx_; ///< Positive right.
  float cy_; ///< Positive down.
};

/// A pan/tilt pair. The units belong to whoever produced it: the sensor
/// control law emits normalised [-1, 1] commands, the image camera solution degrees.
struct AimAngles {
  float pan_;  ///< Positive right.
  float tilt_; ///< Positive down.
};

} // namespace patronus::core
