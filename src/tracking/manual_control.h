#pragma once

#include "core/types.h"

#include <mutex>
#include <string>

namespace patronus::tracking {

/// Operator commands: written by the command listener, read by every tracking thread.
/// The client sends a move per key press and auto-repeat but nothing on release, so a
/// move lapses on its own. That also stops the gimbal if the client or network dies.
class ManualControl {
private:
  struct Axis {
    float direction{0.0F}; ///< -1, 0 or 1.
    double moved_s{0.0};
  };

  mutable std::mutex mtx_;
  bool manual_{false};
  Axis pan_;
  Axis tilt_;

  [[nodiscard]] static float live(const Axis &axis, double now_s, double timeout_s) noexcept {
    return now_s - axis.moved_s < timeout_s ? axis.direction : 0.0F;
  }

public:
  /// "manual" or "auto"; anything else is ignored. True if the mode changed.
  bool set_mode(const std::string &mode) {
    if (mode != "manual" && mode != "auto")
      return false;
    const bool manual = mode == "manual";
    std::scoped_lock lock(mtx_);
    if (manual == manual_)
      return false;
    manual_ = manual;
    pan_ = {};
    tilt_ = {};
    return true;
  }

  /// "left", "right", "up" or "down"; anything else is ignored, as is any move in auto.
  void move(const std::string &direction, double now_s) {
    std::scoped_lock lock(mtx_);
    if (!manual_)
      return;
    // Same signs as the tracking law: positive pan is right, positive tilt is down.
    if (direction == "left")
      pan_ = {-1.0F, now_s};
    else if (direction == "right")
      pan_ = {1.0F, now_s};
    else if (direction == "up")
      tilt_ = {-1.0F, now_s};
    else if (direction == "down")
      tilt_ = {1.0F, now_s};
  }

  [[nodiscard]] bool manual() const {
    std::scoped_lock lock(mtx_);
    return manual_;
  }

  /// Each axis is -1, 0 or 1, as a normalized command like compute_control_sensor's.
  [[nodiscard]] core::AimAngles command(double now_s, double timeout_s) const {
    std::scoped_lock lock(mtx_);
    return {.pan_ = live(pan_, now_s, timeout_s), .tilt_ = live(tilt_, now_s, timeout_s)};
  }
};

} // namespace patronus::tracking
