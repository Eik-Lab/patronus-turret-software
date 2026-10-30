#include "comm/motor_result.h"
#include "tracking/control_state.h"

#include <iostream>
#include <stdexcept>

using namespace patronus::comm;
using namespace patronus::tracking;

static void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}

int main() {
  try {
    // Exactly representable times distinguish the older axis from the newer one.
    MotorReading pose{{0.0F, 10.0, IoStatus::Ok, 1}, {-0.25F, 10.125, IoStatus::Ok, 1}};
    const ControlParameters params{450000000, 500000000, true, true};
    ControlState state{ControlPhase::Tracking, 0};
    auto decide = [&](double now) {
      return step_control(state, params, {0, true, true, true, pose.fresh(now, 0.25)});
    };
    require(decide(10.125).leader == MotorAction::Aim, "valid zero position rejected");
    require(decide(std::nextafter(10.25, 0.0)).leader == MotorAction::Aim,
            "fresh boundary rejected");
    require(decide(10.25).leader == MotorAction::Hold, "stale pan hidden by fresh tilt");
    require(!pose.fresh(9.0, 0.25), "future sample accepted");
    for (auto *axis : {&pose.pan, &pose.tilt}) {
      const auto good = *axis;
      for (auto status : {IoStatus::Error, IoStatus::Unavailable, IoStatus::NotAttempted}) {
        axis->status = status;
        axis->value = 0.0F; // The SDK's failed-read value must never become a valid origin.
        require(decide(10.125).leader == MotorAction::Hold, "failed axis allowed aiming");
      }
      *axis = good;
      require(decide(10.125).leader == MotorAction::Aim, "recovered pose did not resume");
    }
    pose.pan.value = std::numeric_limits<float>::quiet_NaN();
    require(decide(10.125).leader == MotorAction::Hold, "nonfinite pose accepted");

    MotionResult motion;
    require(!motion.reached(), "unavailable hardware reported arrival");
    motion.within_tolerance = true;
    motion.command.pan.status = IoStatus::Ok;
    motion.command.tilt.status = IoStatus::Error;
    motion.command.stop_pan.status = motion.command.stop_tilt.status = IoStatus::Ok;
    require(!motion.reached(), "successful fallback stop hid partial write failure");
    motion.command.tilt.status = IoStatus::Ok;
    require(motion.reached(), "successful arrival rejected");
    motion.command.pan.limit_position.status = IoStatus::Error;
    require(!motion.reached(), "failed limit feedback reported arrival");
    std::cout
      << "Per-axis faults, stale boundaries, recovery and partial command outcomes passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
