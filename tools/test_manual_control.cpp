#include "tracking/control_state.h"
#include "tracking/manual_control.h"

#include <iostream>
#include <stdexcept>

using namespace patronus::tracking;

static void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}

static void test_commands() {
  constexpr double timeout = 0.25; // Exactly representable, like the times below.
  ManualControl manual;
  require(!manual.manual(), "must start in auto");

  manual.move("left", 1.0);
  require(manual.command(1.0, timeout).pan_ == 0.0F, "move accepted in auto");

  require(!manual.set_mode("sideways"), "unknown mode accepted");
  require(manual.set_mode("manual"), "mode change not reported");
  require(!manual.set_mode("manual"), "repeated mode reported as a change");
  require(manual.command(1.0, timeout).pan_ == 0.0F, "entered manual moving");

  manual.move("sideways", 1.0);
  require(manual.command(1.0, timeout).pan_ == 0.0F, "unknown direction moved");

  manual.move("left", 1.0);
  require(manual.command(1.0, timeout).pan_ == -1.0F, "left is negative pan");
  manual.move("right", 1.0);
  require(manual.command(1.0, timeout).pan_ == 1.0F, "right is positive pan");
  manual.move("up", 1.0);
  require(manual.command(1.0, timeout).tilt_ == -1.0F, "up is negative tilt");
  manual.move("down", 1.0);
  require(manual.command(1.0, timeout).tilt_ == 1.0F, "down is positive tilt");
  require(manual.command(1.0, timeout).pan_ == 1.0F, "tilt move cleared pan");

  require(manual.command(1.125, timeout).pan_ == 1.0F, "lapsed before the timeout");
  require(manual.command(1.25, timeout).pan_ == 0.0F, "did not lapse at the timeout");
  require(manual.command(1.25, timeout).tilt_ == 0.0F, "tilt did not lapse");

  manual.move("left", 2.0);
  require(manual.set_mode("auto"), "mode change not reported");
  require(manual.command(2.0, timeout).pan_ == 0.0F, "move survived leaving manual");
  require(!manual.manual(), "did not return to auto");
}

static void test_control() {
  const ControlParameters params{450000000, 500000000, true, true};
  ControlState state{ControlPhase::AtHome, 0};
  ControlInput input{};
  input.now_ns = 1000000000;
  input.raw_detection = true;
  input.estimate_available = true;
  input.driving = true;
  input.manual = true;

  // A target in view must not take the gimbal from the operator.
  auto decision = step_control(state, params, input);
  require(decision.phase == ControlPhase::Manual, "manual phase not reported");
  require(decision.leader == MotorAction::Manual, "leader not manual");
  require(decision.followers == MotorAction::Mirror, "followers not mirroring");
  require(!decision.aiming, "aiming in manual");

  input.pose_usable = false;
  decision = step_control(state, params, input);
  require(decision.leader == MotorAction::Hold, "moved without a pose");
  require(decision.followers == MotorAction::Hold, "followers moved without a pose");
  input.pose_usable = true;

  input.driving = false;
  decision = step_control(state, params, input);
  require(decision.leader == MotorAction::None, "overlay-only commanded a motor");
  input.driving = true;

  // The manual stretch counts as recent, however long it lasted: hold, then go home.
  input.now_ns = 100000000000;
  (void)step_control(state, params, input);
  input.manual = false;
  input.raw_detection = false;
  input.estimate_available = false;
  input.now_ns += 10000000;
  decision = step_control(state, params, input);
  require(decision.phase == ControlPhase::Tracking, "left manual in the wrong phase");
  require(decision.leader == MotorAction::Hold, "did not hold after manual");
  require(state.phase == ControlPhase::Tracking, "started home immediately");
  input.now_ns += 500000000;
  decision = step_control(state, params, input);
  require(decision.leader == MotorAction::Hold, "skipped the stop before going home");
  require(state.phase == ControlPhase::ReturningHome, "did not head home after the delay");

  // Leaving manual with a live target resumes tracking at once.
  state = ControlState{ControlPhase::Manual, 0};
  input.now_ns = 1000000000;
  input.raw_detection = true;
  input.estimate_available = true;
  decision = step_control(state, params, input);
  require(decision.leader == MotorAction::Aim, "did not resume aiming");
}

int main() {
  try {
    test_commands();
    test_control();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
