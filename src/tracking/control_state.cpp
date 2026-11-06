#include "tracking/control_state.h"

namespace patronus::tracking {

ControlParameters make_control_parameters(const config::TrackingConfig &cfg) {
  return {static_cast<int64_t>(cfg.filter_tick_ms_) * 45 * 1000000,
          static_cast<int64_t>(cfg.home_return_delay_ms_) * 1000000, cfg.filter_enabled_,
          cfg.home_return_enabled_};
}

ControlDecision step_control(ControlState &state, const ControlParameters &params,
                             const ControlInput &input) {
  if (input.manual) {
    // Keeps the target age at zero, so the home delay runs from leaving manual.
    state.phase = ControlPhase::Manual;
    state.last_detection_ns = input.now_ns;
    ControlDecision decision{state.phase, false};
    if (input.driving) {
      decision.leader = input.pose_usable ? MotorAction::Manual : MotorAction::Hold;
      decision.followers = input.pose_usable ? MotorAction::Mirror : MotorAction::Hold;
    }
    return decision;
  }
  // AtHome would be wrong: the operator has moved the gimbal away.
  if (state.phase == ControlPhase::Manual)
    state.phase = ControlPhase::Tracking;

  if (input.raw_detection) {
    state.phase = ControlPhase::Tracking;
    state.last_detection_ns = input.now_ns;
  }
  const auto age = input.now_ns - state.last_detection_ns;
  const bool aiming = params.filter_enabled
                        ? input.estimate_available && age < params.stale_after_ns
                        : input.raw_detection && input.driving;
  ControlDecision decision{state.phase, aiming};
  if (!input.driving)
    return decision;
  if (!input.pose_usable) {
    decision.aiming = false;
    decision.leader = decision.followers = MotorAction::Hold;
    return decision;
  }
  // Whatever the leader does, including holding and returning home.
  decision.followers = MotorAction::Mirror;
  if (aiming) {
    decision.leader = MotorAction::Aim;
    return decision;
  }

  if (state.phase == ControlPhase::ReturningHome) {
    decision.leader = MotorAction::ReturnHome;
  } else {
    decision.leader = MotorAction::Hold;
    // Preserve the one-tick delay: stop now, start returning on the next tick.
    if (state.phase == ControlPhase::Tracking && params.home_return_enabled &&
        age >= params.home_delay_ns)
      state.phase = ControlPhase::ReturningHome;
  }
  return decision;
}

bool apply_home_feedback(ControlState &state, bool reached_home) {
  if (state.phase != ControlPhase::ReturningHome || !reached_home)
    return false;
  state.phase = ControlPhase::AtHome;
  return true;
}

} // namespace patronus::tracking
