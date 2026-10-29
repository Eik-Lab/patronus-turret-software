#include "tracking/control_state.h"

namespace patronus::tracking {

ControlParameters make_control_parameters(const config::TrackingConfig &cfg) {
  return {static_cast<int64_t>(cfg.filter_tick_ms_) * 45 * 1000000,
          static_cast<int64_t>(cfg.home_return_delay_ms_) * 1000000, cfg.filter_enabled_,
          cfg.home_return_enabled_};
}

ControlDecision step_control(ControlState &state, const ControlParameters &params,
                             const ControlInput &input) {
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
  if (aiming) {
    decision.leader = MotorAction::Aim;
    decision.followers = MotorAction::Aim;
    return decision;
  }

  decision.followers =
    state.phase == ControlPhase::Tracking ? MotorAction::Hold : MotorAction::ReturnHome;
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
