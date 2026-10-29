#pragma once

#include "core/config.h"

#include <cstdint>

namespace patronus::tracking {

// Values are part of the telemetry/replay format.
enum class ControlPhase { Tracking, ReturningHome, AtHome };
enum class MotorAction { None, Aim, Hold, ReturnHome };

struct ControlParameters {
  int64_t stale_after_ns;
  int64_t home_delay_ns;
  bool filter_enabled;
  bool home_return_enabled;
};

struct ControlState {
  ControlPhase phase{ControlPhase::Tracking};
  int64_t last_detection_ns; ///< Initialize to the loop's startup time.
};

struct ControlInput {
  int64_t now_ns;          ///< Tick-entry time, before pose reads and mailbox wait.
  bool raw_detection;      ///< This tick's observation, even if rejected by the filter.
  bool estimate_available; ///< Retained estimate, including ticks with no frame.
  bool driving;
  bool pose_usable{true}; ///< Both axes valid and fresh; legacy replay assumes true.
};

struct ControlDecision {
  ControlPhase phase; ///< Telemetry phase before end-of-tick home transitions.
  bool aiming;
  MotorAction leader{MotorAction::None};
  MotorAction followers{MotorAction::None};
};

[[nodiscard]] ControlParameters make_control_parameters(const config::TrackingConfig &cfg);

/// No clocks, allocation or I/O. Actions describe intent, not successful motion.
[[nodiscard]] ControlDecision step_control(ControlState &state, const ControlParameters &params,
                                           const ControlInput &input);

/// Apply only after executing ReturnHome. True means stop the leader and log arrival.
[[nodiscard]] bool apply_home_feedback(ControlState &state, bool reached_home);

} // namespace patronus::tracking
