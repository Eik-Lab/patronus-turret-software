#pragma once

#include "patronus/core/config.hpp"

#include <atomic>

namespace patronus::tools {

/// @brief Spin each gimbal briefly to verify the CAN bus, then exit.
///
/// Standalone check for a bring-up where the cameras are not yet available:
/// confirms the CANdle device opens, every configured motor ID resolves, and the
/// encoders report back. Requires a clear field and the turret unloaded.
///
/// @param cfg     Loaded system configuration.
/// @param running Set to false to abort early; the run is bounded by its own
///                timeout regardless.
/// @return 0 if every gimbal responded, 1 otherwise.
int run_motor_test(const patronus::config::SystemConfig &cfg, std::atomic<bool> &running);

} // namespace patronus::tools