#pragma once

#include <atomic>
#include <cstdint>
#include <string>

namespace patronus::comm {

/// @brief Handles a "shoot" command received from the control client.
/// @note Placeholder — will trigger the firing mechanism in a future revision.
void handleShoot();

/// @brief Run the UDP command listener loop (intended for std::thread).
/// @param host Local address to bind to (e.g., "0.0.0.0").
/// @param port UDP port to listen on for incoming commands.
/// @param running Reference to atomic flag to control loop termination.
void runCommandThread(const std::string &host, uint16_t port, std::atomic<bool> &running);

} // namespace patronus::comm
