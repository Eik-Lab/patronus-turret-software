#pragma once

#include <atomic>
#include <cstdint>
#include <string>

namespace patronus::comm {

/// @brief Handles a "shoot" command received from the control client.
void handleShoot();

/// @brief Run the UDP command listener loop (intended for std::thread).
/// @param host Local address to bind to (e.g., "0.0.0.0").
/// @param port UDP port to listen on for incoming commands.
/// @param shoot_port Serial port path for shoot controller (e.g., "/dev/ttyUSB0").
/// @param shoot_baud Baud rate string (e.g., "B9600").
/// @param running Reference to atomic flag to control loop termination.
void runCommandThread(const std::string &host, uint16_t port,
                     const std::string &shoot_port, const std::string &shoot_baud,
                     std::atomic<bool> &running);

} // namespace patronus::comm
