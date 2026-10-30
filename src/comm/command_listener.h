#pragma once

#include "tracking/manual_control.h"

#include <atomic>
#include <cstdint>
#include <string>

namespace patronus::comm {

void handle_capture_image();

void run_command_thread(const std::string &host, uint16_t port, std::atomic<bool> &running,
                        patronus::tracking::ManualControl &manual);

} // namespace patronus::comm
