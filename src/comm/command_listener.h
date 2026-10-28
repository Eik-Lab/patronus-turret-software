#pragma once

#include <atomic>
#include <cstdint>
#include <string>

namespace patronus::comm {

void handle_capture_image();

void run_command_thread(const std::string &host, uint16_t port, std::atomic<bool> &running);

} // namespace patronus::comm
