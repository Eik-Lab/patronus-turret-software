#pragma once

#include <atomic>
#include <cstdint>
#include <string>

namespace patronus::comm {

void handleShoot();

void runCommandThread(const std::string &host, uint16_t port, std::atomic<bool> &running);

} // namespace patronus::comm
