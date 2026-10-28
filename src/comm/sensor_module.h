#pragma once

#include <termios.h>

#include <atomic>
#include <string>

namespace patronus::comm {

struct SensorData {
  float distance = 0.0F; ///< cm
  double latitude = 0.0; ///< Signed decimal degrees.
  double longitude = 0.0;
  bool has_distance = false;
  bool has_gps = false;
};

/// The readings are only logged so far.
void run_sensor_thread(const std::string &port, const std::string &baud_str,
                       std::atomic<bool> &running);

} // namespace patronus::comm
