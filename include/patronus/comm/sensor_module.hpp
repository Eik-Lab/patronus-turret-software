#pragma once

#include <termios.h>

#include <atomic>
#include <string>

namespace patronus::comm {

/// @brief Sensor data read from the serial port.
struct SensorData {
  float distance = 0.0F;  ///< Ultrasonic distance (cm).
  double latitude = 0.0;  ///< GPS latitude (decimal degrees).
  double longitude = 0.0; ///< GPS longitude (decimal degrees).
  bool has_distance = false;
  bool has_gps = false;
};

/// @brief Run sensor data reading loop (intended for std::thread).
///        Logs each received distance/GPS packet. Exits when `running`
///        goes false or a read error occurs.
/// @param port     Serial port path (e.g., "/dev/ttyACM0").
/// @param baud_str Baud rate string (e.g., "B115200").
/// @param running  Atomic flag controlling loop termination.
void run_sensor_thread(const std::string &port, const std::string &baud_str,
                       std::atomic<bool> &running);

} // namespace patronus::comm
