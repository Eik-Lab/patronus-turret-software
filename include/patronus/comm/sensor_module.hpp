#pragma once

#include <atomic>
#include <string>
#include <termios.h>

namespace patronus::comm {

/// @brief Sensor data read from the serial port.
struct SensorData {
  float distance{0.0F};  ///< Distance measurement in cm.
  std::string gps;       ///< GPS coordinates as string.
};

/// @brief Convert baud rate string to speed_t.
/// @param baud_str Baud rate string (e.g., "B115200", "B9600").
/// @return Corresponding speed_t value, or B115200 if unknown.
speed_t baudStringToSpeed(const std::string &baud_str);

/// @brief Open a serial port for reading.
/// @param portname Path to the serial port device (e.g., "/dev/ttyACM0").
/// @return File descriptor on success, or negative value on error.
int openSerialPort(const char *portname);

/// @brief Configure serial port settings (baud rate, parity, etc.).
/// @param fd File descriptor of the serial port.
/// @param speed Baud rate (e.g., B115200, B9600).
/// @return true on success, false on failure.
bool configureSerialPort(int fd, speed_t speed);

/// @brief Read one complete sensor data packet from the serial port.
/// @param fd File descriptor of the serial port.
/// @param data Output parameter for the sensor data.
/// @return true on success, false on error.
bool readSensorData(int fd, SensorData &data);

/// @brief Close the serial port.
/// @param fd File descriptor of the serial port.
void closeSerialPort(int fd);

/// @brief Run sensor data reading loop (intended for std::thread).
/// @param port Serial port path (e.g., "/dev/ttyACM0").
/// @param baud_str Baud rate string (e.g., "B115200").
/// @param running Reference to atomic flag to control loop termination.
void runSensorThread(const std::string &port, const std::string &baud_str,
                     std::atomic<bool> &running);

} // namespace patronus::comm
