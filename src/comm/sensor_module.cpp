#include "patronus/comm/sensor_module.hpp"

#include <glib.h>

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>

namespace patronus::comm {

namespace {

  speed_t baud_string_to_speed(std::string_view baud_str) {
    if (baud_str == "B9600")
      return B9600;
    if (baud_str == "B19200")
      return B19200;
    if (baud_str == "B38400")
      return B38400;
    if (baud_str == "B57600")
      return B57600;
    if (baud_str == "B115200")
      return B115200;

    g_warning("Unsupported baud rate string '%.*s', defaulting to B115200",
              static_cast<int>(baud_str.size()), baud_str.data());
    return B115200;
  }

  int serial_open(const char *portname) {
    const int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
      g_warning("Error opening %s: %s", portname, std::strerror(errno));
    return fd;
  }

  bool serial_configure(int fd, speed_t speed) {
    termios tty{};

    if (tcgetattr(fd, &tty) == -1) {
      g_warning("tcgetattr: %s", std::strerror(errno));
      return false;
    }

    cfmakeraw(&tty);

    if (cfsetispeed(&tty, speed) == -1 || cfsetospeed(&tty, speed) == -1) {
      g_warning("Failed to set speed: %s", std::strerror(errno));
      return false;
    }

    tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |= CS8 | CLOCAL | CREAD;

    tty.c_cc[VMIN] = 0;  // Return immediately...
    tty.c_cc[VTIME] = 5; // ...after at most 0.5 s.

    if (tcsetattr(fd, TCSANOW, &tty) == -1) {
      g_warning("tcsetattr: %s", std::strerror(errno));
      return false;
    }

    if (tcflush(fd, TCIFLUSH) == -1) {
      g_warning("tcflush: %s", std::strerror(errno));
      return false;
    }

    return true;
  }

  // Convert NMEA ddmm.mmmm + hemisphere to signed decimal degrees.
  double nmea_to_decimal(double value, char direction) {
    const double degrees = std::floor(value / 100.0);
    const double minutes = value - degrees * 100.0;
    double result = degrees + minutes / 60.0;

    if (direction == 'S' || direction == 'W')
      result = -result;

    return result;
  }

  // Read one complete Distance: or GLL packet from the serial port.
  // Blocking but bounded: VMIN=0/VTIME=5 caps each read at 0.5 s.
  bool read_sensor_data(int fd, SensorData &data) {
    std::string line;

    while (true) {
      char character = '\0';
      const ssize_t bytes_read = read(fd, &character, 1);

      if (bytes_read < 0) {
        if (errno == EINTR)
          continue;
        g_warning("Sensor read error: %s", std::strerror(errno));
        return false;
      }

      if (bytes_read == 0) // Timeout with no data — keep polling.
        continue;

      if (character == '\r')
        continue;

      if (character != '\n') {
        line.push_back(character);
        continue;
      }

      if (line.compare(0, 9, "Distance:") == 0) {
        const char *value_start = line.c_str() + 9;
        char *value_end = nullptr;

        errno = 0;
        const float value = std::strtof(value_start, &value_end);

        if (value_end != value_start && errno != ERANGE) {
          data.distance = value;
          data.has_distance = true;
          return true;
        }
      } else if (line.compare(0, 6, "$GNGLL") == 0 || line.compare(0, 6, "$GPGLL") == 0) {
        double latitude = 0.0;
        double longitude = 0.0;
        char latitude_direction = '\0';
        char longitude_direction = '\0';
        char utc_time[32]{};
        char status = '\0';

        const int fields =
          std::sscanf(line.c_str() + 7, "%lf,%c,%lf,%c,%31[^,],%c", &latitude, &latitude_direction,
                      &longitude, &longitude_direction, utc_time, &status);

        if (fields == 6 && status == 'A') {
          data.latitude = nmea_to_decimal(latitude, latitude_direction);
          data.longitude = nmea_to_decimal(longitude, longitude_direction);
          data.has_gps = true;
          return true;
        }
      }

      line.clear();
    }
  }

} // namespace

void run_sensor_thread(const std::string &port, const std::string &baud_str,
                       std::atomic<bool> &running) {
  const speed_t baud_speed = baud_string_to_speed(baud_str);

  const int fd = serial_open(port.c_str());
  if (fd < 0) {
    g_critical("Failed to open sensor serial port: %s", port.c_str());
    return;
  }

  if (!serial_configure(fd, baud_speed)) {
    g_critical("Failed to configure sensor serial port");
    close(fd);
    return;
  }

  g_print("Sensor module started: port=%s baud=%s\n", port.c_str(), baud_str.c_str());

  while (running.load()) {
    SensorData data{};

    if (!read_sensor_data(fd, data)) {
      g_warning("Failed to read sensor data");
      break;
    }

    if (data.has_distance)
      g_print("Distance: %.2f cm\n", data.distance);

    if (data.has_gps)
      g_print("GPS: latitude=%.7f longitude=%.7f\n", data.latitude, data.longitude);
  }

  close(fd);
  g_print("Sensor module stopped\n");
}

} // namespace patronus::comm
