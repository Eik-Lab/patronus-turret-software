#include "comm/sensor_module.h"

#include "log.h"

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
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

    LOG(Warning, "Unsupported baud rate string '" << baud_str << "', defaulting to B115200");
    return B115200;
  }

  // The CANdle (USB vendor 0069) also enumerates as /dev/ttyACM*.
  bool is_candle_port(const char *portname) {
    char resolved[PATH_MAX]{};
    if (realpath(portname, resolved) == nullptr)
      return false;
    const std::string_view path{resolved};
    const std::string tty_name{path.substr(path.find_last_of('/') + 1)};
    const std::string vendor_path = "/sys/class/tty/" + tty_name + "/device/../idVendor";

    std::FILE *file = std::fopen(vendor_path.c_str(), "r");
    if (file == nullptr)
      return false;
    char vendor[8]{};
    const bool have_vendor = std::fgets(vendor, sizeof(vendor), file) != nullptr;
    std::fclose(file);
    return have_vendor && std::strncmp(vendor, "0069", 4) == 0;
  }

  int serial_open(const char *portname) {
    const int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
      LOG(Error, "Error opening " << portname << ": " << std::strerror(errno));
    return fd;
  }

  bool serial_configure(int fd, speed_t speed) {
    termios tty{};

    if (tcgetattr(fd, &tty) == -1) {
      LOG(Error, "tcgetattr: " << std::strerror(errno));
      return false;
    }

    cfmakeraw(&tty);

    if (cfsetispeed(&tty, speed) == -1 || cfsetospeed(&tty, speed) == -1) {
      LOG(Error, "Failed to set speed: " << std::strerror(errno));
      return false;
    }

    tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |= CS8 | CLOCAL | CREAD;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    if (tcsetattr(fd, TCSANOW, &tty) == -1) {
      LOG(Error, "tcsetattr: " << std::strerror(errno));
      return false;
    }

    if (tcflush(fd, TCIFLUSH) == -1) {
      LOG(Error, "tcflush: " << std::strerror(errno));
      return false;
    }

    return true;
  }

  // NMEA ddmm.mmmm to signed decimal degrees.
  double nmea_to_decimal(double value, char direction) {
    const double degrees = std::floor(value / 100.0);
    const double minutes = value - (degrees * 100.0);
    double result = degrees + (minutes / 60.0);

    if (direction == 'S' || direction == 'W')
      result = -result;

    return result;
  }

  bool read_sensor_data(int fd, SensorData &data, const std::atomic<bool> &running) {
    std::string line;

    while (running.load()) {
      // A hung-up tty makes read() return 0 at once, like a timeout; poll instead.
      pollfd pfd{fd, POLLIN, 0};
      const int ready = poll(&pfd, 1, 500);

      if (ready < 0) {
        if (errno == EINTR)
          continue;
        LOG(Error, "Sensor poll error: " << std::strerror(errno));
        return false;
      }

      if (ready == 0)
        continue;

      if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        LOG(Warning, "Sensor serial port disconnected");
        return false;
      }

      char character = '\0';
      const ssize_t bytes_read = read(fd, &character, 1);

      if (bytes_read < 0) {
        if (errno == EINTR)
          continue;
        LOG(Error, "Sensor read error: " << std::strerror(errno));
        return false;
      }

      if (bytes_read == 0) {
        LOG(Warning, "Sensor serial port closed");
        return false;
      }

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

    return false;
  }

} // namespace

void run_sensor_thread(const std::string &port, const std::string &baud_str,
                       std::atomic<bool> &running) {
  const speed_t baud_speed = baud_string_to_speed(baud_str);

  if (is_candle_port(port.c_str())) {
    LOG(Warning, "Sensor module: "
                   << port
                   << " is the CANdle adapter, not a sensor — sensor disabled. Set [sensor] port "
                      "to the sensor's device or enabled=false.");
    return;
  }

  const int fd = serial_open(port.c_str());
  if (fd < 0) {
    LOG(Error, "Failed to open sensor serial port: " << port);
    return;
  }

  if (!serial_configure(fd, baud_speed)) {
    LOG(Error, "Failed to configure sensor serial port");
    close(fd);
    return;
  }

  LOG(Info, "Sensor module started: port=" << port << " baud=" << baud_str);

  while (running.load()) {
    SensorData data{};

    if (!read_sensor_data(fd, data, running)) {
      if (running.load())
        LOG(Warning, "Sensor module: stopped reading " << port);
      break;
    }

    if (data.has_distance)
      LOG(Info, "Distance: " << std::fixed << std::setprecision(2) << data.distance << " cm");

    if (data.has_gps)
      LOG(Info, "GPS: latitude=" << std::fixed << std::setprecision(7) << data.latitude
                                 << " longitude=" << data.longitude);
  }

  close(fd);
  LOG(Info, "Sensor module stopped");
}

} // namespace patronus::comm
