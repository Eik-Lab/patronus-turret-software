#include "patronus/comm/sensor_module.hpp"

#include <glib.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

namespace patronus::comm {

speed_t baudStringToSpeed(const std::string &baud_str) {
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

  g_warning("Unsupported baud rate string '%s', defaulting to B115200", baud_str.c_str());

  return B115200;
}

int openSerialPort(const char *portname) {
  int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);

  if (fd < 0) {
    std::cerr << "Error opening " << portname << ": " << std::strerror(errno) << '\n';
  }

  return fd;
}

bool configureSerialPort(int fd, speed_t speed) {
  termios tty{};

  if (tcgetattr(fd, &tty) == -1) {
    std::cerr << "tcgetattr: " << std::strerror(errno) << '\n';
    return false;
  }

  cfmakeraw(&tty);

  if (cfsetispeed(&tty, speed) == -1 || cfsetospeed(&tty, speed) == -1) {
    std::cerr << "Failed to set speed: " << std::strerror(errno) << '\n';
    return false;
  }

  tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
  tty.c_cflag |= CS8 | CLOCAL | CREAD;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 5;

  if (tcsetattr(fd, TCSANOW, &tty) == -1) {
    std::cerr << "tcsetattr: " << std::strerror(errno) << '\n';
    return false;
  }

  if (tcflush(fd, TCIFLUSH) == -1) {
    std::cerr << "tcflush: " << std::strerror(errno) << '\n';
    return false;
  }

  return true;
}

double nmeaToDecimal(double value, char direction) {
  double degrees = std::floor(value / 100.0);
  double minutes = value - degrees * 100.0;
  double result = degrees + minutes / 60.0;

  if (direction == 'S' || direction == 'W') {
    result = -result;
  }

  return result;
}

bool readSensorData(int fd, SensorData &data) {
  std::string line;

  while (true) {
    char character = '\0';
    ssize_t bytesRead = read(fd, &character, 1);

    if (bytesRead < 0) {
      if (errno == EINTR) {
        continue;
      }

      std::cerr << "Read error: " << std::strerror(errno) << '\n';
      return false;
    }

    if (bytesRead == 0) {
      continue;
    }

    if (character == '\r') {
      continue;
    }

    if (character != '\n') {
      line.push_back(character);
      continue;
    }

    if (line.compare(0, 9, "Distance:") == 0) {
      const char *valueStart = line.c_str() + 9;
      char *valueEnd = nullptr;

      errno = 0;

      float value = std::strtof(valueStart, &valueEnd);

      if (valueEnd != valueStart && errno != ERANGE) {
        data.distance = value;
        data.has_distance = true;
        return true;
      }
    }

    if (line.compare(0, 6, "$GNGLL") == 0 || line.compare(0, 6, "$GPGLL") == 0) {
      double latitude = 0.0;
      double longitude = 0.0;
      char latitudeDirection = '\0';
      char longitudeDirection = '\0';
      char utcTime[32]{};
      char status = '\0';

      int fields =
        std::sscanf(line.c_str() + 7, "%lf,%c,%lf,%c,%31[^,],%c", &latitude, &latitudeDirection,
                    &longitude, &longitudeDirection, utcTime, &status);

      if (fields == 6 && status == 'A') {
        data.latitude = nmeaToDecimal(latitude, latitudeDirection);

        data.longitude = nmeaToDecimal(longitude, longitudeDirection);

        data.has_gps = true;
        return true;
      }
    }

    line.clear();
  }
}

void closeSerialPort(int fd) {
  if (fd >= 0) {
    close(fd);
  }
}

void runSensorThread(const std::string &port, const std::string &baud_str,
                     std::atomic<bool> &running) {
  speed_t baud_speed = baudStringToSpeed(baud_str);

  int fd = openSerialPort(port.c_str());

  if (fd < 0) {
    g_critical("Failed to open sensor serial port: %s", port.c_str());
    return;
  }

  if (!configureSerialPort(fd, baud_speed)) {
    g_critical("Failed to configure sensor serial port");

    closeSerialPort(fd);
    return;
  }

  g_print("Sensor module started: port=%s baud=%s\n", port.c_str(), baud_str.c_str());

  while (running.load()) {
    SensorData data{};

    if (!readSensorData(fd, data)) {
      g_warning("Failed to read sensor data");
      break;
    }

    if (data.has_distance) {
      g_print("Distance: %.2f cm\n", data.distance);
    }

    if (data.has_gps) {
      g_print("GPS: latitude=%.7f longitude=%.7f\n", data.latitude, data.longitude);
    }
  }

  closeSerialPort(fd);
  g_print("Sensor module stopped\n");
}

} // namespace patronus::comm

#ifdef SENSOR_MODULE_TEST_MAIN

int main(int argc, char *argv[]) {
  using patronus::comm::closeSerialPort;
  using patronus::comm::configureSerialPort;
  using patronus::comm::openSerialPort;
  using patronus::comm::readSensorData;
  using patronus::comm::SensorData;

  const char *port = argc > 1 ? argv[1] : "/dev/ttyACM0";

  int fd = openSerialPort(port);

  if (fd < 0) {
    return 1;
  }

  if (!configureSerialPort(fd, B115200)) {
    closeSerialPort(fd);
    return 1;
  }

  std::cout << "Waiting for sensor data on " << port << "...\n";

  while (true) {
    SensorData data{};

    if (!readSensorData(fd, data)) {
      break;
    }

    if (data.has_distance) {
      std::cout << "Distance: " << data.distance << " cm\n";
    }

    if (data.has_gps) {
      std::cout << "GPS: latitude=" << data.latitude << " longitude=" << data.longitude << '\n';
    }
  }

  closeSerialPort(fd);
  return 0;
}

#endif