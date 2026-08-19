#include "patronus/comm/sensor_module.hpp"

#include <glib.h>

#include <atomic>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>

namespace patronus::comm {

speed_t baudStringToSpeed(const std::string &baud_str) {
  if (baud_str == "B9600")   return B9600;
  if (baud_str == "B19200")  return B19200;
  if (baud_str == "B38400")  return B38400;
  if (baud_str == "B57600")  return B57600;
  if (baud_str == "B115200") return B115200;

  g_warning("Unsupported baud rate string '%s', defaulting to B115200", baud_str.c_str());
  return B115200;
}

int openSerialPort(const char *portname)
{
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);

    if (fd < 0) {
        std::cerr << "Error opening " << portname << ": "
                  << std::strerror(errno) << '\n';
    }

    return fd;
}

bool configureSerialPort(int fd, speed_t speed)
{
    termios tty{};

    if (tcgetattr(fd, &tty) == -1) {
        std::cerr << "tcgetattr: "
                  << std::strerror(errno) << '\n';
        return false;
    }

    cfmakeraw(&tty);

    if (cfsetispeed(&tty, speed) == -1 ||
        cfsetospeed(&tty, speed) == -1) {
        std::cerr << "Failed to set speed: "
                  << std::strerror(errno) << '\n';
        return false;
    }

    tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |= CS8 | CLOCAL | CREAD;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    if (tcsetattr(fd, TCSANOW, &tty) == -1) {
        std::cerr << "tcsetattr: "
                  << std::strerror(errno) << '\n';
        return false;
    }

    tcflush(fd, TCIFLUSH);
    return true;
}

bool readSensorData(int fd, SensorData& data)
{
    std::string line;
    bool receivedDistance = false;
    bool receivedGps = false;

    while (!receivedDistance || !receivedGps) {
        char character = '\0';
        ssize_t bytesRead = read(fd, &character, 1);

        if (bytesRead < 0) {
            if (errno == EINTR) {
                continue;
            }

            std::cerr << "Read error: "
                      << std::strerror(errno) << '\n';
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

        if (line.compare(0, 9, "DISTANCE:") == 0) {
            const char* valueStart = line.c_str() + 9;
            char* valueEnd = nullptr;

            errno = 0;
            float value = std::strtof(valueStart, &valueEnd);

            if (valueEnd != valueStart && errno != ERANGE) {
                data.distance = value;
                receivedDistance = true;
            }
        } else if (line.compare(0, 4, "GPS:") == 0) {
            data.gps = line.substr(4);
            receivedGps = true;
        }

        line.clear();
    }

    return true;
}

void closeSerialPort(int fd)
{
        close(fd);
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

  while (running) {
    SensorData data;
    if (readSensorData(fd, data)) {
      g_print("Sensor: distance=%.2f cm  GPS=%s\n", data.distance, data.gps.c_str());
    } else {
      g_warning("Failed to read sensor data");
      break;
    }
  }

  closeSerialPort(fd);
  g_print("Sensor module stopped\n");
}

} // namespace patronus::comm

// ── Standalone test main (comment out when building as library) ──────────────
#ifdef SENSOR_MODULE_TEST_MAIN
int main()
{
    using patronus::comm::closeSerialPort;
    using patronus::comm::configureSerialPort;
    using patronus::comm::openSerialPort;
    using patronus::comm::readSensorData;
    using patronus::comm::SensorData;

    int fd = openSerialPort("/dev/ttyACM0");

    if (fd < 0) {
        return 1;
    }

    if (!configureSerialPort(fd, B115200)) {
        closeSerialPort(fd);
        return 1;
    }

    while (true) {
        SensorData data;

        if (!readSensorData(fd, data)) {
            break;
        }

        std::cout << "Distance: "
                  << data.distance << " cm\n";

        std::cout << "GPS: "
                  << data.gps << '\n';
    }

    closeSerialPort(fd);
    return 0;
}
#endif