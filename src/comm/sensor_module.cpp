#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>

struct SensorData
{
    float distance = 0.0F;
    std::string gps;
};

int openSerialPort(const char* portname)
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

int main()
// this functions is for testing the data reading
{
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