#pragma once

#include <array>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

inline int serial_open(const char *port, int baud_rate = B115200)
{
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        fprintf(stderr, "serial_open: failed to open %s: %s\n", port, strerror(errno));
        return -1;
    }
    termios tty{};
    if (tcgetattr(fd, &tty) != 0)
    {
        fprintf(stderr, "serial_open: tcgetattr failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }
    cfsetspeed(&tty, baud_rate);
    tty.c_cflag |= (CLOCAL | CREAD | CS8);
    tty.c_cflag &= ~(PARENB | CSTOPB);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        fprintf(stderr, "serial_open: tcsetattr failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }
    return fd;
}

inline float motor_limit(float value)
{
    if (value >= 0.0f && value < 0.1f)
        return 0.0f;
    if (value >= 0.1f && value < 0.2f)
        return 0.2f;
    return value;
}

inline void motor_send(int fd, float sensor_pan, float sensor_tilt, float shooter_pan, float shooter_tilt)
{
    std::ostringstream cmd;

    cmd << 'A' << motor_limit(sensor_pan) << 'B' << motor_limit(sensor_tilt) << 'C' << motor_limit(shooter_pan) << 'D' << motor_limit(shooter_tilt) << '\n';
    std::string out = cmd.str();
    std::cout << "SEND: " << out;
    write(fd, out.c_str(), out.size());
}

inline bool read_line(int fd, std::string &out_line)
{
    static std::string buffer;
    char temp[64];
    int len = read(fd, temp, sizeof(temp));
    if (len <= 0)
        return false;
    buffer.append(temp, len);
    size_t pos;
    while ((pos = buffer.find('\n')) != std::string::npos)
    {
        out_line = buffer.substr(0, pos);
        buffer.erase(0, pos + 1);
        return true;
    }
    return false;
}

inline std::array<float, 4> motor_read(int fd)
{
    std::string line;
    if (!read_line(fd, line))
    {
        return {};
    }
    std::array<float, 4> positions{};

    int parsed = sscanf(line.c_str(), "A%fB%fC%fD%f", &positions[0], &positions[1], &positions[2], &positions[3]);
    if (parsed != 4)
    {
        fprintf(stderr, "motor_read: bad format '%s'\n", line.c_str());
        return {};
    }
    return positions;
}

inline std::array<float, 2> sensor_read(int fd)
{
    std::string line;

    if (!read_line(fd, line))
    {
        return {};
    }
    std::array<float, 2> sensor{};

    if (strncmp(line.c_str(), "Distance: ", 10) == 0)
    {
        sscanf(line.c_str(), "Distance: %f", &sensor[0]);
    }
    else if (strncmp(line.c_str(), "$GNGGA", 6) == 0)
    {
        sscanf(line.c_str(), "$GNGGA,%*f,%f", &sensor[1]);
    }
    else
    {
        fprintf(stderr, "sensor_read: unknown format '%s'\n", line.c_str());
        return {};
    }

    return sensor;
}