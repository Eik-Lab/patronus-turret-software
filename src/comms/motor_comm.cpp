#include "communication.hpp"

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>

Communication::Communication(const std::string& port, int baud)
{
    fd = open(port.c_str(), O_RDWR | O_NOCTTY);

    if (fd < 0) {
        std::cerr << "Failed to open port\n";
        return;
    }

    struct termios tty{};
    tcgetattr(fd, &tty);

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tcsetattr(fd, TCSANOW, &tty);

    std::cout << "Connected\n";
}

bool Communication::send(float pan, float tilt)
{
    std::string msg =
        "X" + std::to_string(pan) +
        "Y" + std::to_string(tilt) + "\n";

    int n = write(fd, msg.c_str(), msg.size());

    return n > 0;
}


std::string Communication::read()
{
    char buffer[128];
    int n = ::read(fd, buffer, sizeof(buffer) - 1);

    if (n <= 0)
        return "";

    buffer[n] = '\0';
    return std::string(buffer);
}