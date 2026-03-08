#pragma once
#include <string>

class Communication {
public:
    Communication(const std::string& port, int baud = 115200);

    bool send(float pan, float tilt);
    std::string read();

private:
    int fd;
};