#include <string>
#include <array>
#include <sstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

int serial_open(const char* port, int baud_rate = B115200){
    int fd = open(port, O_RDWR | O_NOCTTY);
    termios tty{};
    tcgetattr(fd, &tty);
    cfsetspeed(&tty, baud_rate);
    tty.c_cflag |= (CLOCAL | CREAD | CS8);
    tty.c_cflag &= ~(PARENB | CSTOPB);
    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

void motor_send(int fd, float sensor_pan, float sensor_tilt, float shooter_pan, float shooter_tilt){
    //sending commands to motors, given fixed format 
    std::ostringstream cmd;
    cmd << 'A' << sensor_pan << 'B' << sensor_tilt << 'C' << shooter_pan << 'D' << shooter_tilt << '\n';
    write(fd, cmd.str().c_str(), cmd.str().size());
}

std::array<float, 4> motor_read(int fd){
    fd_set set;
    FD_ZERO(&set);
    FD_SET(fd, &set);
    timeval timeout = {0, 200000}; 

    if (select(fd + 1, &set, NULL, NULL, &timeout) <= 0){
        fprintf(stderr, "motor_read: timeout\n");
        return {};
    }

    char line[64];
    int len = read(fd, line, sizeof(line) - 1);
    if (len < 0) {
        printf("no data motor"); 
        return {}; 
    }
    line[len] = '\0';

    std::array<float, 4> positions{};
    int parsed = sscanf(line, "A%fB%fC%fD%f", &positions[0], &positions[1], &positions[2], &positions[3]);
    if (parsed != 4) {
        fprintf(stderr, "motor_read: unknown format '%s'\n", line);
        return {};
    }
    return positions;
}

std::array<float, 2> sensor_read(int fd){
    fd_set set;
    FD_ZERO(&set);
    FD_SET(fd, &set);
    timeval timeout = {0, 200000};

    if (select(fd + 1, &set, NULL, NULL, &timeout) <= 0){
        fprintf(stderr, "sensor_read: timeout\n");
        return {};
    }

    char line[64];
    int len = read(fd, line, sizeof(line) - 1);
    if (len < 0) { 
        printf("no data sensor"); 
        return {};
    }
    line[len] = '\0';

    std::array<float, 2> sensor{};
    if (strncmp(line, "Distance: ", 10) == 0)
        sscanf(line, "Distance: %f", &sensor[0]);
    else if (strncmp(line, "$GNGGA", 6) == 0)
        sscanf(line, "$GNGGA,%*f,%f", &sensor[1]);
    else
        fprintf(stderr, "sensor_read: unknown format '%s'\n", line);
        return {};

    return sensor;
}


