#pragma once

#include <array>
#include <string>

int serial_open(const char *port, int baud_rate = 115200);

void motor_send(int fd, float sensor_pan, float sensor_tilt, float shooter_pan, float shooter_tilt);

std::array<float, 4> motor_read(int fd);

std::array<float, 2> sensor_read(int fd);
