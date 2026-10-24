#include "comm/command_listener.h"

#include "pipeline/inference_mono.h"
#include "pipeline/inference_rgb.h"

#include "log.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <fcntl.h>
#include <termios.h>

#include <cmath>
#include <cstring>
#include <atomic>
#include <string>

namespace patronus::comm {

namespace {

  // Exposure range in microseconds, split into steps spaced evenly on a log scale so every
  // inc_exp:<camera> / dec_exp:<camera> command changes brightness by the same ratio.
  constexpr double exposure_min_us = 50.0;
  constexpr double exposure_max_us = 20000.0;
  constexpr int exposure_steps = 40;
  const double exposure_step_factor =
    std::pow(exposure_max_us / exposure_min_us, 1.0 / (exposure_steps - 1));

  void adjust_exposure(const std::string &camera, double factor) {
    if (camera == "1") {
      patronus::pipeline::adjust_exposure_rgb(factor, exposure_min_us, exposure_max_us);
    } else {
      patronus::pipeline::adjust_exposure_mono(factor, exposure_min_us, exposure_max_us);
    }
  }

  void decrease_exposure(const std::string &camera) {
    adjust_exposure(camera, 1.0 / exposure_step_factor);
  }

  void increase_exposure(const std::string &camera) {
    adjust_exposure(camera, exposure_step_factor);
  }

  speed_t baud_string_to_speed(std::string_view baud_str) {
    if (baud_str == "B115200") return B115200;
    return B115200;
  }

  std::atomic<int> shoot_fd{-1};
  std::atomic<bool> is_shooting{false};

  void send_shoot_command(const std::string &command) {
    int fd = shoot_fd.load();
    if (fd < 0) {
      LOG(Warning, "Shoot controller not connected");
      return;
    }

    std::string cmd = command + "\n";
    if (write(fd, cmd.c_str(), cmd.length()) < 0) {
      LOG(Error, "Failed to write to shoot controller: " << std::strerror(errno));
    }
  }

} // namespace

void handleShoot() {
    send_shoot_command("fire");
    sleep(1);
    send_shoot_command("hold");
}

void runCommandThread(const std::string &host, uint16_t port,
                     const std::string &shoot_port, const std::string &shoot_baud,
                     std::atomic<bool> &running) {
  // Open shoot controller serial port
  int fd = open(shoot_port.c_str(), O_RDWR | O_NOCTTY);
  if (fd >= 0) {
    termios tty{};
    tcgetattr(fd, &tty);
    cfmakeraw(&tty);
    speed_t baud = baud_string_to_speed(shoot_baud);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    tty.c_cflag |= CS8 | CLOCAL | CREAD;
    tcsetattr(fd, TCSANOW, &tty);
    shoot_fd.store(fd);
    LOG(Info, "Shoot controller: " << shoot_port);
  } else {
    LOG(Warning, "Shoot controller unavailable");
  }

  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    LOG(Error, "CommandListener: Failed to create socket");
    return;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = inet_addr(host.c_str());

  if (bind(sockfd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    LOG(Error, "CommandListener: Failed to bind to " << host << ':' << port);
    close(sockfd);
    return;
  }

  timeval timeout{1, 0};
  setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  LOG(Info, "CommandListener: Listening on " << host << ':' << port);

  char buffer[1024];
  while (running.load()) {
    ssize_t n = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0, nullptr, nullptr);
    if (n < 0)
      continue;

    buffer[n] = '\0';
    std::string command(buffer);

    // Commands may carry an argument after a colon, e.g. "inc_exp:1".
    const std::string::size_type colon = command.find(':');
    const std::string name = command.substr(0, colon);
    const std::string argument = (colon == std::string::npos) ? "" : command.substr(colon + 1);

    if (command == "shoot") {
      handleShoot();
    } else if (name == "inc_exp") {
      increase_exposure(argument);
    } else if (name == "dec_exp") {
      decrease_exposure(argument);
    } else {
    }
  }

  close(sockfd);
  fd = shoot_fd.load();
  if (fd >= 0) close(fd);
  LOG(Info, "CommandListener: Stopped");
}

} // namespace patronus::comm
