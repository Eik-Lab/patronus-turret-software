#include "comm/command_listener.h"

#include "core/state.h"
#include "pipeline/inference_mono.h"
#include "pipeline/inference_rgb.h"

#include "log.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <cstring>

namespace patronus::comm {

namespace {

  // Log-spaced steps, so each command changes brightness by the same ratio.
  // The argument picks the camera: "1" is rgb, anything else mono.
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

} // namespace

// TODO: tell the image camera to take an image.
void handle_capture_image() {
}

void run_command_thread(const std::string &host, uint16_t port, std::atomic<bool> &running,
                        patronus::tracking::ManualControl &manual) {
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

  LOG(Info, "CommandListener: Listening for commands on " << host << ':' << port);

  char buffer[1024];
  while (running.load()) {
    ssize_t n = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0, nullptr, nullptr);
    if (n < 0)
      continue;

    buffer[n] = '\0';
    std::string command(buffer);

    const std::string::size_type colon = command.find(':');
    const std::string name = command.substr(0, colon);
    const std::string argument = (colon == std::string::npos) ? "" : command.substr(colon + 1);

    if (command == "capture_image") {
      handle_capture_image();
    } else if (name == "inc_exp") {
      increase_exposure(argument);
    } else if (name == "dec_exp") {
      decrease_exposure(argument);
    } else if (name == "mode") {
      if (manual.set_mode(argument))
        LOG(Warning, "CommandListener: Control mode " << argument);
    } else if (name == "move") {
      manual.move(argument, patronus::core::steady_now_s());
    } else {
    }
  }

  close(sockfd);
  LOG(Info, "CommandListener: Stopped");
}

} // namespace patronus::comm
