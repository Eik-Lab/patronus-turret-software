#include "patronus/comm/command_listener.hpp"

#include "patronus/pipeline/inference_mono.hpp"
#include "patronus/pipeline/inference_rgb.hpp"

#include <glib.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace patronus::comm {

namespace {

// Exposure time change per inc_exp:<camera> / dec_exp:<camera> command, in microseconds.
constexpr double exposure_step_us = 50.0;

void adjust_exposure(const std::string &camera, double delta_us) {
  if (camera == "1") {
    patronus::pipeline::adjust_exposure_rgb(delta_us);
  } else {
    patronus::pipeline::adjust_exposure_mono(delta_us);
  }
}

void decrease_exposure(const std::string &camera) {
  adjust_exposure(camera, -exposure_step_us);
}

void increase_exposure(const std::string &camera) {
  adjust_exposure(camera, exposure_step_us);
}

} // namespace

void handleShoot() {
  // send command to arduino, needs to initilize serial port. 
/*   "safety-on": skru på safety
"safety-off": skru av safety
"fire": begynn å skyt
"hold": stopp å skyt */
  g_print("SHOOT\n");

}

void runCommandThread(const std::string &host, uint16_t port, std::atomic<bool> &running) {
  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    g_critical("CommandListener: Failed to create socket");
    return;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = inet_addr(host.c_str());

  if (bind(sockfd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    g_critical("CommandListener: Failed to bind to %s:%u", host.c_str(), port);
    close(sockfd);
    return;
  }

  timeval timeout{1, 0};
  setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  g_print("CommandListener: Listening for commands on %s:%u\n", host.c_str(), port);

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
    }
    else if (name == "inc_exp"){
      increase_exposure(argument);
    }
    else if (name == "dec_exp"){
      decrease_exposure(argument);
    } 
    else {
      g_print(":CommandListener: Received command: %s\n", command.c_str());
    }
  }

  close(sockfd);
  g_print(":CommandListener: Stopped\n");
}

} 




 
