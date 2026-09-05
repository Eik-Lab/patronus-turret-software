#include "patronus/comm/command_listener.hpp"

#include <glib.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace patronus::comm {

void handleShoot() {
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

    if (command == "shoot") {
      handleShoot();
    } else {
      g_print(":CommandListener: Received command: %s\n", command.c_str());
    }
  }

  close(sockfd);
  g_print(":CommandListener: Stopped\n");
}

} // namespace patronus::comm
