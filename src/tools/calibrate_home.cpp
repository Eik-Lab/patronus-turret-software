#include "patronus/comm/candle_motor.hpp"
#include "patronus/core/config.hpp"

#include <cstdio>
#include <cstring>
#include <thread>
#include <chrono>
#include <sys/select.h>
#include <unistd.h>

using patronus::config::SystemConfig;
using patronus::config::load_config;

int main(int argc, char *argv[])
{
  const char *config_path = "config/system.ini";
  int gimbal_arg = -1;  // -1 = calibrate all gimbals

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--gimbal") == 0 && i + 1 < argc) {
      gimbal_arg = std::atoi(argv[++i]);
    } else if (argv[i][0] != '-') {
      config_path = argv[i];
    }
  }

  SystemConfig cfg = load_config(config_path);

  patronus::comm::CandleMotor motor(cfg.gimbals, cfg.can_bus);
  if (!motor.init()) {
    std::fprintf(stderr, "calibrate-home: motor init failed\n");
    return 1;
  }

  size_t first = (gimbal_arg >= 0) ?
      static_cast<size_t>(gimbal_arg) : 0;
  size_t last  = (gimbal_arg >= 0) ?
      static_cast<size_t>(gimbal_arg) + 1 : motor.gimbal_count();

  if (first >= motor.gimbal_count() || last > motor.gimbal_count()) {
    std::fprintf(stderr, "calibrate-home: gimbal %d not found (have %zu)\n",
                 gimbal_arg, motor.gimbal_count());
    motor.disable();
    return 1;
  }

  for (size_t g = first; g < last; ++g) {
    std::printf("============================================\n");
    std::printf("  Home Calibration — gimbal %zu\n", g);
    std::printf("============================================\n");
    std::printf("Motors enabled at low torque.\n");
    std::printf("Manually position the turret to your desired\n");
    std::printf("home (centre) position.\n");
    std::printf("Live encoder readings update below.\n");
    std::printf("Press ENTER when centred to save to flash.\n\n");

    bool done = false;
    while (!done) {
      for (int i = 0; i < 10 && !done; ++i) {
        auto [pv, tv] = motor.get_velocity(g);
        auto [pp, pt] = motor.get_position(g);
        std::printf("\r  gimbal %zu  pan=%+.3f rad (%+.1f deg)  "
                    "tilt=%+.3f rad (%+.1f deg)  vel %+.3f %+.3f  ",
                    g,
                    pp, pp * 180.0 / 3.14159,
                    pt, pt * 180.0 / 3.14159,
                    pv, tv);
        std::fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      fd_set stdin_set;
      struct timeval tv = {0, 0};
      FD_ZERO(&stdin_set);
      FD_SET(STDIN_FILENO, &stdin_set);
      if (select(STDIN_FILENO + 1, &stdin_set, nullptr, nullptr, &tv) > 0) {
        std::getchar();
        done = true;
      }
    }

    auto [pp_final, pt_final] = motor.get_position(g);
    std::printf("\n\nCalibrating home at pan=%.3f rad tilt=%.3f rad ...\n",
                pp_final, pt_final);

    if (motor.calibrate_home(g)) {
      std::printf("OK — home position saved to flash for gimbal %zu.\n", g);
      std::printf("Encoder will read 0 at this position on every boot.\n");
    } else {
      std::fprintf(stderr, "ERROR: calibrate_home(%zu) failed.\n", g);
      motor.disable();
      return 1;
    }
  }

  motor.disable();
  return 0;
}
