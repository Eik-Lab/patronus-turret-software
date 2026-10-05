#include "patronus/tools/motor_test.hpp"

#include "patronus/comm/candle_motor.hpp"

#include <glib.h>

#include <chrono>
#include <cmath>
#include <thread>

namespace patronus::tools {
namespace {

  // Short, symmetric, and well under the velocity clamp. Long enough for the PID
  // loop to visibly respond, short enough that a miswired node id cannot walk the
  // gimbal into its limit before the test ends.
  constexpr float k_test_velocity_rad_s = 0.5F;
  constexpr auto k_settle = std::chrono::milliseconds(400);
  constexpr auto k_return = std::chrono::milliseconds(600);

  void report(const char *phase, size_t gimbal, float pan, float tilt) {
    g_print("  gimbal %zu %-7s pan %+.3f rad (%+.1f deg)  tilt %+.3f rad (%+.1f deg)\n", gimbal,
            phase, pan, pan * 180.0F / static_cast<float>(M_PI), tilt,
            tilt * 180.0F / static_cast<float>(M_PI));
  }

} // namespace

int run_motor_test(const patronus::config::SystemConfig &cfg, std::atomic<bool> &running) {
  // RAII: the destructor disables all motors on every exit path, including the
  // early returns below.
  comm::CandleMotor motor(cfg.gimbals_, cfg.can_bus_);
  if (!motor.init()) {
    g_critical("Motor test: CANdle initialisation failed");
    return 1;
  }

  if (motor.gimbal_count() == 0) {
    g_critical("Motor test: no gimbals configured");
    return 1;
  }

  g_print("Motor test — %zu gimbal(s). Clear the field first.\n", motor.gimbal_count());

  int failures = 0;
  for (size_t g = 0; g < motor.gimbal_count() && running; ++g) {
    g_print("Gimbal %zu\n", g);

    const auto [pan0, tilt0] = motor.get_position(g);
    report("start", g, pan0, tilt0);

    // Velocity-mode slew: a non-zero command must produce a non-zero reading.
    motor.set_velocity(g, k_test_velocity_rad_s, 0.0F);
    std::this_thread::sleep_for(k_settle);
    const auto [pan_v, tilt_v] = motor.get_velocity(g);
    report("slewed", g, pan_v, tilt_v);

    if (std::abs(pan_v) < 0.05F) {
      g_warning("Gimbal %zu: no pan response (read %.3f rad/s) — check wiring and node id %u", g,
                static_cast<double>(pan_v), cfg.gimbals_[g].pan_node_id_);
      ++failures;
    }

    // Drive back so the test does not leave the gimbal displaced.
    motor.set_velocity(g, -k_test_velocity_rad_s, 0.0F);
    std::this_thread::sleep_for(k_return);
    motor.set_velocity(g, 0.0F, 0.0F);

    const auto [pan1, tilt1] = motor.get_position(g);
    report("ended", g, pan1, tilt1);

    // Slewing for a second at 0.5 rad/s should move it ~0.5 rad. A much smaller
    // displacement means the encoder is not tracking the command.
    const float moved = std::abs(pan1 - pan0);
    if (moved < 0.1F) {
      g_warning("Gimbal %zu: encoder barely moved (%.3f rad) — encoder or feedback suspect", g,
                static_cast<double>(moved));
      ++failures;
    }
    (void)tilt0;
    (void)tilt1;
  }

  motor.disable();

  if (failures != 0) {
    g_critical("Motor test: %d check(s) failed", failures);
    return 1;
  }

  g_print("Motor test: all gimbals responded\n");
  return 0;
}

} // namespace patronus::tools