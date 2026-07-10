#include "patronus/comm/candle_motor.hpp"
#include "patronus/core/config.hpp"
#include "patronus/core/state.hpp"
#include "patronus/pipeline/inference_mono.hpp"
#include "patronus/pipeline/inference_rgb.hpp"
#include "patronus/tracking/aim_control.hpp"
#include <candlelib.hpp>
#include <glib.h>
#include <gst/gst.h>
#include <cmath>
#include <cstdio>
#include <thread>
#include <chrono>

using patronus::config::SystemConfig;
using patronus::config::load_config;

static LatestValue<Detection> detection_rgb;
static LatestValue<Detection> detection_mono;

int main(int argc, char *argv[])
{
  gchar *config_path = nullptr;
  gboolean rgb_only = FALSE;
  gboolean mono_only = FALSE;
  gboolean test_motors = FALSE;

  GOptionEntry entries[] = {
    {"config", 'c', 0, G_OPTION_ARG_FILENAME, &config_path,
     "Path to system config file (default: config/system.ini)", "FILE"},
    {"rgb-only", 'r', 0, G_OPTION_ARG_NONE, &rgb_only,
     "Run RGB pipeline only", nullptr},
    {"mono-only", 'm', 0, G_OPTION_ARG_NONE, &mono_only,
     "Run mono pipeline only", nullptr},
    {"test-motors", 't', 0, G_OPTION_ARG_NONE, &test_motors,
     "Spin each motor briefly to verify CAN bus, then exit", nullptr},
    {nullptr},
  };

  GOptionContext *ctx = g_option_context_new("- patronus inference system");
  g_option_context_add_main_entries(ctx, entries, nullptr);

  // Only add GStreamer group if not testing motors (avoid needing cameras)
  if (!test_motors)
    g_option_context_add_group(ctx, gst_init_get_option_group());

  GError *error = nullptr;
  if (!g_option_context_parse(ctx, &argc, &argv, &error)) {
    g_printerr("Option parsing failed: %s\n", error->message);
    g_error_free(error);
    g_option_context_free(ctx);
    return 1;
  }
  g_option_context_free(ctx);

  // Load configuration
  std::string cfg_path = config_path ? config_path : "config/system.ini";
  SystemConfig cfg = load_config(cfg_path);
  g_free(config_path);

  // ── --test-motors: standalone CAN bus test, no cameras needed ──────────────
  if (test_motors) {
    patronus::comm::CandleMotor motor(cfg.gimbals, cfg.can_bus);
    if (!motor.init()) {
      g_critical("CANDLE MOTOR TEST FAILED -- init()");
      return 1;
    }
    g_print("CANDLE MOTOR TEST — %zu gimbal(s) enabled\n", motor.gimbal_count());

    // Read gear ratio from motor firmware to convert motor-shaft → output
    auto read_gear_ratio = [](mab::MD* md) -> float {
      if (!md) return 1.0F;
      auto& reg = md->m_mdRegisters.motorGearRatio;
      md->readRegister(reg);
      float ratio = reg.value;
      return (ratio > 0.0F) ? ratio : 1.0F;
    };

    for (size_t g = 0; g < motor.gimbal_count(); ++g) {
      float pan_ratio  = read_gear_ratio(motor.pan_motor(g));
      float tilt_ratio = read_gear_ratio(motor.tilt_motor(g));
      float tilt_limit = cfg.gimbals[g].tilt_max_rad;
      g_print("  gimbal %zu: pan=MD%u tilt=MD%u  gear pan=%.1f:1  tilt=%.1f:1\n",
              g, motor.pan_motor(g)->m_canId, motor.tilt_motor(g)->m_canId,
              pan_ratio, tilt_ratio);
      g_print("  gimbal %zu: output range — pan ±60° (%.2f rad)  tilt ±%.0f° (%.4f rad)\n",
              g, 1.047, tilt_limit * 180.0 / M_PI, tilt_limit);
    }

    auto report_all = [&](const char* label) {
      g_print("  [%s]\n", label);
      for (size_t g = 0; g < motor.gimbal_count(); ++g) {
        auto [ap, at] = motor.get_velocity(g);
        auto [pp, pt] = motor.get_position(g);
        float pan_ratio  = read_gear_ratio(motor.pan_motor(g));
        float tilt_ratio = read_gear_ratio(motor.tilt_motor(g));
        float op = pp / pan_ratio;
        float ot = pt / tilt_ratio;
        g_print("    gimbal %zu  vel %+.3f %+.3f  motor %+.2f %+.2f  output %+.1f° %+.1f°\n",
                g, ap, at, pp, pt,
                op * 180.0 / M_PI, ot * 180.0 / M_PI);
      }
    };

    auto wait_home_all = [&](const char* label, int timeout_ms) {
      g_print("--- %s ---\n", label);
      auto start = std::chrono::steady_clock::now();
      int  count = 0;
      for (;;) {
        bool all_home = true;
        for (size_t g = 0; g < motor.gimbal_count(); ++g) {
          bool at_home = motor.return_to_home(g, 1.0F, 0.05F, 1.5F);
          all_home = all_home && at_home;
        }
        if (++count % 5 == 0) report_all("");
        if (all_home) break;
        if (std::chrono::steady_clock::now() - start >= std::chrono::milliseconds(timeout_ms)) {
          g_print("  (timeout)\n");
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      report_all("at home");
    };

    // Lissajous figure-8: pan at ω, tilt at 2ω — traces a figure-8 pattern
    auto lissajous = [&](const char* label, float amp, float omega,
                          float freq_ratio, int duration_ms) {
      g_print("--- %s ---\n", label);
      auto start = std::chrono::steady_clock::now();
      int  count = 0;
      while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(duration_ms)) {
        float t = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - start).count();
        float pv = amp * std::cos(omega * t);
        float tv = amp * std::cos(freq_ratio * omega * t);
        for (size_t g = 0; g < motor.gimbal_count(); ++g)
          motor.set_velocity(g, pv, tv);
        if (++count % 10 == 0) report_all("");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      for (size_t g = 0; g < motor.gimbal_count(); ++g)
        motor.set_velocity(g, 0.0F, 0.0F);
      report_all("at end of lissajous");
    };

    float sweep_speed = 0.5F;

    // 1. Start at home
    wait_home_all("return to home", 5000);

    // 2. Lissajous figure-8 scan — 4 cycles (ω = 0.5 rad/s, cycle = 12.57 s)
    //    Both axes move simultaneously: pan at ω, tilt at 2ω.
    lissajous("figure-8 scan (4 cycles)", sweep_speed, 0.5F, 2.0F, 50000);

    // 3. Final stop at home
    for (size_t g = 0; g < motor.gimbal_count(); ++g)
      motor.set_velocity(g, 0.0F, 0.0F);
    report_all("test complete — at home");
    motor.disable();
    g_print("CANDLE MOTOR TEST PASSED\n");
    return 0;
  }

  // ── Normal inference + tracking mode ───────────────────────────────────────

  // CLI flags override config file
  if (rgb_only) cfg.mode = "rgb";
  if (mono_only) cfg.mode = "mono";

  g_print("Patronus inference — mode=%s  tracking=%s  gimbals=%zu\n",
          cfg.mode.c_str(), cfg.tracking ? "on" : "off", cfg.gimbals.size());

  // Launch pipeline threads
  std::thread rgb_thread;
  std::thread mono_thread;

  if (cfg.mode != "mono" && cfg.rgb.enabled) {
    rgb_thread = std::thread([&]() {
      run_pipeline_rgb(cfg.rgb, detection_rgb);
    });
  }

  if (cfg.mode != "rgb" && cfg.mono.enabled) {
    mono_thread = std::thread([&]() {
      run_pipeline_mono(cfg.mono, detection_mono);
    });
  }

  // Tracking threads — one per gimbal, each consumes detections and drives motors.
  std::vector<std::thread> tracking_threads;
  if (cfg.tracking) {
    auto motor = std::make_shared<patronus::comm::CandleMotor>(
        cfg.gimbals, cfg.can_bus);
    if (!motor->init()) {
      g_critical("Failed to initialise CANdle motor driver — tracking disabled");
    } else {
      for (size_t g = 0; g < motor->gimbal_count(); ++g) {
        float tilt_limit = cfg.gimbals[g].tilt_max_rad;
        g_print("Tracking gimbal %zu started (max %.2f rad/s, tilt limit ±%.2f rad)\n",
                g, cfg.gimbals[g].max_velocity_rad_s, tilt_limit);

        // Each gimbal gets a copy of its tracking config and shared motor handle
        auto trk = cfg.tracking_cfg[g];
        auto gimbal_cfg = cfg.gimbals[g];
        auto mode = cfg.mode;

        tracking_threads.emplace_back([motor, g, trk, gimbal_cfg, mode]() {
          // Pick the active detection queue — round-robin by gimbal index
          auto* det = (g % 2 == 0) ?
              ((mode == "mono") ? &detection_mono : &detection_rgb) :
              ((mode == "mono") ? &detection_rgb : &detection_mono);

          enum class State { TRACKING, RETURNING_HOME, AT_HOME };
          State state = State::TRACKING;
          auto last_detection = std::chrono::steady_clock::now();

          for (;;) {
            auto opt = det->try_pop(std::chrono::milliseconds(10));

            if (opt.has_value()) {
              state = State::TRACKING;
              last_detection = std::chrono::steady_clock::now();

              Detection d = *opt;
              Point center{d.left + d.width / 2.0F,
                           d.top + d.height / 2.0F};

              AimAngles err = compute_control_sensor(center, 0.006F);

              float pan_vel  = err.pan  * gimbal_cfg.max_velocity_rad_s;
              float tilt_vel = err.tilt * gimbal_cfg.max_velocity_rad_s;

              motor->set_velocity(g, pan_vel, tilt_vel);
            } else {
              switch (state) {
                case State::TRACKING: {
                  if (trk.home_return_enabled) {
                    auto elapsed = std::chrono::steady_clock::now() - last_detection;
                    if (elapsed >= std::chrono::milliseconds(trk.home_return_delay_ms)) {
                      state = State::RETURNING_HOME;
                      g_print("Tracking gimbal %zu: lost target — returning to home\n", g);
                    } else {
                      motor->set_velocity(g, 0.0F, 0.0F);
                    }
                  } else {
                    motor->set_velocity(g, 0.0F, 0.0F);
                  }
                  break;
                }
                case State::RETURNING_HOME: {
                  bool at_home = motor->return_to_home(
                      g, trk.home_return_gain, trk.home_tolerance_rad,
                      trk.home_return_max_velocity);
                  if (at_home) {
                    state = State::AT_HOME;
                    motor->set_velocity(g, 0.0F, 0.0F);
                    g_print("Tracking gimbal %zu: at home position\n", g);
                  }
                  break;
                }
                case State::AT_HOME: {
                  motor->set_velocity(g, 0.0F, 0.0F);
                  break;
                }
              }
            }
          }
        });
      }
    }
  }

  if (rgb_thread.joinable())       rgb_thread.join();
  if (mono_thread.joinable())      mono_thread.join();
  for (auto& t : tracking_threads)
    if (t.joinable()) t.join();

  return 0;
}
