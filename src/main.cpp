#include "patronus/comm/candle_motor.hpp"
#include "patronus/core/config.hpp"
#include "patronus/core/state.hpp"
#include "patronus/pipeline/inference_mono.hpp"
#include "patronus/pipeline/inference_rgb.hpp"
#include "patronus/tracking/aim_control.hpp"
#include <glib.h>
#include <gst/gst.h>
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

  // --test-motors: standalone CAN bus test, no cameras needed
  if (test_motors) {
    patronus::comm::CandleConfig can_cfg;
    can_cfg.pan_node_id         = cfg.can.pan_node_id;
    can_cfg.tilt_node_id        = cfg.can.tilt_node_id;
    can_cfg.datarate            = cfg.can.datarate;
    can_cfg.max_velocity_rad_s  = cfg.can.max_velocity_rad_s;
    can_cfg.pds_node_id         = cfg.can.pds_node_id;

    patronus::comm::CandleMotor motor(can_cfg);
    if (!motor.init()) {
      g_critical("CANDLE MOTOR TEST FAILED -- init()");
      return 1;
    }
    g_print("CANDLE MOTOR TEST -- both motors enabled\n");

    auto run_profile = [&](const char* label, float pv, float tv, int duration_ms) {
      g_print("  %s (pan=%.1f tilt=%.1f rad/s, %d ms)\n", label, pv, tv, duration_ms);
      auto start = std::chrono::steady_clock::now();
      int  count = 0;
      while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(duration_ms)) {
        motor.set_velocity(pv, tv);
        // Read actual velocity every 10 iterations for diagnostics
        if (++count % 10 == 0) {
          auto [ap, at] = motor.get_velocity();
          auto [pp, pt] = motor.get_position();
          g_print("    actual: pan=%.3f tilt=%.3f  pos: pan=%.2f tilt=%.2f\n", ap, at, pp, pt);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    };

    run_profile("spinning pan",   0.5F,  0.0F, 3000);
    run_profile("spinning tilt",  0.0F,  0.5F, 3000);
    run_profile("spinning both",  1.0F,  1.0F, 3000);
    run_profile("reversing both", -1.0F, -1.0F, 3000);
    run_profile("stopping",       0.0F,  0.0F, 1000);

    motor.disable();
    g_print("CANDLE MOTOR TEST PASSED\n");
    return 0;
  }

  // CLI flags override config file
  if (rgb_only) cfg.mode = "rgb";
  if (mono_only) cfg.mode = "mono";

  g_print("Patronus inference — mode=%s  tracking=%s\n",
          cfg.mode.c_str(), cfg.tracking ? "on" : "off");

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

  // Tracking thread — consumes detections and drives CAN bus motors.
  std::thread tracking_thread;
  if (cfg.tracking) {
    patronus::comm::CandleConfig can_cfg;
    can_cfg.pan_node_id         = cfg.can.pan_node_id;
    can_cfg.tilt_node_id        = cfg.can.tilt_node_id;
    can_cfg.datarate            = cfg.can.datarate;
    can_cfg.max_velocity_rad_s  = cfg.can.max_velocity_rad_s;
    can_cfg.pds_node_id         = cfg.can.pds_node_id;

    tracking_thread = std::thread([cfg, can_cfg]() {
      patronus::comm::CandleMotor motor(can_cfg);
      if (!motor.init()) {
        g_critical("Failed to initialise CANdle motor driver — tracking disabled");
        return;
      }
      g_print("Tracking loop started (max %.2f rad/s)\n", can_cfg.max_velocity_rad_s);

      // Pick the active detection queue
      auto* det = (cfg.mode == "mono") ? &detection_mono : &detection_rgb;

      for (;;) {
        // Non-blocking: if no detection arrives within 10ms, command stop
        auto opt = det->try_pop(std::chrono::milliseconds(10));
        if (opt.has_value()) {
          Detection d = *opt;
          Point center{d.left + d.width / 2.0F,
                       d.top + d.height / 2.0F};

          AimAngles err = compute_control_sensor(center, 0.005F);

          float pan_vel  = err.pan  * can_cfg.max_velocity_rad_s;
          float tilt_vel = err.tilt * can_cfg.max_velocity_rad_s;

          motor.set_velocity(pan_vel, tilt_vel);
        } else {
          // No target — stop the gimbal
          motor.set_velocity(0.0F, 0.0F);
        }
      }
    });
  }

  if (rgb_thread.joinable())    rgb_thread.join();
  if (mono_thread.joinable())   mono_thread.join();
  if (tracking_thread.joinable()) tracking_thread.join();

  return 0;
}
