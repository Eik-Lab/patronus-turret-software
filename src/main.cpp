#include "patronus/comm/candle_motor.hpp"
#include "patronus/comm/sensor_module.hpp"
#include "patronus/core/config.hpp"
#include "patronus/core/state.hpp"
#include "patronus/pipeline/inference_mono.hpp"
#include "patronus/pipeline/inference_rgb.hpp"
#include "patronus/tools/calibrate_focal.hpp"
#include "patronus/tools/motor_test.hpp"
#include "patronus/tracking/aim_control.hpp"
#include "patronus/tracking/prediction_overlay.hpp"
#include "patronus/tracking/target_filter.hpp"

#include <glib.h>
#include <gst/gst.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <string>
#include <thread>
#include <vector>

std::atomic<bool> running{true};

static void signal_handler(int) {
  running = false;
  // Unblock the GStreamer main loops so the pipeline threads can join.
  patronus::pipeline::stop_pipeline_rgb();
  patronus::pipeline::stop_pipeline_mono();
}

using patronus::config::load_config;
using patronus::config::SystemConfig;

static patronus::core::LatestValue<patronus::core::Detection> detection_rgb;
static patronus::core::LatestValue<patronus::core::Detection> detection_mono;

// One channel per camera. The tracking threads for the gimbals that read a
// given camera publish into that camera's channel, and that camera's OSD probe
// draws them. Keeping them separate is what lets the round-robin in the loop
// below hand out gimbals without two threads fighting over one slot.
static patronus::tracking::PredictionChannel predictions_rgb;
static patronus::tracking::PredictionChannel predictions_mono;

int main(int argc, char *argv[]) {
  gchar *config_path = nullptr;
  gboolean rgb_only = FALSE;
  gboolean mono_only = FALSE;
  gboolean test_motors = FALSE;
  gboolean calibrate_focal = FALSE;

  GOptionEntry entries[] = {
    {"config", 'c', 0, G_OPTION_ARG_FILENAME, &config_path,
     "Path to system config file (default: config/system.ini)", "FILE"},
    {"rgb-only", 'r', 0, G_OPTION_ARG_NONE, &rgb_only, "Run RGB pipeline only", nullptr},
    {"mono-only", 'm', 0, G_OPTION_ARG_NONE, &mono_only, "Run mono pipeline only", nullptr},
    {"test-motors", 't', 0, G_OPTION_ARG_NONE, &test_motors,
     "Spin each motor briefly to verify CAN bus, then exit", nullptr},
    {"calibrate-focal", 0, 0, G_OPTION_ARG_NONE, &calibrate_focal,
     "Slew each gimbal through known angles to measure the camera focal length, then exit",
     nullptr},
    {nullptr, 0, 0, G_OPTION_ARG_NONE, nullptr, nullptr, nullptr},
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

  std::string cfg_path = config_path ? config_path : "config/system.ini";
  SystemConfig cfg = load_config(cfg_path);
  g_free(config_path);

  std::signal(SIGTERM, signal_handler);
  std::signal(SIGINT, signal_handler);

  // ── --test-motors: standalone CAN bus test, no cameras needed ──────────────
  if (test_motors)
    return patronus::tools::run_motor_test(cfg, running);

  // ── Normal inference + tracking mode ───────────────────────────────────────

  // CLI flags override config file
  if (rgb_only)
    cfg.mode_ = "rgb";
  if (mono_only)
    cfg.mode_ = "mono";

  g_print("Patronus inference — mode=%s  tracking=%s  gimbals=%zu\n", cfg.mode_.c_str(),
          cfg.tracking_ ? "on" : "off", cfg.gimbals_.size());

  // Which pipelines will actually run — gimbal threads must only consume
  // queues that have a producer.
  const bool rgb_active = cfg.mode_ != "mono" && cfg.rgb_.enabled_;
  const bool mono_active = cfg.mode_ != "rgb" && cfg.mono_.enabled_;

  std::thread rgb_thread;
  std::thread mono_thread;
  std::thread sensor_thread;

  if (cfg.sensor_.enabled_) {
    sensor_thread = std::thread([&cfg]() {
      patronus::comm::run_sensor_thread(cfg.sensor_.port_, cfg.sensor_.baud_rate_, running);
    });
  }

  if (rgb_active) {
    rgb_thread = std::thread([&cfg]() {
      patronus::pipeline::run_pipeline_rgb(cfg.rgb_, detection_rgb, predictions_rgb);
    });
  }

  if (mono_active) {
    mono_thread = std::thread([&cfg]() {
      patronus::pipeline::run_pipeline_mono(cfg.mono_, detection_mono, predictions_mono);
    });
  }

  // Tracking threads — one per gimbal, each consumes detections and drives motors.
  std::vector<std::thread> tracking_threads;
  std::shared_ptr<patronus::comm::CandleMotor> motor;
  // Focal calibration also needs the motor, but the two must not run at once:
  // the calibration drives each gimbal to a sequence of known angles, and a
  // tracking thread on the same gimbal would be commanding velocities
  // underneath it, making the regression meaningless and the motion unsafe.
  if (cfg.tracking_ || calibrate_focal) {
    motor = std::make_shared<patronus::comm::CandleMotor>(cfg.gimbals_, cfg.can_bus_);
    if (!motor->init()) {
      g_critical("Failed to initialise CANdle motor driver — tracking disabled");
      motor.reset();
    } else if (!calibrate_focal) {
      for (size_t g = 0; g < motor->gimbal_count(); ++g) {
        g_print("Tracking gimbal %zu started (max %.2f rad/s, tilt limit ±%.2f rad)\n", g,
                cfg.gimbals_[g].max_velocity_rad_s_, cfg.gimbals_[g].tilt_max_rad_);

        // Each gimbal gets a copy of its tracking config and shared motor handle
        const auto trk = cfg.tracking_cfg_[g];
        const auto gimbal_cfg = cfg.gimbals_[g];

        // Round-robin across cameras; when only one pipeline is active, every
        // gimbal reads from it. The pipeline a gimbal reads from is also the
        // one whose geometry and focal lengths its filter must use, so select
        // it once here and capture it by value.
        const bool use_rgb = (rgb_active && mono_active) ? (g % 2 == 0) : rgb_active;
        const auto pipe = use_rgb ? cfg.rgb_ : cfg.mono_;
        auto *predictions = use_rgb ? &predictions_rgb : &predictions_mono;

        tracking_threads.emplace_back([motor, g, trk, gimbal_cfg, pipe, use_rgb, rgb_active,
                                       mono_active, predictions]() {
          auto *det = (rgb_active && mono_active) ? (g % 2 == 0 ? &detection_rgb : &detection_mono)
                                                  : (use_rgb ? &detection_rgb : &detection_mono);

          patronus::tracking::TargetFilter filter(
            trk, pipe.detection_width_, pipe.detection_height_, pipe.focal_x_px_, pipe.focal_y_px_);

          const patronus::tracking::SensorControlGains gains{
            .kp_ = trk.kp_, .kd_ = trk.kd_, .lead_gain_ = trk.lead_gain_};

          if (trk.filter_enabled_ && !filter.ego_compensation_active()) {
            g_print("Tracking gimbal %zu: ego-motion compensation OFF "
                    "(focal_x_px/focal_y_px unset). Velocity terms will include "
                    "self-motion; run --calibrate-focal to enable.\n",
                    g);
          }

          // HOT PATH: ~100 Hz loop. No heap allocs, no blocking I/O, no
          // logging per frame.
          enum class State { Tracking, ReturningHome, AtHome };
          State state = State::Tracking;
          auto last_detection = std::chrono::steady_clock::now();

          // A track is treated as lost once the filter has coasted this
          // long without accepting a measurement. Bounded so that a
          // detection stream which is present but entirely inconsistent
          // with the current estimate (a false positive, or a target that
          // jumped) cannot keep a stale estimate driving the motor
          // indefinitely. Slightly longer than the filter's own
          // re-acquisition window so a brief occlusion is ridden out.
          const auto stale_after = std::chrono::milliseconds(trk.filter_tick_ms_) * 45;

          // The filter's gimbal-angle history, its tick, and the timestamps the
          // pipeline stamps on each detection all come from steady_now_s(), a
          // single process-wide monotonic epoch. They can be subtracted directly,
          // which is the whole reason the filter can compensate for pipeline
          // latency at all.
          auto last_tick = std::chrono::steady_clock::now();
          const auto tick = std::chrono::milliseconds(trk.filter_tick_ms_);

          // Normalisation factors for the overlay. Publishing normalised
          // coordinates keeps the video thread from having to know the detection
          // pixel space, which is not the muxed space and not the network input.
          const float det_w =
            (pipe.detection_width_ > 0U) ? static_cast<float>(pipe.detection_width_) : 1.0F;
          const float det_h =
            (pipe.detection_height_ > 0U) ? static_cast<float>(pipe.detection_height_) : 1.0F;
          const float half_w = det_w * 0.5F;
          const float half_h = det_h * 0.5F;
          const float lead_s = pipe.prediction_lead_s_;

          while (running) {
            const auto now = std::chrono::steady_clock::now();
            const double t_s = patronus::core::steady_now_s();
            const float dt_s = std::chrono::duration<float>(now - last_tick).count();

            // Record the encoder angle before stepping, so the filter can
            // look up the pose that was current when a frame was captured.
            const auto pos = motor->get_position(g);
            filter.push_gimbal_sample(t_s, pos.first, pos.second);

            const auto opt = det->try_pop(tick);
            if (opt.has_value()) {
              state = State::Tracking;
              last_detection = now;
            }

            // True when we have something trustworthy to aim at.
            bool aiming = false;

            if (trk.filter_enabled_) {
              // Predict every tick whether or not a detection arrived; that
              // is what lets the gimbal keep tracking between fixes.
              const auto est = filter.step(dt_s, opt, t_s);

              // Publish for the overlay whenever there is an estimate, not only
              // when we are aiming. A coasting prediction is exactly what an
              // operator wants to see during an occlusion; suppressing it would
              // hide the one moment the filter is interesting.
              if (est.has_value()) {
                patronus::tracking::PredictionSample sample;
                sample.publish_t_s = t_s;
                sample.u_norm = (est->error_u_px() + half_w) / det_w;
                sample.v_norm = (est->error_v_px() + half_h) / det_h;
                const auto lead = est->predicted_error(lead_s);
                sample.lead_u_norm = (lead.cx_ + half_w) / det_w;
                sample.lead_v_norm = (lead.cy_ + half_h) / det_h;
                sample.vu_norm_s = est->vu() / det_w;
                sample.vv_norm_s = est->vv() / det_h;
                sample.box_w_norm = est->box_w_px() / det_w;
                sample.box_h_norm = est->box_h_px() / det_h;
                sample.sigma_norm = std::max(est->sigma_u_px(), est->sigma_v_px()) / det_w;
                sample.coast_ticks = est->coast_ticks();
                sample.measured = est->measured();
                predictions->publish(g, sample);
              }

              // coast_ticks() counts ticks since the last accepted
              // measurement, so a rising value means the gate is rejecting
              // everything and the estimate is extrapolating.
              const bool fresh = now - last_detection < stale_after;
              aiming = est.has_value() && fresh;
              if (aiming) {
                const patronus::core::Point error{est->error_u_px(), est->error_v_px()};
                const patronus::core::Point velocity{est->vu(), est->vv()};
                const auto cmd = patronus::tracking::compute_control_sensor(error, velocity, gains);
                motor->set_velocity(g, cmd.pan_ * gimbal_cfg.max_velocity_rad_s_,
                                    cmd.tilt_ * gimbal_cfg.max_velocity_rad_s_);
              }
            } else if (opt.has_value()) {
              // Filter disabled: drive straight off the raw detection.
              const patronus::core::Detection d = *opt;
              const patronus::core::Point center{d.left_ + (d.width_ / 2.0F),
                                                 d.top_ + (d.height_ / 2.0F)};
              const auto error = patronus::tracking::detection_error(center, pipe.detection_width_,
                                                                     pipe.detection_height_);
              const auto cmd = patronus::tracking::compute_control_sensor(
                error, patronus::core::Point{0.0F, 0.0F}, gains);
              motor->set_velocity(g, cmd.pan_ * gimbal_cfg.max_velocity_rad_s_,
                                  cmd.tilt_ * gimbal_cfg.max_velocity_rad_s_);
              aiming = true;
            }

            last_tick = now;

            if (aiming) {
              continue;
            }

            // Not aiming. Re-enter Tracking as soon as a raw detection
            // arrives, even if it has not yet been accepted by the filter,
            // so the state machine cannot latch into ReturningHome while a
            // target is plainly visible.
            if (opt.has_value() && state == State::ReturningHome) {
              state = State::Tracking;
            }

            if (state == State::ReturningHome) {
              if (motor->return_to_home(g, trk.home_return_gain_, trk.home_tolerance_rad_,
                                        trk.home_return_max_velocity_)) {
                state = State::AtHome;
                motor->set_velocity(g, 0.0F, 0.0F);
                g_print("Tracking gimbal %zu: at home position\n", g);
              }
              continue;
            }

            // Hold position; start the home return once the delay expires.
            motor->set_velocity(g, 0.0F, 0.0F);
            if (state == State::Tracking && trk.home_return_enabled_ &&
                now - last_detection >= std::chrono::milliseconds(trk.home_return_delay_ms_)) {
              state = State::ReturningHome;
              g_print("Tracking gimbal %zu: lost target — returning to home\n", g);
            }
          }

          // Never leave a gimbal commanded on thread exit, and stop drawing a
          // prediction nothing is maintaining any more.
          motor->set_velocity(g, 0.0F, 0.0F);
          predictions->clear(g);
        });
      }
    }
  }

  // ── --calibrate-focal: measure fx/fy from known gimbal angles ─────────────
  // Runs alongside the live pipelines, because the only source of detections is
  // the perception stack. A separate calibration binary would have to open the
  // cameras a second time, which the Basler sources will not allow.
  if (calibrate_focal) {
    g_print("\nFocal calibration — pointing the camera at a high-contrast target is required.\n");
    if (!motor) {
      g_critical("Focal calibration needs the motor driver, which failed to initialise");
      running = false;
    } else {
      for (size_t g = 0; g < motor->gimbal_count() && running; ++g) {
        const bool use_rgb = (rgb_active && mono_active) ? (g % 2 == 0) : rgb_active;
        const auto &pipe = use_rgb ? cfg.rgb_ : cfg.mono_;
        auto *det = (rgb_active && mono_active) ? (g % 2 == 0 ? &detection_rgb : &detection_mono)
                                                : (use_rgb ? &detection_rgb : &detection_mono);
        const int rc = patronus::tools::run_focal_calibration(
          *motor, g, cfg.gimbals_[g], *det, pipe.detection_width_, pipe.detection_height_,
          pipe.focal_x_px_, pipe.focal_y_px_, running);
        if (rc != 0) {
          g_printerr("Gimbal %zu: focal calibration failed\n", g);
        }
      }
      running = false; // always stop the pipelines and exit afterwards
    }
  }

  if (rgb_thread.joinable())
    rgb_thread.join();
  if (mono_thread.joinable())
    mono_thread.join();
  if (sensor_thread.joinable())
    sensor_thread.join();
  for (auto &t : tracking_threads)
    if (t.joinable())
      t.join();

  // Motor destructor (RAII) disables all motors on every exit path.
  return 0;
}
