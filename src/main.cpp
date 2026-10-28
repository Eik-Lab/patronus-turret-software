#include "comm/candle_motor.h"
#include "comm/command_listener.h"
#include "comm/sensor_module.h"
#include "core/config.h"
#include "core/state.h"
#include "pipeline/inference_mono.h"
#include "pipeline/inference_rgb.h"
#include "tracking/aim_control.h"
#include "tracking/prediction_overlay.h"
#include "tracking/target_filter.h"
#include "tracking/telemetry.h"

#include "log.h"

#include <glib.h>
#include <gst/gst.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <memory>
#include <sstream>
#include <optional>
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

// One observation per camera frame, detection or not.
static patronus::core::LatestValue<patronus::core::FrameObservation> detection_rgb;
static patronus::core::LatestValue<patronus::core::FrameObservation> detection_mono;

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

  GOptionEntry entries[] = {
    {"config", 'c', 0, G_OPTION_ARG_FILENAME, &config_path,
     "Path to system config file (default: config/system.ini)", "FILE"},
    {"rgb-only", 'r', 0, G_OPTION_ARG_NONE, &rgb_only, "Run RGB pipeline only", nullptr},
    {"mono-only", 'm', 0, G_OPTION_ARG_NONE, &mono_only, "Run mono pipeline only", nullptr},
    {nullptr, 0, 0, G_OPTION_ARG_NONE, nullptr, nullptr, nullptr},
  };

  GOptionContext *ctx = g_option_context_new("- patronus inference system");
  g_option_context_add_main_entries(ctx, entries, nullptr);
  g_option_context_add_group(ctx, gst_init_get_option_group());

  GError *error = nullptr;
  if (!g_option_context_parse(ctx, &argc, &argv, &error)) {
    LOG(Error, "Option parsing failed: " << error->message);
    g_error_free(error);
    g_option_context_free(ctx);
    return 1;
  }
  g_option_context_free(ctx);

  std::string cfg_path = config_path ? config_path : "config/system.ini";
  SystemConfig cfg = load_config(cfg_path);
  g_free(config_path);

  // Anything logged before this point used the default minimum level.
  if (const auto level = parse_level(cfg.log_level_); level.has_value()) {
    set_log_level(*level);
  } else {
    LOG(Warning, "Unknown [general] log_level '"
                   << cfg.log_level_
                   << "', keeping WARNING. Expected debug, info, warning, error or critical.");
  }

  std::signal(SIGTERM, signal_handler);
  std::signal(SIGINT, signal_handler);

  // ── Normal inference + tracking mode ───────────────────────────────────────

  // CLI flags override config file
  if (rgb_only)
    cfg.mode_ = "rgb";
  if (mono_only)
    cfg.mode_ = "mono";

  LOG(Info, "Patronus inference — mode=" << cfg.mode_
                                         << "  tracking=" << (cfg.tracking_ ? "on" : "off")
                                         << "  gimbals=" << cfg.gimbals_.size());

  // Which pipelines will actually run — gimbal threads must only consume
  // queues that have a producer.
  const bool rgb_active = cfg.mode_ != "mono" && cfg.rgb_.enabled_;
  const bool mono_active = cfg.mode_ != "rgb" && cfg.mono_.enabled_;

  std::thread rgb_thread;
  std::thread mono_thread;
  std::thread sensor_thread;
  std::thread command_thread;

  if (cfg.sensor_.enabled_) {
    sensor_thread = std::thread([&cfg]() {
      patronus::comm::run_sensor_thread(cfg.sensor_.port_, cfg.sensor_.baud_rate_, running);
    });
  }

  // UDP command listener thread (e.g. "shoot" from the client console)
  if (cfg.command_.enabled_) {
    command_thread = std::thread([&]() {
      patronus::comm::runCommandThread(cfg.command_.command_host_, cfg.command_.command_port_,
                                       running);
    });
  }

  if (cfg.mode_ != "mono" && cfg.rgb_.enabled_) {
    rgb_thread = std::thread([&]() {
      patronus::pipeline::run_pipeline_rgb(cfg.rgb_, detection_rgb, predictions_rgb);
    });
  }

  if (mono_active) {
    mono_thread = std::thread([&cfg]() {
      patronus::pipeline::run_pipeline_mono(cfg.mono_, detection_mono, predictions_mono);
    });
  }

  // Tracking threads — one per gimbal. Each consumes detections, runs the target
  // filter and publishes its prediction for the overlay; it drives the motors
  // only when it has a motor handle.
  std::vector<std::thread> tracking_threads;
  std::shared_ptr<patronus::comm::CandleMotor> motor;

  // Shared by every tracking thread; destroyed after they have joined, which
  // flushes the files.
  std::shared_ptr<patronus::tracking::Telemetry> telemetry;
  if (cfg.telemetry_.enabled_ && (rgb_active || mono_active))
    telemetry = patronus::tracking::Telemetry::open(cfg.telemetry_.dir_, cfg_path);
  if (cfg.tracking_) {
    motor = std::make_shared<patronus::comm::CandleMotor>(cfg.gimbals_, cfg.can_bus_);
    if (!motor->init()) {
      LOG(Error, "Failed to initialise CANdle motor driver — tracking disabled");
      motor.reset();
    } else if (!motor->boot_sequence()) {
      // Only runs axes whose [can_N] calibrate_on_boot is set. Each calibrated
      // axis parks at the centre of its range, which becomes its home; an axis
      // that failed keeps its start pose as home.
      LOG(Warning, "Boot calibration failed for at least one axis — treat its limits as unknown");
    }
  }

  // The filter does not depend on the motors: with `tracking=false`, or a CAN bus
  // that failed to come up, the threads still run so the overlay shows the
  // prediction, and simply never command a velocity.
  if (rgb_active || mono_active) {
    // Only hand the threads a motor when tracking was asked for.
    const std::shared_ptr<patronus::comm::CandleMotor> drive_motor =
      cfg.tracking_ ? motor : nullptr;
    const size_t gimbal_count = drive_motor ? drive_motor->gimbal_count() : cfg.gimbals_.size();

    // Gimbals that carry a camera get a tracking thread. The others (follow=N
    // in [can_N]) see nothing themselves: the thread of the gimbal they follow
    // drives them to the target bearing its camera reports.
    std::vector<size_t> leaders;
    std::vector<std::vector<size_t>> followers(gimbal_count);
    for (size_t g = 0; g < gimbal_count; ++g) {
      if (cfg.gimbals_[g].follow_ == 0) {
        leaders.push_back(g);
      } else {
        followers[cfg.gimbals_[g].follow_ - 1U].push_back(g);
        LOG(Info,
            "Gimbal " << g << " follows gimbal " << (cfg.gimbals_[g].follow_ - 1U) << "'s camera");
      }
    }

    for (size_t k = 0; k < leaders.size(); ++k) {
      const size_t g = leaders[k];
      if (drive_motor) {
        LOG(Info, "Tracking gimbal " << g << " started (max " << cfg.gimbals_[g].max_velocity_rad_s_
                                     << " rad/s, tilt limit ±" << cfg.gimbals_[g].tilt_max_rad_
                                     << " rad)");
      } else {
        LOG(Info, "Gimbal " << g << ": prediction overlay only — motors not driven");
      }

      const auto gimbal_cfg = cfg.gimbals_[g];

      // [can_N] camera= picks the pipeline; without it, round-robin across
      // cameras among the gimbals that carry one, mono first. When only one
      // pipeline is active, every such gimbal reads from it. The pipeline a
      // gimbal reads from is also the one whose geometry and focal lengths its
      // filter must use, so select it once here and capture it by value.
      const std::string &want = cfg.gimbals_[g].camera_;
      const bool use_rgb =
        (rgb_active && mono_active) ? (want.empty() ? (k % 2 != 0) : want == "rgb") : rgb_active;
      if (!want.empty() && (want == "rgb") != use_rgb)
        LOG(Warning, "Gimbal " << g << ": camera=" << want << " is not running — tracking from "
                               << (use_rgb ? "rgb" : "mono") << " instead");
      const auto pipe = use_rgb ? cfg.rgb_ : cfg.mono_;
      // Each gimbal gets a copy of its tracking config for that camera and the
      // shared motor handle
      const auto trk = use_rgb ? cfg.tracking_cfg_[g].rgb_ : cfg.tracking_cfg_[g].mono_;
      auto *predictions = use_rgb ? &predictions_rgb : &predictions_mono;
      const std::vector<size_t> follower_ids = followers[g];
      const auto all_gimbals = cfg.gimbals_;

      tracking_threads.emplace_back([motor = drive_motor, g, trk, gimbal_cfg, pipe, use_rgb,
                                     predictions, follower_ids, all_gimbals, telemetry]() {
        auto *det = use_rgb ? &detection_rgb : &detection_mono;

        patronus::tracking::TargetFilter filter(trk, pipe.detection_width_, pipe.detection_height_,
                                                pipe.focal_x_px_, pipe.focal_y_px_);

        const patronus::tracking::SensorControlGains gains{
          .kp_ = trk.kp_, .kd_ = trk.kd_, .lead_gain_ = trk.lead_gain_};

        // Null when this thread only feeds the overlay. Every motor call below
        // is behind this.
        const bool driving = motor != nullptr;

        if (driving && trk.filter_enabled_ && !filter.ego_compensation_active()) {
          LOG(Warning, "Tracking gimbal "
                         << g
                         << ": no focal length set (focal_x_px/focal_y_px). Camera motion is not "
                            "compensated, so the filter will read the gimbal's own slew as target "
                            "motion. Set these from the camera's lens and sensor specs.");
        }

        // Followers are aimed by bearing: this gimbal's pose plus the target's
        // angle off the optical axis, which takes the focal lengths. Bearings
        // are taken relative to each gimbal's home, so the two homes must
        // point the same way up to [can_N] pan/tilt_offset_rad.
        const bool lead_followers =
          driving && !follower_ids.empty() && pipe.focal_x_px_ > 0.0F && pipe.focal_y_px_ > 0.0F;
        if (driving && !follower_ids.empty() && !lead_followers) {
          LOG(Warning, "Tracking gimbal " << g
                                          << ": no focal length set, so its followers cannot be "
                                             "aimed — they stay at home");
        }
        const auto home = driving ? motor->home_position(g) : std::pair<float, float>{0.0F, 0.0F};
        std::pair<float, float> pose{0.0F, 0.0F};

        // Home is the centre of the calibrated tilt range, which is taken as
        // looking at the horizon.
        filter.set_level_tilt(home.second);

        if (telemetry) {
          std::ostringstream note;
          note << "gimbal " << g << " camera " << (use_rgb ? "rgb" : "mono") << " det_w "
               << pipe.detection_width_ << " det_h " << pipe.detection_height_ << " focal_x "
               << pipe.focal_x_px_ << " focal_y " << pipe.focal_y_px_ << " home_pan " << home.first
               << " home_tilt " << home.second << " driving " << (driving ? 1 : 0)
               << " ego " << (filter.ego_compensation_active() ? 1 : 0);
          telemetry->note(note.str());
        }

        const auto aim_followers = [&](patronus::core::Point error_px) {
          if (!lead_followers)
            return;
          const float pan = pose.first - home.first + std::atan(error_px.cx_ / pipe.focal_x_px_);
          const float tilt = pose.second - home.second + std::atan(error_px.cy_ / pipe.focal_y_px_);
          for (const size_t f : follower_ids) {
            const auto &fc = all_gimbals[f];
            const auto [f_pan, f_tilt] = motor->home_position(f);
            motor->drive_to(f, f_pan + fc.pan_offset_rad_ + pan,
                            f_tilt + fc.tilt_offset_rad_ + tilt, fc.follow_gain_,
                            trk.home_tolerance_rad_);
          }
        };

        // Followers mirror this gimbal when it is not aiming: hold during the
        // home-return delay, otherwise head home.
        const auto park_followers = [&](bool hold) {
          if (!driving)
            return;
          for (const size_t f : follower_ids) {
            if (hold)
              motor->set_velocity(f, 0.0F, 0.0F);
            else
              motor->return_to_home(f, trk.home_return_gain_, trk.home_tolerance_rad_,
                                    trk.home_return_max_velocity_);
          }
        };

        // HOT PATH: ~100 Hz loop. No heap allocs, no blocking I/O, no
        // logging per frame.
        enum class State { Tracking, ReturningHome, AtHome };
        State state = State::Tracking;
        auto last_detection = std::chrono::steady_clock::now();

        // A track is treated as lost once this long has passed without a
        // detection. Bounds how long a coasting estimate may keep driving the
        // motor.
        const auto stale_after = std::chrono::milliseconds(trk.filter_tick_ms_) * 45;

        // The loop ticks at this period to service the motors. The filter does
        // not: it steps once per camera frame, when an observation arrives.
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

        // The filter's estimate for the most recent frame. Held between frames;
        // the motor command is not; see below.
        std::optional<patronus::tracking::Estimate> est;

        // This tick's telemetry. Filled in as the tick goes, and queued at the
        // top of the next one, so the early `continue`s below cannot skip it.
        patronus::tracking::TickRecord tick_rec;
        bool tick_pending = false;

        while (running) {
          if (telemetry && tick_pending)
            telemetry->record(tick_rec);
          tick_pending = telemetry != nullptr;
          tick_rec = patronus::tracking::TickRecord{};
          tick_rec.gimbal = static_cast<uint32_t>(g);
          tick_rec.driving = driving;
          tick_rec.cmd_pan_rad_s = 0.0F;
          tick_rec.cmd_tilt_rad_s = 0.0F;

          const auto now = std::chrono::steady_clock::now();
          const double t_s = patronus::core::steady_now_s();
          tick_rec.t_s = t_s;

          // Record the encoder angle every tick, so the filter can look up the
          // pose that was current when a frame was captured. Without a motor
          // there is no pose to record, and camera motion goes uncompensated.
          // Each axis is a CAN register read, so the pose is stamped halfway
          // through the reads rather than before them.
          if (driving) {
            pose = motor->get_position(g);
            const double read_s = 0.5 * (t_s + patronus::core::steady_now_s());
            filter.push_gimbal_sample(read_s, pose.first, pose.second);
            tick_rec.t_s = read_s;
            tick_rec.pan_rad = pose.first;
            tick_rec.tilt_rad = pose.second;
          }

          // One observation per camera frame, with or without a detection.
          const auto opt = det->try_pop(tick);
          if (opt.has_value() && opt->detection_.has_value()) {
            state = State::Tracking;
            last_detection = now;
          }

          // True when we have something trustworthy to aim at.
          bool aiming = false;

          if (trk.filter_enabled_) {
            if (opt.has_value()) {
              const double step_s = patronus::core::steady_now_s();
              est = filter.step(*opt, step_s);
              if (telemetry)
                telemetry->record(
                  patronus::tracking::FrameRecord{static_cast<uint32_t>(g), step_s, *opt, est});

              // Publish for the overlay whenever there is an estimate, not only
              // when we are aiming. A coasting prediction is exactly what an
              // operator wants to see during an occlusion; suppressing it would
              // hide the one moment the filter is interesting.
              if (est.has_value()) {
                patronus::tracking::PredictionSample sample;
                sample.publish_t_s = est->time_s();
                sample.u_norm = (est->error_u_px() + half_w) / det_w;
                sample.v_norm = (est->error_v_px() + half_h) / det_h;
                sample.vu_norm_s = est->vu() / det_w;
                sample.vv_norm_s = est->vv() / det_h;
                sample.camera_vu_norm_s = est->camera_vu() / det_w;
                sample.camera_vv_norm_s = est->camera_vv() / det_h;
                sample.sigma_u_norm = est->sigma_u_px() / det_w;
                sample.sigma_v_norm = est->sigma_v_px() / det_h;
                const patronus::core::Point accepted = est->detection_error_px();
                sample.detection_u_norm = (accepted.cx_ + half_w) / det_w;
                sample.detection_v_norm = (accepted.cy_ + half_h) / det_h;
                sample.detection_frame_id = est->detection_frame_id();
                sample.measured = est->measured();
                predictions->publish(g, sample);
              } else {
                // No track: never seeded, or dropped after coasting too long.
                predictions->clear(g);
              }
            }

            const bool fresh = now - last_detection < stale_after;
            aiming = est.has_value() && fresh;
            if (aiming && driving) {
              // Recomputed every tick, not once per frame: the target estimate
              // only changes when a frame arrives, but the gimbal keeps turning,
              // and the error is re-placed at its newest encoder pose. That
              // closes the loop on the gimbal's own motion at the tick rate;
              // holding a per-frame command overshoots.
              if (const auto aim = filter.aim_at(patronus::core::steady_now_s()); aim.has_value()) {
                const auto cmd = patronus::tracking::compute_control_sensor(
                  aim->error_px, aim->velocity_px_s, gains);
                const auto sent = motor->set_velocity(g, cmd.pan_ * gimbal_cfg.max_velocity_rad_s_,
                                                      cmd.tilt_ * gimbal_cfg.max_velocity_rad_s_);
                tick_rec.aim = aim;
                tick_rec.cmd_pan_rad_s = sent.first;
                tick_rec.cmd_tilt_rad_s = sent.second;
                aim_followers(aim->error_now_px);
              }
            }
          } else if (opt.has_value() && telemetry) {
            telemetry->record(patronus::tracking::FrameRecord{
              static_cast<uint32_t>(g), patronus::core::steady_now_s(), *opt, std::nullopt});
          }
          if (!trk.filter_enabled_ && opt.has_value() && opt->detection_.has_value() && driving) {
            // Filter disabled: drive straight off the raw detection.
            const patronus::core::Detection d = *opt->detection_;
            const patronus::core::Point center{d.left_ + (d.width_ / 2.0F),
                                               d.top_ + (d.height_ / 2.0F)};
            const auto error = patronus::tracking::detection_error(center, pipe.detection_width_,
                                                                   pipe.detection_height_);
            const auto raw_cmd = patronus::tracking::compute_control_sensor(
              error, patronus::core::Point{0.0F, 0.0F}, gains);
            const auto sent = motor->set_velocity(g, raw_cmd.pan_ * gimbal_cfg.max_velocity_rad_s_,
                                                  raw_cmd.tilt_ * gimbal_cfg.max_velocity_rad_s_);
            tick_rec.cmd_pan_rad_s = sent.first;
            tick_rec.cmd_tilt_rad_s = sent.second;
            aim_followers(error);
            aiming = true;
          }

          tick_rec.aiming = aiming;
          tick_rec.state = static_cast<int>(state);

          // Everything below holds position or returns home, which is motor
          // work only.
          if (aiming || !driving) {
            continue;
          }

          // Not aiming. Re-enter Tracking as soon as a raw detection
          // arrives, even if it has not yet been accepted by the filter,
          // so the state machine cannot latch into ReturningHome while a
          // target is plainly visible.
          if (opt.has_value() && opt->detection_.has_value() && state == State::ReturningHome) {
            state = State::Tracking;
          }

          park_followers(state == State::Tracking);

          if (state == State::ReturningHome) {
            tick_rec.cmd_pan_rad_s = std::nanf("");
            tick_rec.cmd_tilt_rad_s = std::nanf("");
            if (motor->return_to_home(g, trk.home_return_gain_, trk.home_tolerance_rad_,
                                      trk.home_return_max_velocity_)) {
              state = State::AtHome;
              motor->set_velocity(g, 0.0F, 0.0F);
              LOG(Info, "Tracking gimbal " << g << ": at home position");
            }
            continue;
          }

          // Hold position; start the home return once the delay expires.
          motor->set_velocity(g, 0.0F, 0.0F);
          if (state == State::Tracking && trk.home_return_enabled_ &&
              now - last_detection >= std::chrono::milliseconds(trk.home_return_delay_ms_)) {
            state = State::ReturningHome;
            LOG(Warning, "Tracking gimbal " << g << ": lost target — returning to home");
          }
        }

        if (telemetry && tick_pending)
          telemetry->record(tick_rec);

        // Never leave a gimbal commanded on thread exit, and stop drawing a
        // prediction nothing is maintaining any more.
        if (driving) {
          motor->set_velocity(g, 0.0F, 0.0F);
          for (const size_t f : follower_ids)
            motor->set_velocity(f, 0.0F, 0.0F);
        }
        predictions->clear(g);
      });
    }
  }

  if (rgb_thread.joinable())
    rgb_thread.join();
  if (mono_thread.joinable())
    mono_thread.join();
  if (sensor_thread.joinable())
    sensor_thread.join();
  if (command_thread.joinable())
    command_thread.join();
  for (auto &t : tracking_threads)
    if (t.joinable())
      t.join();

  // Motor destructor (RAII) disables all motors on every exit path.
  return 0;
}
