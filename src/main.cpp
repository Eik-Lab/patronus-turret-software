#include "comm/candle_motor.h"
#include "comm/command_listener.h"
#include "comm/sensor_module.h"
#include "core/config.h"
#include "core/state.h"
#include "pipeline/inference_mono.h"
#include "pipeline/inference_rgb.h"
#include "tracking/aim_control.h"
#include "tracking/control_state.h"
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
  patronus::pipeline::stop_pipeline_rgb();
  patronus::pipeline::stop_pipeline_mono();
}

using patronus::config::load_config;
using patronus::config::SystemConfig;

static patronus::core::LatestValue<patronus::core::FrameObservation> detection_rgb;
static patronus::core::LatestValue<patronus::core::FrameObservation> detection_mono;

// One channel per camera, so two gimbals on different cameras never share a slot.
static patronus::tracking::PredictionChannel predictions_rgb;
static patronus::tracking::PredictionChannel predictions_mono;

int main(int argc, char *argv[]) {
  gchar *config_path = nullptr;
  gboolean rgb_only = FALSE;
  gboolean mono_only = FALSE;
  gboolean record_only = FALSE;

  GOptionEntry entries[] = {
    {"config", 'c', 0, G_OPTION_ARG_FILENAME, &config_path,
     "Path to system config file (default: config/system.ini)", "FILE"},
    {"rgb-only", 'r', 0, G_OPTION_ARG_NONE, &rgb_only, "Run RGB pipeline only", nullptr},
    {"mono-only", 'm', 0, G_OPTION_ARG_NONE, &mono_only, "Run mono pipeline only", nullptr},
    {"record-only", 0, 0, G_OPTION_ARG_NONE, &record_only,
     "Disable motors (including boot calibration), sensors and external commands", nullptr},
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

  if (record_only) {
    cfg.tracking_ = false;
    cfg.sensor_.enabled_ = false;
    cfg.command_.enabled_ = false;
  }

  if (const auto level = parse_level(cfg.log_level_); level.has_value()) {
    set_log_level(*level);
  } else {
    LOG(Warning, "Unknown [general] log_level '"
                   << cfg.log_level_
                   << "', keeping WARNING. Expected debug, info, warning, error or critical.");
  }

  std::signal(SIGTERM, signal_handler);
  std::signal(SIGINT, signal_handler);

  if (rgb_only)
    cfg.mode_ = "rgb";
  if (mono_only)
    cfg.mode_ = "mono";

  LOG(Info, "Patronus inference — mode=" << cfg.mode_
                                         << "  tracking=" << (cfg.tracking_ ? "on" : "off")
                                         << "  gimbals=" << cfg.gimbals_.size());

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

  if (cfg.command_.enabled_) {
    command_thread = std::thread([&]() {
      patronus::comm::run_command_thread(cfg.command_.command_host_, cfg.command_.command_port_,
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

  std::vector<std::thread> tracking_threads;
  std::shared_ptr<patronus::comm::CandleMotor> motor;

  // Destroyed after the tracking threads join, which flushes it.
  std::shared_ptr<patronus::tracking::Telemetry> telemetry;
  if (cfg.telemetry_.enabled_ && (rgb_active || mono_active))
    telemetry = patronus::tracking::Telemetry::open(cfg.telemetry_.dir_, cfg_path);
  if (cfg.tracking_) {
    motor = std::make_shared<patronus::comm::CandleMotor>(cfg.gimbals_, cfg.can_bus_);
    if (!motor->init()) {
      LOG(Error, "Failed to initialise CANdle motor driver — tracking disabled");
      motor.reset();
    } else if (!motor->boot_sequence()) {
      LOG(Warning, "Boot calibration failed for at least one axis — treat its limits as unknown");
    }
  }

  // Without motors the threads still run, to feed the overlay.
  if (rgb_active || mono_active) {
    const std::shared_ptr<patronus::comm::CandleMotor> drive_motor =
      cfg.tracking_ ? motor : nullptr;
    const size_t gimbal_count = drive_motor ? drive_motor->gimbal_count() : cfg.gimbals_.size();

    // Followers (follow=N) get no thread; their leader's thread drives them.
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

      // camera= picks the pipeline; otherwise round-robin, mono first.
      const std::string &want = cfg.gimbals_[g].camera_;
      const bool use_rgb =
        (rgb_active && mono_active) ? (want.empty() ? (k % 2 != 0) : want == "rgb") : rgb_active;
      if (!want.empty() && (want == "rgb") != use_rgb)
        LOG(Warning, "Gimbal " << g << ": camera=" << want << " is not running — tracking from "
                               << (use_rgb ? "rgb" : "mono") << " instead");
      const auto pipe = use_rgb ? cfg.rgb_ : cfg.mono_;
      const auto trk = use_rgb ? cfg.tracking_cfg_[g].rgb_ : cfg.tracking_cfg_[g].mono_;
      auto *predictions = use_rgb ? &predictions_rgb : &predictions_mono;
      const std::vector<size_t> follower_ids = followers[g];
      const auto all_gimbals = cfg.gimbals_;

      tracking_threads.emplace_back([motor = drive_motor, g, trk, gimbal_cfg, pipe, use_rgb,
                                     predictions, follower_ids, all_gimbals, telemetry]() {
        auto *det = use_rgb ? &detection_rgb : &detection_mono;

        const bool driving = motor != nullptr;
        const auto home = driving ? motor->home_position(g) : std::pair<float, float>{0.0F, 0.0F};
        // Assumes the centre of the calibrated tilt range is level.
        const auto filter_params = patronus::tracking::make_filter_parameters(
          trk, pipe.detection_width_, pipe.detection_height_, pipe.focal_x_px_, pipe.focal_y_px_,
          home.second);
        patronus::tracking::FilterState filter(filter_params);

        const patronus::tracking::SensorControlGains gains{
          .kp_ = trk.kp_, .kd_ = trk.kd_, .lead_gain_ = trk.lead_gain_};

        if (driving && trk.filter_enabled_ && !filter_params.ego_active) {
          LOG(Warning, "Tracking gimbal "
                         << g
                         << ": no focal length set (focal_x_px/focal_y_px). Camera motion is not "
                            "compensated, so the filter will read the gimbal's own slew as target "
                            "motion. Set these from the camera's lens and sensor specs.");
        }

        // Followers aim at the leader's bearing, measured from each gimbal's home.
        const bool lead_followers =
          driving && !follower_ids.empty() && pipe.focal_x_px_ > 0.0F && pipe.focal_y_px_ > 0.0F;
        if (driving && !follower_ids.empty() && !lead_followers) {
          LOG(Warning, "Tracking gimbal " << g
                                          << ": no focal length set, so its followers cannot be "
                                             "aimed — they stay at home");
        }
        std::pair<float, float> pose{0.0F, 0.0F};
        const double max_pose_age_s = static_cast<double>(trk.filter_tick_ms_) * 45 * 0.001;
        patronus::tracking::TickRecord tick_rec;
        auto *bus_metrics = telemetry ? &tick_rec.bus : nullptr;

        if (telemetry) {
          std::ostringstream note;
          note << "gimbal " << g << " camera " << (use_rgb ? "rgb" : "mono") << " det_w "
               << pipe.detection_width_ << " det_h " << pipe.detection_height_ << " focal_x "
               << pipe.focal_x_px_ << " focal_y " << pipe.focal_y_px_ << " home_pan " << home.first
               << " home_tilt " << home.second << " driving " << (driving ? 1 : 0) << " ego "
               << (filter_params.ego_active ? 1 : 0);
          telemetry->note(note.str());
        }

        const auto aim_followers = [&](patronus::core::Point error_px) {
          if (!lead_followers)
            return;
          const double start = telemetry ? patronus::core::steady_now_s() : 0.0;
          const float pan = pose.first - home.first + std::atan(error_px.cx_ / pipe.focal_x_px_);
          const float tilt = pose.second - home.second + std::atan(error_px.cy_ / pipe.focal_y_px_);
          for (const size_t f : follower_ids) {
            const auto &fc = all_gimbals[f];
            const auto [f_pan, f_tilt] = motor->home_position(f);
            motor->drive_to(f, f_pan + fc.pan_offset_rad_ + pan,
                            f_tilt + fc.tilt_offset_rad_ + tilt, fc.follow_gain_,
                            trk.home_tolerance_rad_, 0.0F, max_pose_age_s, bus_metrics);
          }
          if (telemetry)
            tick_rec.follower_us += (patronus::core::steady_now_s() - start) * 1e6;
        };

        const auto park_followers = [&](bool hold) {
          if (!driving)
            return;
          const double start = telemetry ? patronus::core::steady_now_s() : 0.0;
          for (const size_t f : follower_ids) {
            if (hold)
              motor->set_velocity(f, 0.0F, 0.0F, bus_metrics);
            else
              motor->return_to_home(f, trk.home_return_gain_, trk.home_tolerance_rad_,
                                    trk.home_return_max_velocity_, max_pose_age_s, bus_metrics);
          }
          if (telemetry && !follower_ids.empty())
            tick_rec.follower_us += (patronus::core::steady_now_s() - start) * 1e6;
        };

        // HOT PATH: no allocation or file I/O; logs only on state changes.
        using patronus::tracking::ControlPhase;
        using patronus::tracking::MotorAction;
        const auto control_params = patronus::tracking::make_control_parameters(trk);
        const int64_t control_start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                           std::chrono::steady_clock::now().time_since_epoch())
                                           .count();
        patronus::tracking::ControlState control{ControlPhase::Tracking, control_start_ns};

        // Motor period. The filter steps per frame, not per tick.
        const auto tick = std::chrono::milliseconds(trk.filter_tick_ms_);

        const float det_w =
          (pipe.detection_width_ > 0U) ? static_cast<float>(pipe.detection_width_) : 1.0F;
        const float det_h =
          (pipe.detection_height_ > 0U) ? static_cast<float>(pipe.detection_height_) : 1.0F;
        const float half_w = det_w * 0.5F;
        const float half_h = det_h * 0.5F;

        std::optional<patronus::tracking::Estimate> est;
        uint64_t observation_cursor = 0; // Private to this leader and its camera.
        patronus::core::FrameObservation last_observation;
        double previous_start_s = 0.0;

        // Queued at the top of the next tick, so the `continue`s cannot skip it.
        bool tick_pending = false;

        while (running) {
          double submit_us = 0.0;
          if (telemetry && tick_pending) {
            const double submit_start = patronus::core::steady_now_s();
            tick_rec.loop_us = (submit_start - tick_rec.loop_start_s) * 1.0e6;
            telemetry->record(tick_rec);
            submit_us = (patronus::core::steady_now_s() - submit_start) * 1e6;
          }
          tick_pending = telemetry != nullptr;
          tick_rec = patronus::tracking::TickRecord{};
          tick_rec.gimbal = static_cast<uint32_t>(g);
          tick_rec.driving = driving;
          tick_rec.cmd_pan_rad_s = 0.0F;
          tick_rec.cmd_tilt_rad_s = 0.0F;

          const auto now = std::chrono::steady_clock::now();
          const double t_s = patronus::core::steady_now_s();
          tick_rec.t_s = t_s;
          tick_rec.loop_start_s = t_s;
          if (telemetry) {
            tick_rec.previous_submit_us = submit_us;
            tick_rec.iteration_us = previous_start_s > 0.0 ? (t_s - previous_start_s) * 1e6 : 0.0;
            previous_start_s = t_s;
          }

          // Stamped mid-read: each axis is a separate CAN round trip.
          if (driving) {
            tick_rec.position = motor->get_position(g, bus_metrics);
            const double read_end_s = patronus::core::steady_now_s();
            tick_rec.pose_read_us = (read_end_s - t_s) * 1.0e6;
            tick_rec.pan_rad = tick_rec.position.pan.value;
            tick_rec.tilt_rad = tick_rec.position.tilt.value;
          }

          const double wait_s = telemetry ? patronus::core::steady_now_s() : 0.0;
          const auto previous_cursor = observation_cursor;
          const auto opt = det->try_read(observation_cursor, tick);
          if (telemetry) {
            const double received_s = patronus::core::steady_now_s();
            tick_rec.observation_wait_us = (received_s - wait_s) * 1.0e6;
            if (opt) {
              last_observation = *opt;
              tick_rec.observation_received_s = received_s;
              tick_rec.skipped_observations = observation_cursor - previous_cursor - 1;
            }
            tick_rec.source_frame_id = last_observation.frame_id_;
            tick_rec.source_probe_s = last_observation.timestamp_s_;
            tick_rec.source_publish_s = last_observation.published_s_;
            tick_rec.source_capture_s = last_observation.captured_at_s_;
          }
          tick_rec.pose_usable =
            !driving || tick_rec.position.fresh(patronus::core::steady_now_s(), max_pose_age_s);
          if (driving && tick_rec.pose_usable) {
            pose = {tick_rec.position.pan.value, tick_rec.position.tilt.value};
            tick_rec.t_s = 0.5 * (tick_rec.position.pan.time_s + tick_rec.position.tilt.time_s);
            patronus::tracking::push_gimbal_sample(filter.poses, tick_rec.t_s, pose.first,
                                                   pose.second, filter_params.level_tilt_rad);
          } else if (!tick_rec.pose_usable) {
            // Never coast using a fabricated pose or bridge a gap with old pose history.
            patronus::tracking::reset_filter(filter);
            est.reset();
            predictions->clear(g);
          }
          if (trk.filter_enabled_ && tick_rec.pose_usable) {
            if (opt.has_value()) {
              const double step_s = patronus::core::steady_now_s();
              est = patronus::tracking::step_filter(filter, filter_params, *opt, step_s);
              if (telemetry)
                tick_rec.filter_us = (patronus::core::steady_now_s() - step_s) * 1.0e6;
              if (telemetry)
                telemetry->record(
                  patronus::tracking::FrameRecord{static_cast<uint32_t>(g), step_s, *opt, est});

              // Published even when not aiming: a coasting prediction is what an
              // operator wants to see during an occlusion.
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
                predictions->clear(g);
              }
            }

          } else if (opt.has_value() && telemetry) {
            telemetry->record(patronus::tracking::FrameRecord{
              static_cast<uint32_t>(g), patronus::core::steady_now_s(), *opt, std::nullopt});
          }

          const patronus::tracking::ControlInput control_input{
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count(),
            opt.has_value() && opt->detection_.has_value(), est.has_value(), driving,
            tick_rec.pose_usable};
          const double control_start_s = telemetry ? patronus::core::steady_now_s() : 0.0;
          const auto decision =
            patronus::tracking::step_control(control, control_params, control_input);
          if (telemetry)
            tick_rec.control_us = (patronus::core::steady_now_s() - control_start_s) * 1e6;
          tick_rec.control_elapsed_ns = control_input.now_ns - control_start_ns;
          tick_rec.raw_detection = control_input.raw_detection;
          tick_rec.estimate_available = control_input.estimate_available;
          if (decision.leader == MotorAction::Aim && trk.filter_enabled_) {
            // Every tick, not per frame: see tracking::aim_at().
            tick_rec.aim_s = patronus::core::steady_now_s();
            if (const auto aim = patronus::tracking::aim_at(filter, filter_params, tick_rec.aim_s);
                aim.has_value()) {
              const auto cmd = patronus::tracking::compute_control_sensor(
                aim->error_px, aim->velocity_px_s, gains);
              if (telemetry) {
                tick_rec.command_start_s = patronus::core::steady_now_s();
                tick_rec.aim_compute_us = (tick_rec.command_start_s - tick_rec.aim_s) * 1e6;
              }
              tick_rec.command = motor->set_velocity(g, cmd.pan_ * gimbal_cfg.max_velocity_rad_s_,
                                                     cmd.tilt_ * gimbal_cfg.max_velocity_rad_s_, bus_metrics);
              if (telemetry)
                tick_rec.command_done_s = patronus::core::steady_now_s();
              tick_rec.aim = aim;
              if (tick_rec.command.ok())
                aim_followers(aim->error_now_px);
              else
                park_followers(true);
            } else if (telemetry) {
              tick_rec.aim_compute_us = (patronus::core::steady_now_s() - tick_rec.aim_s) * 1e6;
            }
          } else if (decision.leader == MotorAction::Aim) {
            const double aim_start = telemetry ? patronus::core::steady_now_s() : 0.0;
            const patronus::core::Detection d = *opt->detection_;
            const patronus::core::Point center{d.left_ + (d.width_ / 2.0F),
                                               d.top_ + (d.height_ / 2.0F)};
            const auto error = patronus::tracking::detection_error(center, pipe.detection_width_,
                                                                   pipe.detection_height_);
            const auto raw_cmd = patronus::tracking::compute_control_sensor(
              error, patronus::core::Point{0.0F, 0.0F}, gains);
            if (telemetry) {
              tick_rec.command_start_s = patronus::core::steady_now_s();
              tick_rec.aim_compute_us = (tick_rec.command_start_s - aim_start) * 1e6;
            }
            tick_rec.command = motor->set_velocity(g, raw_cmd.pan_ * gimbal_cfg.max_velocity_rad_s_,
                                                   raw_cmd.tilt_ * gimbal_cfg.max_velocity_rad_s_, bus_metrics);
            if (telemetry)
              tick_rec.command_done_s = patronus::core::steady_now_s();
            if (tick_rec.command.ok())
              aim_followers(error);
            else
              park_followers(true);
          }

          tick_rec.aiming = decision.aiming;
          tick_rec.state = static_cast<int>(decision.phase);

          if (decision.followers == MotorAction::Hold)
            park_followers(true);
          else if (decision.followers == MotorAction::ReturnHome)
            park_followers(false);

          if (decision.leader == MotorAction::ReturnHome) {
            if (telemetry)
              tick_rec.command_start_s = patronus::core::steady_now_s();
            const auto home_result =
              motor->return_to_home(g, trk.home_return_gain_, trk.home_tolerance_rad_,
                                    trk.home_return_max_velocity_, max_pose_age_s, bus_metrics);
            tick_rec.command = home_result.command;
            tick_rec.return_position = home_result.position;
            const bool reached_home = home_result.reached();
            tick_rec.reached_home = reached_home;
            const bool arrived = patronus::tracking::apply_home_feedback(control, reached_home);
            // return_to_home reports arrival only after its paired hold succeeds.
            if (telemetry)
              tick_rec.command_done_s = patronus::core::steady_now_s();
            if (arrived)
              LOG(Info, "Tracking gimbal " << g << ": at home position");
          } else if (decision.leader == MotorAction::Hold) {
            if (telemetry)
              tick_rec.command_start_s = patronus::core::steady_now_s();
            tick_rec.command = motor->set_velocity(g, 0.0F, 0.0F, bus_metrics);
            if (telemetry)
              tick_rec.command_done_s = patronus::core::steady_now_s();
          }

          tick_rec.cmd_pan_rad_s = tick_rec.command.pan.status == patronus::comm::IoStatus::Ok
                                     ? tick_rec.command.pan.attempted
                                     : std::nanf("");
          tick_rec.cmd_tilt_rad_s = tick_rec.command.tilt.status == patronus::comm::IoStatus::Ok
                                      ? tick_rec.command.tilt.attempted
                                      : std::nanf("");

          if (decision.phase == ControlPhase::Tracking &&
              control.phase == ControlPhase::ReturningHome) {
            LOG(Warning, "Tracking gimbal " << g << ": lost target — returning to home");
          }
        }

        if (telemetry && tick_pending) {
          tick_rec.loop_us = (patronus::core::steady_now_s() - tick_rec.loop_start_s) * 1.0e6;
          telemetry->record(tick_rec);
        }

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

  return 0;
}
