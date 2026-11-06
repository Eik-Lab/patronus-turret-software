#include "core/config.h"
#include "core/state.h"
#include "tracking/aim_control.h"
#include "tracking/control_state.h"
#include "tracking/target_filter.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;

enum class Kind { Pose, Frame, Aim, Control };

struct Event {
  Kind kind{Kind::Aim};
  double time{};
  float pan{}, tilt{};
  patronus::core::FrameObservation frame{};
  patronus::tracking::ControlInput control{};
  bool reached_home{false};
};

std::vector<Event> load_events(const std::string &path) {
  std::ifstream input(path);
  if (!input)
    throw std::runtime_error("Cannot open events: " + path);
  std::vector<Event> events;
  std::string line;
  double previous = 0.0;
  int64_t previous_control_ns = 0;
  size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty() || line.front() == '#')
      continue;
    std::istringstream row(line);
    Event e;
    std::string kind;
    row >> kind;
    if (kind == "control") {
      row >> e.control.now_ns;
      e.time = static_cast<double>(e.control.now_ns) * 1e-9;
    } else {
      row >> e.time;
    }
    if (kind == "pose") {
      e.kind = Kind::Pose;
      row >> e.pan >> e.tilt;
    } else if (kind == "frame") {
      e.kind = Kind::Frame;
      int present = -1;
      patronus::core::Detection d{};
      row >> e.frame.timestamp_s_ >> e.frame.capture_s_ >> e.frame.captured_at_s_ >>
        e.frame.frame_id_ >> present >> d.left_ >> d.top_ >> d.width_ >> d.height_ >> d.confidence_;
      if (present != 0 && present != 1)
        row.setstate(std::ios::failbit);
      if (present == 1) {
        d.timestamp_s_ = e.frame.timestamp_s_;
        d.frame_id_ = e.frame.frame_id_;
        e.frame.detection_ = d;
      }
    } else if (kind == "control") {
      e.kind = Kind::Control;
      int raw = -1, estimate = -1, driving = -1, home = -1;
      row >> raw >> estimate >> driving >> home;
      if ((raw != 0 && raw != 1) || (estimate != 0 && estimate != 1) ||
          (driving != 0 && driving != 1) || (home != 0 && home != 1))
        row.setstate(std::ios::failbit);
      if (row) {
        e.control = {e.control.now_ns, raw == 1, estimate == 1, driving == 1};
        e.reached_home = home == 1;
        // Optional for compatibility with control streams recorded before hardware outcomes.
        if (!row.eof())
          row >> std::ws;
        if (!row.eof()) {
          int usable = -1;
          row >> usable;
          if (usable != 0 && usable != 1)
            row.setstate(std::ios::failbit);
          e.control.pose_usable = usable == 1;
        }
      }
    } else if (kind != "aim") {
      row.setstate(std::ios::failbit);
    }
    if (!row || !std::isfinite(e.time) || e.time < previous ||
        (e.kind == Kind::Control && e.control.now_ns < previous_control_ns))
      throw std::runtime_error("Invalid or unordered event at line " + std::to_string(line_number));
    std::string extra;
    if (row >> extra)
      throw std::runtime_error("Extra fields at line " + std::to_string(line_number));
    previous = e.time;
    if (e.kind == Kind::Control)
      previous_control_ns = e.control.now_ns;
    events.push_back(e);
  }
  if (events.empty())
    throw std::runtime_error("No events");
  return events;
}

struct Output {
  std::optional<patronus::tracking::Estimate> estimate;
  std::optional<patronus::tracking::AimPoint> aim;
  patronus::core::AimAngles command{};
  std::optional<patronus::tracking::ControlDecision> decision;
  patronus::tracking::ControlPhase next_phase{};
  bool stop_after_home{false};
};

void report(const char *name, std::vector<double> &samples) {
  if (samples.empty())
    return;
  std::sort(samples.begin(), samples.end());
  const auto percentile = [&](double q) {
    return samples[static_cast<size_t>(std::ceil(q * samples.size())) - 1];
  };
  std::cerr << name << " n=" << samples.size() << " p50_us=" << percentile(0.50)
            << " p99_us=" << percentile(0.99) << " max_us=" << samples.back() << '\n';
}
} // namespace

int main(int argc, char **argv) {
  try {
    if (argc != 7) {
      std::cerr << "Usage: tracking_replay CONFIG CAMERA GIMBAL LEVEL_TILT EVENTS REPEATS\n"
                   "CAMERA is rgb or mono; GIMBAL is zero-based; LEVEL_TILT is radians.\n";
      return 2;
    }
    if (!std::ifstream(argv[1]))
      throw std::runtime_error("Cannot open config");
    const auto cfg = patronus::config::load_config(argv[1]);
    const std::string camera = argv[2];
    if (camera != "rgb" && camera != "mono")
      throw std::runtime_error("Camera must be rgb or mono");
    const size_t gimbal = std::stoul(argv[3]);
    const float level = std::stof(argv[4]);
    const int repeats = std::stoi(argv[6]);
    if (gimbal >= cfg.tracking_cfg_.size() || !std::isfinite(level) || repeats < 1)
      throw std::runtime_error("Invalid gimbal, level tilt or repeats");
    const auto &pipe = camera == "rgb" ? cfg.rgb_ : cfg.mono_;
    const auto &trk =
      camera == "rgb" ? cfg.tracking_cfg_[gimbal].rgb_ : cfg.tracking_cfg_[gimbal].mono_;
    const patronus::tracking::SensorControlGains gains{trk.kp_, trk.kd_, trk.lead_gain_};
    const auto events = load_events(argv[5]);
    if (!trk.filter_enabled_ && std::any_of(events.begin(), events.end(), [](const Event &e) {
          return e.kind != Kind::Control;
        }))
      throw std::runtime_error("Filter-disabled replay supports control events only");
    std::vector<Output> output(events.size());
    std::vector<double> pose_times, frame_times, aim_times, control_times;
    std::vector<double> update_times, seed_times;
    size_t poses = 0, frames = 0, aims = 0, controls = 0;
    for (const auto &e : events) {
      poses += e.kind == Kind::Pose;
      frames += e.kind == Kind::Frame;
      aims += e.kind == Kind::Aim;
      controls += e.kind == Kind::Control;
    }
    pose_times.reserve(poses * repeats);
    frame_times.reserve(frames * repeats);
    aim_times.reserve(aims * repeats);
    control_times.reserve(controls * repeats);
    update_times.reserve(frames * repeats);
    seed_times.reserve(frames * repeats);
    const auto control_params = patronus::tracking::make_control_parameters(trk);
    const auto filter_params =
      patronus::tracking::make_filter_parameters(trk, pipe.detection_width_, pipe.detection_height_,
                                                 pipe.focal_x_px_, pipe.focal_y_px_, level);
    // One complete warm-up replay. Each measured pass starts with fresh state.
    for (int pass = -1; pass < repeats; ++pass) {
      patronus::tracking::FilterState filter(filter_params);
      patronus::tracking::ControlState control{patronus::tracking::ControlPhase::Tracking, 0};
      for (size_t i = 0; i < events.size(); ++i) {
        const Event &e = events[i];
        Output &out = output[i];
        std::vector<double> *times = nullptr;
        const bool had_track = filter.initialized;
        const auto start = Clock::now();
        if (e.kind == Kind::Pose) {
          patronus::tracking::push_gimbal_sample(filter.poses, e.time, e.pan, e.tilt,
                                                 filter_params.level_tilt_rad);
          times = &pose_times;
        } else if (e.kind == Kind::Frame) {
          out.estimate = patronus::tracking::step_filter(filter, filter_params, e.frame, e.time);
          times = &frame_times;
        } else if (e.kind == Kind::Aim) {
          out.aim = patronus::tracking::aim_at(filter, filter_params, e.time);
          if (out.aim)
            out.command = patronus::tracking::compute_control_sensor(out.aim->error_px,
                                                                     out.aim->velocity_px_s, gains);
          times = &aim_times;
        } else {
          out.decision = patronus::tracking::step_control(control, control_params, e.control);
          out.stop_after_home =
            out.decision->leader == patronus::tracking::MotorAction::ReturnHome &&
            patronus::tracking::apply_home_feedback(control, e.reached_home);
          out.next_phase = control.phase;
          times = &control_times;
        }
        const auto end = Clock::now();
        if (pass >= 0) {
          const double elapsed = std::chrono::duration<double, std::micro>(end - start).count();
          times->push_back(elapsed);
          // Seed/reacquisition also returns measured=true, but does not run an update.
          // Classification and sample storage stay outside the measured region.
          if (e.kind == Kind::Frame && out.estimate && out.estimate->measured())
            (had_track ? update_times : seed_times).push_back(elapsed);
        }
      }
    }
    // Output and parsing are outside the measured regions. No hardware commands.
    std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);
    std::cout << "event,time_s,present,measured,u,v,vu,vv,sigma_u,sigma_v,coast,cmd_pan,cmd_tilt";
    if (controls)
      std::cout << ",phase,next_phase,leader_action,follower_action,aiming,stop_after_home";
    std::cout << '\n';
    for (size_t i = 0; i < events.size(); ++i) {
      const auto &e = events[i];
      const auto &out = output[i];
      if (e.kind == Kind::Pose)
        continue;
      std::cout << (e.kind == Kind::Frame ? "frame"
                    : e.kind == Kind::Aim ? "aim"
                                          : "control")
                << ',' << e.time << ',';
      if (e.kind == Kind::Frame && out.estimate) {
        const auto &v = *out.estimate;
        std::cout << "1," << v.measured() << ',' << v.error_u_px() << ',' << v.error_v_px() << ','
                  << v.vu() << ',' << v.vv() << ',' << v.sigma_u_px() << ',' << v.sigma_v_px()
                  << ',' << v.coast_frames() << ",,";
      } else if (e.kind == Kind::Aim && out.aim) {
        const auto &v = *out.aim;
        std::cout << "1,," << v.error_px.cx_ << ',' << v.error_px.cy_ << ',' << v.velocity_px_s.cx_
                  << ',' << v.velocity_px_s.cy_ << ",,,," << out.command.pan_ << ','
                  << out.command.tilt_;
      } else if (e.kind == Kind::Control) {
        std::cout << "1,,,,,,,,,,";
      } else {
        std::cout << "0,,,,,,,,,,";
      }
      if (out.decision) {
        const auto &d = *out.decision;
        std::cout << ',' << static_cast<int>(d.phase) << ',' << static_cast<int>(out.next_phase)
                  << ',' << static_cast<int>(d.leader) << ',' << static_cast<int>(d.followers)
                  << ',' << d.aiming << ',' << out.stop_after_home;
      } else if (controls) {
        std::cout << ",,,,,,";
      }
      std::cout << '\n';
    }
    report("pose", pose_times);
    report("filter_step", frame_times);
    report("filter_successful_update", update_times);
    report("filter_seed", seed_times);
    report("aim_and_control", aim_times);
    report("control_state", control_times);
  } catch (const std::exception &error) {
    std::cerr << "Replay: " << error.what() << '\n';
    return 1;
  }
}
