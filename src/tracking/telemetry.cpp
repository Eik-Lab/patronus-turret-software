#include "tracking/telemetry.h"

#include "log.h"

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace patronus::tracking {
namespace {

  // Tens of seconds of headroom.
  constexpr size_t k_ring_capacity = 4096;
  constexpr auto k_drain_period = std::chrono::milliseconds(100);

  template <typename T> void push(std::vector<T> &slots, size_t &head, size_t &count,
                                  uint64_t &dropped, const T &r) noexcept {
    if (count == slots.size()) {
      ++dropped;
      return;
    }
    slots[(head + count) % slots.size()] = r;
    ++count;
  }

  template <typename T> void take(std::vector<T> &slots, size_t &head, size_t &count,
                                  std::vector<T> &out) {
    out.clear();
    for (size_t i = 0; i < count; ++i)
      out.push_back(slots[(head + i) % slots.size()]);
    head = (head + count) % slots.size();
    count = 0;
  }

  constexpr const char *k_frames_header =
    "gimbal,frame_id,step_s,timestamp_s,captured_at_s,capture_s,frame_dt_s,readout_s,"
    "exposure_s,det,det_cx,det_cy,det_w,det_h,det_conf,est,measured,coast,cam_comp,est_capture_s,"
    "frame_pan,frame_tilt,est_u,est_v,vu,vv,au,av,cam_vu,cam_vv,sigma_u,sigma_v,nis,lead_h,"
    "lead_u,lead_v,published_s\n";

  constexpr const char *k_ticks_header =
    "gimbal,t_s,state,aiming,driving,pan,tilt,aim,aim_h,aim_u,aim_v,aim_now_u,aim_now_v,aim_vu,"
    "aim_vv,cmd_pan,cmd_tilt,loop_start_s,loop_us,pose_read_us,observation_wait_us,filter_us,"
    "aim_s,command_done_s,control_elapsed_ns,raw_detection,estimate_available,reached_home,"
    "pose_usable,pan_read_s,pan_read_status,pan_read_error,tilt_read_s,tilt_read_status,tilt_read_"
    "error,"
    "pan_requested,pan_attempted,pan_write_status,pan_write_error,pan_limit_value,pan_limit_s,pan_"
    "limit_status,pan_limit_error,"
    "tilt_requested,tilt_attempted,tilt_write_status,tilt_write_error,tilt_limit_value,tilt_limit_"
    "s,tilt_limit_status,tilt_limit_error,"
    "pan_stop_status,pan_stop_error,tilt_stop_status,tilt_stop_error,"
    "return_pan,return_pan_s,return_pan_status,return_pan_error,"
    "return_tilt,return_tilt_s,return_tilt_status,return_tilt_error,"
    "previous_submit_us,iteration_us,control_us,aim_compute_us,follower_us,"
    "command_start_s,observation_received_s,skipped_observations,source_frame_id,"
    "source_probe_s,source_publish_s,source_capture_s,bus_lock_wait_us,sdk_read_us,sdk_write_us,"
    "sdk_read_calls,sdk_write_calls\n";

} // namespace

std::unique_ptr<Telemetry> Telemetry::open(const std::string &parent_dir,
                                           const std::string &config_path) {
  char stamp[32];
  const std::time_t now = std::time(nullptr);
  std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", std::localtime(&now));
  const std::filesystem::path dir = std::filesystem::path(parent_dir) / stamp;

  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {
    LOG(Error, "Telemetry: cannot create " << dir.string() << ": " << ec.message());
    return nullptr;
  }

  std::unique_ptr<Telemetry> t(new Telemetry());
  t->dir_ = dir.string();
  t->frames_ = std::fopen((dir / "frames.csv").c_str(), "w");
  t->ticks_ = std::fopen((dir / "ticks.csv").c_str(), "w");
  t->run_ = std::fopen((dir / "run.txt").c_str(), "w");
  if (t->frames_ == nullptr || t->ticks_ == nullptr || t->run_ == nullptr) {
    LOG(Error, "Telemetry: cannot create files in " << t->dir_);
    return nullptr;
  }
  std::fputs(k_frames_header, t->frames_);
  std::fputs(k_ticks_header, t->ticks_);

  // Pins steady_now_s() to the wall clock.
  char wall[32];
  std::strftime(wall, sizeof(wall), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
  std::fprintf(t->run_, "start %s steady_s %.6f\n", wall, core::steady_now_s());

  std::ifstream cfg(config_path);
  std::stringstream text;
  text << cfg.rdbuf();
  std::fprintf(t->run_, "--- config %s ---\n%s\n--- end config ---\n", config_path.c_str(),
               text.str().c_str());
  std::fflush(t->run_);

  t->frame_ring_.slots.resize(k_ring_capacity);
  t->tick_ring_.slots.resize(k_ring_capacity);
  t->frame_batch_.reserve(k_ring_capacity);
  t->tick_batch_.reserve(k_ring_capacity);
  t->writer_ = std::thread([p = t.get()]() {
    p->writer_loop();
  });

  LOG(Info, "Telemetry: writing to " << t->dir_);
  return t;
}

Telemetry::~Telemetry() {
  {
    const std::scoped_lock lock(mutex_);
    stop_ = true;
  }
  cv_.notify_one();
  if (writer_.joinable())
    writer_.join();

  if (run_ != nullptr) {
    std::fprintf(run_, "end steady_s %.6f dropped_frames %llu dropped_ticks %llu\n",
                 core::steady_now_s(), static_cast<unsigned long long>(frame_ring_.dropped),
                 static_cast<unsigned long long>(tick_ring_.dropped));
    std::fclose(run_);
  }
  if (frames_ != nullptr)
    std::fclose(frames_);
  if (ticks_ != nullptr)
    std::fclose(ticks_);
}

void Telemetry::note(const std::string &line) {
  const std::scoped_lock lock(run_mutex_);
  std::fprintf(run_, "%s\n", line.c_str());
  std::fflush(run_);
}

void Telemetry::record(const FrameRecord &r) noexcept {
  const std::scoped_lock lock(mutex_);
  push(frame_ring_.slots, frame_ring_.head, frame_ring_.count, frame_ring_.dropped, r);
}

void Telemetry::record(const TickRecord &r) noexcept {
  const std::scoped_lock lock(mutex_);
  push(tick_ring_.slots, tick_ring_.head, tick_ring_.count, tick_ring_.dropped, r);
}

void Telemetry::writer_loop() {
  std::unique_lock lock(mutex_);
  for (;;) {
    cv_.wait_for(lock, k_drain_period, [this]() {
      return stop_;
    });
    const bool stopping = stop_;
    take(frame_ring_.slots, frame_ring_.head, frame_ring_.count, frame_batch_);
    take(tick_ring_.slots, tick_ring_.head, tick_ring_.count, tick_batch_);
    lock.unlock();
    drain();
    // Stop only after taking the final batch under the same lock as stop_.
    if (stopping)
      return;
    lock.lock();
  }
}

void Telemetry::drain() {
  for (const FrameRecord &r : frame_batch_)
    write_frame(r);
  for (const TickRecord &r : tick_batch_)
    write_tick(r);
  std::fflush(frames_);
  std::fflush(ticks_);
}

void Telemetry::write_frame(const FrameRecord &r) {
  const core::FrameObservation &o = r.obs;
  std::fprintf(frames_, "%u,%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,", r.gimbal,
               static_cast<unsigned long long>(o.frame_id_), r.step_s, o.timestamp_s_,
               o.captured_at_s_, o.capture_s_, o.frame_dt_s_, o.readout_s_, o.exposure_s_);
  if (o.detection_.has_value()) {
    // Raw box centre; the estimate columns are aim errors from frame centre.
    const core::Detection &d = *o.detection_;
    std::fprintf(frames_, "1,%.3f,%.3f,%.2f,%.2f,%.3f,", d.left_ + (d.width_ * 0.5F),
                 d.top_ + (d.height_ * 0.5F), d.width_, d.height_, d.confidence_);
  } else {
    std::fputs("0,,,,,,", frames_);
  }
  if (r.est.has_value()) {
    const Estimate &e = *r.est;
    const core::Point a = e.acceleration();
    const core::Point le = e.lead_error();
    std::fprintf(frames_,
                 "1,%d,%d,%d,%.6f,%.7f,%.7f,%.3f,%.3f,%.2f,%.2f,%.1f,%.1f,%.2f,%.2f,%.3f,%.3f,"
                 "%.3f,%.4f,%.3f,%.3f",
                 e.measured() ? 1 : 0, e.coast_frames(), e.camera_compensated() ? 1 : 0,
                 e.capture_s(), e.frame_pan_rad(), e.frame_tilt_rad(), e.error_u_px(),
                 e.error_v_px(), e.vu(), e.vv(), a.cx_, a.cy_, e.camera_vu(), e.camera_vv(),
                 e.sigma_u_px(), e.sigma_v_px(), e.nis(), e.lead_horizon_s(), le.cx_, le.cy_);
  } else {
    std::fputs("0,,,,,,,,,,,,,,,,,,,,", frames_);
  }
  std::fprintf(frames_, ",%.9f\n", o.published_s_);
}

void Telemetry::write_tick(const TickRecord &r) {
  std::fprintf(ticks_, "%u,%.6f,%d,%d,%d,%.7f,%.7f,", r.gimbal, r.t_s, r.state, r.aiming ? 1 : 0,
               r.driving ? 1 : 0, r.pan_rad, r.tilt_rad);
  if (r.aim.has_value()) {
    const AimPoint &a = *r.aim;
    std::fprintf(ticks_, "1,%.4f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,", a.horizon_s, a.error_px.cx_,
                 a.error_px.cy_, a.error_now_px.cx_, a.error_now_px.cy_, a.velocity_px_s.cx_,
                 a.velocity_px_s.cy_);
  } else {
    std::fputs("0,,,,,,,,", ticks_);
  }
  if (std::isfinite(r.cmd_pan_rad_s) && std::isfinite(r.cmd_tilt_rad_s))
    std::fprintf(ticks_, "%.5f,%.5f", r.cmd_pan_rad_s, r.cmd_tilt_rad_s);
  else
    std::fputs(",", ticks_);
  std::fprintf(ticks_, ",%.9f,%.3f,%.3f,%.3f,%.3f,%.9f,%.9f,%lld,%d,%d,%d,%d", r.loop_start_s,
               r.loop_us, r.pose_read_us, r.observation_wait_us, r.filter_us, r.aim_s,
               r.command_done_s, static_cast<long long>(r.control_elapsed_ns),
               r.raw_detection ? 1 : 0, r.estimate_available ? 1 : 0, r.reached_home ? 1 : 0,
               r.pose_usable ? 1 : 0);
  for (const auto *axis : {&r.position.pan, &r.position.tilt})
    std::fprintf(ticks_, ",%.9f,%d,%d", axis->time_s, static_cast<int>(axis->status),
                 axis->sdk_error);
  for (const auto *axis : {&r.command.pan, &r.command.tilt})
    std::fprintf(ticks_, ",%.5f,%.5f,%d,%d,%.7f,%.9f,%d,%d", axis->requested, axis->attempted,
                 static_cast<int>(axis->status), axis->sdk_error, axis->limit_position.value,
                 axis->limit_position.time_s, static_cast<int>(axis->limit_position.status),
                 axis->limit_position.sdk_error);
  for (const auto *axis : {&r.command.stop_pan, &r.command.stop_tilt})
    std::fprintf(ticks_, ",%d,%d", static_cast<int>(axis->status), axis->sdk_error);
  for (const auto *axis : {&r.return_position.pan, &r.return_position.tilt})
    std::fprintf(ticks_, ",%.7f,%.9f,%d,%d", axis->value, axis->time_s,
                 static_cast<int>(axis->status), axis->sdk_error);
  std::fprintf(ticks_, ",%.3f,%.3f,%.3f,%.3f,%.3f", r.previous_submit_us, r.iteration_us,
               r.control_us, r.aim_compute_us, r.follower_us);
  std::fprintf(ticks_, ",%.9f,%.9f,%llu,%llu,%.9f,%.9f,%.9f", r.command_start_s,
               r.observation_received_s, static_cast<unsigned long long>(r.skipped_observations),
               static_cast<unsigned long long>(r.source_frame_id), r.source_probe_s,
               r.source_publish_s, r.source_capture_s);
  std::fprintf(ticks_, ",%.3f,%.3f,%.3f,%u,%u\n", r.bus.lock_wait_us, r.bus.read_us,
               r.bus.write_us, r.bus.read_calls, r.bus.write_calls);
}

} // namespace patronus::tracking
