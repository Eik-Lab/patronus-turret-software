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
    "lead_u,lead_v\n";

  constexpr const char *k_ticks_header =
    "gimbal,t_s,state,aiming,driving,pan,tilt,aim,aim_h,aim_u,aim_v,aim_now_u,aim_now_v,aim_vu,"
    "aim_vv,cmd_pan,cmd_tilt\n";

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
  while (!stop_) {
    cv_.wait_for(lock, k_drain_period, [this]() {
      return stop_;
    });
    take(frame_ring_.slots, frame_ring_.head, frame_ring_.count, frame_batch_);
    take(tick_ring_.slots, tick_ring_.head, tick_ring_.count, tick_batch_);
    lock.unlock();
    drain();
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
                 "%.3f,%.4f,%.3f,%.3f\n",
                 e.measured() ? 1 : 0, e.coast_frames(), e.camera_compensated() ? 1 : 0,
                 e.capture_s(), e.frame_pan_rad(), e.frame_tilt_rad(), e.error_u_px(),
                 e.error_v_px(), e.vu(), e.vv(), a.cx_, a.cy_, e.camera_vu(), e.camera_vv(),
                 e.sigma_u_px(), e.sigma_v_px(), e.nis(), e.lead_horizon_s(), le.cx_, le.cy_);
  } else {
    std::fputs("0,,,,,,,,,,,,,,,,,,,,\n", frames_);
  }
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
    std::fprintf(ticks_, "%.5f,%.5f\n", r.cmd_pan_rad_s, r.cmd_tilt_rad_s);
  else
    std::fputs(",\n", ticks_);
}

} // namespace patronus::tracking
