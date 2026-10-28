#pragma once

#include "core/state.h"
#include "tracking/target_filter.h"

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace patronus::tracking {

struct FrameRecord {
  uint32_t gimbal{0};
  double step_s{0.0};
  core::FrameObservation obs{};
  std::optional<Estimate> est;
};

struct TickRecord {
  uint32_t gimbal{0};
  double t_s{0.0};
  int state{0}; ///< The loop's State: 0 tracking, 1 returning home, 2 at home.
  bool aiming{false};
  bool driving{false};
  float pan_rad{0.0F}; ///< Encoder angles; zero when not driving.
  float tilt_rad{0.0F};
  std::optional<AimPoint> aim;
  /// rad/s actually sent; NaN while returning home.
  float cmd_pan_rad_s{0.0F};
  float cmd_tilt_rad_s{0.0F};
};

/// Per-run CSVs of every frame and tick, read by tools/telemetry_analyse.py.
/// record() only copies into a ring; a writer thread does the I/O. A full
/// ring drops new records and counts them in run.txt.
class Telemetry {
public:
  /// nullptr if the run directory cannot be created.
  [[nodiscard]] static std::unique_ptr<Telemetry> open(const std::string &parent_dir,
                                                       const std::string &config_path);

  ~Telemetry();
  Telemetry(const Telemetry &) = delete;
  Telemetry &operator=(const Telemetry &) = delete;

  [[nodiscard]] const std::string &directory() const noexcept {
    return dir_;
  }

  /// Not for the hot path.
  void note(const std::string &line);

  /// Hot path safe.
  void record(const FrameRecord &r) noexcept;
  void record(const TickRecord &r) noexcept;

private:
  template <typename T> struct Ring {
    std::vector<T> slots;
    size_t head{0};
    size_t count{0};
    uint64_t dropped{0};
  };

  Telemetry() = default;
  void writer_loop();
  void drain();
  void write_frame(const FrameRecord &r);
  void write_tick(const TickRecord &r);

  std::string dir_;
  std::FILE *frames_{nullptr};
  std::FILE *ticks_{nullptr};
  std::FILE *run_{nullptr};

  std::mutex mutex_;
  std::condition_variable cv_;
  bool stop_{false};
  Ring<FrameRecord> frame_ring_;
  Ring<TickRecord> tick_ring_;
  // Writer-side copies, so formatting runs outside the lock.
  std::vector<FrameRecord> frame_batch_;
  std::vector<TickRecord> tick_batch_;
  std::mutex run_mutex_;
  std::thread writer_;
};

} // namespace patronus::tracking
