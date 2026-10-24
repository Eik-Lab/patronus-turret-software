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

/// @brief One camera frame as the tracking loop saw it: the observation, and
///        the filter's estimate after stepping on it.
struct FrameRecord {
  uint32_t gimbal{0};
  double step_s{0.0}; ///< `steady_now_s()` handed to `TargetFilter::step()`.
  core::FrameObservation obs{};
  std::optional<Estimate> est; ///< Empty with no track, or the filter disabled.
};

/// @brief One tracking-loop tick: the gimbal's pose and what it was told to do.
struct TickRecord {
  uint32_t gimbal{0};
  double t_s{0.0}; ///< When the encoders were read, `steady_now_s()` seconds.
  int state{0};    ///< The loop's State: 0 tracking, 1 returning home, 2 at home.
  bool aiming{false};
  bool driving{false};
  float pan_rad{0.0F};  ///< Raw pan encoder angle.
  float tilt_rad{0.0F}; ///< Raw tilt encoder angle.
  std::optional<AimPoint> aim; ///< What the control law acted on, when aiming.
  /// Velocity sent to the drives after scaling, clamping and limits, rad/s.
  /// NaN when this tick's command is not known (home return drives itself).
  float cmd_pan_rad_s{0.0F};
  float cmd_tilt_rad_s{0.0F};
};

/// @brief Tracking telemetry for tuning: every camera frame and every tick, as
///        CSV, one directory per run.
///
/// `<parent>/<YYYYmmdd-HHMMSS>/` holds `frames.csv`, `ticks.csv` and `run.txt`
/// (a copy of the config, plus notes such as each gimbal's home pose).
/// `tools/telemetry_analyse.py` summarises a run.
///
/// `record()` is hot-path safe: it copies the record into a preallocated ring
/// under a mutex held for the copy only. A writer thread formats and writes
/// the files. If the writer falls behind the ring overwrites nothing; new
/// records are dropped and counted, and the count lands in `run.txt`.
///
/// Thread-safe: every tracking thread may share one instance.
class Telemetry {
public:
  /// @brief Create the run directory and start the writer.
  /// @param parent_dir  Directory the run directory is created in.
  /// @param config_path Config file copied into `run.txt`.
  /// @return nullptr, after logging why, when the files cannot be created.
  [[nodiscard]] static std::unique_ptr<Telemetry> open(const std::string &parent_dir,
                                                       const std::string &config_path);

  ~Telemetry();
  Telemetry(const Telemetry &) = delete;
  Telemetry &operator=(const Telemetry &) = delete;

  /// @brief The run directory.
  [[nodiscard]] const std::string &directory() const noexcept {
    return dir_;
  }

  /// @brief Append a line to `run.txt`. Not for the hot path.
  void note(const std::string &line);

  /// @brief Queue a frame. HOT PATH: no allocation, no I/O.
  void record(const FrameRecord &r) noexcept;
  /// @brief Queue a tick. HOT PATH: no allocation, no I/O.
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
