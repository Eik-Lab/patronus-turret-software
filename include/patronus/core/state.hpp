#pragma once

#include "patronus/core/types.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>

namespace patronus::core {

/// @brief Seconds on a process-wide monotonic epoch shared by every thread.
///
/// `std::chrono::steady_clock` is CLOCK_MONOTONIC on Linux, so it is already
/// consistent between threads. This gives all of them the same *origin*, which
/// is the part that is otherwise missing: a tracking thread that started its own
/// clock at thread launch produces timestamps that no other thread can compare
/// against its own `now()`.
///
/// Both ends of the prediction path depend on this agreeing. The pipeline stamps
/// `Detection::timestamp_s_` from here, the tracking loop stamps the filter's
/// tick from here, and the overlay compares publish times from here. Any
/// divergence shows up as a filter fed a negative or wildly large `dt`.
///
/// @return Seconds since the first call on any thread. Never negative.
[[nodiscard]] inline double steady_now_s() {
  static const std::chrono::steady_clock::time_point epoch = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - epoch).count();
}

/// @brief Lightweight detection result extracted from NvDsObjectMeta.
///        POD type — safe to copy across threads without dangling pointers.
struct Detection {
  float left_;       ///< Bounding-box left edge (pixels).
  float top_;        ///< Bounding-box top edge (pixels).
  float width_;      ///< Bounding-box width (pixels).
  float height_;     ///< Bounding-box height (pixels).
  int class_id_;     ///< Detection class (0 = drone).
  float confidence_; ///< Detection confidence [0, 1].

  /// Capture time of the frame this detection came from, in seconds.
  ///
  /// Stamped with `patronus::core::steady_now_s()` by the pipeline thread. The
  /// target filter needs this: a Kalman prediction is a function of dt, and the
  /// tracking loop is event-driven, so "one loop iteration" is not a meaningful
  /// time step. The cameras run at 30/40 fps while the loop ticks far faster, so
  /// assuming a fixed dt would bias the velocity estimate badly during gimbals.
  ///
  /// Not the GStreamer buffer PTS directly: PTS is wall-clock, and the filter's
  /// tick and encoder history are on the steady clock, so those two cannot be
  /// subtracted. Using one process-wide monotonic epoch for both is what makes
  /// the latency compensation possible at all.
  ///
  /// 0.0 means the timestamp was unavailable (see Detection::valid_timestamp).
  double timestamp_s_ = 0.0;

  /// Monotonic frame counter, for diagnostics and duplicate detection.
  uint64_t frame_id_ = 0;

  /// True when timestamp_s_ carries a usable value.
  [[nodiscard]] bool valid_timestamp() const noexcept {
    return timestamp_s_ > 0.0;
  }
};

/// @brief Everything the perception pipeline learned from one camera frame.
///
/// One of these is published per frame, whether or not anything was detected:
/// the target filter runs once per frame, and a frame with no detection is still
/// a time step it has to predict across. POD — no heap, safe to copy between
/// threads.
///
/// All pixel quantities are in *detection* pixels
/// (`PipelineConfig::detection_width_`), the space `Detection` boxes are in.
struct FrameObservation {
  /// When the frame reached the pipeline probe, `steady_now_s()` seconds. This
  /// is the stamp to compare against other threads' clocks (encoder history,
  /// overlay age).
  double timestamp_s_{0.0};

  /// Capture time of the frame on the camera's own cadence, seconds, with
  /// arrival jitter removed. Only differences are meaningful: this is what the
  /// filter's `dt` comes from, so inference-time jitter does not become velocity
  /// noise. 0.0 means unavailable.
  double capture_s_{0.0};

  /// Interval since the previous frame on the capture clock, seconds. 0.0 on the
  /// first frame.
  double frame_dt_s_{0.0};

  /// Monotonic per-camera frame counter.
  uint64_t frame_id_{0};

  /// The single highest-confidence drone detection in the frame, if any.
  std::optional<Detection> detection_;
};

/// @brief Thread-safe single-slot container for the most recent value.
///        Uses a mutex + condition variable so the consumer blocks until
///        a new value is available.
///        Hot path: always use try_pop() with a short timeout.
template <typename T>
class LatestValue {
private:
  std::optional<T> slot_;
  std::mutex mtx_;
  std::condition_variable cv_;

public:
  /// @brief Publish a new value, waking one blocked consumer.
  void push(T value) {
    {
      std::lock_guard<std::mutex> lock(mtx_);
      slot_ = std::move(value);
    }
    cv_.notify_one();
  }

  /// @brief Wait up to `timeout` for a value; returns empty optional if none arrives.
  template <typename Rep, typename Period>
  std::optional<T> try_pop(const std::chrono::duration<Rep, Period> &timeout) {
    std::unique_lock<std::mutex> lock(mtx_);
    if (!cv_.wait_for(lock, timeout, [this]() {
          return slot_.has_value();
        }))
      return std::nullopt;
    T value = std::move(*slot_);
    slot_.reset();
    return value;
  }

  /// @brief Discard any pending value without waiting.
  ///
  /// Used when the consumer is about to change what it considers a fresh
  /// sample, so that a queued value captured before the change is not mistaken
  /// for one captured after it.
  void clear() {
    std::lock_guard<std::mutex> lock(mtx_);
    slot_.reset();
  }
};

} // namespace patronus::core
