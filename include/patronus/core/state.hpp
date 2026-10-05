#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>

namespace patronus::core {

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
  /// Taken from the GStreamer buffer PTS, which DeepStream has already
  /// converted to wall-clock nanoseconds. The target filter needs this: a
  /// Kalman prediction is a function of dt, and the tracking loop is
  /// event-driven, so "one loop iteration" is not a meaningful time step. The
  /// cameras run at 30/40 fps while the loop ticks far faster, so assuming a
  /// fixed dt would bias the velocity estimate badly during gimbals.
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
