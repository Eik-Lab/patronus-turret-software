#pragma once

#include "core/types.h"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>

namespace patronus::core {

/// Shared monotonic clock for every cross-thread timestamp.
[[nodiscard]] inline double steady_now_s() {
  static const std::chrono::steady_clock::time_point epoch = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - epoch).count();
}

/// Copied out of NvDsObjectMeta so it can cross threads.
struct Detection {
  float left_;
  float top_;
  float width_;
  float height_;
  int class_id_;     ///< 0 = bird.
  float confidence_; ///< [0, 1]

  /// Copies of the owning FrameObservation's fields.
  double timestamp_s_ = 0.0;
  uint64_t frame_id_ = 0;

  [[nodiscard]] bool valid_timestamp() const noexcept {
    return timestamp_s_ > 0.0;
  }
};

/// Published for every frame: one without a detection is still a filter step.
struct FrameObservation {
  /// Probe arrival, after inference.
  double timestamp_s_{0.0};

  /// De-jittered camera cadence; only differences matter. 0 = unavailable.
  double capture_s_{0.0};

  /// Mid-exposure instant, where the gimbal pose is looked up. 0 = unavailable.
  double captured_at_s_{0.0};

  /// Kept so telemetry can recompute captured_at_s_.
  double readout_s_{0.0};
  double exposure_s_{0.0};

  double frame_dt_s_{0.0};
  double published_s_{0.0}; ///< Just before publishing to the observation channel.

  uint64_t frame_id_{0};

  /// Most confident bird only.
  std::optional<Detection> detection_;
};

/// Single-slot broadcast; push overwrites. Slow readers skip superseded values.
/// Each reader owns a cursor initialized to zero, used with this channel only.
/// Reads copy the latest value without consuming it for other readers.
template <typename T>
class LatestValue {
private:
  std::optional<T> slot_;
  uint64_t sequence_{0};
  std::mutex mtx_;
  std::condition_variable cv_;

public:
  void push(T value) {
    {
      std::scoped_lock lock(mtx_);
      slot_ = std::move(value);
      ++sequence_;
    }
    cv_.notify_all();
  }

  template <typename Rep, typename Period>
  std::optional<T> try_read(uint64_t &cursor, const std::chrono::duration<Rep, Period> &timeout) {
    std::unique_lock<std::mutex> lock(mtx_);
    if (!cv_.wait_for(lock, timeout, [this, &cursor]() {
          return slot_.has_value() && sequence_ != cursor;
        }))
      return std::nullopt;
    T value = *slot_;
    cursor = sequence_;
    return value;
  }

  void clear() {
    std::scoped_lock lock(mtx_);
    slot_.reset();
  }
};

} // namespace patronus::core
