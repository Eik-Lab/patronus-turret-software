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

  uint64_t frame_id_{0};

  /// Most confident bird only.
  std::optional<Detection> detection_;
};

/// Single-slot mailbox; push overwrites.
template <typename T>
class LatestValue {
private:
  std::optional<T> slot_;
  std::mutex mtx_;
  std::condition_variable cv_;

public:
  void push(T value) {
    {
      std::scoped_lock lock(mtx_);
      slot_ = std::move(value);
    }
    cv_.notify_one();
  }

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

  void clear() {
    std::scoped_lock lock(mtx_);
    slot_.reset();
  }
};

} // namespace patronus::core
