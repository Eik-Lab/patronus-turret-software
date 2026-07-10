#pragma once

#include <condition_variable>
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
};

/// @brief Thread-safe single-slot container for the most recent value.
///        Uses a mutex + condition variable so the consumer blocks until
///        a new value is available.
///        Hot path: always use try_pop() with short timeouts — never wait() or pop().
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

  /// @brief Block until a value is available, consume it, and return it.
  T pop() {
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock, [this]() {
      return slot_.has_value();
    });
    T value = std::move(*slot_);
    slot_.reset();
    return value;
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
};

} // namespace patronus::core
