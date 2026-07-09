#pragma once

#include <mutex>
#include <optional>
#include <condition_variable>

/// @brief Lightweight detection result extracted from NvDsObjectMeta.
///        POD type — safe to copy across threads without dangling pointers.
struct Detection {
  float left, top, width, height;
  int class_id;
  float confidence;
};

template <typename T>
class LatestValue {
private:
  std::optional<T> slot;
  std::mutex mtx;
  std::condition_variable cv;

public:
  void push(T value) {
    {
      std::lock_guard<std::mutex> lock(mtx);
      slot = std::move(value);
    }
    cv.notify_one();
  }

  T pop() {
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [this]() { return slot.has_value(); });
    T value = std::move(*slot);
    slot.reset();
    return value;
  }

  /// @brief Wait up to `timeout` for a value; returns empty optional if none arrives.
  template <typename Rep, typename Period>
  std::optional<T> try_pop(const std::chrono::duration<Rep, Period>& timeout) {
    std::unique_lock<std::mutex> lock(mtx);
    if (!cv.wait_for(lock, timeout, [this]() { return slot.has_value(); }))
      return std::nullopt;
    T value = std::move(*slot);
    slot.reset();
    return value;
  }
};
