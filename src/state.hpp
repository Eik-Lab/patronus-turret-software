#pragma once

#include "gstnvdsmeta.h"
#include <mutex>
#include <optional>
#include <condition_variable>

struct DetectionPair {
  NvDsObjectMeta rgb;
  NvDsObjectMeta mono;
};

class DetectionState {
public:
  void update(const NvDsObjectMeta& rgb, const NvDsObjectMeta& mono) {
    std::lock_guard<std::mutex> lock(mutex_);
    detections_.rgb = rgb;
    detections_.mono = mono;
  }

  DetectionPair get() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return detections_;
  }

private:
  DetectionPair detections_{};
  mutable std::mutex mutex_;
};

// Single-slot latest-value holder. push() always overwrites (never blocks).
// pop() blocks until a value is available, then clears the slot.
template <typename T>
class LatestValue {
private:
  std::optional<T> slot;
  std::mutex mtx;
  std::condition_variable cv;

public:
  // Overwrites whatever is in the slot. Safe to call from GStreamer probe callbacks.
  void push(T value) {
    {
      std::lock_guard<std::mutex> lock(mtx);
      slot = std::move(value);
    }
    cv.notify_one();
  }

  // Blocks until a value is available, then returns it and clears the slot.
  T pop() {
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [this]() { return slot.has_value(); });
    T value = std::move(*slot);
    slot.reset();
    return value;
  }
};