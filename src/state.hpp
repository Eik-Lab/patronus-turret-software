#pragma once

#include "gstnvdsmeta.h"
#include <mutex>
#include <queue>
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

template <typename T>
class ThreadSafeQueue {
private:
  std::queue<T> q;
  std::mutex mtx;
  std::condition_variable cv_not_empty;
  std::condition_variable cv_not_full;
  size_t max_size;

public:
  explicit ThreadSafeQueue(size_t max_size)
      : max_size(max_size) {}

  void push(T value) {
    std::unique_lock<std::mutex> lock(mtx);

    cv_not_full.wait(lock, [this]() {
      return q.size() < max_size;
    });
    q.push(std::move(value));

    cv_not_empty.notify_one();
  }

  T pop() {
    std::unique_lock<std::mutex> lock(mtx);

    cv_not_empty.wait(lock, [this]() {
      return !q.empty();
    });
    T value = std::move(q.front());
    q.pop();

    cv_not_full.notify_one();
    return value;
  }
};