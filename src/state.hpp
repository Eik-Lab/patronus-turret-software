#pragma once

#include "gstnvdsmeta.h"
#include <mutex>

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
