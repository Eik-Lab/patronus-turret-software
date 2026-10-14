#include "patronus/tracking/prediction_overlay.hpp"

namespace patronus::tracking {

void PredictionChannel::publish(size_t gimbal, const PredictionSample &sample) noexcept {
  if (gimbal >= k_max_gimbals)
    return;
  const std::lock_guard<std::mutex> lock(mutex_);
  slots_[gimbal] = sample;
}

std::optional<PredictionSample> PredictionChannel::get(size_t gimbal) const noexcept {
  if (gimbal >= k_max_gimbals)
    return std::nullopt;
  const std::lock_guard<std::mutex> lock(mutex_);
  return slots_[gimbal];
}

void PredictionChannel::clear(size_t gimbal) noexcept {
  if (gimbal >= k_max_gimbals)
    return;
  const std::lock_guard<std::mutex> lock(mutex_);
  slots_[gimbal].reset();
}

void PredictionChannel::clear() noexcept {
  const std::lock_guard<std::mutex> lock(mutex_);
  for (auto &slot : slots_)
    slot.reset();
}

} // namespace patronus::tracking