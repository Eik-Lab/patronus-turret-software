#include "tracking/prediction_overlay.h"

namespace patronus::tracking {

void PredictionChannel::publish(size_t gimbal, const PredictionSample &sample) noexcept {
  if (gimbal >= k_max_gimbals)
    return;
  const std::scoped_lock lock(mutex_);
  slots_[gimbal] = sample;
}

std::optional<PredictionSample> PredictionChannel::get(size_t gimbal) const noexcept {
  if (gimbal >= k_max_gimbals)
    return std::nullopt;
  const std::scoped_lock lock(mutex_);
  return slots_[gimbal];
}

void PredictionChannel::clear(size_t gimbal) noexcept {
  if (gimbal >= k_max_gimbals)
    return;
  const std::scoped_lock lock(mutex_);
  slots_[gimbal].reset();
}

void PredictionChannel::clear() noexcept {
  const std::scoped_lock lock(mutex_);
  for (auto &slot : slots_)
    slot.reset();
}

} // namespace patronus::tracking