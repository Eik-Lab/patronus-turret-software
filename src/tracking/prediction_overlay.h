#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>

namespace patronus::tracking {

/// Normalised [0, 1] detection-image coordinates, so the video thread can
/// scale to muxed pixels without knowing the detection size.
struct PredictionSample {
  /// steady_now_s() of the frame the estimate is for.
  double publish_t_s{0.0};

  float u_norm{0.0F};
  float v_norm{0.0F};

  /// Camera motion removed.
  float vu_norm_s{0.0F};
  float vv_norm_s{0.0F};

  /// Image motion caused by the camera; the target moves at the sum.
  float camera_vu_norm_s{0.0F};
  float camera_vv_norm_s{0.0F};

  float sigma_u_norm{0.0F};
  float sigma_v_norm{0.0F};

  /// Last accepted detection, unfiltered.
  float detection_u_norm{0.0F};
  float detection_v_norm{0.0F};

  uint64_t detection_frame_id{0};

  /// This frame's detection was accepted.
  bool measured{false};
};

/// One slot per gimbal that reads do not empty: the overlay redraws every
/// gimbal's latest prediction on each frame.
class PredictionChannel {
public:
  static constexpr size_t k_max_gimbals = 8;

  /// Out-of-range gimbals are ignored.
  void publish(size_t gimbal, const PredictionSample &sample) noexcept;

  [[nodiscard]] std::optional<PredictionSample> get(size_t gimbal) const noexcept;

  /// On track loss and thread exit, so no stale marker is drawn.
  void clear(size_t gimbal) noexcept;
  void clear() noexcept;

private:
  mutable std::mutex mutex_;
  std::array<std::optional<PredictionSample>, k_max_gimbals> slots_;
};

} // namespace patronus::tracking