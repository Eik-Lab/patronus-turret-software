#pragma once

#include <array>
#include <cstddef>
#include <mutex>
#include <optional>

namespace patronus::tracking {

/// @brief One gimbal's prediction, as published for the video thread to draw.
///
/// Positions are **normalised** detection-image coordinates in [0, 1], not
/// pixels. The filter works in detection pixels while nvdsosd draws in muxed
/// stream pixels, and the two spaces differ by a ratio the video thread owns
/// (`muxer_width / detection_width`). Normalising here keeps that conversion on
/// the side that already knows both numbers, so the overlay cannot silently use
/// the wrong scale factor.
struct PredictionSample {
  /// When this sample was published, on the shared steady-clock base, seconds.
  /// The video thread drops samples older than a fraction of a second, so a
  /// tracking thread that died mid-run cannot leave a frozen marker on screen.
  double publish_t_s{0.0};

  /// Predicted target centre, normalised.
  float u_norm{0.0F};
  float v_norm{0.0F};

  /// Predicted centre `lead_s` seconds ahead, normalised. The overlay draws a
  /// vector from the current estimate to this point.
  float lead_u_norm{0.0F};
  float lead_v_norm{0.0F};

  /// Target velocity, ego-motion removed, normalised units per second.
  float vu_norm_s{0.0F};
  float vv_norm_s{0.0F};

  /// Size of the most recent accepted detection box, normalised.
  float box_w_norm{0.0F};
  float box_h_norm{0.0F};

  /// 1-sigma position uncertainty, normalised. Drives the overlay's confidence
  /// cue so a coasting, growing-uncertain estimate looks different from a
  /// freshly corrected one.
  float sigma_norm{0.0F};

  /// Ticks since the last accepted measurement.
  int coast_ticks{0};

  /// Whether the most recent filter tick accepted a detection.
  bool measured{false};
};

/// @brief Thread-safe hand-off of filter output from tracking threads to the
///        video thread that draws the overlay.
///
/// `LatestValue` is the right tool for detections: one producer, one consumer,
/// and a stale value is worthless. This is the opposite shape. There is one
/// producer *per gimbal* and a single consumer that wants the newest value for
/// each gimbal every time it draws a frame — draining a slot would starve the
/// overlay whenever the video thread outran the tracking tick. So this keeps one
/// non-consuming latest-value slot per gimbal instead.
///
/// The critical section is a single 40-byte copy per publish. At the 100 Hz
/// tick rate that is negligible against a per-tick cost that already includes
/// an IMM predict and update, and it is a different cache line per gimbal.
class PredictionChannel {
public:
  /// @brief Maximum gimbals whose predictions can be published at once.
  static constexpr size_t k_max_gimbals = 8;

  /// @brief Publish the newest prediction for a gimbal.
  /// @param gimbal Zero-based gimbal index; ignored if out of range.
  /// @param sample Prediction to store. Copied, not referenced.
  void publish(size_t gimbal, const PredictionSample &sample) noexcept;

  /// @brief Read the newest prediction for a gimbal, without consuming it.
  /// @param gimbal Zero-based gimbal index.
  /// @return The sample, or `std::nullopt` if that gimbal has never published.
  [[nodiscard]] std::optional<PredictionSample> get(size_t gimbal) const noexcept;

  /// @brief Discard a gimbal's prediction so the overlay stops drawing it.
  ///
  /// Call when a tracking thread exits: the motor has been commanded to zero and
  /// the estimate is no longer being maintained, so continuing to draw it would
  /// show a marker that nothing is driving.
  void clear(size_t gimbal) noexcept;

  /// @brief Discard every gimbal's prediction.
  void clear() noexcept;

private:
  mutable std::mutex mutex_;
  std::array<std::optional<PredictionSample>, k_max_gimbals> slots_;
};

} // namespace patronus::tracking