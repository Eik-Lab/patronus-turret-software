#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
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
  /// Time of the camera frame this estimate is valid at, on the shared
  /// steady-clock base, seconds. The video thread extrapolates from here to the
  /// frame it is drawing, and drops samples older than a fraction of a second so
  /// a tracking thread that died mid-run cannot leave a frozen marker on screen.
  double publish_t_s{0.0};

  /// Estimated target centre at `publish_t_s`, normalised.
  float u_norm{0.0F};
  float v_norm{0.0F};

  /// Target velocity, camera motion removed, normalised units per second. The
  /// overlay draws the velocity arrow along this.
  float vu_norm_s{0.0F};
  float vv_norm_s{0.0F};

  /// Image velocity induced by the camera's own motion, normalised units per
  /// second. The target moves through the image at the sum of this and its own
  /// velocity, which is what carries the estimate forward to the drawn frame.
  float camera_vu_norm_s{0.0F};
  float camera_vv_norm_s{0.0F};

  /// 1-sigma position uncertainty per axis, normalised by the detection width
  /// and height respectively. The overlay draws the 3-sigma ellipse from these,
  /// so a coasting estimate visibly balloons while it extrapolates.
  float sigma_u_norm{0.0F};
  float sigma_v_norm{0.0F};

  /// Centre of the most recent detection the filter accepted, normalised. This
  /// is the raw measurement, not a filter output.
  float detection_u_norm{0.0F};
  float detection_v_norm{0.0F};

  /// `FrameObservation::frame_id_` of that detection. The pipeline compares it
  /// with the frame it sent to learn whether its detection was accepted.
  uint64_t detection_frame_id{0};

  /// Whether the detection of the frame at `publish_t_s` was accepted. False
  /// while the filter is coasting.
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
/// The critical section is a single small trivially-copyable struct copy per
/// publish. At the 100 Hz
/// tick rate that is negligible against a per-tick cost that already includes
/// a Kalman predict and update, and it is a different cache line per gimbal.
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