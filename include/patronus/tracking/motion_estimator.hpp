#pragma once

#include "patronus/core/types.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace patronus::tracking {

/// @brief Non-owning view of one image plane in CPU memory.
struct ImageView {
  const uint8_t *data_{nullptr}; ///< First byte of the top row.
  int width_{0};                 ///< Width in pixels.
  int height_{0};                ///< Height in pixels.
  int stride_{0};                ///< Bytes per row, including padding.
  int channels_{1};              ///< 1 (luma / grey) or 4 (RGBA).
};

/// @brief Axis-aligned rectangle in image pixels.
struct PixelRect {
  float left_{0.0F};
  float top_{0.0F};
  float width_{0.0F};
  float height_{0.0F};
};

/// @brief What `MotionEstimator::track()` measured between two frames.
struct MotionResult {
  /// Displacement of the tracked target since the previous frame, image pixels.
  /// Camera motion is *not* removed. Empty when no target is being tracked or
  /// too few features survived.
  std::optional<core::Point> target_flow_px_;

  /// 1-sigma uncertainty of `target_flow_px_`, pixels.
  float target_flow_sigma_px_{0.0F};

  /// Number of features behind `target_flow_px_`.
  int target_features_{0};

  /// Displacement of the background since the previous frame, image pixels.
  /// Empty when the background has too little texture, or its motion is not
  /// consistent enough, to trust.
  std::optional<core::Point> camera_shift_px_;
};

/// @brief Frame-to-frame motion measurement for one camera: KLT optical flow of
///        the target, and the background shift caused by the camera moving.
///
/// This is the turret's counterpart of kalman-cpp's `FlowTracker` and
/// `GlobalMotionEstimator` (apps/video_track), and follows the same algorithms:
/// pyramidal Lucas-Kanade with a forward-backward check and median residual
/// rejection for the target, and RANSAC over background corners for the camera.
/// It differs where a live, continuously slewing camera needs it to: background
/// corners are re-detected as they leave the frame, and the target is tracked in
/// a crop around it, with a window no larger than the target and a search seeded
/// from where it is expected to have gone. A drone is a few dozen pixels across:
/// with the stock full-frame pyramid the coarse levels see mostly background and
/// the features follow that instead.
///
/// Feed it every frame, in order. All coordinates are in the pixels of the
/// images passed to `track()`.
///
/// @note Runs on the pipeline thread, not the tracking loop: it allocates and
///       takes a few milliseconds per frame.
/// @warning Not thread-safe. One instance per camera.
class MotionEstimator {
public:
  MotionEstimator();
  ~MotionEstimator();

  MotionEstimator(const MotionEstimator &) = delete;
  MotionEstimator &operator=(const MotionEstimator &) = delete;

  /// @brief True when the build includes OpenCV and this class can measure
  ///        anything. When false every `track()` returns an empty result.
  [[nodiscard]] static bool available() noexcept;

  /// @brief Measure motion from the previous frame to this one.
  /// @param frame      The new frame. Copied; need not outlive the call.
  /// @param search_roi Where the filter expects the target. Target features that
  ///                   stray far outside it are discarded, and it is masked out
  ///                   of the background estimate. Empty when there is no track.
  /// @param expected_target_shift Rough guess of how far the target moved since
  ///                   the previous frame, e.g. the displacement of the
  ///                   detector's box, or the filter's predicted velocity times
  ///                   the frame interval. Only seeds the search: the result is
  ///                   measured from the image, so the guess's own jitter does
  ///                   not carry through. Without it a small target crossing a
  ///                   textured background cannot be told from that background.
  /// @return The measured motion. Empty on the first frame.
  MotionResult track(const ImageView &frame, const std::optional<PixelRect> &search_roi,
                     const std::optional<core::Point> &expected_target_shift = std::nullopt);

  /// @brief Pick fresh target features inside a detection box.
  /// @param box The box, in the most recent frame passed to `track()`.
  ///
  /// Call when the filter has accepted that frame's detection, before the next
  /// `track()`. Replaces the current features.
  void refresh_target(const PixelRect &box);

  /// @brief Stop tracking the target; the background estimate carries on.
  void drop_target() noexcept;

  /// @brief True while target features are being tracked.
  [[nodiscard]] bool has_target() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace patronus::tracking
