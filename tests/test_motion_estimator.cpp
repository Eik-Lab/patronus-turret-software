// Standalone verification of MotionEstimator on synthetic frames: a textured
// background that shifts as a slewing camera would, with a small textured target
// moving over it independently. Also reports the per-frame cost at the muxed
// resolution, since this runs on the streaming thread.
#include "patronus/tracking/motion_estimator.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>

using patronus::tracking::ImageView;
using patronus::tracking::MotionEstimator;
using patronus::tracking::MotionResult;
using patronus::tracking::PixelRect;

namespace {

int g_failures = 0;

void check(bool ok, const char *what) {
  std::printf("  %-62s %s\n", what, ok ? "PASS" : "FAIL");
  if (!ok)
    ++g_failures;
}

constexpr int k_width = 1920;
constexpr int k_height = 1088;
constexpr int k_target_size = 36;

/// Smooth random texture: enough corners for LK, no aliasing under sub-pixel shifts.
cv::Mat make_texture(int width, int height, int seed, double blur) {
  cv::Mat noise(height, width, CV_8UC1);
  cv::RNG rng(static_cast<uint64_t>(seed));
  rng.fill(noise, cv::RNG::UNIFORM, 0, 256);
  cv::GaussianBlur(noise, noise, cv::Size(), blur);
  cv::normalize(noise, noise, 0, 255, cv::NORM_MINMAX);
  return noise;
}

struct Scene {
  cv::Mat world;  // background, larger than the frame so it can scroll
  cv::Mat target; // small patch pasted on top
  bool textured_background{true};

  /// Render the frame with the background scrolled by `camera` and the target's
  /// top-left corner at `target_pos`, both in frame pixels.
  [[nodiscard]] cv::Mat render(cv::Point2f camera, cv::Point2f target_pos) const {
    cv::Mat frame;
    if (textured_background) {
      // A background point at world (x, y) appears at (x, y) - origin + camera.
      const cv::Mat shift =
        (cv::Mat_<double>(2, 3) << 1.0, 0.0, camera.x - 200.0, 0.0, 1.0, camera.y - 200.0);
      cv::warpAffine(world, frame, shift, cv::Size(k_width, k_height), cv::INTER_LINEAR);
    } else {
      frame = cv::Mat(k_height, k_width, CV_8UC1, cv::Scalar(180)); // open sky
    }
    const cv::Mat place =
      (cv::Mat_<double>(2, 3) << 1.0, 0.0, target_pos.x, 0.0, 1.0, target_pos.y);
    cv::Mat layer;
    cv::Mat mask;
    cv::warpAffine(target, layer, place, frame.size(), cv::INTER_LINEAR);
    cv::warpAffine(cv::Mat(target.size(), CV_8UC1, cv::Scalar(255)), mask, place, frame.size(),
                   cv::INTER_NEAREST);
    layer.copyTo(frame, mask);
    return frame;
  }
};

ImageView view_of(const cv::Mat &frame) {
  ImageView view;
  view.data_ = frame.data;
  view.width_ = frame.cols;
  view.height_ = frame.rows;
  view.stride_ = static_cast<int>(frame.step);
  view.channels_ = frame.channels();
  return view;
}

PixelRect box_at(cv::Point2f target_pos) {
  PixelRect box;
  box.left_ = target_pos.x;
  box.top_ = target_pos.y;
  box.width_ = static_cast<float>(k_target_size);
  box.height_ = static_cast<float>(k_target_size);
  return box;
}

PixelRect search_around(cv::Point2f target_pos) {
  PixelRect roi;
  roi.left_ = target_pos.x - 50.0F;
  roi.top_ = target_pos.y - 50.0F;
  roi.width_ = 100.0F + static_cast<float>(k_target_size);
  roi.height_ = 100.0F + static_cast<float>(k_target_size);
  return roi;
}

struct RunStats {
  int frames{0};
  int flow_frames{0};
  int camera_frames{0};
  double flow_err_max{0.0};
  double camera_err_max{0.0};
  double mean_ms{0.0};
  double max_ms{0.0};
};

/// Slew the camera and fly the target for `frames` frames, feeding the estimator
/// the way the pipeline does: track the new frame, then (as the filter accepts
/// each detection) refresh the features from that frame's box.
RunStats run(const Scene &scene, cv::Point2f camera_step, cv::Point2f target_step, int frames,
             bool rgba, cv::Point2f hint_error = cv::Point2f(3.0F, -2.0F)) {
  MotionEstimator motion;
  RunStats stats;
  cv::Point2f camera(0.0F, 0.0F);
  cv::Point2f target(900.0F, 500.0F);
  double total_ms = 0.0;

  for (int i = 0; i < frames; ++i) {
    cv::Mat frame = scene.render(camera, target);
    if (rgba)
      cv::cvtColor(frame, frame, cv::COLOR_GRAY2RGBA);

    const auto t0 = std::chrono::steady_clock::now();
    if (i > 0)
      motion.refresh_target(box_at(target - target_step));
    // The hint the pipeline has: the detector's box moved about this far, give
    // or take a few pixels of box jitter.
    const patronus::core::Point hint{target_step.x + hint_error.x, target_step.y + hint_error.y};
    const MotionResult result = motion.track(
      view_of(frame), i > 0 ? std::optional<PixelRect>(search_around(target)) : std::nullopt, hint);
    const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

    if (i > 0) {
      ++stats.frames;
      total_ms += ms;
      stats.max_ms = std::max(stats.max_ms, ms);
      if (result.target_flow_px_.has_value()) {
        ++stats.flow_frames;
        stats.flow_err_max =
          std::max(stats.flow_err_max,
                   static_cast<double>(std::hypot(result.target_flow_px_->cx_ - target_step.x,
                                                  result.target_flow_px_->cy_ - target_step.y)));
      }
      if (result.camera_shift_px_.has_value()) {
        ++stats.camera_frames;
        stats.camera_err_max =
          std::max(stats.camera_err_max,
                   static_cast<double>(std::hypot(result.camera_shift_px_->cx_ - camera_step.x,
                                                  result.camera_shift_px_->cy_ - camera_step.y)));
      }
    }
    camera += camera_step;
    target += target_step;
  }
  stats.mean_ms = total_ms / static_cast<double>(std::max(stats.frames, 1));
  return stats;
}

void report(const RunStats &s) {
  std::printf("    target flow on %d/%d frames (max err %.2f px), camera shift on %d/%d "
              "(max err %.2f px)\n",
              s.flow_frames, s.frames, s.flow_err_max, s.camera_frames, s.frames, s.camera_err_max);
  std::printf("    cost %.1f ms/frame mean, %.1f ms max at %dx%d\n", s.mean_ms, s.max_ms, k_width,
              k_height);
}

Scene make_scene(bool textured_background) {
  Scene scene;
  scene.world = make_texture(k_width + 400, k_height + 400, 1, 3.0);
  scene.target = make_texture(k_target_size, k_target_size, 2, 1.2);
  scene.textured_background = textured_background;
  return scene;
}

// Camera slewing one way, target flying another: both motions must come out, and
// separately. The slew runs long enough to sweep the original corners out of
// frame, which is what kills an estimator that never re-detects them.
void test_separates_target_from_camera() {
  std::printf("\n[textured background] camera slews (-9, +2) px/frame, target (+12, -6)\n");
  const cv::Point2f camera_step(-9.0F, 2.0F);
  const cv::Point2f target_step(12.0F, -6.0F);
  const RunStats s = run(make_scene(true), camera_step, target_step, 40, false);
  report(s);
  check(s.camera_frames >= s.frames - 1, "camera shift measured on (almost) every frame");
  check(s.camera_err_max < 0.5, "camera shift within 0.5 px");
  check(s.flow_frames >= s.frames - 2, "target flow measured on (almost) every frame");
  check(s.flow_err_max < 1.0, "target flow within 1 px");
}

// Open sky: there is nothing to measure the camera against, and the estimator
// must say so rather than invent a shift. The target flow must still work.
void test_plain_background_reports_no_camera_shift() {
  std::printf("\n[plain background] open sky, target (+5, -3) px/frame\n");
  const RunStats s =
    run(make_scene(false), cv::Point2f(0.0F, 0.0F), cv::Point2f(5.0F, -3.0F), 20, false);
  report(s);
  check(s.camera_frames == 0, "no camera shift is claimed");
  check(s.flow_frames >= s.frames - 2, "target flow still measured");
  check(s.flow_err_max < 1.0, "target flow within 1 px");
}

// The OSD pad hands over RGBA; it must behave like the luma plane does.
void test_accepts_rgba() {
  std::printf("\n[RGBA input] same scene as RGBA\n");
  const RunStats s =
    run(make_scene(true), cv::Point2f(-9.0F, 2.0F), cv::Point2f(5.0F, -3.0F), 12, true);
  report(s);
  check(s.camera_frames >= s.frames - 1 && s.camera_err_max < 0.5, "camera shift within 0.5 px");
  check(s.flow_frames >= s.frames - 2 && s.flow_err_max < 1.0, "target flow within 1 px");
}

// Without a search region there is no track, so no target flow may be reported.
void test_no_track_no_flow() {
  std::printf("\n[no track] features are dropped when the filter has no estimate\n");
  const Scene scene = make_scene(true);
  MotionEstimator motion;
  const cv::Point2f target(900.0F, 500.0F);
  cv::Mat frame = scene.render(cv::Point2f(0.0F, 0.0F), target);
  (void)motion.track(view_of(frame), std::nullopt);
  motion.refresh_target(box_at(target));
  check(motion.has_target(), "features picked inside the detection box");
  frame = scene.render(cv::Point2f(0.0F, 0.0F), target + cv::Point2f(4.0F, 0.0F));
  const MotionResult result = motion.track(view_of(frame), std::nullopt);
  check(!result.target_flow_px_.has_value(), "no flow reported without a search region");
  check(!motion.has_target(), "features dropped");
}

} // namespace

int main() {
  std::printf("MotionEstimator verification\n============================\n");
  check(MotionEstimator::available(), "built with OpenCV");

  test_separates_target_from_camera();
  test_plain_background_reports_no_camera_shift();
  test_accepts_rgba();
  test_no_track_no_flow();

  std::printf("\n============================\n");
  std::printf("%s (%d failure%s)\n", g_failures == 0 ? "ALL PASS" : "FAILURES", g_failures,
              g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
