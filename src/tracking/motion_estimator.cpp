#include "patronus/tracking/motion_estimator.hpp"

#if defined(PATRONUS_HAVE_OPENCV)
  #include <opencv2/calib3d.hpp>
  #include <opencv2/core.hpp>
  #include <opencv2/imgproc.hpp>
  #include <opencv2/video/tracking.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace patronus::tracking {

#if defined(PATRONUS_HAVE_OPENCV)

namespace {

  // --- Target KLT (kalman-cpp FlowTracker::Config defaults) ---
  constexpr int k_target_max_corners = 25;
  constexpr double k_target_quality = 0.02;
  constexpr double k_target_min_distance = 5.0;
  constexpr int k_target_block_size = 7;
  constexpr int k_win_size = 21;
  constexpr int k_max_level = 3;
  /// The target is tracked with a smaller window and a shallower pyramid than
  /// the background: both are bounded by the target's size, not the frame's.
  constexpr int k_target_min_win = 7;
  constexpr int k_target_max_level = 1;
  constexpr float k_fb_threshold_px = 1.5F;
  constexpr float k_residual_threshold_px = 5.0F;
  constexpr int k_target_min_features = 4;
  constexpr double k_min_flow_sigma_px = 0.3;
  /// Context kept around the target features when cropping for LK: the reach of
  /// the top pyramid level's window, plus room for the target to move.
  constexpr int k_target_crop_margin_px = ((k_win_size << k_target_max_level) / 2) + 48;

  // --- Camera motion (kalman-cpp GlobalMotionEstimator::Config defaults) ---
  constexpr int k_background_max_features = 200;
  constexpr double k_background_quality = 0.01;
  constexpr double k_background_min_distance = 10.0;
  constexpr int k_background_block_size = 5;
  constexpr double k_downsample = 0.5;
  constexpr double k_ransac_threshold_px = 3.0;
  constexpr double k_min_inlier_ratio = 0.4;
  constexpr int k_min_inliers = 8;
  /// Re-detect background corners once this few are left. A slewing camera
  /// sweeps its corners out of frame; without this the estimate dies for good.
  constexpr size_t k_background_replenish_below = k_background_max_features / 2;
  /// Frames to wait before re-detecting again when the last attempt still came
  /// up short. Corner detection is the expensive step, and against open sky it
  /// finds nothing however often it is run.
  constexpr int k_background_retry_frames = 15;
  /// A target feature moving within this of the background's shift is taken to
  /// be sitting on the background, when enough others are not.
  constexpr float k_background_match_px = 1.5F;

  cv::Rect to_rect(const PixelRect &r) {
    return {static_cast<int>(std::floor(r.left_)), static_cast<int>(std::floor(r.top_)),
            static_cast<int>(std::ceil(r.width_)), static_cast<int>(std::ceil(r.height_))};
  }

  float median(std::vector<float> &values) {
    const auto mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), mid, values.end());
    return *mid;
  }

} // namespace

struct MotionEstimator::Impl {
  // Full-resolution grey frames. Swapped each frame, so neither is reallocated.
  cv::Mat prev_gray;
  cv::Mat cur_gray;
  // Downsampled copies for the background estimate.
  cv::Mat prev_small;
  cv::Mat cur_small;
  bool have_prev{false};

  std::vector<cv::Point2f> target_pts;     // in prev_gray, full-resolution pixels
  std::vector<cv::Point2f> background_pts; // in prev_small pixels
  int background_retry_in{0};              // frames until re-detection is allowed
  int target_win{k_win_size};              // LK window for the target, odd

  void load(const ImageView &frame) {
    if (frame.channels_ == 4) {
      const cv::Mat rgba(frame.height_, frame.width_, CV_8UC4, const_cast<uint8_t *>(frame.data_),
                         static_cast<size_t>(frame.stride_));
      cv::cvtColor(rgba, cur_gray, cv::COLOR_RGBA2GRAY);
    } else {
      const cv::Mat grey(frame.height_, frame.width_, CV_8UC1, const_cast<uint8_t *>(frame.data_),
                         static_cast<size_t>(frame.stride_));
      grey.copyTo(cur_gray);
    }
    cv::resize(cur_gray, cur_small, cv::Size(), k_downsample, k_downsample, cv::INTER_AREA);
  }

  void detect_background(const cv::Rect &excluded) {
    cv::Mat mask(cur_small.size(), CV_8UC1, cv::Scalar(255));
    const cv::Rect exclude = excluded & cv::Rect(0, 0, cur_small.cols, cur_small.rows);
    if (!exclude.empty())
      mask(exclude).setTo(0);
    background_pts.clear();
    cv::goodFeaturesToTrack(cur_small, background_pts, k_background_max_features,
                            k_background_quality, k_background_min_distance, mask,
                            k_background_block_size, true, 0.04);
  }

  /// Robust background translation prev_small -> cur_small, in full-res pixels.
  std::optional<core::Point> track_background(const cv::Rect &excluded) {
    std::optional<core::Point> out;
    std::vector<cv::Point2f> tracked;

    if (have_prev && !background_pts.empty()) {
      std::vector<cv::Point2f> cur_pts;
      std::vector<uchar> status;
      std::vector<float> err;
      cv::calcOpticalFlowPyrLK(prev_small, cur_small, background_pts, cur_pts, status, err,
                               cv::Size(k_win_size, k_win_size), k_max_level);

      const cv::Rect2f bounds(0.0F, 0.0F, static_cast<float>(cur_small.cols),
                              static_cast<float>(cur_small.rows));
      std::vector<cv::Point2f> prev_in;
      std::vector<cv::Point2f> cur_in;
      for (size_t i = 0; i < background_pts.size(); ++i) {
        if (status[i] == 0 || !bounds.contains(cur_pts[i]))
          continue;
        tracked.push_back(cur_pts[i]);
        // Corners on the target would drag the estimate towards its motion.
        if (excluded.contains(background_pts[i]))
          continue;
        prev_in.push_back(background_pts[i]);
        cur_in.push_back(cur_pts[i]);
      }

      if (prev_in.size() >= static_cast<size_t>(k_min_inliers)) {
        cv::Mat inliers;
        cv::estimateAffinePartial2D(prev_in, cur_in, inliers, cv::RANSAC, k_ransac_threshold_px);

        int count = 0;
        cv::Point2f sum(0.0F, 0.0F);
        for (int i = 0; i < inliers.rows; ++i) {
          if (inliers.at<uchar>(i) == 0)
            continue;
          sum += cur_in[static_cast<size_t>(i)] - prev_in[static_cast<size_t>(i)];
          ++count;
        }
        const double ratio = static_cast<double>(count) / static_cast<double>(prev_in.size());
        if (count >= k_min_inliers && ratio >= k_min_inlier_ratio) {
          const float scale = 1.0F / (static_cast<float>(count) * static_cast<float>(k_downsample));
          out = core::Point{sum.x * scale, sum.y * scale};
        }
      }
    }

    background_pts = std::move(tracked);
    if (background_retry_in > 0)
      --background_retry_in;
    if (background_pts.size() < k_background_replenish_below && background_retry_in == 0) {
      detect_background(excluded);
      if (background_pts.size() < k_background_replenish_below)
        background_retry_in = k_background_retry_frames;
    }
    return out;
  }

  /// KLT flow of the target features prev_gray -> cur_gray.
  void track_target(const cv::Rect &search_roi, const std::optional<core::Point> &camera_shift,
                    const std::optional<core::Point> &expected_shift, MotionResult &result) {
    // Both frames are cropped to the same window around the features, so LK
    // builds its pyramids over a few hundred pixels instead of the whole frame.
    cv::Rect crop = cv::boundingRect(target_pts);
    crop -= cv::Point(k_target_crop_margin_px, k_target_crop_margin_px);
    crop += cv::Size(2 * k_target_crop_margin_px, 2 * k_target_crop_margin_px);
    crop &= cv::Rect(0, 0, cur_gray.cols, cur_gray.rows);
    if (crop.width <= target_win || crop.height <= target_win) {
      target_pts.clear();
      return;
    }
    const cv::Point2f origin(static_cast<float>(crop.x), static_cast<float>(crop.y));

    std::vector<cv::Point2f> prev_pts;
    prev_pts.reserve(target_pts.size());
    for (const cv::Point2f &p : target_pts)
      prev_pts.push_back(p - origin);

    const cv::Mat prev_crop = prev_gray(crop);
    const cv::Mat cur_crop = cur_gray(crop);
    const cv::Size window(target_win, target_win);
    const cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

    // Start each feature where the target is expected to have gone. The window
    // is too small to find a fast target from a standing start, and a window
    // large enough to would be looking mostly at the background.
    const cv::Point2f guess = expected_shift.has_value()
                                ? cv::Point2f(expected_shift->cx_, expected_shift->cy_)
                                : cv::Point2f(0.0F, 0.0F);
    std::vector<cv::Point2f> next_pts;
    next_pts.reserve(prev_pts.size());
    for (const cv::Point2f &p : prev_pts)
      next_pts.push_back(p + guess);

    std::vector<cv::Point2f> back_pts = prev_pts;
    std::vector<uchar> fwd_status;
    std::vector<uchar> back_status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev_crop, cur_crop, prev_pts, next_pts, fwd_status, err, window,
                             k_target_max_level, criteria, cv::OPTFLOW_USE_INITIAL_FLOW);
    // Track back again: a feature that does not return to where it started was
    // not really followed.
    cv::calcOpticalFlowPyrLK(cur_crop, prev_crop, next_pts, back_pts, back_status, err, window,
                             k_target_max_level, criteria, cv::OPTFLOW_USE_INITIAL_FLOW);

    // Features may wander a little outside the search region and still be valid.
    const cv::Rect roi_margin(search_roi.x - (search_roi.width / 2),
                              search_roi.y - (search_roi.height / 2), search_roi.width * 2,
                              search_roi.height * 2);

    std::vector<cv::Point2f> kept_pts;
    std::vector<cv::Point2f> kept_disp;
    for (size_t i = 0; i < prev_pts.size(); ++i) {
      if (fwd_status[i] == 0 || back_status[i] == 0)
        continue;
      const cv::Point2f fb = prev_pts[i] - back_pts[i];
      if (std::hypot(fb.x, fb.y) > k_fb_threshold_px)
        continue;
      const cv::Point2f next_full = next_pts[i] + origin;
      if (!roi_margin.contains(
            cv::Point(static_cast<int>(next_full.x), static_cast<int>(next_full.y))))
        continue;
      kept_pts.push_back(next_full);
      kept_disp.push_back(next_pts[i] - prev_pts[i]);
    }

    // A detection box takes in some of the scene behind a small target, and
    // corners there are often stronger than the target's own. They move with
    // the camera. Where enough features move differently from the background,
    // those are the target; where none do, the target is simply holding still
    // against it and every feature is as good as another.
    if (camera_shift.has_value()) {
      std::vector<cv::Point2f> pts;
      std::vector<cv::Point2f> disp;
      for (size_t i = 0; i < kept_disp.size(); ++i) {
        if (std::hypot(kept_disp[i].x - camera_shift->cx_, kept_disp[i].y - camera_shift->cy_) >
            k_background_match_px) {
          pts.push_back(kept_pts[i]);
          disp.push_back(kept_disp[i]);
        }
      }
      if (pts.size() >= static_cast<size_t>(k_target_min_features)) {
        kept_pts = std::move(pts);
        kept_disp = std::move(disp);
      }
    }

    if (kept_disp.size() >= static_cast<size_t>(k_target_min_features)) {
      // Reject displacements far from the median: features that slid onto the
      // background move with it, not with the target.
      std::vector<float> dx;
      std::vector<float> dy;
      for (const cv::Point2f &d : kept_disp) {
        dx.push_back(d.x);
        dy.push_back(d.y);
      }
      const float med_dx = median(dx);
      const float med_dy = median(dy);

      std::vector<cv::Point2f> pts;
      std::vector<cv::Point2f> disp;
      for (size_t i = 0; i < kept_disp.size(); ++i) {
        if (std::hypot(kept_disp[i].x - med_dx, kept_disp[i].y - med_dy) <
            k_residual_threshold_px) {
          pts.push_back(kept_pts[i]);
          disp.push_back(kept_disp[i]);
        }
      }
      kept_pts = std::move(pts);
      kept_disp = std::move(disp);
    }

    if (kept_disp.size() < static_cast<size_t>(k_target_min_features)) {
      // Lost. Wait for the next accepted detection to pick new features.
      target_pts.clear();
      return;
    }

    const double n = static_cast<double>(kept_disp.size());
    double mean_x = 0.0;
    double mean_y = 0.0;
    for (const cv::Point2f &d : kept_disp) {
      mean_x += static_cast<double>(d.x);
      mean_y += static_cast<double>(d.y);
    }
    mean_x /= n;
    mean_y /= n;

    double var_x = 0.0;
    double var_y = 0.0;
    for (const cv::Point2f &d : kept_disp) {
      var_x += std::pow(static_cast<double>(d.x) - mean_x, 2);
      var_y += std::pow(static_cast<double>(d.y) - mean_y, 2);
    }
    var_x /= (n - 1.0);
    var_y /= (n - 1.0);

    result.target_flow_px_ = core::Point{static_cast<float>(mean_x), static_cast<float>(mean_y)};
    result.target_flow_sigma_px_ =
      static_cast<float>(std::max(k_min_flow_sigma_px, std::sqrt((var_x + var_y) / (2.0 * n))));
    result.target_features_ = static_cast<int>(kept_disp.size());
    target_pts = std::move(kept_pts);
  }
};

MotionEstimator::MotionEstimator() : impl_(std::make_unique<Impl>()) {
}

MotionEstimator::~MotionEstimator() = default;

bool MotionEstimator::available() noexcept {
  return true;
}

MotionResult MotionEstimator::track(const ImageView &frame,
                                    const std::optional<PixelRect> &search_roi,
                                    const std::optional<core::Point> &expected_target_shift) {
  MotionResult result;
  Impl &m = *impl_;
  if (frame.data_ == nullptr || frame.width_ <= 0 || frame.height_ <= 0 ||
      (frame.channels_ != 1 && frame.channels_ != 4))
    return result;

  m.load(frame);

  // A resolution change invalidates everything measured against the old frame.
  if (m.have_prev && m.prev_gray.size() != m.cur_gray.size()) {
    m.have_prev = false;
    m.target_pts.clear();
    m.background_pts.clear();
  }

  const cv::Rect roi = search_roi.has_value() ? to_rect(*search_roi) : cv::Rect();
  const cv::Rect excluded(
    static_cast<int>(roi.x * k_downsample), static_cast<int>(roi.y * k_downsample),
    static_cast<int>(roi.width * k_downsample), static_cast<int>(roi.height * k_downsample));
  result.camera_shift_px_ = m.track_background(excluded);

  if (m.have_prev && !m.target_pts.empty()) {
    if (search_roi.has_value())
      m.track_target(roi, result.camera_shift_px_, expected_target_shift, result);
    else
      m.target_pts.clear();
  }

  cv::swap(m.prev_gray, m.cur_gray);
  cv::swap(m.prev_small, m.cur_small);
  m.have_prev = true;
  return result;
}

void MotionEstimator::refresh_target(const PixelRect &box) {
  Impl &m = *impl_;
  m.target_pts.clear();
  if (!m.have_prev)
    return;

  // Corners are taken strictly inside the box, with no margin: around a small
  // target the margin is background, and its corners are often the stronger.
  cv::Rect roi = to_rect(box);
  roi &= cv::Rect(0, 0, m.prev_gray.cols, m.prev_gray.rows);
  if (roi.width < k_target_block_size || roi.height < k_target_block_size)
    return;

  // Window about half the target across, so most of what it sees is target.
  const int half = std::min(roi.width, roi.height) / 2;
  m.target_win = std::clamp(half | 1, k_target_min_win, k_win_size);

  cv::goodFeaturesToTrack(m.prev_gray(roi), m.target_pts, k_target_max_corners, k_target_quality,
                          k_target_min_distance, cv::noArray(), k_target_block_size, true, 0.04);
  const cv::Point2f origin(static_cast<float>(roi.x), static_cast<float>(roi.y));
  for (cv::Point2f &p : m.target_pts)
    p += origin;
}

void MotionEstimator::drop_target() noexcept {
  impl_->target_pts.clear();
}

bool MotionEstimator::has_target() const noexcept {
  return !impl_->target_pts.empty();
}

#else // !PATRONUS_HAVE_OPENCV

struct MotionEstimator::Impl {};

MotionEstimator::MotionEstimator() = default;

MotionEstimator::~MotionEstimator() = default;

bool MotionEstimator::available() noexcept {
  return false;
}

MotionResult MotionEstimator::track(const ImageView & /*frame*/,
                                    const std::optional<PixelRect> & /*search_roi*/,
                                    const std::optional<core::Point> & /*expected_target_shift*/) {
  return {};
}

void MotionEstimator::refresh_target(const PixelRect & /*box*/) {
}

void MotionEstimator::drop_target() noexcept {
}

bool MotionEstimator::has_target() const noexcept {
  return false;
}

#endif

} // namespace patronus::tracking
