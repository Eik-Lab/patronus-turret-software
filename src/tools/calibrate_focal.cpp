#include "patronus/tools/calibrate_focal.hpp"

#include "patronus/tracking/aim_control.hpp"

#include <glib.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <thread>
#include <vector>

namespace patronus::tools {
namespace {

  // Pan sweep, in radians, centred on wherever the target was centred. Small
  // enough to stay well clear of the gimbal's mechanical limits and to keep the
  // target inside the frame, wide enough that a 1920 px frame yields a
  // displacement far larger than detection noise.
  constexpr std::array<float, 5> k_sweep_rad{-0.12F, -0.06F, 0.0F, 0.06F, 0.12F};

  constexpr auto k_settle = std::chrono::milliseconds(500);
  constexpr auto k_sample_period = std::chrono::milliseconds(40);
  constexpr int k_samples_per_step = 9; // odd, so the median is a real sample
  constexpr auto k_center_timeout = std::chrono::seconds(8);
  constexpr float k_slew_gain = 2.0F; // rad/s per rad of position error
  constexpr float k_slew_tolerance = 0.002F;
  constexpr float k_slew_max_velocity = 0.6F;
  constexpr auto k_pop_timeout = std::chrono::milliseconds(120);

  /// Drive the gimbal to a pan angle, leaving tilt alone.
  /// @return true once inside `k_slew_tolerance`.
  bool slew_to_pan(comm::CandleMotor &motor, size_t gimbal, float target_pan_rad,
                   std::atomic<bool> &running) {
    while (running) {
      const auto [pan, tilt] = motor.get_position(gimbal);
      (void)tilt;
      const float error = target_pan_rad - pan;
      if (std::abs(error) < k_slew_tolerance) {
        motor.set_velocity(gimbal, 0.0F, 0.0F);
        return true;
      }
      const float vel = std::clamp(k_slew_gain * error, -k_slew_max_velocity, k_slew_max_velocity);
      motor.set_velocity(gimbal, vel, 0.0F);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    motor.set_velocity(gimbal, 0.0F, 0.0F);
    return false;
  }

  /// Bring the target to the centre of frame with a plain proportional loop on the
  /// raw detection. The IMM is deliberately not used here: it is the thing being
  /// calibrated, and its velocity estimate depends on the focal length being
  /// solved for.
  bool centre_target(comm::CandleMotor &motor, size_t gimbal, uint32_t detection_w,
                     uint32_t detection_h,
                     patronus::core::LatestValue<patronus::core::Detection> &detections,
                     std::atomic<bool> &running) {
    const auto deadline = std::chrono::steady_clock::now() + k_center_timeout;
    bool seen_target = false;

    while (running && std::chrono::steady_clock::now() < deadline) {
      const auto det = detections.try_pop(k_pop_timeout);
      if (!det.has_value())
        continue;
      seen_target = true;

      const float center_u = det->left_ + (det->width_ * 0.5F);
      const float center_v = det->top_ + (det->height_ * 0.5F);
      const auto error = tracking::detection_error({center_u, center_v}, detection_w, detection_h);

      // P gain on the pixel error, no velocity term: exactly the behaviour
      // calibrate_focal is validating, minus the filter.
      const float vel_u = std::clamp(-0.002F * error.cx_, -0.5F, 0.5F);
      const float vel_v = std::clamp(-0.002F * error.cy_, -0.5F, 0.5F);
      motor.set_velocity(gimbal, vel_u, vel_v);

      if (std::abs(error.cx_) < 4.0F && std::abs(error.cy_) < 4.0F) {
        motor.set_velocity(gimbal, 0.0F, 0.0F);
        return true;
      }
    }

    motor.set_velocity(gimbal, 0.0F, 0.0F);
    return seen_target;
  }

} // namespace

int run_focal_calibration(comm::CandleMotor &motor, size_t gimbal,
                          const patronus::config::GimbalConfig &gimbal_cfg,
                          patronus::core::LatestValue<patronus::core::Detection> &detections,
                          uint32_t detection_w, uint32_t detection_h, float focal_x_px,
                          float focal_y_px, std::atomic<bool> &running) {
  (void)gimbal_cfg;

  g_print("\n--- Focal calibration, gimbal %zu ---\n", gimbal);
  g_print("Keep a fixed, high-contrast target in view and keep the turret clear.\n");

  if (detection_w == 0U || detection_h == 0U) {
    g_critical("Gimbal %zu: detection_width/height must be non-zero", gimbal);
    return 1;
  }

  if (!centre_target(motor, gimbal, detection_w, detection_h, detections, running)) {
    g_critical("Gimbal %zu: no drone detection — nothing to calibrate against", gimbal);
    return 1;
  }
  if (!running) {
    g_print("Gimbal %zu: aborted while centring\n", gimbal);
    return 1;
  }
  g_print("Target centred. Beginning sweep.\n");

  // (pan angle, target x) pairs.
  std::vector<std::pair<float, float>> samples;
  samples.reserve(k_sweep_rad.size() * k_samples_per_step);

  const auto [pan_home, _tilt_home] = motor.get_position(gimbal);

  for (const float offset : k_sweep_rad) {
    if (!running) {
      g_print("Gimbal %zu: aborted during sweep\n", gimbal);
      motor.set_velocity(gimbal, 0.0F, 0.0F);
      return 1;
    }

    if (!slew_to_pan(motor, gimbal, pan_home + offset, running)) {
      motor.set_velocity(gimbal, 0.0F, 0.0F);
      g_critical("Gimbal %zu: slew did not complete", gimbal);
      return 1;
    }
    std::this_thread::sleep_for(k_settle);

    // Median over several frames: the detection lags the command by the pipeline
    // latency, so individual samples are noisy, but the bias is common-mode and
    // the median discards the outliers from a badly detected frame.
    std::vector<float> xs;
    xs.reserve(k_samples_per_step);
    for (int i = 0; i < k_samples_per_step && running; ++i) {
      const auto det = detections.try_pop(k_pop_timeout);
      if (det.has_value())
        xs.push_back(det->left_ + (det->width_ * 0.5F));
      else
        std::this_thread::sleep_for(k_sample_period);
    }

    const auto [pan_now, _tilt_now] = motor.get_position(gimbal);
    if (xs.size() < 3U) {
      g_warning("Gimbal %zu: only %zu/%d samples at %+.3f rad — skipping", gimbal, xs.size(),
                k_samples_per_step, static_cast<double>(offset));
      continue;
    }

    std::sort(xs.begin(), xs.end());
    const float median_x = xs[xs.size() / 2U];
    samples.emplace_back(pan_now, median_x);
    g_print("  pan %+.4f rad -> x %7.1f px\n", static_cast<double>(pan_now),
            static_cast<double>(median_x));
  }

  motor.set_velocity(gimbal, 0.0F, 0.0F);

  if (samples.size() < 2U) {
    g_critical("Gimbal %zu: not enough usable samples (%zu)", gimbal, samples.size());
    return 1;
  }

  // Least squares x = fx * theta + b. A two-point difference would be shorter to
  // write but throws away the intermediate samples and is far more sensitive to a
  // single bad detection.
  double sum_t = 0.0, sum_x = 0.0, sum_tt = 0.0, sum_tx = 0.0;
  const auto n = static_cast<double>(samples.size());
  for (const auto &[theta, x] : samples) {
    sum_t += theta;
    sum_x += x;
    sum_tt += static_cast<double>(theta) * theta;
    sum_tx += static_cast<double>(theta) * x;
  }
  const double denom = (n * sum_tt) - (sum_t * sum_t);
  if (std::abs(denom) < 1.0e-9) {
    g_critical("Gimbal %zu: degenerate sweep — all samples at the same angle", gimbal);
    return 1;
  }

  const double fx = ((n * sum_tx) - (sum_t * sum_x)) / denom;
  if (!(fx > 1.0)) {
    g_critical("Gimbal %zu: implausible fx %.1f px/rad — is the target fixed?", gimbal, fx);
    return 1;
  }

  // Square pixels mean fy = fx. If the detection space is not the same aspect
  // ratio as the sensor, this is where it would show.
  const double fy = fx;

  g_print("\nGimbal %zu focal length: fx = %.1f px/rad, fy = %.1f px/rad\n", gimbal, fx, fy);
  g_print("Horizontal field of view: %.1f deg\n",
          2.0 * std::atan((detection_w * 0.5) / fx) * 180.0 / M_PI);
  g_print("\nSet these in the [rgb] and/or [mono] section of the config:\n");
  g_print("  focal_x_px=%.0f\n", fx);
  g_print("  focal_y_px=%.0f\n", fy);
  g_print("\nCurrently configured: focal_x_px=%.1f focal_y_px=%.1f\n",
          static_cast<double>(focal_x_px), static_cast<double>(focal_y_px));
  g_print("Ego-motion compensation is %s until both are non-zero.\n",
          (focal_x_px > 0.0F && focal_y_px > 0.0F) ? "already active" : "OFF");

  return 0;
}

} // namespace patronus::tools