// Standalone verification of TargetFilter: replays synthetic targets through the
// real filter, one observation per camera frame, and asserts the camera-motion
// compensation, latency lead, gating and track lifetime behave.
// Built out-of-tree so it can be run without DeepStream or a camera.
#include "patronus/tracking/target_filter.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <random>

using patronus::config::TrackingConfig;
using patronus::core::Detection;
using patronus::core::FrameObservation;
using patronus::core::Point;
using patronus::tracking::Estimate;
using patronus::tracking::TargetFilter;

namespace {

int g_failures = 0;

void check(bool ok, const char *what) {
  std::printf("  %-62s %s\n", what, ok ? "PASS" : "FAIL");
  if (!ok)
    ++g_failures;
}

constexpr double k_dt = 1.0 / 30.0; // camera frame period
constexpr float k_focal = 1000.0F;  // detection px per radian
constexpr float k_omega = 0.3F;     // rad/s, so 300 px/s apparent drift
constexpr double k_t0 = 1.0;        // the steady clock is never 0 in the app

Detection make_det(float center_u, float center_v, float confidence = 0.9F) {
  Detection d{};
  const float size = 40.0F;
  d.left_ = center_u - (size * 0.5F);
  d.top_ = center_v - (size * 0.5F);
  d.width_ = size;
  d.height_ = size;
  d.class_id_ = 0;
  d.confidence_ = confidence;
  return d;
}

/// Builds the per-frame observations a camera would produce.
class Camera {
public:
  /// Next frame, `k_dt` after the previous one.
  FrameObservation frame(const std::optional<Detection> &det) {
    FrameObservation obs;
    obs.frame_id_ = ++frame_id_;
    obs.frame_dt_s_ = (frame_id_ == 1) ? 0.0 : k_dt;
    obs.timestamp_s_ = now();
    obs.capture_s_ = now();
    obs.detection_ = det;
    return obs;
  }

  [[nodiscard]] double now() const {
    return k_t0 + (static_cast<double>(frame_id_ == 0 ? 0 : frame_id_ - 1) * k_dt);
  }
  /// Time of the frame the next call to `frame()` will produce.
  [[nodiscard]] double next() const {
    return k_t0 + (static_cast<double>(frame_id_) * k_dt);
  }

private:
  uint64_t frame_id_{0};
};

TrackingConfig base_config() {
  TrackingConfig cfg;
  cfg.filter_pipeline_latency_ms_ = 0;
  return cfg;
}

/// Step the filter on the camera's next frame, with the control loop running the
/// instant the frame arrives.
std::optional<Estimate> step_frame(TargetFilter &filter, Camera &cam,
                                   const std::optional<Detection> &det) {
  const FrameObservation obs = cam.frame(det);
  return filter.step(obs, obs.timestamp_s_);
}

// A world-static target seen by a slewing gimbal drifts across the frame at
// -focal*omega px/s. The encoders must remove that, so the filter reports ~zero
// *target* velocity while still reporting the true image displacement.
void test_encoders_remove_gimbal_motion() {
  std::printf("\n[camera motion: encoders] gimbal slews, target static in the world\n");
  TargetFilter filter(base_config(), 1920U, 1088U, k_focal, k_focal);
  check(filter.ego_compensation_active(), "encoder fallback reports active");

  Camera cam;
  std::optional<Estimate> est;
  float center_u = 0.0F;
  for (int i = 0; i < 60; ++i) {
    const double t = cam.next();
    const float pan = k_omega * static_cast<float>(t - k_t0);
    filter.push_gimbal_sample(t, pan, 0.0F);
    // Panning right drags a static target left across the image.
    center_u = 1500.0F - (k_focal * pan);
    est = step_frame(filter, cam, make_det(center_u, 544.0F));
  }

  check(est.has_value(), "filter produced an estimate");
  std::printf("    velocity = (%.1f, %.1f) px/s, camera = %.1f px/s\n", est->vu(), est->vv(),
              est->camera_vu());
  check(est->camera_compensated(), "frame reports camera motion as compensated");
  check(std::abs(est->vu()) < 25.0F, "target velocity ~0 despite 300 px/s apparent slew");
  check(std::abs(est->camera_vu() + (k_focal * k_omega)) < 5.0F,
        "camera velocity is the apparent slew rate");
  check(std::abs(est->error_u_px() - (center_u - 960.0F)) < 10.0F,
        "aim error is still the true image position");
}

// No focal length: the filter must still track, in image space, and say so.
void test_uncompensated_tracks_in_image_space() {
  std::printf("\n[camera motion: none] same slew, nothing to measure it with\n");
  TargetFilter filter(base_config(), 1920U, 1088U, 0.0F, 0.0F);

  const float shift = -k_focal * k_omega * static_cast<float>(k_dt);
  Camera cam;
  std::optional<Estimate> est;
  float center_u = 1500.0F;
  for (int i = 0; i < 60; ++i) {
    est = step_frame(filter, cam, make_det(center_u, 544.0F));
    center_u += shift;
  }

  check(est.has_value(), "filter produced an estimate");
  std::printf("    velocity = (%.1f, %.1f) px/s (apparent rate %.0f)\n", est->vu(), est->vv(),
              static_cast<double>(-k_focal * k_omega));
  check(!est->camera_compensated(), "frame reports camera motion as not compensated");
  check(std::abs(est->vu() + (k_focal * k_omega)) < 40.0F,
        "velocity is the apparent image rate, as expected without compensation");
}

// Box centres jitter by several pixels a frame. The velocity must still come out
// unbiased, and much steadier than differencing consecutive boxes would give.
void test_velocity_from_jittery_boxes() {
  std::printf("\n[jitter] 300 px/s target, box centres jittering 6 px RMS\n");
  TargetFilter filter(base_config(), 1920U, 1088U, 0.0F, 0.0F);
  std::mt19937 rng(7);
  std::normal_distribution<float> box_jitter(0.0F, 6.0F);

  const float v_true = 300.0F;
  const float step = v_true * static_cast<float>(k_dt);
  Camera cam;
  double sum_sq = 0.0;
  double sum_err = 0.0;
  int n = 0;
  float u = 300.0F;
  for (int i = 0; i < 150; ++i) {
    const auto est =
      step_frame(filter, cam, make_det(u + box_jitter(rng), 544.0F + box_jitter(rng)));
    if (i >= 30 && est.has_value()) {
      const double eu = static_cast<double>(est->vu() - v_true);
      const double ev = static_cast<double>(est->vv());
      sum_sq += (eu * eu) + (ev * ev);
      sum_err += eu;
      ++n;
    }
    u += step;
  }
  const double rms = std::sqrt(sum_sq / static_cast<double>(n));
  const double bias = sum_err / static_cast<double>(n);
  // Differencing two boxes 1/30 s apart: sqrt(2) * 6 px * 30 /s per axis.
  const double differencing_rms = std::sqrt(2.0) * 6.0 * 30.0 * std::sqrt(2.0);
  std::printf("    velocity error: bias %.1f px/s, RMS %.1f px/s (box differencing ~%.0f)\n", bias,
              rms, differencing_rms);
  check(std::abs(bias) < 15.0, "velocity is unbiased");
  check(rms < differencing_rms * 0.25, "filter at least quarters the differencing noise");
}

// The gate must reject a wild outlier and the filter must coast instead of
// jumping. This is the safety property: a false positive must not slew the gun.
void test_gate_rejects_outlier() {
  std::printf("\n[gating] outlier must be rejected, filter coasts\n");
  TargetFilter filter(base_config(), 1920U, 1088U, 0.0F, 0.0F);

  Camera cam;
  std::optional<Estimate> before;
  for (int i = 0; i < 40; ++i)
    before = step_frame(filter, cam, make_det(900.0F, 544.0F));

  // A detection most of a frame away, as a false positive on some other object.
  const auto after = step_frame(filter, cam, make_det(150.0F, 544.0F));

  check(after.has_value(), "filter still produces an estimate");
  check(!after->measured(), "outlier was not accepted as a measurement");
  check(after->coast_frames() == before->coast_frames() + 1, "coast counter advanced");
  check(after->detection_frame_id() == before->detection_frame_id(),
        "accepted-detection frame id did not move");
  check(std::abs(after->error_u_px() - before->error_u_px()) < 10.0F,
        "estimate barely moved on the outlier");
}

// With no detections the estimate must extrapolate along its velocity, grow less
// certain, and finally be dropped so the next detection starts a new track.
void test_coasts_then_drops_and_reseeds() {
  std::printf("\n[coasting] detections stop: extrapolate, then drop, then re-seed\n");
  TrackingConfig cfg = base_config();
  cfg.filter_max_coast_ms_ = 500;
  TargetFilter filter(cfg, 1920U, 1088U, 0.0F, 0.0F);

  const float v_true = 200.0F;
  Camera cam;
  std::optional<Estimate> at_cutoff;
  float u = 400.0F;
  for (int i = 0; i < 60; ++i) {
    at_cutoff = step_frame(filter, cam, make_det(u, 544.0F));
    u += v_true * static_cast<float>(k_dt);
  }

  std::optional<Estimate> coasting;
  for (int i = 0; i < 6; ++i)
    coasting = step_frame(filter, cam, std::nullopt);

  check(coasting.has_value(), "still tracking 0.2 s into the gap");
  const float drift = coasting->error_u_px() - at_cutoff->error_u_px();
  std::printf("    drifted %.1f px in 0.2 s (expected ~%.1f)\n", static_cast<double>(drift),
              static_cast<double>(v_true * 0.2F));
  check(std::abs(drift - (v_true * 0.2F)) < 8.0F, "coast drift matches velocity * time");
  check(coasting->coast_frames() == 6, "coast counter counts frames");
  check(coasting->sigma_u_px() > at_cutoff->sigma_u_px(), "uncertainty grew while coasting");

  std::optional<Estimate> late = coasting;
  for (int i = 0; i < 12; ++i)
    late = step_frame(filter, cam, std::nullopt);
  check(!late.has_value(), "track dropped after coasting past the limit");
  check(!filter.initialized(), "filter reports no track");

  // Somewhere the old estimate would have gated out.
  const auto reseeded = step_frame(filter, cam, make_det(1500.0F, 300.0F));
  check(reseeded.has_value() && reseeded->measured(), "next detection re-seeds immediately");
  check(reseeded.has_value() && std::abs(reseeded->error_u_px() - (1500.0F - 960.0F)) < 1.0F,
        "new track starts at the new detection");
}

// Frames arrive when inference finishes, which jitters; the camera's own
// cadence does not. The step must follow the capture clock, or that jitter is
// divided straight into the velocity.
void test_uses_capture_clock_not_arrival_time() {
  std::printf("\n[timing] arrival jitters +/-8 ms, capture cadence steady\n");
  TargetFilter filter(base_config(), 1920U, 1088U, 0.0F, 0.0F);
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> arrival_jitter(-0.008, 0.008);

  const float v_true = 250.0F;
  Camera cam;
  std::optional<Estimate> est;
  float u = 300.0F;
  for (int i = 0; i < 90; ++i) {
    FrameObservation obs = cam.frame(make_det(u, 544.0F));
    obs.timestamp_s_ += 0.05 + arrival_jitter(rng);
    est = filter.step(obs, obs.timestamp_s_);
    u += v_true * static_cast<float>(k_dt);
  }
  std::printf("    velocity %.1f px/s (truth %.0f)\n", est->vu(), static_cast<double>(v_true));
  check(std::abs(est->vu() - v_true) < 10.0F, "velocity unbiased despite arrival jitter");
}

// A repeated or out-of-order frame must not advance or corrupt the filter.
void test_ignores_stale_observation() {
  std::printf("\n[robustness] repeated frame is ignored\n");
  TargetFilter filter(base_config(), 1920U, 1088U, 0.0F, 0.0F);
  Camera cam;
  FrameObservation obs;
  std::optional<Estimate> est;
  for (int i = 0; i < 20; ++i) {
    obs = cam.frame(make_det(900.0F, 544.0F));
    est = filter.step(obs, obs.timestamp_s_);
  }
  const auto again = filter.step(obs, obs.timestamp_s_);
  check(again.has_value(), "filter still produces an estimate");
  check(again.has_value() && std::abs(again->error_u_px() - est->error_u_px()) < 1.0e-3F &&
          std::isfinite(again->vu()),
        "state unchanged and finite");
}

// The control law acts on the estimate extrapolated across the pipeline latency:
// a target crossing at 200 px/s seen 50 ms late is 10 px further on by now.
void test_lead_covers_pipeline_latency() {
  std::printf("\n[lead] 200 px/s target, 50 ms pipeline latency\n");
  TrackingConfig cfg = base_config();
  cfg.filter_pipeline_latency_ms_ = 50;
  TargetFilter filter(cfg, 1920U, 1088U, 0.0F, 0.0F);

  const float v_true = 200.0F;
  Camera cam;
  std::optional<Estimate> est;
  float u = 400.0F;
  for (int i = 0; i < 90; ++i) {
    est = step_frame(filter, cam, make_det(u, 544.0F));
    u += v_true * static_cast<float>(k_dt);
  }

  check(est.has_value(), "filter produced an estimate");
  const float lead = est->lead_error().cx_ - est->error_u_px();
  std::printf("    horizon %.3f s, lead %.1f px (expected %.1f)\n",
              static_cast<double>(est->lead_horizon_s()), static_cast<double>(lead),
              static_cast<double>(v_true * 0.05F));
  check(std::abs(est->lead_horizon_s() - 0.05F) < 1.0e-4F, "horizon is the configured latency");
  check(std::abs(lead - (v_true * 0.05F)) < 2.0F, "lead is velocity * latency");
  check(std::abs(est->lead_velocity().cx_ - v_true) < 15.0F, "lead velocity is the target's");
}

// Between a frame's capture and the control instant the gimbal keeps slewing.
// The lead error must place the target using the gimbal's newest pose, not the
// pose at capture, or the motors chase where the camera used to point.
void test_lead_uses_current_gimbal_pose() {
  std::printf("\n[lead] gimbal slews, target static, observation handled 30 ms late\n");
  TargetFilter filter(base_config(), 1920U, 1088U, k_focal, k_focal);

  // Under one frame period, so the encoder history stays in time order.
  const double wait_s = 0.03;
  Camera cam;
  std::optional<Estimate> est;
  float center_u = 0.0F;
  for (int i = 0; i < 60; ++i) {
    const double t = cam.next();
    filter.push_gimbal_sample(t, k_omega * static_cast<float>(t - k_t0), 0.0F);
    // The control loop keeps sampling the encoder while the frame is in flight.
    filter.push_gimbal_sample(t + wait_s, k_omega * static_cast<float>(t + wait_s - k_t0), 0.0F);
    center_u = 1500.0F - (k_focal * k_omega * static_cast<float>(t - k_t0));
    const FrameObservation obs = cam.frame(make_det(center_u, 544.0F));
    est = filter.step(obs, obs.timestamp_s_ + wait_s);
  }

  check(est.has_value(), "filter produced an estimate");
  const float expected_shift = -k_focal * k_omega * static_cast<float>(wait_s);
  const float shift = est->lead_error().cx_ - est->error_u_px();
  std::printf("    lead shift %.1f px (camera moved the target %.1f px)\n",
              static_cast<double>(shift), static_cast<double>(expected_shift));
  check(std::abs(shift - expected_shift) < 2.0F, "lead error follows the gimbal's newest pose");
}

// Absolute encoders wrap at +/-pi. A gimbal slewing through the seam must not
// look like a 2*pi*focal pixel jump, or every detection after it gates out.
void test_encoder_wrap_does_not_break_track() {
  std::printf("\n[camera motion: encoders] gimbal slews through the +/-pi seam\n");
  TargetFilter filter(base_config(), 1920U, 1088U, k_focal, k_focal);

  const float pi = 3.14159265F;
  const float start = pi - 0.2F;
  Camera cam;
  std::optional<Estimate> est;
  int rejected = 0;
  for (int i = 0; i < 60; ++i) {
    const double t = cam.next();
    const float travel = k_omega * static_cast<float>(t - k_t0); // continuous
    const float raw = std::remainder(start + travel, 2.0F * pi); // what the encoder reports
    filter.push_gimbal_sample(t, raw, 0.0F);
    est = step_frame(filter, cam, make_det(1500.0F - (k_focal * travel), 544.0F));
    if (est.has_value() && !est->measured())
      ++rejected;
  }

  check(est.has_value(), "filter produced an estimate");
  std::printf("    velocity = %.1f px/s, %d detections rejected\n", static_cast<double>(est->vu()),
              rejected);
  check(rejected == 0, "no detection gated out across the seam");
  check(std::abs(est->vu()) < 25.0F, "target velocity ~0 across the seam");
  check(std::abs(est->camera_vu() + (k_focal * k_omega)) < 5.0F,
        "camera velocity has no wrap spike");
}

} // namespace

int main() {
  std::printf("TargetFilter verification\n==========================\n");

  test_encoders_remove_gimbal_motion();
  test_uncompensated_tracks_in_image_space();
  test_velocity_from_jittery_boxes();
  test_gate_rejects_outlier();
  test_coasts_then_drops_and_reseeds();
  test_uses_capture_clock_not_arrival_time();
  test_ignores_stale_observation();
  test_lead_covers_pipeline_latency();
  test_lead_uses_current_gimbal_pose();
  test_encoder_wrap_does_not_break_track();

  std::printf("\n==========================\n");
  std::printf("%s (%d failure%s)\n", g_failures == 0 ? "ALL PASS" : "FAILURES", g_failures,
              g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
