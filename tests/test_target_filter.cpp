// Standalone verification of TargetFilter: replays synthetic targets through the
// real filter and asserts the ego-motion compensation, gating and timestamping
// behave. Built out-of-tree so it can be run without DeepStream or a camera.
#include "patronus/tracking/target_filter.hpp"

#include <cmath>
#include <cstdio>
#include <optional>

using patronus::config::TrackingConfig;
using patronus::core::Detection;
using patronus::tracking::TargetFilter;

namespace {

int g_failures = 0;

void check(bool ok, const char *what) {
  std::printf("  %-58s %s\n", what, ok ? "PASS" : "FAIL");
  if (!ok)
    ++g_failures;
}

Detection make_det(float center_u, float center_v, double t_s, float w = 40.0F, float h = 40.0F) {
  Detection d;
  d.left_ = center_u - (w * 0.5F);
  d.top_ = center_v - (h * 0.5F);
  d.width_ = w;
  d.height_ = h;
  d.class_id_ = 0;
  d.confidence_ = 0.9F;
  d.timestamp_s_ = t_s;
  return d;
}

constexpr float k_focal = 1000.0F; // detection px per radian
constexpr float k_omega = 0.3F;    // rad/s, so 300 px/s apparent drift

// A world-static target seen by a slewing gimbal drifts across the frame at
// -focal*omega px/s. Confirm the filter reports ~zero *target* velocity instead
// of the full apparent rate, while still reporting the true image displacement.
void test_ego_compensation_removes_gimbal_motion() {
  std::printf("\n[ego compensation] gimbal slews, target holds still in the world\n");
  const float home_u = 960.0F;
  const float home_v = 544.0F;

  TrackingConfig cfg;
  cfg.filter_meas_sigma_px_ = 3.0F;
  cfg.filter_qc_ = 2000.0F;
  cfg.filter_gate_ = 9.21F;
  cfg.filter_pipeline_latency_ms_ = 0;

  TargetFilter filter(cfg, 1920U, 1088U, k_focal, k_focal);
  check(filter.ego_compensation_active(), "ego compensation reports active");

  double t = 0.0;
  for (int i = 0; i < 120; ++i) {
    const float pan = k_omega * static_cast<float>(t);
    filter.push_gimbal_sample(t, pan, 0.0F);
    // Panning right drags a static target left across the image.
    const float center_u = home_u - (k_focal * pan);
    (void)filter.step(0.01F, make_det(center_u, home_v, t), t);
    t += 0.01;
  }
  filter.push_gimbal_sample(t, k_omega * static_cast<float>(t), 0.0F);
  const auto est = filter.step(0.01F, std::nullopt, t);

  check(est.has_value(), "filter produced an estimate");

  // The aim error must report the true image displacement: the target has really
  // moved -focal*pan in the frame and the gimbal has to null that out.
  const float true_error = -(k_focal * (k_omega * static_cast<float>(t)));
  std::printf("    error = (%.1f, %.1f) px, velocity = (%.2f, %.2f) px/s\n", est->error_u_px(),
              est->error_v_px(), est->vu(), est->vv());
  std::printf("    apparent image rate %.0f px/s, true image displacement %.1f px\n",
              static_cast<double>(k_focal * k_omega), static_cast<double>(true_error));

  check(std::abs(est->vu()) < 25.0F, "target velocity ~0 despite 300 px/s apparent slew");
  check(std::abs(est->error_u_px() - true_error) < 40.0F,
        "aim error matches the true image displacement");
}

// Same detections, no focal length configured: the filter must mistake the
// gimbal's slewing for a target manoeuvre. This is the exact failure the
// compensation exists to prevent, so demonstrate it rather than assert it.
void test_without_focal_mistakes_gimbal_for_target() {
  std::printf("\n[no compensation] same slew, focal unset\n");
  const float home_u = 960.0F;
  const float home_v = 544.0F;

  TrackingConfig cfg;
  cfg.filter_meas_sigma_px_ = 3.0F;
  cfg.filter_qc_ = 2000.0F;
  cfg.filter_gate_ = 9.21F;
  cfg.filter_pipeline_latency_ms_ = 0;

  TargetFilter filter(cfg, 1920U, 1088U, 0.0F, 0.0F);
  check(!filter.ego_compensation_active(), "ego compensation reports inactive");

  double t = 0.0;
  for (int i = 0; i < 120; ++i) {
    const float pan = k_omega * static_cast<float>(t);
    filter.push_gimbal_sample(t, pan, 0.0F);
    const float center_u = home_u - (k_focal * pan);
    (void)filter.step(0.01F, make_det(center_u, home_v, t), t);
    t += 0.01;
  }
  filter.push_gimbal_sample(t, k_omega * static_cast<float>(t), 0.0F);
  const auto est = filter.step(0.01F, std::nullopt, t);

  check(est.has_value(), "filter produced an estimate");

  const float apparent = -(k_focal * k_omega);
  std::printf("    velocity = (%.1f, %.1f) px/s; gimbal motion shown as %.1f px/s\n", est->vu(),
              est->vv(), static_cast<double>(apparent));
  check(std::abs(est->vu() - apparent) < 40.0F,
        "ego motion is misattributed to the target (the bug compensation fixes)");
}

// A genuinely moving target with a static gimbal must be tracked, and its
// velocity estimated well enough for the lead term to be useful.
void test_tracks_moving_target() {
  std::printf("\n[moving target] static gimbal, target crosses at 300 px/s\n");
  TrackingConfig cfg;
  cfg.filter_meas_sigma_px_ = 2.0F;
  cfg.filter_qc_ = 50.0F;
  cfg.filter_pipeline_latency_ms_ = 0;

  TargetFilter filter(cfg, 1920U, 1088U, k_focal, k_focal);

  const float v_true = 300.0F;
  double t = 0.0;
  for (int i = 0; i < 200; ++i) {
    filter.push_gimbal_sample(t, 0.0F, 0.0F);
    const float center_u = 400.0F + (v_true * static_cast<float>(t));
    (void)filter.step(0.01F, make_det(center_u, 544.0F, t), t);
    t += 0.01;
  }
  const auto est = filter.step(0.01F, std::nullopt, t);

  check(est.has_value(), "filter produced an estimate");
  std::printf("    velocity = (%.1f, %.1f) px/s, truth = (%.1f, 0.0)\n", est->vu(), est->vv(),
              static_cast<double>(v_true));
  check(std::abs(est->vu() - v_true) < 25.0F, "velocity within 25 px/s of truth");

  const auto lead = est->predicted_error(0.15F);
  check(std::abs((lead.cx_ - est->error_u_px()) - (v_true * 0.15F)) < 6.0F,
        "0.15 s lead projection matches v * lead");
}

// The gate must reject a wild outlier and the filter must coast instead of
// jumping. This is the safety property: a false positive must not slew the gun.
void test_gate_rejects_outlier() {
  std::printf("\n[gating] outlier must be rejected, filter coasts\n");
  TrackingConfig cfg;
  cfg.filter_meas_sigma_px_ = 2.0F;
  cfg.filter_qc_ = 50.0F;
  cfg.filter_gate_ = 9.21F;
  cfg.filter_pipeline_latency_ms_ = 0;

  TargetFilter filter(cfg, 1920U, 1088U, k_focal, k_focal);

  double t = 0.0;
  for (int i = 0; i < 80; ++i) {
    filter.push_gimbal_sample(t, 0.0F, 0.0F);
    (void)filter.step(0.01F, make_det(900.0F, 544.0F, t), t);
    t += 0.01;
  }
  const auto before = filter.step(0.01F, std::nullopt, t);
  const int coast_before = before->coast_ticks();

  // A detection a full frame away, as a false positive on some other object.
  filter.push_gimbal_sample(t, 0.0F, 0.0F);
  const auto after = filter.step(0.01F, make_det(150.0F, 544.0F, t), t);

  check(after.has_value(), "filter still produces an estimate");
  check(!after->measured(), "outlier was not accepted as a measurement");
  check(after->coast_ticks() > coast_before, "coast counter advanced");
  check(std::abs(after->error_u_px() - before->error_u_px()) < 40.0F,
        "estimate barely moved on the outlier");

  std::printf("    error %.1f -> %.1f px, coast %d -> %d\n",
              static_cast<double>(before->error_u_px()), static_cast<double>(after->error_u_px()),
              coast_before, after->coast_ticks());
}

// Coasting between detections must extrapolate along the estimated velocity.
void test_coasts_between_detections() {
  std::printf("\n[coasting] no detections, prediction extrapolates\n");
  TrackingConfig cfg;
  cfg.filter_meas_sigma_px_ = 2.0F;
  cfg.filter_qc_ = 50.0F;
  cfg.filter_pipeline_latency_ms_ = 0;

  TargetFilter filter(cfg, 1920U, 1088U, k_focal, k_focal);

  const float v_true = 200.0F;
  double t = 0.0;
  for (int i = 0; i < 100; ++i) {
    filter.push_gimbal_sample(t, 0.0F, 0.0F);
    (void)filter.step(0.01F, make_det(400.0F + (v_true * static_cast<float>(t)), 544.0F, t), t);
    t += 0.01;
  }
  const auto at_cutoff = filter.step(0.01F, std::nullopt, t);
  const float u0 = at_cutoff->error_u_px();

  constexpr int k_steps = 20;
  std::optional<patronus::tracking::Estimate> last;
  for (int i = 0; i < k_steps; ++i) {
    filter.push_gimbal_sample(t, 0.0F, 0.0F);
    last = filter.step(0.01F, std::nullopt, t);
    t += 0.01;
  }

  const float drift = last->error_u_px() - u0;
  std::printf("    drifted %.1f px in 0.2 s at %.0f px/s (expected ~%.1f)\n",
              static_cast<double>(drift), static_cast<double>(last->vu()),
              static_cast<double>(v_true * 0.2));

  check(std::abs(drift - (v_true * 0.2F)) < 12.0F, "coast drift matches velocity * time");
  check(last->coast_ticks() >= k_steps, "coast counter accumulated");
  check(last->sigma_u_px() > at_cutoff->sigma_u_px(), "uncertainty grew while coasting");
}

// Timestamps must drive the update, not tick counting. Deliver detections at
// irregular intervals and with a realistic declared latency, and confirm the
// velocity is unbiased -- the loop is event-driven, so assuming a fixed dt
// biases the estimate that the lead term depends on.
void test_uses_detection_timestamps_not_tick_count() {
  std::printf("\n[timestamps] irregular 25-40 ms frames, 20 ms declared latency\n");
  TrackingConfig cfg;
  cfg.filter_meas_sigma_px_ = 2.0F;
  cfg.filter_qc_ = 50.0F;
  cfg.filter_pipeline_latency_ms_ = 20;

  TargetFilter filter(cfg, 1920U, 1088U, k_focal, k_focal);

  const float v_true = 250.0F;
  // Camera at ~33 ms, tracking loop at 10 ms: detection times are not multiples
  // of the tick period, so tick counting cannot reconstruct this dt.
  const double frame_periods[] = {0.033, 0.040, 0.033, 0.025, 0.033, 0.040, 0.033};
  double t = 0.0;
  double last_detection_t = -1.0;
  size_t idx = 0;
  int ticks = 0;
  int accepted = 0;

  while (t < 3.0) {
    filter.push_gimbal_sample(t, 0.0F, 0.0F);
    std::optional<Detection> det;
    if (last_detection_t < 0.0 || t >= last_detection_t + frame_periods[idx % 7]) {
      det = make_det(400.0F + (v_true * static_cast<float>(t)), 544.0F, t);
      last_detection_t = t;
      ++idx;
    }
    const auto est = filter.step(0.01F, det, t);
    if (est.has_value() && est->measured())
      ++accepted;
    t += 0.01;
    ++ticks;
  }

  const auto est = filter.step(0.01F, std::nullopt, t);
  std::printf("    %d ticks, %d detections (%zu offered), velocity %.1f px/s (truth %.0f)\n", ticks,
              accepted, idx, est->vu(), static_cast<double>(v_true));
  check(std::abs(est->vu() - v_true) < 20.0F,
        "velocity unbiased despite irregular timing + latency");
  check(est->measured() || est->coast_ticks() <= 1, "estimate is fresh, not stuck coasting");
}

// A detection whose timestamp is ahead of the tick time would mean a clock-domain
// mix-up. It must be clamped, not trusted, or the filter runs updates backwards.
void test_future_timestamp_is_clamped() {
  std::printf("\n[robustness] timestamp ahead of now must be clamped\n");
  TrackingConfig cfg;
  cfg.filter_meas_sigma_px_ = 2.0F;
  cfg.filter_qc_ = 50.0F;
  cfg.filter_pipeline_latency_ms_ = 0;

  TargetFilter filter(cfg, 1920U, 1088U, k_focal, k_focal);

  double t = 0.0;
  for (int i = 0; i < 60; ++i) {
    filter.push_gimbal_sample(t, 0.0F, 0.0F);
    (void)filter.step(0.01F, make_det(900.0F, 544.0F, t), t);
    t += 0.01;
  }

  // Stamp the next detection half a second in the future.
  filter.push_gimbal_sample(t, 0.0F, 0.0F);
  const auto est = filter.step(0.01F, make_det(905.0F, 544.0F, t + 0.5), t);
  check(est.has_value(), "filter survives a future-dated measurement");
  check(std::isfinite(est->error_u_px()) && std::isfinite(est->vu()), "state stayed finite");
  std::printf("    error %.1f px, velocity %.1f px/s\n", static_cast<double>(est->error_u_px()),
              static_cast<double>(est->vu()));
}

} // namespace

int main() {
  std::printf("TargetFilter verification\n==========================\n");

  test_ego_compensation_removes_gimbal_motion();
  test_without_focal_mistakes_gimbal_for_target();
  test_tracks_moving_target();
  test_gate_rejects_outlier();
  test_coasts_between_detections();
  test_uses_detection_timestamps_not_tick_count();
  test_future_timestamp_is_clamped();

  std::printf("\n==========================\n");
  std::printf("%s (%d failure%s)\n", g_failures == 0 ? "ALL PASS" : "FAILURES", g_failures,
              g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}