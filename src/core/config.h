#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace patronus::config {

// system.ini documents every key.
struct PipelineConfig {
  std::string camera_serial_;
  std::string camera_caps_;
  std::string infer_config_;
  std::string udp_host_;
  uint16_t udp_port_{5000};
  uint32_t encoder_bitrate_{3000};
  uint32_t muxer_width_{1920};
  uint32_t muxer_height_{1088};
  bool enabled_{true};

  /// The space boxes are reported in; not the network input size.
  uint32_t detection_width_{1920};
  uint32_t detection_height_{1088};

  /// Detection px per radian; 0 disables camera-motion compensation.
  float focal_x_px_{0.0f};
  float focal_y_px_{0.0f};

  bool draw_predictions_{true};
  float prediction_lead_s_{0.15f};

  /// Readout + USB transfer after exposure; <= 0 estimates it from the caps.
  float readout_ms_{0.0f};

  uint32_t binning_{1};      ///< camera_caps must give the binned size.
  uint32_t grab_buffers_{0}; ///< 0 = Pylon default (16).
};

struct MotorPidConfig {
  float kp_{2.0f};
  float ki_{0.1f};
  float kd_{0.0f};
  float integral_max_{0.5f};
};

struct GimbalConfig {
  uint16_t pan_node_id_{16};
  uint16_t tilt_node_id_{18};
  float max_velocity_rad_s_{6.28f};
  float tilt_max_rad_{0.611f}; ///< Only logged; boot calibration sets the real limits.
  bool calibrate_on_boot_{false};
  MotorPidConfig pan_pid_;
  MotorPidConfig tilt_pid_;
  /// Command = clamp(command * scale, +/- axis max).
  float pan_velocity_scale_{1.0f};
  float tilt_velocity_scale_{1.0f};
  float pan_max_velocity_rad_s_{6.28f};
  float tilt_max_velocity_rad_s_{6.28f};

  /// 1-based leader index, or 0. A follower aims at its leader's bearing.
  uint16_t follow_{0};
  float follow_gain_{5.0f};
  float pan_offset_rad_{0.0f};
  float tilt_offset_rad_{0.0f};

  /// "rgb", "mono", or empty to round-robin.
  std::string camera_;
};

struct TrackingConfig {
  bool home_return_enabled_{true};
  float home_return_gain_{2.0f};
  float home_tolerance_rad_{0.05f};
  int home_return_delay_ms_{500};
  float home_return_max_velocity_{1.5f};

  bool filter_enabled_{true};
  float filter_meas_sigma_px_{8.0f};
  float filter_gate_{9.21f};
  int filter_tick_ms_{10};
  float filter_q_jerk_{2.0e5f};
  float filter_conf_noise_scale_{10.0f};
  int filter_max_coast_ms_{1000}; ///< 0 disables.

  /// Fallback for frames without a PTS.
  int filter_pipeline_latency_ms_{20};
  int filter_max_lead_ms_{250};
  int filter_actuation_latency_ms_{0};

  // See SensorControlGains.
  float kp_{0.006f};
  float kd_{0.0f};
  float lead_gain_{0.0f};
};

/// Gains are in pixels, so they differ per camera.
struct GimbalTrackingConfig {
  TrackingConfig rgb_;
  TrackingConfig mono_;
};

struct CanBusConfig {
  uint8_t datarate_{1};
  uint16_t pds_node_id_{100};
};

struct SensorConfig {
  bool enabled_{false};
  std::string port_{"/dev/ttyACM0"};
  std::string baud_rate_{"B115200"};
};

struct CommandConfig {
  bool enabled_{false};
  std::string command_host_;
  uint16_t command_port_;
};

struct TelemetryConfig {
  bool enabled_{false};
  std::string dir_{"telemetry"};
};

struct SystemConfig {
  std::string mode_{"both"};
  bool tracking_{false};
  std::string log_level_{"warning"};
  PipelineConfig rgb_;
  PipelineConfig mono_;
  SensorConfig sensor_;
  CommandConfig command_;
  TelemetryConfig telemetry_;
  CanBusConfig can_bus_;
  std::vector<GimbalConfig> gimbals_;
  std::vector<GimbalTrackingConfig> tracking_cfg_;
};

/// Missing or invalid values fall back to defaults.
[[nodiscard]] SystemConfig load_config(const std::string &path);

} // namespace patronus::config
