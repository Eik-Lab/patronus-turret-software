#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace patronus::config {

/// @brief Per-pipeline configuration loaded from system.ini.
struct PipelineConfig {
  std::string camera_serial_;      ///< Basler camera serial number.
  std::string camera_caps_;        ///< GStreamer caps string for the source.
  std::string infer_config_;       ///< Path to DeepStream inference config.
  std::string udp_host_;           ///< Destination host for H.264 UDP stream.
  uint16_t udp_port_{5000};        ///< Destination port for H.264 UDP stream.
  uint32_t encoder_bitrate_{3000}; ///< x264enc bitrate (kbps).
  uint32_t muxer_width_{1920};     ///< Stream muxer output width.
  uint32_t muxer_height_{1088};    ///< Stream muxer output height.
  bool enabled_{true};             ///< Whether this pipeline is active.

  /// Pixel space in which the detector reports bounding boxes.
  uint32_t detection_width_{1920};  ///< Detection-space width in pixels.
  uint32_t detection_height_{1088}; ///< Detection-space height in pixels.

  /// Focal length in pixels, in *muxed* coordinates.
  float focal_x_px_{0.0f}; ///< Horizontal focal length, pixels.
  float focal_y_px_{0.0f}; ///< Vertical focal length, pixels.

  /// Draw the target filter's prediction on the video overlay.
  bool draw_predictions_{true};

  /// Look-ahead horizon for the drawn velocity arrow, seconds.
  float prediction_lead_s_{0.15f};

  /// Time from the end of exposure until the frame has arrived on the host, ms:
  /// the sensor readout and USB transfer. pylonsrc stamps a frame's PTS only
  /// once it has arrived, so this (plus half the exposure) is subtracted to find
  /// when the image was actually taken. 0 or less computes it from the caps as
  /// frame bytes over the USB 3 link rate.
  float readout_ms_{0.0f};

  /// Sensor binning, the same factor horizontally and vertically (1 = none).
  /// The field of view is unchanged, so the focal lengths in detection pixels
  /// are too; `camera_caps` must give the binned size.
  uint32_t binning_{1};

  /// Most frames the Pylon stream grabber may hold, 0 for its default (16).
  /// When the camera delivers as fast as the pipeline consumes, any backlog
  /// in those buffers never drains and every frame arrives that much late; a
  /// small pool makes the camera drop frames instead.
  uint32_t grab_buffers_{0};
};

/// @brief Velocity-PID gains for one motor axis (VELOCITY_PID mode).
struct MotorPidConfig {
  float kp_{2.0f};           ///< Proportional gain.
  float ki_{0.1f};           ///< Integral gain.
  float kd_{0.0f};           ///< Derivative gain.
  float integral_max_{0.5f}; ///< Integral term clamp.
};

/// @brief CAN bus configuration for a single gimbal.
struct GimbalConfig {
  uint16_t pan_node_id_{16};        ///< CAN node ID for the pan motor.
  uint16_t tilt_node_id_{18};       ///< CAN node ID for the tilt motor.
  float max_velocity_rad_s_{6.28f}; ///< Maximum velocity command.
  float tilt_max_rad_{0.611f};      ///< Maximum tilt output angle (±rad).
  bool calibrate_on_boot_{false};   ///< Run axis barrier calibration in boot_sequence() at start.
  MotorPidConfig pan_pid_;          ///< Velocity-PID gains for the pan motor.
  MotorPidConfig tilt_pid_;         ///< Velocity-PID gains for the tilt motor.
  float pan_velocity_scale_{1.0f};  ///< Multiplier on every pan velocity command.
  float tilt_velocity_scale_{1.0f}; ///< Multiplier on every tilt velocity command.
  float pan_max_velocity_rad_s_{6.28f};  ///< Pan command clamp after scaling (<= max_velocity).
  float tilt_max_velocity_rad_s_{6.28f}; ///< Tilt command clamp after scaling (<= max_velocity).

  /// 1-based [can_N] index of the gimbal whose camera this one follows, or 0
  /// when it tracks from a camera of its own. A follower (e.g. a pointer on a
  /// separate gimbal) runs no image loop: it is driven to the target bearing
  /// that the leader's camera reports.
  uint16_t follow_{0};
  float follow_gain_{5.0f};     ///< Follower P-gain, rad/s per rad of bearing error.
  float pan_offset_rad_{0.0f};  ///< Follower pan home minus leader pan home, when aligned.
  float tilt_offset_rad_{0.0f}; ///< Follower tilt home minus leader tilt home, when aligned.

  /// Camera a leader tracks from: "rgb", "mono", or empty to round-robin the
  /// active pipelines across leaders. Falls back to the other pipeline when the
  /// named one is not running.
  std::string camera_;
};

/// @brief Tracking loop configuration for a single gimbal.
struct TrackingConfig {
  bool home_return_enabled_{true};       ///< Return to home when no target detected.
  float home_return_gain_{2.0f};         ///< P-gain for home return (rad/s per rad of error).
  float home_tolerance_rad_{0.05f};      ///< Deadband for "at home" (~3°).
  int home_return_delay_ms_{500};        ///< Wait (ms) before initiating return.
  float home_return_max_velocity_{1.5f}; ///< Max return velocity (rad/s motor shaft).

  // --- Target filter (kalman-cpp constant-acceleration tracker) ------------
  bool filter_enabled_{true};        ///< Run the filter instead of raw detections.
  float filter_meas_sigma_px_{8.0f}; ///< Detection position sigma (1σ, pixels).
  float filter_gate_{9.21f};         ///< Innovation gate (χ², 2 dof at 99%).
  int filter_tick_ms_{10};           ///< Tracking-loop tick: motor command period (ms).

  /// White-jerk spectral density, px²/s⁵: the one tuning knob that matters.
  /// The target's acceleration can change by `sqrt(q * T)` px/s² within `T`
  /// seconds. Raise to follow sharper manoeuvres, lower for a smoother estimate.
  float filter_q_jerk_{2.0e5f};

  /// Extra distrust of low-confidence detections: the position variance is
  /// scaled by `1 + filter_conf_noise_scale * (1 - confidence)`.
  float filter_conf_noise_scale_{10.0f};

  /// Drop the track after this long without an accepted detection (ms), so the
  /// next detection re-seeds the filter instead of being gated against an
  /// estimate that has long since drifted away. 0 disables.
  int filter_max_coast_ms_{1000};

  /// Age of a frame when it reaches the tracking loop, in ms. Only a fallback
  /// for frames without a usable PTS: the pipeline normally measures the age of
  /// every frame, and the filter looks the gimbal pose up at that capture time.
  int filter_pipeline_latency_ms_{20};
  /// Longest capture-to-control extrapolation of the target's motion, ms. Caps
  /// how far ahead the aim point is predicted, including while coasting between
  /// detections. The gimbal's own rotation is measured, so it is never capped.
  int filter_max_lead_ms_{250};
  /// How long the gimbal keeps turning at its current rate after a velocity
  /// command, ms: the drive's velocity loop, the inertia and the CAN round trip.
  /// The aim error is predicted this far ahead along the gimbal's measured rate,
  /// so a command never asks for rotation the gimbal is already going to make.
  /// 0 aims at the pose at the control instant.
  int filter_actuation_latency_ms_{0};

  // --- Control law -------------------------------------------------------
  float kp_{0.006f};      ///< Proportional gain (per pixel of error).
  float kd_{0.0f};        ///< Derivative gain on filtered velocity.
  float lead_gain_{0.0f}; ///< Lead term on filtered velocity (s).
};

/// @brief Tracking config of one gimbal for each camera it can track from:
/// [tracking_N] overlaid with [tracking_N.rgb] or [tracking_N.mono].
struct GimbalTrackingConfig {
  TrackingConfig rgb_;
  TrackingConfig mono_;
};

/// @brief Shared CAN bus settings (apply to all gimbals on the bus).
struct CanBusConfig {
  uint8_t datarate_{1};       ///< 1, 2, 5, or 8 Mbps.
  uint16_t pds_node_id_{100}; ///< PDS module CAN ID (0 to skip).
};

/// @brief Sensor module configuration for serial communication.
struct SensorConfig {
  bool enabled_{false};              ///< Enable sensor data reading.
  std::string port_{"/dev/ttyACM0"}; ///< Serial port device path.
  std::string baud_rate_{"B115200"}; ///< Baud rate string (e.g., "B9600", "B115200").
};

/// @brief Command listener configuration for the UDP control channel.
struct CommandConfig {
  bool enabled_{false};      ///< Enable the UDP command listener.
  std::string command_host_; ///< ip address to bind to.
  uint16_t command_port_;    ///< UDP port to listen on for commands.
};

/// @brief Tracking telemetry: per-frame and per-tick CSV logs for tuning.
struct TelemetryConfig {
  bool enabled_{false};          ///< Write telemetry at all.
  std::string dir_{"telemetry"}; ///< Parent directory; each run gets a subdirectory.
};

/// @brief Top-level system configuration.
struct SystemConfig {
  std::string mode_{"both"};          ///< Pipeline mode: "both", "rgb", or "mono".
  bool tracking_{false};              ///< Enable tracking loop.
  std::string log_level_{"warning"};  ///< Min level: debug, info, warning, error, critical.
  PipelineConfig rgb_;                ///< RGB pipeline config.
  PipelineConfig mono_;               ///< Mono pipeline config.
  SensorConfig sensor_;               ///< Sensor module config.
  CommandConfig command_;             ///< UDP command listener config.
  TelemetryConfig telemetry_;         ///< Tracking telemetry logs.
  CanBusConfig can_bus_;              ///< Shared CAN bus settings.
  std::vector<GimbalConfig> gimbals_; ///< Per-gimbal CAN + limits.
  std::vector<GimbalTrackingConfig> tracking_cfg_; ///< Per-gimbal tracking params.
};

/// @brief Load system configuration from a GLib-format .ini file.
/// @param path  Absolute or CWD-relative path to the .ini file.
/// @return Populated SystemConfig with defaults for any missing keys.
/// @note Uses GKeyFile under the hood. Reports parse issues through
///       LOG(level, ...) (see log.h) but always returns a valid config.
[[nodiscard]] SystemConfig load_config(const std::string &path);

} // namespace patronus::config
