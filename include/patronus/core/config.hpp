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
  ///
  /// This is NOT necessarily the muxed stream size and NOT the network input
  /// size. The network runs at 640x640, and DeepStream-Yolo's
  /// `NvDsInferParseYolo` emits boxes in whichever space the parser was
  /// written against; nothing in the pad probe rescales them. Getting this
  /// wrong is not a rounding error -- a detector emitting 640x640 while the
  /// code assumes 1920x1088 produces a standing ~-640 px error and sends the
  /// gimbal hard over.
  ///
  /// Verify with `--calibrate-focal`, which measures the effective focal length
  /// and compares it against the value this width implies. If the measured fx
  /// is off by roughly the width ratio (1920/640 = 3), these are wrong.
  uint32_t detection_width_{1920};  ///< Detection-space width in pixels.
  uint32_t detection_height_{1088}; ///< Detection-space height in pixels.

  /// Focal length in pixels, in *muxed* coordinates.
  ///
  /// Detections are produced from the 1920x1088 muxed stream, not from the
  /// native sensor resolution, so the intrinsics that matter are the post-mux
  /// ones. They are needed to convert between angular and pixel rates: a
  /// gimbal slewing at omega rad/s moves a target across the frame at
  /// fx*omega pixels/s, and the target filter removes that self-induced
  /// apparent motion before estimating the target's own velocity.
  ///
  /// Derive from the lens and sensor geometry, or measure directly with the
  /// `calibrate_focal` tool, which slews the gimbal through known angles and
  /// solves fx = delta_px / delta_rad. Leaving these at zero disables
  /// ego-motion compensation (degraded but safe: the filter then treats
  /// gimbal motion as target manoeuvre).
  float focal_x_px_{0.0f}; ///< Horizontal focal length, pixels.
  float focal_y_px_{0.0f}; ///< Vertical focal length, pixels.

  /// Draw the target filter's prediction on the video overlay.
  ///
  /// The overlay is drawn by the video thread from predictions published by the
  /// tracking threads, so it keeps working (and coasting) while detections are
  /// rejected or absent — which is exactly when it is worth watching. Purely
  /// cosmetic: turning it off does not change what the motors are commanded.
  bool draw_predictions_{true};

  /// Look-ahead horizon for the drawn velocity arrow, seconds.
  ///
  /// This is a *visualisation* horizon, chosen to be visible at the frame rates
  /// involved. It is deliberately independent of the per-gimbal `lead_gain`,
  /// which is a control-law parameter: tuning one must not silently redraw the
  /// other.
  float prediction_lead_s_{0.15f};
};

/// @brief CAN bus configuration for a single gimbal.
struct GimbalConfig {
  uint16_t pan_node_id_{16};        ///< CAN node ID for the pan motor.
  uint16_t tilt_node_id_{18};       ///< CAN node ID for the tilt motor.
  float max_velocity_rad_s_{6.28f}; ///< Maximum velocity command.
  float tilt_max_rad_{0.611f};      ///< Maximum tilt output angle (±rad).
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

  /// Age of a detection at the moment it reaches the tracking loop, in ms.
  ///
  /// The ego-motion correction has to apply the gimbal angle from when the
  /// frame was *captured*, not from when we processed it. Most of that delay
  /// the filter can recover on its own by looking up a timestamped angle
  /// history, but the sensor-side portion (exposure, ISP, TensorRT inference)
  /// is not observable from the tracking thread, so it is declared here.
  ///
  /// Leaving this at 0 biases the correction in proportion to slew rate: at
  /// 6.28 rad/s and f = 1000 px, an uncorrected 20 ms is ~125 px of error.
  /// Measure it once with a fixed target and correct it.
  int filter_pipeline_latency_ms_{20};

  // --- Control law -------------------------------------------------------
  float kp_{0.006f};      ///< Proportional gain (per pixel of error).
  float kd_{0.0f};        ///< Derivative gain on filtered velocity.
  float lead_gain_{0.0f}; ///< Lead term on filtered velocity (s).
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

/// @brief Top-level system configuration.
struct SystemConfig {
  std::string mode_{"both"};                 ///< Pipeline mode: "both", "rgb", or "mono".
  bool tracking_{false};                     ///< Enable tracking loop.
  PipelineConfig rgb_;                       ///< RGB pipeline config.
  PipelineConfig mono_;                      ///< Mono pipeline config.
  SensorConfig sensor_;                      ///< Sensor module config.
  CanBusConfig can_bus_;                     ///< Shared CAN bus settings.
  std::vector<GimbalConfig> gimbals_;        ///< Per-gimbal CAN + limits.
  std::vector<TrackingConfig> tracking_cfg_; ///< Per-gimbal tracking params.
};

/// @brief Load system configuration from a GLib-format .ini file.
/// @param path  Absolute or CWD-relative path to the .ini file.
/// @return Populated SystemConfig with defaults for any missing keys.
/// @note Uses GKeyFile under the hood. Logs warnings via g_warning for
///       parse issues but always returns a valid (defaulted) config.
[[nodiscard]] SystemConfig load_config(const std::string &path);

} // namespace patronus::config
