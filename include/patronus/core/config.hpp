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
