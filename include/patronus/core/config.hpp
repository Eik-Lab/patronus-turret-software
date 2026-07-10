#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace patronus::config {

/// @brief Per-pipeline configuration loaded from system.ini.
struct PipelineConfig {
  std::string camera_serial;
  std::string camera_caps;
  std::string infer_config;
  std::string udp_host;
  uint16_t udp_port{5000};
  uint32_t encoder_bitrate{3000};
  uint32_t muxer_width{1920};
  uint32_t muxer_height{1088};
  bool enabled{true};
};

/// @brief CAN bus configuration for a single gimbal.
struct GimbalConfig {
  uint16_t pan_node_id{16};
  uint16_t tilt_node_id{18};
  float    max_velocity_rad_s{6.28f}; ///< Maximum velocity command.
  float    tilt_max_rad{0.611f};      ///< Maximum tilt output angle (±rad).
};

/// @brief Tracking loop configuration for a single gimbal.
struct TrackingConfig {
  bool   home_return_enabled{true};        ///< Return to home when no target detected.
  float  home_return_gain{2.0f};           ///< P-gain for home return (rad/s per rad of error).
  float  home_tolerance_rad{0.05f};        ///< Deadband for "at home" (~3°).
  int    home_return_delay_ms{500};        ///< Wait (ms) before initiating return.
  float  home_return_max_velocity{1.5f};   ///< Max return velocity (rad/s motor shaft).
};

/// @brief Shared CAN bus settings (apply to all gimbals on the bus).
struct CanBusConfig {
  uint8_t  datarate{1};       ///< 1, 2, 5, or 8 Mbps.
  uint16_t pds_node_id{100};  ///< PDS module CAN ID (0 to skip).
};

/// @brief Top-level system configuration.
struct SystemConfig {
  std::string mode{"both"};
  bool tracking{false};
  PipelineConfig rgb;
  PipelineConfig mono;
  CanBusConfig can_bus;
  std::vector<GimbalConfig> gimbals;       ///< Per-gimbal CAN + limits.
  std::vector<TrackingConfig> tracking_cfg; ///< Per-gimbal tracking params.
};

/// @brief Load system configuration from a GLib-format .ini file.
/// @param path  Absolute or CWD-relative path to the .ini file.
/// @return Populated SystemConfig with defaults for any missing keys.
/// @note Uses GKeyFile under the hood. Logs warnings via g_warning for
///       parse issues but always returns a valid (defaulted) config.
[[nodiscard]] SystemConfig load_config(const std::string &path);

}  // namespace patronus::config
