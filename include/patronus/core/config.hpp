#pragma once

#include <cstdint>
#include <string>

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

/// @brief Top-level system configuration.
struct SystemConfig {
  std::string mode{"both"};
  bool tracking{false};
  PipelineConfig rgb;
  PipelineConfig mono;
};

/// @brief Load system configuration from a GLib-format .ini file.
/// @param path  Absolute or CWD-relative path to the .ini file.
/// @return Populated SystemConfig with defaults for any missing keys.
/// @note Uses GKeyFile under the hood. Logs warnings via g_warning for
///       parse issues but always returns a valid (defaulted) config.
[[nodiscard]] SystemConfig load_config(const std::string &path);

}  // namespace patronus::config
