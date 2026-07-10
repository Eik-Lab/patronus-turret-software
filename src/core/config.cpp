#include "patronus/core/config.hpp"
#include <glib.h>
#include <cstdlib>
#include <cstring>

namespace patronus::config {

static std::string read_string(GKeyFile *kf, const gchar *group,
                                const gchar *key, const gchar *fallback)
{
  gchar *val = g_key_file_get_string(kf, group, key, nullptr);
  if (!val) return fallback;
  std::string result(val);
  g_free(val);
  return result;
}

static uint16_t read_uint16(GKeyFile *kf, const gchar *group,
                             const gchar *key, uint16_t fallback)
{
  GError *err = nullptr;
  gint64 val = g_key_file_get_int64(kf, group, key, &err);
  if (err) {
    g_clear_error(&err);
    return fallback;
  }
  return static_cast<uint16_t>(val);
}

static uint32_t read_uint32(GKeyFile *kf, const gchar *group,
                             const gchar *key, uint32_t fallback)
{
  GError *err = nullptr;
  gint64 val = g_key_file_get_int64(kf, group, key, &err);
  if (err) {
    g_clear_error(&err);
    return fallback;
  }
  return static_cast<uint32_t>(val);
}

static bool read_bool(GKeyFile *kf, const gchar *group,
                       const gchar *key, bool fallback)
{
  GError *err = nullptr;
  gboolean val = g_key_file_get_boolean(kf, group, key, &err);
  if (err) {
    g_clear_error(&err);
    return fallback;
  }
  return !!val;
}

static float read_float(GKeyFile *kf, const gchar *group,
                          const gchar *key, float fallback)
{
  GError *err = nullptr;
  double val = g_key_file_get_double(kf, group, key, &err);
  if (err) {
    g_clear_error(&err);
    return fallback;
  }
  return static_cast<float>(val);
}

static PipelineConfig read_pipeline(GKeyFile *kf, const gchar *group)
{
  PipelineConfig cfg;
  cfg.camera_serial = read_string(kf, group, "camera_serial", "");
  cfg.camera_caps = read_string(kf, group, "camera_caps",
      "video/x-raw(memory:NVMM),format=YUY2,width=1920,height=1080");
  cfg.infer_config = read_string(kf, group, "infer_config", "");
  cfg.udp_host = read_string(kf, group, "udp_host", "127.0.0.1");
  cfg.udp_port = read_uint16(kf, group, "udp_port", 5000);
  cfg.encoder_bitrate = read_uint32(kf, group, "encoder_bitrate", 3000);
  cfg.muxer_width = read_uint32(kf, group, "muxer_width", 1920);
  cfg.muxer_height = read_uint32(kf, group, "muxer_height", 1088);
  cfg.enabled = read_bool(kf, group, "enabled", true);
  return cfg;
}

static GimbalConfig read_gimbal(GKeyFile *kf, const gchar *group,
                                 uint16_t fallback_pan,
                                 uint16_t fallback_tilt)
{
  GimbalConfig cfg;
  cfg.pan_node_id      = read_uint16(kf, group, "pan_node_id", fallback_pan);
  cfg.tilt_node_id     = read_uint16(kf, group, "tilt_node_id", fallback_tilt);
  cfg.max_velocity_rad_s = read_float(kf, group, "max_velocity_rad_s", 6.28f);
  if (cfg.max_velocity_rad_s <= 0.0f)
    cfg.max_velocity_rad_s = 6.28f;
  cfg.tilt_max_rad     = read_float(kf, group, "tilt_max_rad", 0.611f);
  if (cfg.tilt_max_rad <= 0.0f)
    cfg.tilt_max_rad = 0.611f;
  return cfg;
}

static TrackingConfig read_tracking(GKeyFile *kf, const gchar *group)
{
  TrackingConfig cfg;
  cfg.home_return_enabled  = read_bool(kf, group, "home_return_enabled", true);
  cfg.home_return_gain     = read_float(kf, group, "home_return_gain", 2.0f);
  if (cfg.home_return_gain <= 0.0f)
    cfg.home_return_gain = 2.0f;
  cfg.home_tolerance_rad   = read_float(kf, group, "home_tolerance_rad", 0.05f);
  if (cfg.home_tolerance_rad <= 0.0f)
    cfg.home_tolerance_rad = 0.05f;
  cfg.home_return_delay_ms = static_cast<int>(
      read_uint16(kf, group, "home_return_delay_ms", 500));
  cfg.home_return_max_velocity = read_float(kf, group,
      "home_return_max_velocity", 1.5f);
  if (cfg.home_return_max_velocity <= 0.0f)
    cfg.home_return_max_velocity = 1.5f;
  return cfg;
}

SystemConfig load_config(const std::string &path)
{
  SystemConfig sys;

  GKeyFile *kf = g_key_file_new();
  GError *err = nullptr;

  if (!g_key_file_load_from_file(kf, path.c_str(), G_KEY_FILE_NONE, &err)) {
    g_warning("Failed to load config '%s': %s", path.c_str(), err->message);
    g_error_free(err);
    g_key_file_free(kf);
    return sys;
  }

  // [general]
  sys.mode = read_string(kf, "general", "mode", "both");
  sys.tracking = read_bool(kf, "general", "tracking", false);

  // [rgb]
  if (g_key_file_has_group(kf, "rgb"))
    sys.rgb = read_pipeline(kf, "rgb");

  // [mono]
  if (g_key_file_has_group(kf, "mono"))
    sys.mono = read_pipeline(kf, "mono");

  // Shared CAN bus settings from [can]
  if (g_key_file_has_group(kf, "can")) {
    sys.can_bus.datarate    = static_cast<uint8_t>(
        read_uint16(kf, "can", "datarate", 1));
    sys.can_bus.pds_node_id = read_uint16(kf, "can", "pds_node_id", 100);
  }

  // Per-gimbal configs: try [can_1], [can_2], ... then fall back to [can].
  // Also try [tracking_1], [tracking_2], ... then fall back to [tracking].
  bool found_numbered = false;
  for (int i = 1; i <= 8; ++i) {
    char can_group[16];
    char trk_group[16];
    std::snprintf(can_group, sizeof(can_group), "can_%d", i);
    std::snprintf(trk_group, sizeof(trk_group), "tracking_%d", i);

    if (!g_key_file_has_group(kf, can_group))
      break;

    found_numbered = true;
    uint16_t fallback_pan  = static_cast<uint16_t>(16 + (i - 1) * 2);
    uint16_t fallback_tilt = static_cast<uint16_t>(17 + (i - 1) * 2);
    sys.gimbals.push_back(read_gimbal(kf, can_group,
                                       fallback_pan, fallback_tilt));

    if (g_key_file_has_group(kf, trk_group))
      sys.tracking_cfg.push_back(read_tracking(kf, trk_group));
    else
      sys.tracking_cfg.push_back(TrackingConfig{});
  }

  // Legacy single-gimbal: [can] + [tracking]
  if (!found_numbered) {
    if (g_key_file_has_group(kf, "can")) {
      sys.gimbals.push_back(read_gimbal(kf, "can", 16, 18));
    } else {
      sys.gimbals.push_back(GimbalConfig{});
    }

    if (g_key_file_has_group(kf, "tracking"))
      sys.tracking_cfg.push_back(read_tracking(kf, "tracking"));
    else
      sys.tracking_cfg.push_back(TrackingConfig{});
  }

  g_key_file_free(kf);
  return sys;
}

}  // namespace patronus::config
