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

  // [can]
  if (g_key_file_has_group(kf, "can")) {
    sys.can.pan_node_id      = read_uint16(kf, "can", "pan_node_id", 16);
    sys.can.tilt_node_id     = read_uint16(kf, "can", "tilt_node_id", 18);
    sys.can.datarate         = static_cast<uint8_t>(read_uint16(kf, "can", "datarate", 1));
    sys.can.max_velocity_rad_s =
        static_cast<float>(g_key_file_get_double(kf, "can", "max_velocity_rad_s", nullptr));
    if (sys.can.max_velocity_rad_s <= 0.0f)
      sys.can.max_velocity_rad_s = 6.28f;
    sys.can.pds_node_id      = read_uint16(kf, "can", "pds_node_id", 100);
  }

  g_key_file_free(kf);
  return sys;
}

}  // namespace patronus::config
