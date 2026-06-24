#include "patronus/core/config.hpp"
#include "patronus/core/state.hpp"
#include "patronus/pipeline/inference_mono.hpp"
#include "patronus/pipeline/inference_rgb.hpp"
#include <glib.h>
#include <gst/gst.h>
#include <cstdio>
#include <thread>

using patronus::config::SystemConfig;
using patronus::config::load_config;

static LatestValue<Detection> detection_rgb;
static LatestValue<Detection> detection_mono;

int main(int argc, char *argv[])
{
  // Default config path resolved from CWD (project root by convention)
  gchar *config_path = nullptr;
  gboolean rgb_only = FALSE;
  gboolean mono_only = FALSE;

  GOptionEntry entries[] = {
    {"config", 'c', 0, G_OPTION_ARG_FILENAME, &config_path,
     "Path to system config file (default: config/system.ini)", "FILE"},
    {"rgb-only", 'r', 0, G_OPTION_ARG_NONE, &rgb_only,
     "Run RGB pipeline only", nullptr},
    {"mono-only", 'm', 0, G_OPTION_ARG_NONE, &mono_only,
     "Run mono pipeline only", nullptr},
    {nullptr},
  };

  GOptionContext *ctx = g_option_context_new("- patronus inference system");
  g_option_context_add_main_entries(ctx, entries, nullptr);
  g_option_context_add_group(ctx, gst_init_get_option_group());

  GError *error = nullptr;
  if (!g_option_context_parse(ctx, &argc, &argv, &error)) {
    g_printerr("Option parsing failed: %s\n", error->message);
    g_error_free(error);
    g_option_context_free(ctx);
    return 1;
  }
  g_option_context_free(ctx);

  // Load configuration
  std::string cfg_path = config_path ? config_path : "config/system.ini";
  SystemConfig cfg = load_config(cfg_path);
  g_free(config_path);

  // CLI flags override config file
  if (rgb_only) cfg.mode = "rgb";
  if (mono_only) cfg.mode = "mono";

  g_print("Patronus inference — mode=%s  tracking=%s\n",
          cfg.mode.c_str(), cfg.tracking ? "on" : "off");

  // Launch pipeline threads
  std::thread rgb_thread;
  std::thread mono_thread;

  if (cfg.mode != "mono" && cfg.rgb.enabled) {
    rgb_thread = std::thread([&]() {
      run_pipeline_rgb(cfg.rgb, detection_rgb);
    });
  }

  if (cfg.mode != "rgb" && cfg.mono.enabled) {
    mono_thread = std::thread([&]() {
      run_pipeline_mono(cfg.mono, detection_mono);
    });
  }

  // Tracking thread is instantiated here only if cfg.tracking == true.
  // Currently unused — motor comm is disabled by default.

  if (rgb_thread.joinable()) rgb_thread.join();
  if (mono_thread.joinable()) mono_thread.join();

  return 0;
}
