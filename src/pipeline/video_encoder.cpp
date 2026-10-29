#include "pipeline/video_encoder.h"

#include "log.h"

#include <limits>

namespace patronus::pipeline {

GstElement *make_video_encoder(const config::PipelineConfig &config) {
  const bool hardware = config.encoder_backend_ == "thor";
  if ((!hardware && config.encoder_backend_ != "x264") || config.encoder_bitrate_ == 0 ||
      (hardware && config.encoder_bitrate_ > std::numeric_limits<guint>::max() / 1000U)) {
    LOG(Error, "Invalid encoder backend/bitrate: " << config.encoder_backend_ << "/"
                                                   << config.encoder_bitrate_);
    return nullptr;
  }
  GstElement *bin = gst_bin_new("video-encoder");
  // Attach each element immediately so all failure paths release the whole bin.
  const auto add = [bin](const char *factory, const char *name) {
    GstElement *element = gst_element_factory_make(factory, name);
    if (element != nullptr)
      gst_bin_add(GST_BIN(bin), element);
    return element;
  };
  GstElement *input = add("capsfilter", "caps-encoder");
  GstElement *encoder = add(hardware ? "nvv4l2h264enc" : "x264enc", "encoder");
  GstElement *parser = hardware ? add("h264parse", "encoder-parser") : nullptr;
  GstElement *output = hardware ? add("capsfilter", "caps-h264") : encoder;
  const auto fail = [bin, &config]() {
    LOG(Error,
        "Cannot build encoder backend '" << config.encoder_backend_ << "'; no fallback performed");
    gst_object_unref(bin);
    return static_cast<GstElement *>(nullptr);
  };
  if (input == nullptr || encoder == nullptr ||
      (hardware && (parser == nullptr || output == nullptr)))
    return fail();

  GstCaps *caps = gst_caps_from_string(hardware ? "video/x-raw(memory:NVMM),format=I420"
                                                : "video/x-raw,format=I420");
  g_object_set(input, "caps", caps, nullptr);
  gst_caps_unref(caps);
  if (hardware) {
    // These are the Thor settings exercised by the offline encoder benchmark.
    // Older Jetson plugins must not silently ignore the Thor tuning properties.
    for (const char *property :
         {"bitrate", "control-rate", "iframeinterval", "idrinterval", "num-B-Frames",
          "insert-sps-pps", "insert-aud", "insert-vui", "profile", "preset-id", "tuning-info-id",
          "vbvbufsize", "vbvinit"}) {
      if (g_object_class_find_property(G_OBJECT_GET_CLASS(encoder), property) == nullptr)
        return fail();
    }
    g_object_set(encoder, "bitrate", config.encoder_bitrate_ * 1000U, "iframeinterval", 60U,
                 "idrinterval", 60U, "num-B-Frames", 0U, "insert-sps-pps", TRUE, "insert-aud", TRUE,
                 "insert-vui", TRUE, "preset-id", 4U, "vbvbufsize", config.encoder_bitrate_ * 600U,
                 "vbvinit", config.encoder_bitrate_ * 540U, nullptr);
    gst_util_set_object_arg(G_OBJECT(encoder), "control-rate", "constant_bitrate");
    gst_util_set_object_arg(G_OBJECT(encoder), "profile", "High");
    gst_util_set_object_arg(G_OBJECT(encoder), "tuning-info-id", "UltraLowLatencyPreset");
    g_object_set(parser, "config-interval", -1, nullptr);
    caps = gst_caps_from_string("video/x-h264,stream-format=byte-stream,alignment=au");
    g_object_set(output, "caps", caps, nullptr);
    gst_caps_unref(caps);
  } else {
    // Keep the existing software path, including its default GOP/rate control.
    g_object_set(encoder, "bitrate", config.encoder_bitrate_, nullptr);
    gst_util_set_object_arg(G_OBJECT(encoder), "tune", "zerolatency");
    gst_util_set_object_arg(G_OBJECT(encoder), "speed-preset", "superfast");
  }
  if (!(hardware ? gst_element_link_many(input, encoder, parser, output, nullptr)
                 : gst_element_link(input, encoder)))
    return fail();

  GstPad *sink = gst_element_get_static_pad(input, "sink");
  GstPad *src = gst_element_get_static_pad(output, "src");
  gst_element_add_pad(bin, gst_ghost_pad_new("sink", sink));
  gst_element_add_pad(bin, gst_ghost_pad_new("src", src));
  gst_object_unref(sink);
  gst_object_unref(src);
  return bin;
}

} // namespace patronus::pipeline
