// Offline integration driver. Generates frames; never opens a camera or network sink.
#include "core/config.h"
#include "pipeline/video_encoder.h"

#include <gst/gst.h>

#include <iostream>
#include <string>

int main(int argc, char **argv) {
  if (argc != 4)
    return 2;
  gst_init(nullptr, nullptr);
  const auto system = patronus::config::load_config(argv[1]);
  const auto &config = std::string(argv[2]) == "rgb" ? system.rgb_ : system.mono_;
  GstElement *encoder = patronus::pipeline::make_video_encoder(config);
  if (encoder == nullptr)
    return 1;
  const bool hardware = config.encoder_backend_ == "thor";
  // Exercise the real encoder bin plus an in-process RTP roundtrip, not a copy
  // of the production encoder setup. Thor tests include the NVMM upload.
  const std::string graph =
    "videotestsrc num-buffers=120 pattern=ball ! "
    "video/x-raw,format=I420,width=1920,height=1088,framerate=30/1 ! " +
    std::string(hardware ? "nvvideoconvert" : "identity") +
    " name=upstream rtph264pay name=pay config-interval=-1 ! rtph264depay ! h264parse ! "
    "video/x-h264,stream-format=byte-stream,alignment=au ! filesink name=output";
  GError *error = nullptr;
  GstElement *pipeline = gst_parse_launch(graph.c_str(), &error);
  if (error != nullptr || pipeline == nullptr) {
    if (error != nullptr) {
      std::cerr << error->message << '\n';
      g_error_free(error);
    }
    if (pipeline != nullptr)
      gst_object_unref(pipeline);
    gst_object_unref(encoder);
    return 3;
  }
  gst_bin_add(GST_BIN(pipeline), encoder);
  GstElement *upstream = gst_bin_get_by_name(GST_BIN(pipeline), "upstream");
  GstElement *pay = gst_bin_get_by_name(GST_BIN(pipeline), "pay");
  GstElement *output = gst_bin_get_by_name(GST_BIN(pipeline), "output");
  g_object_set(output, "location", argv[3], nullptr);
  const bool linked = gst_element_link_many(upstream, encoder, pay, nullptr);
  gst_object_unref(upstream);
  gst_object_unref(pay);
  gst_object_unref(output);
  GstBus *bus = gst_element_get_bus(pipeline);
  bool ok =
    linked && gst_element_set_state(pipeline, GST_STATE_PLAYING) != GST_STATE_CHANGE_FAILURE;
  GstMessage *message =
    ok ? gst_bus_timed_pop_filtered(
           bus, 30 * GST_SECOND, static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR))
       : nullptr;
  ok = message != nullptr && GST_MESSAGE_TYPE(message) == GST_MESSAGE_EOS;
  if (message != nullptr) {
    if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
      gst_message_parse_error(message, &error, nullptr);
      std::cerr << error->message << '\n';
      g_error_free(error);
    }
    gst_message_unref(message);
  }
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(bus);
  gst_object_unref(pipeline);
  return ok ? 0 : 3;
}
