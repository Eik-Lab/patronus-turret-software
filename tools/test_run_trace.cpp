// Hardware-independent trace fixture: no camera, CUDA, motor or network access.
#include "pipeline/run_trace.h"

#include <gst/gst.h>

#include <cstdlib>
#include <initializer_list>

int main(int argc, char **argv) {
  if (argc != 2)
    return 2;
  gst_init(nullptr, nullptr);
  g_setenv("PATRONUS_TRACE_DIR", argv[1], TRUE);
  for (const char *camera : {"rgb", "mono"}) {
    GError *error = nullptr;
    GstElement *pipeline = gst_parse_launch(
      "fakesrc name=pylon-source num-buffers=17 sizetype=fixed sizemax=64 do-timestamp=true ! "
      "identity name=nvvideo-converter-pre ! identity name=stream-muxer ! "
      "identity name=primary-nvinference-engine ! identity name=nv-onscreendisplay ! "
      "identity name=nvvideo-converter-post ! identity name=video-encoder ! fakesink",
      &error);
    if (error != nullptr || pipeline == nullptr ||
        !patronus::pipeline::install_run_trace(pipeline, camera))
      return 3;
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    GstBus *bus = gst_element_get_bus(pipeline);
    GstMessage *message = gst_bus_timed_pop_filtered(
      bus, 5 * GST_SECOND, static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
    const bool ok = message != nullptr && GST_MESSAGE_TYPE(message) == GST_MESSAGE_EOS;
    if (message != nullptr)
      gst_message_unref(message);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(bus);
    gst_object_unref(pipeline);
    if (!ok)
      return 4;
  }
  // Reusing a run must not overwrite its records.
  GstElement *empty = gst_pipeline_new("empty");
  const bool overwritten = patronus::pipeline::install_run_trace(empty, "rgb");
  gst_object_unref(empty);
  return overwritten ? 5 : 0;
}
