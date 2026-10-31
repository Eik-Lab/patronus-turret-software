// Real GStreamer pushes and loopback UDP, without cameras, CUDA or motors.
#include "pipeline/profiling.h"

#include "patronus/profiling.h"

#include <gst/gst.h>

#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>

static GstPadProbeReturn observation(GstPad *, GstPadProbeInfo *, gpointer data) {
  const auto *camera = static_cast<const char *>(data);
  PATRONUS_PROFILE(osd_sink_pad_buffer_probe, camera, UINT64_MAX);
  {
    PATRONUS_PROFILE(select_detection, camera, UINT64_MAX);
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }
  {
    PATRONUS_PROFILE(draw_predictions, camera, UINT64_MAX);
    PATRONUS_PROFILE(project_prediction, "overlay", UINT64_MAX);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  return GST_PAD_PROBE_OK;
}

int main(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "--overflow") {
    patronus::profiling::Session session(patronus::pipeline::profiling_report());
    for (int64_t i = 0; i < 1000100; ++i)
      patronus::profiling::record("span", "overflow", "test", i * 1000, 500);
    return 0;
  }
  if (argc == 2 && std::string(argv[1]) == "--summary") {
    using patronus::profiling::record;
    patronus::profiling::Session session(patronus::pipeline::profiling_report());
    int64_t start = 1000000;
    // Asymmetric tail: p999 differs from p99 and max.
    for (int i = 0; i < 1000; ++i) {
      const int64_t ms = i < 500 ? 1 : i < 900 ? 20 : i < 990 ? 70 : i < 999 ? 300 : 9999;
      record("span", "tail", "test", start, ms * 1000000, 0, UINT64_MAX, 0);
      start += (ms + 1) * 1000000;
    }
    record("span", "parent", "test", start, 100000000, 0, UINT64_MAX, 60000000);
    record("span", "child", "other", start + 5000000, 90000000, 0, UINT64_MAX, 40000000);
    record("span", "unknown", "test", start + 200000000, 10000000);
    // Synthetic motor intervals verify report nesting, never touch hardware.
    record("span", "set_velocity", "motor", start + 300000000, 100000000, 0, 2);
    record("span", "lock_bus", "motor", start + 305000000, 10000000);
    record("span", "set_velocity_locked", "motor", start + 320000000, 60000000, 0, 2);
    record("span", "write_axis", "motor", start + 330000000, 20000000);
    record("span", "setTargetVelocity", "motor", start + 335000000, 5000000);
    record("span", "osd_sink_pad_buffer_probe", "rgb", start + 500000000, 100000000);
    record("span", "draw_predictions", "rgb", start + 510000000, 60000000);
    record("span", "project_prediction", "overlay", start + 520000000, 20000000);
    std::thread ambiguous([] {
      record("span", "crossing", "test", 1000000, 10000000);
      record("span", "crossing", "test", 2000000, 20000000);
    });
    ambiguous.join();
    for (const auto &entry : {std::pair<uint64_t, int>{1, 10},
                              {2, 20},
                              {2, 21},
                              {3, 30},
                              {4, 40},
                              {UINT64_MAX, 50},
                              {6, 60},
                              {7, 70}})
      record("marker", "infer_in", "rgb", entry.second * 1000000LL, 0, 0, entry.first);
    for (const auto &entry : {std::pair<uint64_t, int>{1, 15},
                              {2, 25},
                              {3, 35},
                              {3, 36},
                              {5, 45},
                              {UINT64_MAX, 55},
                              {6, 59},
                              {8, 80}})
      record("marker", "infer_out", "rgb", entry.second * 1000000LL, 0, 0, entry.first);
    record("marker", "infer_in", "mono", 1000000, 0, 0, 1);
    record("marker", "infer_out", "mono", 10000000, 0, 0, 1);
    record("marker", "source", "rgb", 1000000, 0, 0, 1);
    record("marker", "encode_out", "mono", 8000000, 0, 0, 1);
    return 0;
  }
  if (argc != 3)
    return 2;
  g_setenv("PATRONUS_PROFILE_DIR", argv[1], TRUE);
  gst_init(nullptr, nullptr);
  {
    patronus::profiling::Session session(patronus::pipeline::profiling_report());
    // Distinct durations and IDs exercise concurrent recording and shutdown drain.
    auto worker = [](const char *stream) {
      PATRONUS_PROFILE(worker, stream, UINT64_MAX);
      for (uint64_t i = 0; i < 31; ++i) {
        PATRONUS_PROFILE(prediction, stream, i);
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
      {
        PATRONUS_PROFILE(cpu_work, stream, UINT64_MAX);
        const auto start = patronus::profiling::thread_cpu_ns();
        if (start < 0)
          return;
        while (patronus::profiling::thread_cpu_ns() - start < 5000000) {
        }
      }
      {
        PATRONUS_PROFILE(sleep, stream, UINT64_MAX);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    };
    std::thread left(worker, "gimbal-0"), right(worker, "gimbal-1");
    left.join();
    right.join();
    for (const char *camera : {"rgb", "mono"}) {
      const std::string description =
        "fakesrc name=pylon-source num-buffers=17 sizetype=fixed sizemax=64 do-timestamp=true ! "
        "identity name=nvvideo-converter-pre ! stream-muxer.sink_0 funnel name=stream-muxer ! "
        "identity name=primary-nvinference-engine sleep-time=1000 ! identity "
        "name=nvvideo-converter ! identity "
        "name=nv-onscreendisplay ! "
        "identity name=nvvideo-converter-post ! identity name=video-encoder ! "
        "identity name=payload_encode ! udpsink name=udp-sink host=127.0.0.1 port=" +
        std::string(argv[2]) + " sync=false async=false";
      GError *error = nullptr;
      auto *pipeline = gst_parse_launch(description.c_str(), &error);
      if (error || !pipeline)
        return 3;
      auto *osd = gst_bin_get_by_name(GST_BIN(pipeline), "nv-onscreendisplay");
      auto *osd_pad = gst_element_get_static_pad(osd, "sink");
      gst_pad_add_probe(osd_pad, GST_PAD_PROBE_TYPE_BUFFER, observation, const_cast<char *>(camera),
                        nullptr);
      gst_object_unref(osd_pad);
      gst_object_unref(osd);
      if (error || !pipeline || !PATRONUS_PROFILE_PIPELINE(pipeline, camera))
        return 3;
      gst_element_set_state(pipeline, GST_STATE_PLAYING);
      auto *bus = gst_element_get_bus(pipeline);
      auto *message = gst_bus_timed_pop_filtered(
        bus, 5 * GST_SECOND, static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
      const bool ok = message && GST_MESSAGE_TYPE(message) == GST_MESSAGE_EOS;
      if (message)
        gst_message_unref(message);
      gst_element_set_state(pipeline, GST_STATE_NULL);
      gst_object_unref(bus);
      gst_object_unref(pipeline);
      if (!ok)
        return 4;
    }
    // Explicit list pushes exercise the actual fast path used by RTP payloaders.
    auto *pipeline = gst_parse_launch(
      ("fakesrc name=pylon-source num-buffers=0 ! identity name=nvvideo-converter-pre ! "
       "stream-muxer.sink_0 funnel name=stream-muxer ! identity name=primary-nvinference-engine ! "
       "identity name=nvvideo-converter ! "
       "identity name=nv-onscreendisplay ! identity name=nvvideo-converter-post ! "
       "identity name=video-encoder ! identity name=payload_encode ! "
       "udpsink name=udp-sink host=127.0.0.1 sync=false async=false port=" +
       std::string(argv[2]))
        .c_str(),
      nullptr);
    if (!pipeline || !PATRONUS_PROFILE_PIPELINE(pipeline, "rgb"))
      return 5;
    // Keep the source stopped; manually supply valid sticky events before the list.
    auto *source = gst_bin_get_by_name(GST_BIN(pipeline), "pylon-source");
    gst_element_set_locked_state(source, TRUE);
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    auto *payload = gst_bin_get_by_name(GST_BIN(pipeline), "payload_encode");
    auto *pad = gst_element_get_static_pad(payload, "src");
    gst_pad_push_event(pad, gst_event_new_stream_start("profile-list"));
    GstSegment segment;
    gst_segment_init(&segment, GST_FORMAT_TIME);
    gst_pad_push_event(pad, gst_event_new_segment(&segment));
    auto *list = gst_buffer_list_new();
    for (int i = 0; i < 3; ++i) {
      auto *buffer = gst_buffer_new_allocate(nullptr, 23, nullptr);
      GST_BUFFER_PTS(buffer) = 9000000000;
      gst_buffer_list_add(list, buffer);
    }
    const bool sent = gst_pad_push_list(pad, list) == GST_FLOW_OK;
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pad);
    gst_object_unref(payload);
    gst_object_unref(source);
    gst_object_unref(pipeline);
    if (!sent)
      return 6;
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
  }
  gst_deinit();
}
