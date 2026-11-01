#include "pipeline/profiling.h"

#include "patronus/profiling.h"

#include <patronus/logging.h>

#include <mutex>
#include <string>
#include <vector>

namespace patronus::pipeline {
namespace {
  // A tracer measures the entire synchronous call into udpsink, not just the
  // instant a packet reaches its sink pad. Covers RTP's buffer-list fast path.
  struct ProfileTracer {
    GstTracer parent;
  };
  struct ProfileTracerClass {
    GstTracerClass parent;
  };
  // GLib's required C type-registration macro expands to C casts/type traits.
  // NOLINTNEXTLINE(modernize-type-traits,performance-no-int-to-ptr)
  G_DEFINE_TYPE(ProfileTracer, profile_tracer, GST_TYPE_TRACER)

  GQuark stream_key() {
    static const GQuark key = g_quark_from_static_string("patronus-profile-udp-stream");
    return key;
  }

  struct Send {
    int64_t start;
    int64_t cpu_start;
    uint64_t pts;
    size_t bytes;
  };
  thread_local std::vector<Send> sends;

  void push_pre(GstTracer *, guint64, GstPad *pad, GstBuffer *buffer) {
    if (g_object_get_qdata(G_OBJECT(pad), stream_key()))
      sends.push_back({profiling::now_ns(), profiling::thread_cpu_ns(), GST_BUFFER_PTS(buffer),
                       gst_buffer_get_size(buffer)});
  }

  void list_pre(GstTracer *, guint64, GstPad *pad, GstBufferList *list) {
    if (!g_object_get_qdata(G_OBJECT(pad), stream_key()))
      return;
    const guint count = gst_buffer_list_length(list);
    sends.push_back({profiling::now_ns(), profiling::thread_cpu_ns(),
                     count ? GST_BUFFER_PTS(gst_buffer_list_get(list, 0)) : GST_CLOCK_TIME_NONE,
                     gst_buffer_list_calculate_size(list)});
  }

  void push_post(GstTracer *, guint64, GstPad *pad, GstFlowReturn result) {
    const auto *stream = static_cast<const char *>(g_object_get_qdata(G_OBJECT(pad), stream_key()));
    if (!stream || sends.empty())
      return;
    const auto cpu_end = profiling::thread_cpu_ns();
    const auto send = sends.back();
    sends.pop_back();
    profiling::record(
      "span", "udp_send", stream, send.start, profiling::now_ns() - send.start, result, send.pts,
      send.cpu_start >= 0 && cpu_end >= send.cpu_start ? cpu_end - send.cpu_start : -1);
    profiling::record("counter", "udp_send_bytes", stream, send.start, 0,
                      static_cast<double>(send.bytes), send.pts);
  }

  void profile_tracer_class_init(ProfileTracerClass *) {
  }
  void profile_tracer_init(ProfileTracer *self) {
    auto *tracer = GST_TRACER(self);
    gst_tracing_register_hook(tracer, "pad-push-pre", G_CALLBACK(push_pre));
    gst_tracing_register_hook(tracer, "pad-push-post", G_CALLBACK(push_post));
    gst_tracing_register_hook(tracer, "pad-push-list-pre", G_CALLBACK(list_pre));
    gst_tracing_register_hook(tracer, "pad-push-list-post", G_CALLBACK(push_post));
  }

  struct Probe {
    const char *stage;
    const char *camera;
    bool inspected = false;
  };

  GstPadProbeReturn marker(GstPad *pad, GstPadProbeInfo *info, gpointer data) {
    auto &probe = *static_cast<Probe *>(data);
    auto record = [&](GstBuffer *buffer) {
      profiling::record("marker", probe.stage, probe.camera, profiling::now_ns(), 0,
                        static_cast<double>(gst_buffer_get_size(buffer)), GST_BUFFER_PTS(buffer));
      if (!probe.inspected &&
          (g_str_equal(probe.stage, "source") || g_str_equal(probe.stage, "convert_out"))) {
        probe.inspected = true;
        // Inspect allocator/caps metadata only. Mapping the buffer here could
        // itself cause a synchronization or copy and perturb the measurement.
        std::string description = std::string(probe.stage) + " buffer memory: ";
        for (guint i = 0; i < gst_buffer_n_memory(buffer); ++i) {
          const auto *memory = gst_buffer_peek_memory(buffer, i);
          if (i)
            description += ", ";
          description += memory->allocator ? memory->allocator->mem_type : "unknown";
        }
        if (gst_buffer_n_memory(buffer) == 0)
          description += "no memory blocks";
        if (auto *caps = gst_pad_get_current_caps(pad)) {
          if (gst_caps_get_size(caps) > 0) {
            auto *features = gst_caps_features_to_string(gst_caps_get_features(caps, 0));
            description += std::string("; caps features: ") + features;
            g_free(features);
          }
          gst_caps_unref(caps);
        }
        profiling::record("info", description.c_str(), probe.camera, profiling::now_ns());
      }
    };
    if ((GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) != 0)
      record(GST_PAD_PROBE_INFO_BUFFER(info));
    else if ((GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER_LIST) != 0) {
      auto *list = GST_PAD_PROBE_INFO_BUFFER_LIST(info);
      for (guint i = 0; i < gst_buffer_list_length(list); ++i)
        record(gst_buffer_list_get(list, i));
    }
    return GST_PAD_PROBE_OK;
  }
} // namespace

bool install_profiling(GstElement *pipeline, const char *camera) {
  static std::once_flag once;
  std::call_once(once, [] {
    // GStreamer's hook registry holds the tracer until gst_deinit().
    auto *tracer = g_object_new(profile_tracer_get_type(), nullptr);
    gst_object_unref(tracer);
  });
  struct Location {
    const char *element;
    const char *pad;
    const char *stage;
  };
  for (const auto &location :
       {Location{"pylon-source", "src", "source"},
        Location{"nvvideo-converter-pre", "sink", "convert_in"},
        Location{"nvvideo-converter-pre", "src", "convert_out"},
        Location{"stream-muxer", "sink_0", "mux_in"}, Location{"stream-muxer", "src", "mux_out"},
        Location{"primary-nvinference-engine", "sink", "infer_in"},
        Location{"primary-nvinference-engine", "src", "infer_out"},
        Location{"nvvideo-converter", "sink", "display_convert_in"},
        Location{"nvvideo-converter", "src", "display_convert_out"},
        Location{"nv-onscreendisplay", "sink", "osd_in"},
        Location{"nv-onscreendisplay", "src", "osd_out"},
        Location{"nvvideo-converter-post", "sink", "post_in"},
        Location{"nvvideo-converter-post", "src", "post_out"},
        Location{"video-encoder", "sink", "encode_in"},
        Location{"video-encoder", "src", "encode_out"},
        Location{"payload_encode", "sink", "payload_in"},
        Location{"payload_encode", "src", "payload_out"}, Location{"udp-sink", "sink", "udp_in"}}) {
    auto *element = gst_bin_get_by_name(GST_BIN(pipeline), location.element);
    auto *pad = element ? gst_element_get_static_pad(element, location.pad) : nullptr;
    if (element)
      gst_object_unref(element);
    if (!pad) {
      LOG(Error, "Cannot profile missing pad: " << location.stage);
      return false;
    }
    if (g_str_equal(location.stage, "payload_out"))
      g_object_set_qdata_full(G_OBJECT(pad), stream_key(), g_strdup(camera), g_free);
    gst_pad_add_probe(
      pad, static_cast<GstPadProbeType>(GST_PAD_PROBE_TYPE_BUFFER | GST_PAD_PROBE_TYPE_BUFFER_LIST),
      marker, new Probe{location.stage, camera}, [](gpointer data) {
        delete static_cast<Probe *>(data);
      });
    gst_object_unref(pad);
  }
  return true;
}
} // namespace patronus::pipeline
