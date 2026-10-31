#include "pipeline/profiling.h"

#include <algorithm>
#include <ostream>
#include <tuple>

namespace patronus::pipeline {
namespace {
  using profiling::Event;
  using profiling::Groups;

  struct PipelineTimings {
    Groups residence;
    std::map<std::string, std::vector<const Event *>> markers;

    explicit PipelineTimings(const std::vector<Event> &events) {
      std::map<std::tuple<std::string, std::string, uint64_t>, std::vector<int64_t>> endpoints;
      for (const auto &event : events) {
        if (event.kind != "marker")
          continue;
        markers[event.stream + "/" + event.name].push_back(&event);
        if (event.id != UINT64_MAX)
          endpoints[{event.stream, event.name, event.id}].push_back(event.start);
      }
      for (auto &[name, samples] : markers) {
        if (name.size() >= 7 && name.substr(name.size() - 7) == "/source") {
          std::sort(samples.begin(), samples.end(), [](const Event *a, const Event *b) {
            return a->start < b->start;
          });
          for (size_t i = 1; i < samples.size(); ++i)
            residence[name + "_interarrival"].push_back(
              (samples[i]->start - samples[i - 1]->start) / 1e6);
        }
      }
      struct Pair {
        const char *name, *in, *out;
      };
      for (const auto &pair :
           {Pair{"frame_ready", "source", "convert_out"},
            Pair{"source_delivery", "source", "convert_in"},
            Pair{"inference_total", "infer_in", "osd_in"},
            Pair{"display_conversion", "display_convert_in", "display_convert_out"},
            Pair{"conversion", "convert_in", "convert_out"}, Pair{"mux", "mux_in", "mux_out"},
            Pair{"inference", "infer_in", "infer_out"}, Pair{"osd", "osd_in", "osd_out"},
            Pair{"postconversion", "post_in", "post_out"},
            Pair{"encoding", "encode_in", "encode_out"},
            Pair{"payload", "payload_in", "payload_out"},
            Pair{"source_to_encoder", "source", "encode_out"}}) {
        for (const auto &[key, times] : endpoints) {
          const auto &[stream, stage, id] = key;
          if (stage != pair.in)
            continue;
          const auto other = endpoints.find({stream, pair.out, id});
          if (times.size() == 1 && other != endpoints.end() && other->second.size() == 1 &&
              other->second[0] >= times[0])
            residence[stream + "/" + pair.name].push_back((other->second[0] - times[0]) / 1e6);
        }
      }
    }
  };

  profiling::TreeRoot tree_root(const Event &event) {
    if (event.stream == "motor")
      return {"Motor function tree",
              event.id != UINT64_MAX ? "gimbal-" + std::to_string(event.id) : event.stream};
    if (event.name == "osd_sink_pad_buffer_probe" || event.name == "step_filter" ||
        event.name == "project_aim")
      return {"Inference function tree", event.stream};
    return {};
  }

  void summary(std::ostream &out, const std::vector<Event> &events,
               const std::vector<const Event *> &roots, double seconds) {
    PipelineTimings timings(events);
    Groups stages;
    for (const auto *event : roots)
      if (event->stream == "motor") {
        const auto context = tree_root(*event).context;
        stages[context + " motor host operations"].push_back(event->duration / 1e6);
      }
    if (stages.empty())
      stages.try_emplace("motor host operations (not exercised)");
    for (const auto &[name, samples] : timings.markers) {
      if (name.size() < 7 || name.substr(name.size() - 7) != "/source")
        continue;
      const auto camera = name.substr(0, name.size() - 7);
      stages[camera + " inference host latency"] = timings.residence[camera + "/inference_total"];
      stages[camera + " post-capture buffer preparation"] =
        timings.residence[camera + "/frame_ready"];
      stages[camera + " delivery cadence (not acquisition)"] =
        timings.residence[camera + "/source_interarrival"];
    }
    profiling::print_table(out, "Stage summary (host timings)", std::move(stages), seconds, true,
                           true);
    out << "Camera stages match unique same-camera PTS; they are NOT synchronous function calls.\n"
        << "Inference camera total: nvinfer input through observation callback completion (OSD "
           "input). Percentiles do not add.\n"
        << "Unmeasured internals: pylonsrc sensor acquisition/buffer filling and NVIDIA "
           "preprocessing/TensorRT kernels/output parsing. No capture function tree is available.\n"
        << "Buffer preparation starts AFTER acquisition; host residence is NOT GPU completion or "
           "a separately measured CPU-GPU memcpy. Shared memory does NOT establish zero-copy.\n";
    for (const auto &event : events)
      if (event.kind == "info")
        out << event.stream << ": " << event.name << '\n';
  }

  void details(std::ostream &out, const std::vector<Event> &events,
               const std::vector<const Event *> &roots, double seconds) {
    if (std::none_of(roots.begin(), roots.end(), [](const Event *event) {
          return event->stream == "motor";
        }))
      out << "No motor functions recorded (not exercised or nesting unavailable).\n";
    out << "Motor totals cover recorded host operations, including SDK/lock waiting, NOT "
           "mechanical response. No samples means not exercised, never zero.\n";
    PipelineTimings timings(events);
    profiling::print_table(
      out, "Pipeline residence/capture delivery cadence (matched subset, not GPU kernel time)",
      std::move(timings.residence), seconds);
    out << "\nMarker coverage (counts are buffers/packets, not necessarily frames):\n";
    for (const auto &[name, samples] : timings.markers)
      out << name << " count=" << samples.size() << '\n';
    out << "Residence requires unique valid same-camera PTS at both endpoints; rewritten, "
           "duplicate, missing or backwards PTS are not paired.\n"
        << "Missing residence rows are unavailable, never zero. No source-to-UDP/client latency "
           "is inferred.\n";
    struct Io {
      uint64_t operations = 0, failures = 0;
      double bytes = 0;
    };
    std::map<std::string, Io> io;
    for (const auto &event : events) {
      if (event.kind == "span" && event.name == "udp_send") {
        ++io[event.stream].operations;
        if (event.value < 0)
          ++io[event.stream].failures;
      } else if (event.kind == "counter" && event.name == "udp_send_bytes") {
        io[event.stream].bytes += event.value;
      }
    }
    out << "\nUDP I/O (attempted bytes; push/list operations, NOT packet counts or delivery "
           "acknowledgements):\n";
    for (const auto &[stream, counts] : io)
      out << stream << " operations=" << counts.operations << " failures=" << counts.failures
          << " bytes=" << counts.bytes << " operations/s=" << counts.operations / seconds
          << " bytes/s=" << counts.bytes / seconds << '\n';
  }
} // namespace

profiling::ReportOptions profiling_report() {
  return {tree_root, summary, details};
}
} // namespace patronus::pipeline
