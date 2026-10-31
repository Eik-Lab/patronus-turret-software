#include "pipeline/run_trace.h"

#include "patronus/tracing.h"

#include <cstdlib>
#include <string>

namespace patronus::pipeline {

bool install_run_trace(GstElement *pipeline, const char *camera) {
  const char *directory = std::getenv("PATRONUS_TRACE_DIR");
  if (directory == nullptr || *directory == '\0')
    return true;
  const std::string path = std::string(directory) + "/pipeline-" + camera + ".csv";
  return tracing::install(pipeline, path,
                          {{"pylon-source", "src", "source"},
                           {"nvvideo-converter-pre", "sink", "convert_in"},
                           {"nvvideo-converter-pre", "src", "convert_out"},
                           {"stream-muxer", "src", "mux_out"},
                           {"primary-nvinference-engine", "sink", "infer_in"},
                           {"primary-nvinference-engine", "src", "infer_out"},
                           {"nv-onscreendisplay", "sink", "osd_in"},
                           {"nv-onscreendisplay", "src", "osd_out"},
                           {"nvvideo-converter-post", "sink", "post_in"},
                           {"nvvideo-converter-post", "src", "post_out"},
                           {"video-encoder", "sink", "encode_in"},
                           {"video-encoder", "src", "encode_out"}});
}

} // namespace patronus::pipeline
