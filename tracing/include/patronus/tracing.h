#pragma once

#include <gst/gst.h>

#include <string>
#include <vector>

namespace patronus::tracing {

struct Location {
  std::string element, pad, stage;
};

// Install before PLAYING on a bin/pipeline with the specified static pads.
// Locations and labels are copied; the caller need not keep them alive.
// Stage labels must not contain commas, quotes, CR or LF. The output must not exist.
// Probes own the writer: stop and unref the pipeline to drain and close it.
// Returns false for file/pad errors; discard the pipeline on failure because
// earlier probes may already have been installed. Errors go to stderr.
bool install(GstElement *pipeline, const std::string &path, const std::vector<Location> &locations);

} // namespace patronus::tracing
