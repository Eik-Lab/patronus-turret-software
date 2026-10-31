#pragma once

#ifdef PATRONUS_PROFILING
  #include "patronus/profiling.h"

  #include <gst/gst.h>

namespace patronus::pipeline {
bool install_profiling(GstElement *pipeline, const char *camera);
profiling::ReportOptions profiling_report();
} // namespace patronus::pipeline

  #define PATRONUS_PROFILE_PIPELINE(pipeline, camera) \
    patronus::pipeline::install_profiling(pipeline, camera)
#else
// Discard arguments as tokens: no evaluation, calls or GStreamer dependency.
  #define PATRONUS_PROFILE_PIPELINE(pipeline, camera) true
#endif
