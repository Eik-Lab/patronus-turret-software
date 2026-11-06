#pragma once

#include <gst/gst.h>

namespace patronus::pipeline {

/// Opt-in, bounded, asynchronous per-camera pad tracing via PATRONUS_TRACE_DIR.
/// Install before PLAYING. No probes or writer threads when the variable is unset.
/// Returns false if an explicitly requested trace cannot be initialized.
bool install_run_trace(GstElement *pipeline, const char *camera);

} // namespace patronus::pipeline
