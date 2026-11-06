#pragma once

#include "core/config.h"

#include <gst/gst.h>

namespace patronus::pipeline {

/// Floating bin with sink/src pads. Precede with nvvideoconvert: x264 requests
/// host I420; thor requests NVMM I420. nullptr on unsupported configuration or
/// missing plugins/properties; never silently falls back to another backend.
GstElement *make_video_encoder(const config::PipelineConfig &config);

} // namespace patronus::pipeline
