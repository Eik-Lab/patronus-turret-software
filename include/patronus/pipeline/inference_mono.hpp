#pragma once

#include "patronus/core/config.hpp"
#include "patronus/core/state.hpp"
#include "patronus/tracking/prediction_overlay.hpp"

namespace patronus::pipeline {

/// @brief Launch the mono-camera DeepStream inference pipeline.
///        Blocks until the pipeline terminates (EOS or error).
/// @param config Pipeline configuration (camera serial, caps, inference config, UDP sink).
/// @param output Thread-safe output slot; receives one observation per frame.
/// @param predictions Where to read filter predictions for the overlay. This
///        thread only ever reads from it; the tracking threads publish.
/// @return 0 on success, negative on error.
int run_pipeline_mono(const patronus::config::PipelineConfig &config,
                      patronus::core::LatestValue<patronus::core::FrameObservation> &output,
                      patronus::tracking::PredictionChannel &predictions);

/// @brief Signal the mono pipeline to quit its GStreamer main loop.
///        Safe to call from a signal handler (only touches a file-scope pointer).
void stop_pipeline_mono();

} // namespace patronus::pipeline
