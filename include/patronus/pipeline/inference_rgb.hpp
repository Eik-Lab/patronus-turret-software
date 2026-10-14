#pragma once

#include "patronus/core/config.hpp"
#include "patronus/core/state.hpp"
#include "patronus/tracking/prediction_overlay.hpp"

namespace patronus::pipeline {

/// @brief Launch the RGB-camera DeepStream inference pipeline.
///        Blocks until the pipeline terminates (EOS or error).
/// @param config Pipeline configuration (camera serial, caps, inference config, UDP sink).
/// @param output Thread-safe output slot; receives one observation per frame.
/// @param predictions Where to read filter predictions for the overlay. This
///        thread only ever reads from it; the tracking threads publish.
/// @return 0 on success, negative on error.
int run_pipeline_rgb(const patronus::config::PipelineConfig &config,
                     patronus::core::LatestValue<patronus::core::FrameObservation> &output,
                     patronus::tracking::PredictionChannel &predictions);

/// @brief Signal the RGB pipeline to quit its GStreamer main loop.
///        Safe to call from a signal handler (only touches a file-scope pointer).
void stop_pipeline_rgb();

/// @brief Change the rgb camera's exposure time by a relative amount.
/// @param factor Multiplier for the current exposure time; values below 1 shorten the exposure.
/// @param min_us Lower limit for the new exposure time, in microseconds.
/// @param max_us Upper limit for the new exposure time, in microseconds.
/// @note Does nothing if the rgb pipeline is not running. Has no lasting effect while the
///       camera's ExposureAuto is enabled.
void adjust_exposure_rgb(double factor, double min_us, double max_us);

} // namespace patronus::pipeline
