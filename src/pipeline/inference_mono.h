#pragma once

#include "core/config.h"
#include "core/state.h"
#include "tracking/prediction_overlay.h"

namespace patronus::pipeline {

/// Blocks until EOS, an error or stop_pipeline_mono(). Negative on setup failure.
int run_pipeline_mono(const patronus::config::PipelineConfig &config,
                      patronus::core::LatestValue<patronus::core::FrameObservation> &output,
                      patronus::tracking::PredictionChannel &predictions);

/// Called from the signal handler.
void stop_pipeline_mono();

/// No lasting effect while ExposureAuto is on.
void adjust_exposure_mono(double factor, double min_us, double max_us);

} // namespace patronus::pipeline
