#pragma once

#include "patronus/core/config.hpp"
#include "patronus/core/state.hpp"

int run_pipeline_mono(const patronus::config::PipelineConfig &config,
                      LatestValue<Detection> &output);
