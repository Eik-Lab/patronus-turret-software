#pragma once

#include "patronus/comm/candle_motor.hpp"
#include "patronus/core/config.hpp"
#include "patronus/core/state.hpp"

#include <atomic>

namespace patronus::tools {

/// @brief Measure the camera focal length for one gimbal by slewing through
///        known angles and solving fx = delta_px / delta_rad.
///
/// The IMM's ego-motion compensation converts gimbal slew rate into apparent
/// pixel rate using these values. Left at zero the filter treats the gimbal's own
/// motion as a target manoeuvre, which makes the D and lead terms fight the
/// gimbal and causes overshoot.
///
/// Runs against the live pipelines rather than opening the cameras itself: the
/// Basler sources will not accept a second client, so a separate binary could
/// only measure this by duplicating the whole perception stack.
///
/// The operator must keep a fixed, high-contrast target in view for the whole
/// sweep. Pan is swept because it has the most travel; tilt is assumed to share
/// the horizontal focal length, which holds for a square pixel pitch.
///
/// @param motor        Initialised motor driver.
/// @param gimbal       Zero-based gimbal index.
/// @param gimbal_cfg   That gimbal's motor ids and limits.
/// @param detections   Detection slot for the camera this gimbal tracks.
/// @param detection_w  Detection-space width in pixels.
/// @param detection_h  Detection-space height in pixels.
/// @param focal_x_px   Currently configured focal length, echoed back.
/// @param focal_y_px   Currently configured focal length, echoed back.
/// @param running      Set to false to abort early.
/// @return 0 on success, non-zero if no usable displacement was observed.
int run_focal_calibration(comm::CandleMotor &motor, size_t gimbal,
                          const patronus::config::GimbalConfig &gimbal_cfg,
                          patronus::core::LatestValue<patronus::core::Detection> &detections,
                          uint32_t detection_w, uint32_t detection_h, float focal_x_px,
                          float focal_y_px, std::atomic<bool> &running);

} // namespace patronus::tools