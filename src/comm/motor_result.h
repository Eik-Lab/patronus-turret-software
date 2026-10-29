#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace patronus::comm {

/// Caller-owned totals for one tick, including followers and fallback stops.
/// SDK calls are not physical CAN-frame counts (the SDK may retry internally).
struct BusMetrics {
  double lock_wait_us{0.0};
  double read_us{0.0};
  double write_us{0.0};
  uint32_t read_calls{0};
  uint32_t write_calls{0};
};

// Stable telemetry values. SDK error codes are retained separately; 0 is NOT SDK success.
enum class IoStatus { NotAttempted, Ok, Error, Unavailable };

struct AxisReading {
  float value{std::numeric_limits<float>::quiet_NaN()};
  double time_s{0.0}; ///< Host midpoint of the exchange, not an encoder hardware timestamp.
  IoStatus status{IoStatus::NotAttempted};
  int sdk_error{-1}; ///< -1 when no SDK call was made.

  bool fresh(double now_s, double max_age_s) const {
    return status == IoStatus::Ok && std::isfinite(value) && std::isfinite(time_s) &&
           now_s >= time_s && now_s - time_s < max_age_s;
  }
};

struct MotorReading {
  AxisReading pan, tilt;

  bool fresh(double now_s, double max_age_s) const {
    return pan.fresh(now_s, max_age_s) && tilt.fresh(now_s, max_age_s);
  }
};

struct AxisCommand {
  float requested{0.0F}; ///< Before scaling, clamping and soft limits.
  float attempted{std::numeric_limits<float>::quiet_NaN()};
  IoStatus status{IoStatus::NotAttempted};
  int sdk_error{-1};
  AxisReading limit_position; ///< Populated only when a soft-limit read was required.
};

struct MotorCommand {
  AxisCommand pan, tilt;
  // Best-effort zero writes after a partial/failed nonzero command or home arrival.
  // The original outcome remains above; even a successful stop is not motion feedback.
  AxisCommand stop_pan, stop_tilt;

  /// Both writes succeeded and neither request was rejected for invalid input/limit feedback.
  bool ok() const {
    return pan.status == IoStatus::Ok && tilt.status == IoStatus::Ok &&
           pan.limit_position.status != IoStatus::Error &&
           tilt.limit_position.status != IoStatus::Error && std::isfinite(pan.requested) &&
           std::isfinite(tilt.requested);
  }
};

struct MotionResult {
  MotorReading position;
  MotorCommand command;
  bool within_tolerance{false};

  bool reached() const {
    return within_tolerance && command.ok();
  }
};

} // namespace patronus::comm
