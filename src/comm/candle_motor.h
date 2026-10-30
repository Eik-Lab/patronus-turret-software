#pragma once

#include "comm/motor_result.h"
#include "core/config.h"
#include "core/types.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

namespace mab {
class Candle;
class MD;
class Pds;
class PowerStage;
} // namespace mab

namespace patronus::comm {

/// All gimbals' pan/tilt drives on one CANdle bus, in VELOCITY_PID mode.
/// Safe to call from one thread per gimbal: bus exchanges are serialised.
class CandleMotor {
public:
  explicit CandleMotor(const std::vector<config::GimbalConfig> &gimbals,
                       const config::CanBusConfig &bus = config::CanBusConfig{});

  ~CandleMotor();

  CandleMotor(const CandleMotor &) = delete;
  CandleMotor &operator=(const CandleMotor &) = delete;

  [[nodiscard]] bool init();

  [[nodiscard]] size_t gimbal_count() const {
    return gimbals_.size();
  }

  /// Scales, clamps and applies soft limits, then commands the drives.
  /// SDK write success does not confirm physical motion. Hot path.
  MotorCommand set_velocity(size_t gimbal, float pan_rad_s, float tilt_rad_s,
                            BusMetrics *metrics = nullptr);

  /// Per-axis status and host exchange timestamps; failed values are NaN.
  /// Position reads stop on pan failure; tilt then remains NotAttempted.
  MotorReading get_velocity(size_t gimbal, BusMetrics *metrics = nullptr);
  MotorReading get_position(size_t gimbal, BusMetrics *metrics = nullptr);

  /// For axes with calibrate_on_boot_: drive into both end stops to find soft
  /// limits, then park at the centre, which becomes home. Blocks up to ~70 s
  /// per axis; call before tracking starts. Failed axes are left disabled.
  [[nodiscard]] bool boot_sequence();

  /// Requests paired zero velocity inside tolerance; reached() requires both writes.
  MotionResult return_to_home(size_t gimbal, float position_gain, float tolerance_rad,
                              float max_velocity_rad_s, double max_pose_age_s,
                              BusMetrics *metrics = nullptr);

  /// One P-control step. Invalid/stale reads request zero velocity, never arrival.
  /// reached() requires an in-tolerance measurement and successful command writes.
  /// Holds the bus lock through feedback, computation, limit reads and writes.
  /// Feedback reuse is disabled: soft limits always acquire separate readings.
  /// max_velocity_rad_s 0 means the gimbal's max.
  MotionResult drive_to(size_t gimbal, float pan_rad, float tilt_rad, float position_gain,
                        float tolerance_rad, float max_velocity_rad_s, double max_pose_age_s,
                        BusMetrics *metrics = nullptr);

  /// One step of following another gimbal: its velocity (drive rad/s, as written to its
  /// drives) plus P-control on the pose error, with no deadband. position_gain and its
  /// clamp at the gimbal's max are before velocity scaling. Same feedback, stale-read
  /// and locking rules as drive_to.
  MotionResult mirror_pose(size_t gimbal, float pan_rad, float tilt_rad, float pan_velocity_rad_s,
                           float tilt_velocity_rad_s, float position_gain, double max_pose_age_s,
                           BusMetrics *metrics = nullptr);

  /// Centre of the calibrated range, or 0 for an uncalibrated axis.
  std::pair<float, float> home_position(size_t gimbal) const;

  /// Bypasses the bus lock: diagnostics only, never while tracking.
  mab::MD *pan_motor(size_t gimbal);
  mab::MD *tilt_motor(size_t gimbal);

  /// Idempotent.
  void disable();

private:
  bool enable_pds();

  /// Radians from the zero set when calibration started.
  struct SoftLimit {
    bool calibrated{false};
    float min_pos{0.0F};
    float max_pos{0.0F};
    float home_pos{0.0F};
  };

  // The helpers below expect bus_mutex_ held.
  MotorReading get_position_locked(size_t gimbal, BusMetrics *metrics);
  MotorCommand set_velocity_locked(size_t gimbal, float pan_rad_s, float tilt_rad_s,
                                   BusMetrics *metrics);
  MotionResult drive_to_locked(size_t gimbal, float pan_rad, float tilt_rad, float position_gain,
                               float tolerance_rad, float max_velocity_rad_s, double max_pose_age_s,
                               bool stop_at_target, BusMetrics *metrics);

  bool find_barrier(mab::MD *md, uint16_t node_id, float direction, float step, float max_lead,
                    float &barrier_pos);

  bool calibrate_axis(mab::MD *md, uint16_t node_id, float kp, float kd, float torque,
                      float backoff, SoftLimit &limit);

  bool park_axis(mab::MD *md, uint16_t node_id, float target);

  /// An MD drops to IDLE after 100-200 ms without a CAN frame, so the drives
  /// not being calibrated must keep being addressed.
  void feed_watchdogs(const mab::MD *active);

  void wait_feeding(const mab::MD *active, int ms);

  float apply_limit(mab::MD *md, const SoftLimit &limit, float velocity, AxisReading &position,
                    BusMetrics *metrics);

  config::CanBusConfig bus_cfg_;
  std::vector<config::GimbalConfig> gimbals_;
  mab::Candle *candle_{nullptr};
  std::vector<std::unique_ptr<mab::MD>> pan_motors_;
  std::vector<std::unique_ptr<mab::MD>> tilt_motors_;
  std::vector<SoftLimit> pan_limits_;
  std::vector<SoftLimit> tilt_limits_;
  std::unique_ptr<mab::Pds> pds_;
  std::vector<std::shared_ptr<mab::PowerStage>> power_stages_;
  // The SDK locks USB write and read separately; without this, concurrent
  // callers read each other's replies.
  std::mutex bus_mutex_;
};

} // namespace patronus::comm
