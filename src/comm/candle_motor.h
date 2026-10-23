#pragma once

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

/// @brief CANdle-based motor driver for pan/tilt gimbal control.
///
/// Wraps the CANdle-SDK's mab::Candle (USB-to-CAN adapter) and two
/// mab::MD (motor controller) instances per gimbal. Operates in
/// VELOCITY_PID mode. Supports multiple gimbals on a single CAN bus.
///
/// @note The per-gimbal calls are safe to make from one thread per gimbal: each
///       CAN request/response exchange is serialised on the shared CANdle.
class CandleMotor {
public:
  /// @brief Construct for one or more gimbals on one CANdle bus.
  /// @param gimbals Per-gimbal motor IDs and velocity limits (may be empty).
  /// @param bus     Shared CAN bus settings.
  explicit CandleMotor(const std::vector<config::GimbalConfig> &gimbals,
                       const config::CanBusConfig &bus = config::CanBusConfig{});

  ~CandleMotor();

  CandleMotor(const CandleMotor &) = delete;
  CandleMotor &operator=(const CandleMotor &) = delete;

  /// @brief Open CANdle device and initialise all gimbal motors.
  /// @return true on success.
  [[nodiscard]] bool init();

  /// @brief Number of gimbals managed by this instance.
  [[nodiscard]] size_t gimbal_count() const {
    return gimbals_.size();
  }

  /// @brief Set pan/tilt velocity for a specific gimbal (rad/s).
  ///        Clamped to the gimbal's max_velocity_rad_s. Hot path: must
  ///        return within 1 ms. After a boot_sequence() run, a command that
  ///        would push a calibrated axis further past its software limit is
  ///        zeroed (moving away from the limit is always allowed), which
  ///        costs one position read per axis on the bus.
  void set_velocity(size_t gimbal, float pan_rad_s, float tilt_rad_s);

  /// @brief Read back actual motor velocities for a specific gimbal.
  /// @return Pair of (pan_rad_s, tilt_rad_s).
  std::pair<float, float> get_velocity(size_t gimbal);

  /// @brief Read back actual motor positions for a specific gimbal.
  /// @return Pair of (pan_rad, tilt_rad).
  std::pair<float, float> get_position(size_t gimbal);

  /// @brief Run the boot-up calibration sequence (from the standalone turret
  ///        boot tool).
  ///
  /// For every gimbal with calibrate_on_boot_ set, each axis is zeroed at its
  /// current pose, driven into both physical end stops with a low-stiffness
  /// impedance controller, and the recorded barrier positions are stored as
  /// software limits set 'backoff' inside the barriers. The centre of those
  /// limits becomes home for return_to_home(). The axis is put back into its
  /// configured VELOCITY_PID mode, enabled, and parked at the centre with a
  /// closed position loop, ready for the tracking threads.
  ///
  /// Blocking: up to ~30 s per axis while searching for a barrier, plus up
  /// to ~10 s to park. Called
  /// once after init(), before any tracking thread starts. Axes that fail
  /// calibration are disabled and left uncalibrated; velocity commands to
  /// uncalibrated axes are not position-limited (find_barrier() results are
  /// only recorded for axes calibrated in this boot).
  /// @return true when everything requested (or nothing at all) calibrated.
  [[nodiscard]] bool boot_sequence();

  /// @brief Drive a specific gimbal toward home using velocity-mode P-control.
  ///        Home is the centre of each calibrated axis's range, or the start
  ///        pose (0) for an axis not calibrated this boot.
  /// @param position_gain      P-gain (rad/s per rad of position error).
  /// @param tolerance_rad      Position deadband for "at home".
  /// @param max_velocity_rad_s Upper bound; 0 uses the gimbal's configured max.
  /// @return true when the gimbal is within tolerance of home.
  bool return_to_home(size_t gimbal, float position_gain, float tolerance_rad,
                      float max_velocity_rad_s = 0.0F);

  /// @brief Access raw pan MD for a specific gimbal (diagnostic use).
  /// @warning Calls on the returned MD bypass the bus lock; do not use while
  ///          tracking threads are running.
  mab::MD *pan_motor(size_t gimbal);

  /// @brief Access raw tilt MD for a specific gimbal (diagnostic use).
  /// @warning Calls on the returned MD bypass the bus lock; do not use while
  ///          tracking threads are running.
  mab::MD *tilt_motor(size_t gimbal);

  /// @brief Disable PWM output on all motors and detach the CANdle device.
  ///        Idempotent. Also called by the destructor.
  void disable();

private:
  bool enable_pds();

  /// Software limit for one axis, in radians relative to the zero set at the
  /// start of its calibration. Filled in by boot_sequence().
  struct SoftLimit {
    bool calibrated{false};
    float min_pos{0.0F}; ///< Lower stop + backoff.
    float max_pos{0.0F}; ///< Upper stop - backoff.
    float home_pos{0.0F}; ///< Centre of [min_pos, max_pos]; 0 until calibrated.
  };

  /// Drive one axis in one direction (+1 or -1) until it stalls against a
  /// physical barrier. Returns true and writes the stall position to
  /// barrier_pos. Caller must hold bus_mutex_ and have the axis in IMPEDANCE
  /// mode with suitable impedance params and torque limit.
  bool find_barrier(mab::MD *md, uint16_t node_id, float direction, float step,
                    float max_lead, float &barrier_pos);

  /// Find both barriers for one axis and store the soft limits. Caller must
  /// hold bus_mutex_.
  bool calibrate_axis(mab::MD *md, uint16_t node_id, float kp, float kd, float torque,
                      float backoff, SoftLimit &limit);

  /// Drive one axis to 'target' (rad) with a P+I position loop on top of its
  /// VELOCITY_PID mode. Returns false if it never got within tolerance.
  /// Caller must hold bus_mutex_ and have the axis enabled in VELOCITY_PID.
  bool park_axis(mab::MD *md, uint16_t node_id, float target);

  /// Zero a velocity command that would push a calibrated axis past its
  /// software limit. Caller must hold bus_mutex_.
  float apply_limit(mab::MD *md, const SoftLimit &limit, float velocity);

  config::CanBusConfig bus_cfg_;
  std::vector<config::GimbalConfig> gimbals_;
  mab::Candle *candle_{nullptr};
  std::vector<std::unique_ptr<mab::MD>> pan_motors_;
  std::vector<std::unique_ptr<mab::MD>> tilt_motors_;
  std::vector<SoftLimit> pan_limits_;  ///< Parallel to gimbals_; pan axes.
  std::vector<SoftLimit> tilt_limits_; ///< Parallel to gimbals_; tilt axes.
  std::unique_ptr<mab::Pds> pds_;
  std::vector<std::shared_ptr<mab::PowerStage>> power_stages_;
  // Serialises CAN exchanges. The SDK locks the USB write and the USB read
  // separately, so concurrent callers would otherwise read each other's replies.
  std::mutex bus_mutex_;
};

} // namespace patronus::comm
