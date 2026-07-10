#pragma once

#include "patronus/core/config.hpp"
#include "patronus/core/types.hpp"

#include <cstdint>
#include <memory>
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
/// Wraps the CANdle-SDK's mab::Candle (USB-to-CAN adapter) and two or more
/// mab::MD (motor controller) instances.  Operates in VELOCITY_PID mode.
/// Supports multiple gimbals on a single CAN bus.
class CandleMotor {
public:
  using GimbalCfg = config::GimbalConfig;
  using BusCfg = config::CanBusConfig;

  /// @brief Construct for a single gimbal (backward-compatible).
  explicit CandleMotor(const GimbalCfg &gimbal, const BusCfg &bus = BusCfg{});

  /// @brief Construct for multiple gimbals on one CANdle bus.
  explicit CandleMotor(const std::vector<GimbalCfg> &gimbals, const BusCfg &bus = BusCfg{});

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

  // ── Per-gimbal API ────────────────────────────────────────────────────

  /// @brief Set pan/tilt velocity for a specific gimbal (rad/s).
  void set_velocity(size_t gimbal, float pan_rad_s, float tilt_rad_s);

  /// @brief Read back actual motor velocities for a specific gimbal.
  /// @return Pair of (pan_rad_s, tilt_rad_s).
  std::pair<float, float> get_velocity(size_t gimbal);

  /// @brief Read back actual motor positions for a specific gimbal.
  /// @return Pair of (pan_rad, tilt_rad).
  std::pair<float, float> get_position(size_t gimbal);

  /// @brief Zero the encoder at the current position for a specific gimbal.
  /// @return true on success.
  [[nodiscard]] bool calibrate_home(size_t gimbal);

  /// @brief Drive a specific gimbal toward home using velocity-mode P-control.
  bool return_to_home(size_t gimbal, float position_gain, float tolerance_rad,
                      float max_velocity_rad_s = 0.0F);

  /// @brief Check whether a specific gimbal is within tolerance of home.
  [[nodiscard]] bool is_at_home(size_t gimbal, float tolerance_rad);

  /// @brief Access raw pan MD for a specific gimbal (diagnostic use).
  mab::MD *pan_motor(size_t gimbal);

  /// @brief Access raw tilt MD for a specific gimbal (diagnostic use).
  mab::MD *tilt_motor(size_t gimbal);

  // ── Legacy single-gimbal API (operates on gimbal 0) ───────────────────

  /// @brief Set pan/tilt velocity on gimbal 0 (rad/s).
  void set_velocity(float pan_rad_s, float tilt_rad_s);

  /// @brief Read back actual motor velocities for gimbal 0.
  /// @return Pair of (pan_rad_s, tilt_rad_s).
  std::pair<float, float> get_velocity();

  /// @brief Read back actual motor positions for gimbal 0.
  /// @return Pair of (pan_rad, tilt_rad).
  std::pair<float, float> get_position();

  /// @brief Zero the encoder at the current position for gimbal 0.
  /// @return true on success.
  [[nodiscard]] bool calibrate_home();

  /// @brief Drive gimbal 0 toward home using velocity-mode P-control.
  bool return_to_home(float position_gain, float tolerance_rad, float max_velocity_rad_s = 0.0F);

  /// @brief Check whether gimbal 0 is within tolerance of home.
  [[nodiscard]] bool is_at_home(float tolerance_rad);

  /// @brief Access raw pan MD for gimbal 0 (diagnostic use).
  mab::MD *pan_motor();

  /// @brief Access raw tilt MD for gimbal 0 (diagnostic use).
  mab::MD *tilt_motor();

  /// @brief Disable PWM output on all motors and detach the CANdle device.
  void disable();

private:
  bool enable_pds();
  // Discover MDs on bus and verify all configured CAN IDs are present
  std::vector<uint16_t> try_resolve_motor_ids();

  BusCfg bus_cfg_;
  std::vector<GimbalCfg> gimbals_;
  mab::Candle *candle_{nullptr};
  std::vector<std::unique_ptr<mab::MD>> pan_motors_;
  std::vector<std::unique_ptr<mab::MD>> tilt_motors_;
  std::vector<float> tilt_gear_ratios_;
  std::unique_ptr<mab::Pds> pds_;
  std::vector<std::shared_ptr<mab::PowerStage>> power_stages_;
};

} // namespace patronus::comm
