#pragma once

#include "patronus/core/types.hpp"
#include <cstdint>
#include <memory>
#include <vector>

namespace mab
{
class Candle;
class MD;
class Pds;
class PowerStage;
}  // namespace mab

namespace patronus::comm {

/// @brief Configuration parameters for the CANdle motor driver.
struct CandleConfig {
  uint16_t pan_node_id{16};
  uint16_t tilt_node_id{18};
  uint8_t  datarate{1};               ///< 1, 2, 5, or 8 Mbps
  float    max_velocity_rad_s{6.28f}; ///< Maximum velocity command (~1 rev/s)
  uint16_t pds_node_id{100};          ///< PDS module CAN ID (0 to skip PDS init)
};

/// @brief CANdle-based motor driver for pan/tilt gimbal control.
///
/// Wraps the CANdle-SDK's mab::Candle (USB-to-CAN adapter) and two mab::MD
/// (motor controller) instances.  Operates in VELOCITY_PID mode.
class CandleMotor {
 public:
  explicit CandleMotor(const CandleConfig& cfg);
  ~CandleMotor();

  CandleMotor(const CandleMotor&) = delete;
  CandleMotor& operator=(const CandleMotor&) = delete;

  /// @brief Open CANdle device and initialise both motors.
  /// @return true on success.
  [[nodiscard]] bool init();

  /// @brief Set pan/tilt velocity commands in radians per second.
  void set_velocity(float pan_rad_s, float tilt_rad_s);

  /// @brief Read back actual motor velocities.
  /// @return Pair of (pan_rad_s, tilt_rad_s).
  std::pair<float, float> get_velocity();

  /// @brief Read back actual motor positions.
  /// @return Pair of (pan_rad, tilt_rad).
  std::pair<float, float> get_position();

  /// @brief Disable PWM output on both motors and detach the CANdle device.
  void disable();

 private:
  bool enable_pds();

  /// @brief Resolve pan/tilt CAN IDs — try configured IDs first, then discover.
  /// @return Two IDs on success; empty vector on failure.
  std::vector<uint16_t> try_resolve_motor_ids();

  CandleConfig                        cfg_;
  mab::Candle*                        candle_{nullptr};
  std::unique_ptr<mab::MD>            pan_motor_;
  std::unique_ptr<mab::MD>            tilt_motor_;
  std::unique_ptr<mab::Pds>           pds_;
  std::vector<std::shared_ptr<mab::PowerStage>> power_stages_;
};

}  // namespace patronus::comm
