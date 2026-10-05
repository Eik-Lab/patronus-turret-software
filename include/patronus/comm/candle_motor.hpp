#pragma once

#include "patronus/core/config.hpp"
#include "patronus/core/types.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
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
  ///        return within 1 ms.
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
  /// @param position_gain      P-gain (rad/s per rad of position error).
  /// @param tolerance_rad      Position deadband for "at home".
  /// @param max_velocity_rad_s Upper bound; 0 uses the gimbal's configured max.
  /// @return true when the gimbal is within tolerance of home.
  bool return_to_home(size_t gimbal, float position_gain, float tolerance_rad,
                      float max_velocity_rad_s = 0.0F);

  /// @brief Access raw pan MD for a specific gimbal (diagnostic use).
  mab::MD *pan_motor(size_t gimbal);

  /// @brief Access raw tilt MD for a specific gimbal (diagnostic use).
  mab::MD *tilt_motor(size_t gimbal);

  /// @brief Disable PWM output on all motors and detach the CANdle device.
  ///        Idempotent. Also called by the destructor.
  void disable();

private:
  bool enable_pds();
  // Discover MDs on the bus; empty result means a required CAN ID is missing.
  std::vector<uint16_t> try_resolve_motor_ids();

  config::CanBusConfig bus_cfg_;
  std::vector<config::GimbalConfig> gimbals_;
  mab::Candle *candle_{nullptr};
  std::vector<std::unique_ptr<mab::MD>> pan_motors_;
  std::vector<std::unique_ptr<mab::MD>> tilt_motors_;
  std::unique_ptr<mab::Pds> pds_;
  std::vector<std::shared_ptr<mab::PowerStage>> power_stages_;
};

} // namespace patronus::comm
