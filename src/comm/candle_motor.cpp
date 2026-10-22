#include "comm/candle_motor.h"

#include "log.h"

#include <algorithm>
#include <candlelib.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace patronus::comm {

static mab::CANdleDatarate_E to_datarate(uint8_t v) {
  if (v == 2)
    return mab::CAN_DATARATE_2M;
  if (v == 5)
    return mab::CAN_DATARATE_5M;
  if (v == 8)
    return mab::CAN_DATARATE_8M;
  return mab::CAN_DATARATE_1M;
}

// ── Lifecycle ────────────────────────────────────────────────────────────────

CandleMotor::CandleMotor(const std::vector<config::GimbalConfig> &gimbals,
                         const config::CanBusConfig &bus)
  : bus_cfg_(bus), gimbals_(gimbals) {
  pan_limits_.resize(gimbals_.size());
  tilt_limits_.resize(gimbals_.size());
}

CandleMotor::~CandleMotor() {
  disable();
}

// ── PDS power stage ──────────────────────────────────────────────────────────

bool CandleMotor::enable_pds() {
  if (bus_cfg_.pds_node_id_ == 0) {
    LOG(Debug, "CandleMotor: PDS init skipped (pds_node_id_=0)");
    return true;
  }

  pds_ = std::make_unique<mab::Pds>(bus_cfg_.pds_node_id_, candle_);
  if (pds_->init() != mab::PdsModule::error_E::OK) {
    LOG(Warning, "CandleMotor: PDS" << bus_cfg_.pds_node_id_
                                    << " not found on bus — continuing without PDS");
    pds_.reset();
    return true;
  }
  LOG(Info, "CandleMotor: PDS" << bus_cfg_.pds_node_id_ << " initialised");

  const auto modules = pds_->getModules();
  const mab::socketIndex_E sockets[] = {
    mab::socketIndex_E::SOCKET_1, mab::socketIndex_E::SOCKET_2, mab::socketIndex_E::SOCKET_3,
    mab::socketIndex_E::SOCKET_4, mab::socketIndex_E::SOCKET_5, mab::socketIndex_E::SOCKET_6,
  };
  const mab::moduleType_E types[] = {
    modules.moduleTypeSocket1, modules.moduleTypeSocket2, modules.moduleTypeSocket3,
    modules.moduleTypeSocket4, modules.moduleTypeSocket5, modules.moduleTypeSocket6,
  };

  for (size_t i = 0; i < 6; ++i) {
    if (types[i] != mab::moduleType_E::POWER_STAGE)
      continue;
    auto ps = pds_->attachPowerStage(sockets[i]);
    if (ps) {
      ps->enable();
      power_stages_.push_back(ps);
      LOG(Info, "CandleMotor: power stage enabled on socket " << (i + 1));
    }
  }

  if (power_stages_.empty()) {
    LOG(Error, "CandleMotor: no power stages found on PDS" << bus_cfg_.pds_node_id_);
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return true;
}

// ── Initialisation ───────────────────────────────────────────────────────────

bool CandleMotor::init() {
  try {
    candle_ = mab::attachCandle(to_datarate(bus_cfg_.datarate_), mab::candleTypes::busTypes_t::USB);
  } catch (const std::runtime_error &e) {
    LOG(Error, "CandleMotor: failed to attach CANdle device — " << e.what());
    return false;
  }
  if (candle_ == nullptr) {
    LOG(Error, "CandleMotor: failed to attach CANdle device");
    return false;
  }

  if (!enable_pds()) {
    LOG(Error, "CandleMotor: PDS power stage enable failed");
    disable();
    return false;
  }

  pan_motors_.resize(gimbals_.size());
  tilt_motors_.resize(gimbals_.size());

  // Motors are addressed by their configured node ids, with no bus scan; a
  // wrong or absent id surfaces as a failed MD::init() below.
  for (size_t g = 0; g < gimbals_.size(); ++g) {
    pan_motors_[g] = std::make_unique<mab::MD>(gimbals_[g].pan_node_id_, candle_);
    tilt_motors_[g] = std::make_unique<mab::MD>(gimbals_[g].tilt_node_id_, candle_);

    // Per-axis velocity-PID gains from the gimbal's config (system.ini)
    auto setup_md = [&](mab::MD *md, const config::MotorPidConfig &pid) {
      if (md->init() != mab::MD::Error_t::OK) {
        LOG(Error, "CandleMotor: MD" << md->m_canId << " (gimbal " << g
                                     << ") did not respond — check node id");
        return false;
      }

      md->setMotionMode(mab::MdMode_E::VELOCITY_PID);
      md->setVelocityPIDparam(pid.kp_, pid.ki_, pid.kd_, pid.integral_max_);
      md->setMaxTorque(10.0F);
      md->enable();
      LOG(Info, "CandleMotor: MD" << md->m_canId << " velocity PID (kp=" << pid.kp_
                                  << " ki=" << pid.ki_ << " kd=" << pid.kd_
                                  << " integral_max=" << pid.integral_max_ << ")");
      return true;
    };

    if (!setup_md(pan_motors_[g].get(), gimbals_[g].pan_pid_)) {
      disable();
      return false;
    }
    if (!setup_md(tilt_motors_[g].get(), gimbals_[g].tilt_pid_)) {
      disable();
      return false;
    }

    LOG(Info, "CandleMotor: gimbal " << g << " — pan=MD" << pan_motors_[g]->m_canId << " tilt=MD"
                                     << tilt_motors_[g]->m_canId);
  }

  return true;
}

// ── Boot-up calibration (ported from the standalone turret boot tool) ───────

// Calibration parameters, matching the boot tool: pan gears up against
// barriers with a stiff low-torque push, tilt with a soft one. 'backoff' is
// the software-limit margin kept inside each real barrier, in radians.
static constexpr float k_boot_backoff_rad = 0.3F;
static constexpr float k_pan_boot_kp = 10.0F;
static constexpr float k_pan_boot_kd = 0.5F;
static constexpr float k_pan_boot_torque = 2.0F;
static constexpr float k_tilt_boot_kp = 1.0F;
static constexpr float k_tilt_boot_kd = 1.0F;
static constexpr float k_tilt_boot_torque = 1.0F;

float CandleMotor::apply_limit(mab::MD *md, const SoftLimit &limit, float velocity) {
  if (md == nullptr || !limit.calibrated || velocity == 0.0F)
    return velocity;

  // A failed read returns 0, and with the axis position unknown it is not
  // safe to move it. No log here: this runs in the 100 Hz hot path and a
  // failing bus would otherwise flood it.
  const auto pos = md->getPosition();
  if (pos.second != mab::MD::Error_t::OK)
    return 0.0F;

  if (velocity > 0.0F && pos.first >= limit.max_pos)
    return 0.0F;
  if (velocity < 0.0F && pos.first <= limit.min_pos)
    return 0.0F;
  return velocity;
}

bool CandleMotor::find_barrier(mab::MD *md, uint16_t node_id, float direction, float step,
                               float max_lead, float &barrier_pos) {
  constexpr int grace_samples = 30;   // ignore stalls while the motor gets going (~0.3 s)
  constexpr int max_samples = 3000;   // give up after ~30 s
  constexpr int max_read_errors = 20; // give up after this many failed position reads in a row

  // Start from where the motor actually is
  const auto start = md->getPosition();
  if (start.second != mab::MD::Error_t::OK) {
    LOG(Error, "CandleMotor: MD" << node_id << ": could not read position");
    return false;
  }

  float current = start.first;
  float last = current;
  float target = current;
  int stall_count = 0;
  int read_errors = 0;

  for (int sample = 0; sample < max_samples; ++sample) {
    // Move the target along, but never further than max_lead ahead of the
    // motor. Otherwise the target runs away from the motor and cannot be
    // backed off from later.
    target += direction * step;
    if (std::abs(target - current) > max_lead)
      target = current + direction * max_lead;
    md->setTargetPosition(target);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    const auto pos = md->getPosition();
    if (pos.second != mab::MD::Error_t::OK) {
      // A failed read returns 0, so don't use it
      if (++read_errors > max_read_errors) {
        LOG(Error, "CandleMotor: MD" << node_id << ": too many failed position reads");
        return false;
      }
      continue;
    }
    read_errors = 0;
    current = pos.first;

    // If the motor cannot move for 20 samples, it has pushed into a barrier
    if (sample >= grace_samples && std::abs(current - last) < 0.01F)
      ++stall_count;
    else
      stall_count = 0;
    last = current;

    if (stall_count > 20) {
      barrier_pos = current;
      return true;
    }
  }

  LOG(Error, "CandleMotor: MD" << node_id << ": no barrier found (timeout)");
  return false;
}

bool CandleMotor::calibrate_axis(mab::MD *md, uint16_t node_id, float kp, float kd,
                                 float torque, float backoff, SoftLimit &limit) {
  constexpr float step = 0.05F; // calibration speed, rad per 10 ms tick

  // The target may sit ahead of the motor by at most this much: enough to
  // saturate the torque limit against a hard barrier.
  const float max_lead = (kp > 0.0F) ? 2.0F * torque / kp : 1.0F;

  limit.calibrated = false;

  // Zero at the pose the axis is found in: that pose becomes position 0,
  // i.e. home for return_to_home(). The gimbal must be parked where it
  // should call home before boot_sequence() is invoked.
  md->zero();
  md->setMotionMode(mab::MdMode_E::IMPEDANCE);
  md->setImpedanceParams(kp, kd);
  md->setMaxTorque(torque);
  md->enable();

  // Go up to one barrier, then down to the other, and record both
  float upper = 0.0F;
  float lower = 0.0F;
  const bool ok = find_barrier(md, node_id, +1.0F, step, max_lead, upper) &&
                  find_barrier(md, node_id, -1.0F, step, max_lead, lower);

  if (!ok) {
    LOG(Error, "CandleMotor: MD" << node_id << ": calibration FAILED");
    md->disable();
    return false;
  }

  if (upper - lower <= 2.0F * backoff) {
    LOG(Warning, "CandleMotor: MD" << node_id << ": range (" << (upper - lower)
                                   << " rad) is too small for a backoff of " << backoff
                                   << " rad");
    md->disable();
    return false;
  }

  // Software limits sit 'backoff' inside the real barriers
  limit.min_pos = lower + backoff;
  limit.max_pos = upper - backoff;
  limit.calibrated = true;
  LOG(Info, "CandleMotor: MD" << node_id << " calibrated, soft limits: [" << limit.min_pos
                              << ", " << limit.max_pos << "] rad");

  // The axis is at the lower barrier now. Ramp it up to the middle of the
  // range so it is not left loaded against a stop.
  const float centre = (limit.min_pos + limit.max_pos) * 1.0F;
  float target = lower;
  for (int i = 0; i < 2000 && target < centre; ++i) {
    target = std::min(target + step, centre);
    md->setTargetPosition(target);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Hold ~1 s so the motor can get there (also feeds the watchdog)
  for (int i = 0; i < 100; ++i) {
    md->setTargetPosition(centre);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return true;
}

bool CandleMotor::boot_sequence() {
  const std::scoped_lock lock(bus_mutex_);

  bool all_ok = true;
  for (size_t g = 0; g < gimbals_.size(); ++g) {
    if (!gimbals_[g].calibrate_on_boot_)
      continue;

    LOG(Info, "CandleMotor: boot calibration gimbal " << g << " (this zeroes both axes at their "
                                                      "current pose — make sure the gimbal is "
                                                      "parked at its home position)");

    // Put an axis back into its velocity-PID mission configuration after
    // calibration, whether it succeeded or not, so the tracking threads
    // always see enabled motors in the mode the config describes.
    const auto settle = [g](mab::MD *md, const config::MotorPidConfig &pid) {
      if (md == nullptr)
        return;
      md->setMotionMode(mab::MdMode_E::VELOCITY_PID);
      md->setVelocityPIDparam(pid.kp_, pid.ki_, pid.kd_, pid.integral_max_);
      md->setMaxTorque(10.0F);
      md->enable();
      md->setTargetVelocity(0.0F);
      LOG(Info, "CandleMotor: gimbal " << g << " axis rearmed (velocity PID)");
    };

    if (calibrate_axis(pan_motors_[g].get(), gimbals_[g].pan_node_id_, k_pan_boot_kp, k_pan_boot_kd,
                       k_pan_boot_torque, k_boot_backoff_rad, pan_limits_[g]))
      settle(pan_motors_[g].get(), gimbals_[g].pan_pid_);
    else
      all_ok = false;

    if (calibrate_axis(tilt_motors_[g].get(), gimbals_[g].tilt_node_id_, k_tilt_boot_kp,
                       k_tilt_boot_kd, k_tilt_boot_torque, k_boot_backoff_rad, tilt_limits_[g]))
      settle(tilt_motors_[g].get(), gimbals_[g].tilt_pid_);
    else
      all_ok = false;
  }

  return all_ok;
}

// ── Per-gimbal API ───────────────────────────────────────────────────────────

void CandleMotor::set_velocity(size_t gimbal, float pan_rad_s, float tilt_rad_s) {
  if (gimbal >= gimbals_.size())
    return;
  auto *pan = pan_motors_[gimbal].get();
  auto *tilt = tilt_motors_[gimbal].get();
  if (pan == nullptr || tilt == nullptr)
    return;

  const float max_vel = gimbals_[gimbal].max_velocity_rad_s_;
  pan_rad_s = std::clamp(pan_rad_s, -max_vel, max_vel);
  tilt_rad_s = std::clamp(tilt_rad_s, -max_vel, max_vel);

  const std::scoped_lock lock(bus_mutex_);
  // Software limits from boot calibration: a command that would push a
  // calibrated axis past its (backed-off) barrier is zeroed. Moving away
  // from a limit is always allowed. Uncalibrated axes get the plain
  // max_velocity clamp only.
  pan_rad_s = apply_limit(pan, pan_limits_[gimbal], pan_rad_s);
  tilt_rad_s = apply_limit(tilt, tilt_limits_[gimbal], tilt_rad_s);

  pan->setTargetVelocity(pan_rad_s);
  tilt->setTargetVelocity(tilt_rad_s);
}

std::pair<float, float> CandleMotor::get_velocity(size_t gimbal) {
  if (gimbal >= gimbals_.size())
    return {0.0F, 0.0F};
  if (pan_motors_[gimbal] == nullptr || tilt_motors_[gimbal] == nullptr)
    return {0.0F, 0.0F};
  const std::scoped_lock lock(bus_mutex_);
  const auto pv = pan_motors_[gimbal]->getVelocity();
  const auto tv = tilt_motors_[gimbal]->getVelocity();
  return {pv.first, tv.first};
}

std::pair<float, float> CandleMotor::get_position(size_t gimbal) {
  if (gimbal >= gimbals_.size())
    return {0.0F, 0.0F};
  if (pan_motors_[gimbal] == nullptr || tilt_motors_[gimbal] == nullptr)
    return {0.0F, 0.0F};
  const std::scoped_lock lock(bus_mutex_);
  const auto pp = pan_motors_[gimbal]->getPosition();
  const auto tp = tilt_motors_[gimbal]->getPosition();
  return {pp.first, tp.first};
}

bool CandleMotor::return_to_home(size_t gimbal, float position_gain, float tolerance_rad,
                                 float max_velocity_rad_s) {
  if (gimbal >= gimbals_.size())
    return true;
  auto *pan = pan_motors_[gimbal].get();
  auto *tilt = tilt_motors_[gimbal].get();
  if (pan == nullptr || tilt == nullptr)
    return true;

  const float limit = (max_velocity_rad_s > 0.0F)
                        ? std::min(max_velocity_rad_s, gimbals_[gimbal].max_velocity_rad_s_)
                        : gimbals_[gimbal].max_velocity_rad_s_;

  const auto [pp, pt] = get_position(gimbal);
  const float pan_vel = std::clamp(-pp * position_gain, -limit, limit);
  const float tilt_vel = std::clamp(-pt * position_gain, -limit, limit);
  set_velocity(gimbal, pan_vel, tilt_vel);
  return std::abs(pp) < tolerance_rad && std::abs(pt) < tolerance_rad;
}

mab::MD *CandleMotor::pan_motor(size_t gimbal) {
  if (gimbal >= gimbals_.size())
    return nullptr;
  return pan_motors_[gimbal].get();
}

mab::MD *CandleMotor::tilt_motor(size_t gimbal) {
  if (gimbal >= gimbals_.size())
    return nullptr;
  return tilt_motors_[gimbal].get();
}

// ── Disable ──────────────────────────────────────────────────────────────────

void CandleMotor::disable() {
  const std::scoped_lock lock(bus_mutex_);
  // Silence SDK logging while tearing down motors
  const auto prev_verbosity = Logger::g_m_verbosity;
  Logger::g_m_verbosity = Logger::Verbosity_E::SILENT;

  for (auto &md : pan_motors_)
    if (md)
      md->disable();
  for (auto &md : tilt_motors_)
    if (md)
      md->disable();

  Logger::g_m_verbosity = prev_verbosity;

  pan_motors_.clear();
  tilt_motors_.clear();
  power_stages_.clear();
  pds_.reset();
  if (candle_ != nullptr)
    mab::detachCandle(candle_);
  candle_ = nullptr;
}

} // namespace patronus::comm
