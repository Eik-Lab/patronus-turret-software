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

CandleMotor::CandleMotor(const std::vector<config::GimbalConfig> &gimbals,
                         const config::CanBusConfig &bus)
  : bus_cfg_(bus), gimbals_(gimbals) {
  pan_limits_.resize(gimbals_.size());
  tilt_limits_.resize(gimbals_.size());
}

CandleMotor::~CandleMotor() {
  disable();
}

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

  for (size_t g = 0; g < gimbals_.size(); ++g) {
    pan_motors_[g] = std::make_unique<mab::MD>(gimbals_[g].pan_node_id_, candle_);
    tilt_motors_[g] = std::make_unique<mab::MD>(gimbals_[g].tilt_node_id_, candle_);

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

// Pan gets a stiff push into its end stops, tilt (direct drive) a soft one.
static constexpr float k_boot_backoff_rad = 0.3F;
static constexpr float k_pan_boot_kp = 10.0F;
static constexpr float k_pan_boot_kd = 0.5F;
static constexpr float k_pan_boot_torque = 5.0F;
static constexpr float k_tilt_boot_kp = 1.0F;
static constexpr float k_tilt_boot_kd = 1.0F;
static constexpr float k_tilt_boot_torque = 3.0F;

static bool mode_is(mab::MD *md, mab::MdMode_E mode) {
  md->m_mdRegisters.motionModeStatus = 0;
  return md->readRegisters(md->m_mdRegisters.motionModeStatus) == mab::MD::Error_t::OK &&
         md->m_mdRegisters.motionModeStatus.value == mode;
}

// A mode written to an enabled drive, or right after disable()/zero(), often
// does not take: the drive stays IDLE and ignores targets, which calibration
// reads as an instant stall. So disable, settle, write, read back, retry.
template <typename Configure, typename Wait>
static bool arm_drive(mab::MD *md, mab::MdMode_E mode, const Configure &configure,
                      const Wait &wait) {
  constexpr int attempts = 3;
  constexpr int verify_polls = 10; // ~100 ms for the mode to show up after enable()

  for (int attempt = 0; attempt < attempts; ++attempt) {
    md->disable();
    wait(100);
    md->setMotionMode(mode);
    configure();
    md->enable();

    for (int poll = 0; poll < verify_polls; ++poll) {
      if (mode_is(md, mode))
        return true;
      wait(10);
    }
  }

  md->disable();
  return false;
}

void CandleMotor::feed_watchdogs(const mab::MD *active) {
  for (auto *motors : {&pan_motors_, &tilt_motors_})
    for (auto &md : *motors)
      if (md != nullptr && md.get() != active)
        md->setTargetVelocity(0.0F);
}

void CandleMotor::wait_feeding(const mab::MD *active, int ms) {
  for (int elapsed = 0; elapsed < ms; elapsed += 10) {
    feed_watchdogs(active);
    std::this_thread::sleep_for(std::chrono::milliseconds(std::min(10, ms - elapsed)));
  }
}

float CandleMotor::apply_limit(mab::MD *md, const SoftLimit &limit, float velocity) {
  if (md == nullptr || !limit.calibrated || velocity == 0.0F)
    return velocity;

  // Unknown position: not safe to move. No log, this is the hot path.
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
  constexpr int grace_samples = 50;   // ignore stalls while the motor gets going (~0.5 s)
  constexpr int stall_samples = 40;   // barely moving this long (~0.4 s) = barrier
  constexpr int max_samples = 3000;   // give up after ~30 s
  constexpr int max_read_errors = 20; // give up after this many failed position reads in a row

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
    // Capped lead, or the target runs away and cannot be backed off from.
    target += direction * step;
    if (std::abs(target - current) > max_lead)
      target = current + direction * max_lead;
    md->setTargetPosition(target);

    wait_feeding(md, 10);

    const auto pos = md->getPosition();
    if (pos.second != mab::MD::Error_t::OK) {
      // A failed read returns 0.
      if (++read_errors > max_read_errors) {
        LOG(Error, "CandleMotor: MD" << node_id << ": too many failed position reads");
        return false;
      }
      continue;
    }
    read_errors = 0;
    current = pos.first;

    if (sample >= grace_samples && std::abs(current - last) < 0.01F)
      ++stall_count;
    else
      stall_count = 0;
    last = current;

    if (stall_count > stall_samples) {
      barrier_pos = current;
      return true;
    }
  }

  LOG(Error, "CandleMotor: MD" << node_id << ": no barrier found (timeout)");
  return false;
}

bool CandleMotor::calibrate_axis(mab::MD *md, uint16_t node_id, float kp, float kd, float torque,
                                 float backoff, SoftLimit &limit) {
  constexpr float step = 0.05F; // calibration speed, rad per 10 ms tick

  // Just enough lead to saturate the torque limit.
  const float max_lead = (kp > 0.0F) ? 2.0F * torque / kp : 1.0F;

  limit.calibrated = false;

  // Limits are relative to where the axis starts.
  md->disable();
  md->zero();
  if (!arm_drive(
        md, mab::MdMode_E::IMPEDANCE,
        [&] {
          md->setImpedanceParams(kp, kd);
          md->setMaxTorque(torque);
        },
        [&](int ms) {
          wait_feeding(md, ms);
        })) {
    LOG(Error, "CandleMotor: MD" << node_id << ": IMPEDANCE mode did not take (mode "
                                 << static_cast<int>(md->m_mdRegisters.motionModeStatus.value)
                                 << ") — not calibrating");
    return false;
  }

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
                                   << " rad) is too small for a backoff of " << backoff << " rad");
    md->disable();
    return false;
  }

  limit.min_pos = lower + backoff;
  limit.max_pos = upper - backoff;
  limit.calibrated = true;
  LOG(Info, "CandleMotor: MD" << node_id << " calibrated, soft limits: [" << limit.min_pos << ", "
                              << limit.max_pos << "] rad");

  // park_axis() moves it to the centre later: an impedance ramp here stopped
  // short, the soft spring cannot beat friction and gravity.
  limit.home_pos = (limit.min_pos + limit.max_pos) * 0.5F;
  md->setTargetPosition(lower);

  return true;
}

bool CandleMotor::park_axis(mab::MD *md, uint16_t node_id, float target) {
  constexpr float gain = 2.0F;        // rad/s per rad of position error
  constexpr float i_gain = 1.0F;      // rad/s per rad*s of accumulated error
  constexpr float max_vel = 1.0F;     // rad/s, parking speed limit
  constexpr float tolerance = 0.02F;  // rad, close enough to the centre
  constexpr int settle_samples = 20;  // inside tolerance this long (~0.2 s) = parked
  constexpr int max_samples = 1000;   // give up after ~10 s
  constexpr int max_read_errors = 20; // give up after this many failed position reads in a row
  constexpr int stuck_samples = 100;  // no progress for ~1 s = axis is not responding
  constexpr float stuck_rad = 0.02F;  // ...where progress means moving this far
  constexpr float dt = 0.01F;

  // The integral pushes through friction and gravity, even on axes whose
  // velocity PID has none.
  float integral = 0.0F;
  float err = 0.0F;
  int in_tol = 0;
  int read_errors = 0;
  int stuck = 0;
  bool have_anchor = false;
  float anchor = 0.0F; // position at the start of the current no-progress window

  for (int sample = 0; sample < max_samples; ++sample) {
    const auto pos = md->getPosition();
    if (pos.second != mab::MD::Error_t::OK) {
      // A failed read returns 0.
      if (++read_errors > max_read_errors) {
        LOG(Error,
            "CandleMotor: MD" << node_id << ": too many failed position reads while parking");
        md->setTargetVelocity(0.0F);
        return false;
      }
      wait_feeding(md, 10);
      continue;
    }
    read_errors = 0;

    err = target - pos.first;

    // Wrong mode, torque limit or jammed: give up early.
    if (!have_anchor || std::abs(pos.first - anchor) >= stuck_rad) {
      have_anchor = true;
      anchor = pos.first;
      stuck = 0;
    } else if (std::abs(err) >= tolerance && ++stuck >= stuck_samples) {
      md->setTargetVelocity(0.0F);
      LOG(Warning, "CandleMotor: MD" << node_id << ": not moving while parking (" << err
                                     << " rad from the centre) — left where it is");
      return false;
    }

    if (std::abs(err) < tolerance) {
      if (++in_tol >= settle_samples) {
        md->setTargetVelocity(0.0F);
        LOG(Info, "CandleMotor: MD" << node_id << " parked at centre " << target << " rad");
        return true;
      }
    } else {
      in_tol = 0;
    }

    integral = std::clamp(integral + err * dt, -max_vel / i_gain, max_vel / i_gain);
    md->setTargetVelocity(std::clamp(gain * err + i_gain * integral, -max_vel, max_vel));
    wait_feeding(md, 10);
  }

  md->setTargetVelocity(0.0F);
  LOG(Warning, "CandleMotor: MD" << node_id << ": did not reach the centre (" << err
                                 << " rad short) — parked where it stopped");
  return false;
}

bool CandleMotor::boot_sequence() {
  const std::scoped_lock lock(bus_mutex_);

  bool all_ok = true;
  for (size_t g = 0; g < gimbals_.size(); ++g) {
    if (!gimbals_[g].calibrate_on_boot_)
      continue;

    LOG(Info, "CandleMotor: boot calibration gimbal "
                << g << " (both axes sweep end stop to end stop, then park at the centre)");

    const auto settle = [this, g](mab::MD *md, uint16_t node_id,
                                  const config::MotorPidConfig &pid) {
      if (md == nullptr)
        return false;
      if (!arm_drive(
            md, mab::MdMode_E::VELOCITY_PID,
            [&] {
              md->setVelocityPIDparam(pid.kp_, pid.ki_, pid.kd_, pid.integral_max_);
              md->setMaxTorque(10.0F);
              md->setTargetVelocity(0.0F);
            },
            [&](int ms) {
              wait_feeding(md, ms);
            })) {
        LOG(Error, "CandleMotor: MD" << node_id << ": VELOCITY_PID mode did not take (mode "
                                     << static_cast<int>(md->m_mdRegisters.motionModeStatus.value)
                                     << ") — axis disabled");
        return false;
      }
      LOG(Info, "CandleMotor: gimbal " << g << " axis rearmed (velocity PID)");
      return true;
    };

    // A parking miss is fine: return_to_home() finishes the job.
    if (calibrate_axis(pan_motors_[g].get(), gimbals_[g].pan_node_id_, k_pan_boot_kp, k_pan_boot_kd,
                       k_pan_boot_torque, k_boot_backoff_rad, pan_limits_[g]) &&
        settle(pan_motors_[g].get(), gimbals_[g].pan_node_id_, gimbals_[g].pan_pid_))
      park_axis(pan_motors_[g].get(), gimbals_[g].pan_node_id_, pan_limits_[g].home_pos);
    else
      all_ok = false;

    if (calibrate_axis(tilt_motors_[g].get(), gimbals_[g].tilt_node_id_, k_tilt_boot_kp,
                       k_tilt_boot_kd, k_tilt_boot_torque, k_boot_backoff_rad, tilt_limits_[g]) &&
        settle(tilt_motors_[g].get(), gimbals_[g].tilt_node_id_, gimbals_[g].tilt_pid_))
      park_axis(tilt_motors_[g].get(), gimbals_[g].tilt_node_id_, tilt_limits_[g].home_pos);
    else
      all_ok = false;
  }

  return all_ok;
}

std::pair<float, float> CandleMotor::set_velocity(size_t gimbal, float pan_rad_s,
                                                  float tilt_rad_s) {
  if (gimbal >= gimbals_.size())
    return {0.0F, 0.0F};
  auto *pan = pan_motors_[gimbal].get();
  auto *tilt = tilt_motors_[gimbal].get();
  if (pan == nullptr || tilt == nullptr)
    return {0.0F, 0.0F};

  const auto &cfg = gimbals_[gimbal];
  pan_rad_s = std::clamp(pan_rad_s * cfg.pan_velocity_scale_, -cfg.pan_max_velocity_rad_s_,
                         cfg.pan_max_velocity_rad_s_);
  tilt_rad_s = std::clamp(tilt_rad_s * cfg.tilt_velocity_scale_, -cfg.tilt_max_velocity_rad_s_,
                          cfg.tilt_max_velocity_rad_s_);

  const std::scoped_lock lock(bus_mutex_);
  pan_rad_s = apply_limit(pan, pan_limits_[gimbal], pan_rad_s);
  tilt_rad_s = apply_limit(tilt, tilt_limits_[gimbal], tilt_rad_s);

  pan->setTargetVelocity(pan_rad_s);
  tilt->setTargetVelocity(tilt_rad_s);
  return {pan_rad_s, tilt_rad_s};
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

std::pair<float, float> CandleMotor::home_position(size_t gimbal) const {
  if (gimbal >= gimbals_.size())
    return {0.0F, 0.0F};
  return {pan_limits_[gimbal].home_pos, tilt_limits_[gimbal].home_pos};
}

bool CandleMotor::return_to_home(size_t gimbal, float position_gain, float tolerance_rad,
                                 float max_velocity_rad_s) {
  const auto [home_pan, home_tilt] = home_position(gimbal);
  return drive_to(gimbal, home_pan, home_tilt, position_gain, tolerance_rad, max_velocity_rad_s);
}

bool CandleMotor::drive_to(size_t gimbal, float pan_rad, float tilt_rad, float position_gain,
                           float tolerance_rad, float max_velocity_rad_s) {
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
  const float pan_err = pp - pan_rad;
  const float tilt_err = pt - tilt_rad;
  const float pan_vel = std::clamp(-pan_err * position_gain, -limit, limit);
  const float tilt_vel = std::clamp(-tilt_err * position_gain, -limit, limit);
  set_velocity(gimbal, pan_vel, tilt_vel);
  return std::abs(pan_err) < tolerance_rad && std::abs(tilt_err) < tolerance_rad;
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

void CandleMotor::disable() {
  const std::scoped_lock lock(bus_mutex_);
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
