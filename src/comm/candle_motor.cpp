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
