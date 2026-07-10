#include "patronus/comm/candle_motor.hpp"

#include <algorithm>
#include <candlelib.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <logger.hpp>
#include <stdexcept>
#include <thread>
#include <unordered_set>

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

// ── Constructors ─────────────────────────────────────────────────────────────

CandleMotor::CandleMotor(const GimbalCfg &gimbal, const BusCfg &bus)
  : bus_cfg_(bus), gimbals_({gimbal}) {
}

CandleMotor::CandleMotor(const std::vector<GimbalCfg> &gimbals, const BusCfg &bus)
  : bus_cfg_(bus), gimbals_(gimbals) {
  if (gimbals_.empty())
    gimbals_.push_back(GimbalCfg{});
}

CandleMotor::~CandleMotor() {
  disable();
}

// ── PDS power stage ──────────────────────────────────────────────────────────

bool CandleMotor::enable_pds() {
  if (bus_cfg_.pds_node_id_ == 0) {
    std::printf("CandleMotor: PDS init skipped (pds_node_id_=0)\n");
    return true;
  }

  pds_ = std::make_unique<mab::Pds>(bus_cfg_.pds_node_id_, candle_);
  if (pds_->init() != mab::PdsModule::error_E::OK) {
    std::fprintf(stderr, "CandleMotor: PDS%u not found on bus — continuing without PDS\n",
                 bus_cfg_.pds_node_id_);
    pds_.reset();
    return true;
  }
  std::printf("CandleMotor: PDS%u initialised\n", bus_cfg_.pds_node_id_);

  auto modules = pds_->getModules();
  mab::socketIndex_E sockets[] = {
    mab::socketIndex_E::SOCKET_1, mab::socketIndex_E::SOCKET_2, mab::socketIndex_E::SOCKET_3,
    mab::socketIndex_E::SOCKET_4, mab::socketIndex_E::SOCKET_5, mab::socketIndex_E::SOCKET_6,
  };
  mab::moduleType_E types[] = {
    modules.moduleTypeSocket1, modules.moduleTypeSocket2, modules.moduleTypeSocket3,
    modules.moduleTypeSocket4, modules.moduleTypeSocket5, modules.moduleTypeSocket6,
  };

  for (size_t i = 0; i < 6; ++i) {
    if (types[i] == mab::moduleType_E::POWER_STAGE) {
      auto ps = pds_->attachPowerStage(sockets[i]);
      if (ps) {
        ps->enable();
        power_stages_.push_back(ps);
        std::printf("CandleMotor: power stage enabled on socket %zu\n", i + 1);
      }
    }
  }

  if (power_stages_.empty()) {
    std::fprintf(stderr, "CandleMotor: no power stages found on PDS%u\n", bus_cfg_.pds_node_id_);
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return true;
}

// ── Motor discovery ──────────────────────────────────────────────────────────

std::vector<uint16_t> CandleMotor::try_resolve_motor_ids() {
  auto discovered = mab::MD::discoverMDs(candle_);

  size_t needed = gimbals_.size() * 2;
  if (discovered.size() < needed) {
    std::fprintf(stderr, "CandleMotor: found %zu MD(s) on bus — need %zu for %zu gimbal(s)\n",
                 discovered.size(), needed, gimbals_.size());
    for (auto id : discovered)
      std::fprintf(stderr, "  — MD%u\n", id);
    return {};
  }

  std::printf("CandleMotor: discovered %zu MDs on bus\n", discovered.size());
  for (auto id : discovered)
    std::printf("  — MD%u\n", id);

  // Build a lookup set for fast membership check
  std::unordered_set<uint16_t> avail(discovered.begin(), discovered.end());

  // Verify every configured node ID exists on the bus
  for (size_t g = 0; g < gimbals_.size(); ++g) {
    if (!avail.count(gimbals_[g].pan_node_id_)) {
      std::fprintf(stderr, "CandleMotor: configured pan node %u (gimbal %zu) not found on bus\n",
                   gimbals_[g].pan_node_id_, g);
      return {};
    }
    if (!avail.count(gimbals_[g].tilt_node_id_)) {
      std::fprintf(stderr, "CandleMotor: configured tilt node %u (gimbal %zu) not found on bus\n",
                   gimbals_[g].tilt_node_id_, g);
      return {};
    }
  }

  return discovered;
}

// ── Initialisation ───────────────────────────────────────────────────────────

bool CandleMotor::init() {
  try {
    candle_ = mab::attachCandle(to_datarate(bus_cfg_.datarate_), mab::candleTypes::busTypes_t::USB);
  } catch (const std::runtime_error &e) {
    std::fprintf(stderr, "CandleMotor: failed to attach CANdle device — %s\n", e.what());
    return false;
  }
  if (!candle_) {
    std::fprintf(stderr, "CandleMotor: failed to attach CANdle device\n");
    return false;
  }

  if (!enable_pds()) {
    std::fprintf(stderr, "CandleMotor: PDS power stage enable failed\n");
    disable();
    return false;
  }

  auto ids = try_resolve_motor_ids();
  if (ids.empty()) {
    disable();
    return false;
  }

  // Create motor objects — match configured node IDs against discovered bus IDs
  pan_motors_.resize(gimbals_.size());
  tilt_motors_.resize(gimbals_.size());
  tilt_gear_ratios_.resize(gimbals_.size(), 1.0F);

  for (size_t g = 0; g < gimbals_.size(); ++g) {
    pan_motors_[g] = std::make_unique<mab::MD>(gimbals_[g].pan_node_id_, candle_);
    tilt_motors_[g] = std::make_unique<mab::MD>(gimbals_[g].tilt_node_id_, candle_);
  }

  // Initialise all motors with shared velocity-PID settings
  for (size_t g = 0; g < gimbals_.size(); ++g) {
    for (auto *md : {pan_motors_[g].get(), tilt_motors_[g].get()}) {
      if (md->init() != mab::MD::Error_t::OK) {
        std::fprintf(stderr, "CandleMotor: MD%u init failed\n", md->m_canId);
        disable();
        return false;
      }

      md->setMotionMode(mab::MdMode_E::VELOCITY_PID);
      md->setVelocityPIDparam(2.0F, 0.1F, 0.0F, 0.5F);
      md->setMaxTorque(10.0F);
      md->enable();
    }

    // Read tilt gear ratio for diagnostic reporting
    auto &reg = tilt_motors_[g]->m_mdRegisters.motorGearRatio;
    tilt_motors_[g]->readRegister(reg);
    float ratio = reg.value;
    tilt_gear_ratios_[g] = (ratio > 0.0F) ? ratio : 1.0F;

    std::printf("CandleMotor: gimbal %zu — pan=MD%u tilt=MD%u  tilt gear %.1f:1\n", g,
                pan_motors_[g]->m_canId, tilt_motors_[g]->m_canId, tilt_gear_ratios_[g]);
  }

  return true;
}

// ── Per-gimbal API ───────────────────────────────────────────────────────────

// HOT PATH: called every frame (~100 Hz). Must return within 1 ms.
void CandleMotor::set_velocity(size_t gimbal, float pan_rad_s, float tilt_rad_s) {
  if (gimbal >= gimbals_.size())
    return;
  auto *pan = pan_motors_[gimbal].get();
  auto *tilt = tilt_motors_[gimbal].get();
  if (!pan || !tilt)
    return;

  float max_vel = gimbals_[gimbal].max_velocity_rad_s_;
  pan_rad_s = std::clamp(pan_rad_s, -max_vel, max_vel);
  tilt_rad_s = std::clamp(tilt_rad_s, -max_vel, max_vel);

  pan->setTargetVelocity(pan_rad_s);
  tilt->setTargetVelocity(tilt_rad_s);
}

std::pair<float, float> CandleMotor::get_velocity(size_t gimbal) {
  if (gimbal >= gimbals_.size())
    return {0, 0};
  if (!pan_motors_[gimbal] || !tilt_motors_[gimbal])
    return {0, 0};
  auto pv = pan_motors_[gimbal]->getVelocity();
  auto tv = tilt_motors_[gimbal]->getVelocity();
  return {pv.first, tv.first};
}

std::pair<float, float> CandleMotor::get_position(size_t gimbal) {
  if (gimbal >= gimbals_.size())
    return {0, 0};
  if (!pan_motors_[gimbal] || !tilt_motors_[gimbal])
    return {0, 0};
  auto pp = pan_motors_[gimbal]->getPosition();
  auto tp = tilt_motors_[gimbal]->getPosition();
  return {pp.first, tp.first};
}

bool CandleMotor::calibrate_home(size_t gimbal) {
  if (gimbal >= gimbals_.size())
    return false;
  auto *pan = pan_motors_[gimbal].get();
  auto *tilt = tilt_motors_[gimbal].get();
  if (!pan || !tilt)
    return false;

  bool ok = true;
  for (auto *md : {pan, tilt}) {
    ok &= (md->zero() == mab::MD::Error_t::OK);
    ok &= (md->save() == mab::MD::Error_t::OK);
  }
  return ok;
}

bool CandleMotor::return_to_home(size_t gimbal, float position_gain, float tolerance_rad,
                                 float max_velocity_rad_s) {
  if (gimbal >= gimbals_.size())
    return true;
  auto *pan = pan_motors_[gimbal].get();
  auto *tilt = tilt_motors_[gimbal].get();
  if (!pan || !tilt)
    return true;

  float limit = (max_velocity_rad_s > 0.0F)
                  ? std::min(max_velocity_rad_s, gimbals_[gimbal].max_velocity_rad_s_)
                  : gimbals_[gimbal].max_velocity_rad_s_;

  auto [pp, pt] = get_position(gimbal);
  float pan_vel = std::clamp(-pp * position_gain, -limit, limit);
  float tilt_vel = std::clamp(-pt * position_gain, -limit, limit);
  set_velocity(gimbal, pan_vel, tilt_vel);
  return std::abs(pp) < tolerance_rad && std::abs(pt) < tolerance_rad;
}

bool CandleMotor::is_at_home(size_t gimbal, float tolerance_rad) {
  if (gimbal >= gimbals_.size())
    return true;
  auto [pp, pt] = get_position(gimbal);
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

// ── Legacy single-gimbal API (gimbal 0) ──────────────────────────────────────

void CandleMotor::set_velocity(float pan_rad_s, float tilt_rad_s) {
  set_velocity(0, pan_rad_s, tilt_rad_s);
}

std::pair<float, float> CandleMotor::get_velocity() {
  return get_velocity(0);
}

std::pair<float, float> CandleMotor::get_position() {
  return get_position(0);
}

bool CandleMotor::calibrate_home() {
  return calibrate_home(0);
}

bool CandleMotor::return_to_home(float position_gain, float tolerance_rad,
                                 float max_velocity_rad_s) {
  return return_to_home(0, position_gain, tolerance_rad, max_velocity_rad_s);
}

bool CandleMotor::is_at_home(float tolerance_rad) {
  return is_at_home(0, tolerance_rad);
}

mab::MD *CandleMotor::pan_motor() {
  return pan_motor(0);
}

mab::MD *CandleMotor::tilt_motor() {
  return tilt_motor(0);
}

// ── Disable ──────────────────────────────────────────────────────────────────

void CandleMotor::disable() {
  auto prev = Logger::g_m_verbosity;
  Logger::g_m_verbosity = Logger::Verbosity_E::SILENT;

  for (auto &md : pan_motors_)
    if (md)
      md->disable();
  for (auto &md : tilt_motors_)
    if (md)
      md->disable();

  Logger::g_m_verbosity = prev;

  pan_motors_.clear();
  tilt_motors_.clear();
  tilt_gear_ratios_.clear();
  power_stages_.clear();
  pds_.reset();
  if (candle_)
    mab::detachCandle(candle_);
  candle_ = nullptr;
}

} // namespace patronus::comm
