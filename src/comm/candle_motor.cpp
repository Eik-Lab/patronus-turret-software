#include "patronus/comm/candle_motor.hpp"

#include <candlelib.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <thread>

namespace patronus::comm {

static mab::CANdleDatarate_E to_datarate(uint8_t v)
{
  if (v == 2) return mab::CAN_DATARATE_2M;
  if (v == 5) return mab::CAN_DATARATE_5M;
  if (v == 8) return mab::CAN_DATARATE_8M;
  return mab::CAN_DATARATE_1M;
}

CandleMotor::CandleMotor(const CandleConfig& cfg) : cfg_(cfg) {}

CandleMotor::~CandleMotor()
{
  disable();
}

bool CandleMotor::enable_pds()
{
  if (cfg_.pds_node_id == 0) {
    std::printf("CandleMotor: PDS init skipped (pds_node_id=0)\n");
    return true;
  }

  pds_ = std::make_unique<mab::Pds>(cfg_.pds_node_id, candle_);
  if (pds_->init() != mab::PdsModule::error_E::OK) {
    std::fprintf(stderr, "CandleMotor: PDS%u not found on bus — continuing without PDS\n",
                 cfg_.pds_node_id);
    pds_.reset();
    return true;
  }
  std::printf("CandleMotor: PDS%u initialised\n", cfg_.pds_node_id);

  // Discover which sockets have power stages
  auto modules = pds_->getModules();
  mab::socketIndex_E sockets[] = {
    mab::socketIndex_E::SOCKET_1, mab::socketIndex_E::SOCKET_2,
    mab::socketIndex_E::SOCKET_3, mab::socketIndex_E::SOCKET_4,
    mab::socketIndex_E::SOCKET_5, mab::socketIndex_E::SOCKET_6,
  };
  mab::moduleType_E types[] = {
    modules.moduleTypeSocket1, modules.moduleTypeSocket2,
    modules.moduleTypeSocket3, modules.moduleTypeSocket4,
    modules.moduleTypeSocket5, modules.moduleTypeSocket6,
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
    std::fprintf(stderr, "CandleMotor: no power stages found on PDS%u\n", cfg_.pds_node_id);
    return false;
  }

  // Allow power to stabilise before contacting motors
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return true;
}

bool CandleMotor::init()
{
  candle_ = mab::attachCandle(to_datarate(cfg_.datarate),
                              mab::candleTypes::busTypes_t::USB);
  if (!candle_) {
    std::fprintf(stderr, "CandleMotor: failed to attach CANdle device\n");
    return false;
  }

  // Enable PDS power stages before motors
  if (!enable_pds()) {
    std::fprintf(stderr, "CandleMotor: PDS power stage enable failed\n");
    disable();
    return false;
  }

  pan_motor_  = std::make_unique<mab::MD>(cfg_.pan_node_id, candle_);
  tilt_motor_ = std::make_unique<mab::MD>(cfg_.tilt_node_id, candle_);

  for (auto* md : {pan_motor_.get(), tilt_motor_.get()}) {
    if (md->init() != mab::MD::Error_t::OK) {
      std::fprintf(stderr, "CandleMotor: MD%u init failed\n", md->m_canId);
      disable();
      return false;
    }

    md->setMotionMode(mab::MdMode_E::VELOCITY_PID);
    md->setVelocityPIDparam(1.0F, 0.05F, 0.0F, 0.5F);
    md->setMaxTorque(10.0F);
    md->enable();
  }

  return true;
}

std::pair<float, float> CandleMotor::get_velocity()
{
  if (!pan_motor_ || !tilt_motor_) return {0, 0};
  auto pv = pan_motor_->getVelocity();
  auto tv = tilt_motor_->getVelocity();
  return {pv.first, tv.first};
}

std::pair<float, float> CandleMotor::get_position()
{
  if (!pan_motor_ || !tilt_motor_) return {0, 0};
  auto pp = pan_motor_->getPosition();
  auto tp = tilt_motor_->getPosition();
  return {pp.first, tp.first};
}

void CandleMotor::set_velocity(float pan_rad_s, float tilt_rad_s)
{
  if (!pan_motor_ || !tilt_motor_) return;

  pan_rad_s  = std::clamp(pan_rad_s,  -cfg_.max_velocity_rad_s, cfg_.max_velocity_rad_s);
  tilt_rad_s = std::clamp(tilt_rad_s, -cfg_.max_velocity_rad_s, cfg_.max_velocity_rad_s);

  pan_motor_->setTargetVelocity(pan_rad_s);
  tilt_motor_->setTargetVelocity(tilt_rad_s);
}

void CandleMotor::disable()
{
  if (pan_motor_)  pan_motor_->disable();
  if (tilt_motor_) tilt_motor_->disable();
  pan_motor_.reset();
  tilt_motor_.reset();
  power_stages_.clear();
  pds_.reset();
  if (candle_) mab::detachCandle(candle_);
  candle_ = nullptr;
}

}  // namespace patronus::comm
