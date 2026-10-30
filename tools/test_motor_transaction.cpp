// Hardware-free production-wrapper tests. SDK I/O is replaced at link time.
// Include SDK/standard headers before test-only fixture access below.
#include "comm/motor_result.h"
#include "core/config.h"
#include "core/types.h"

#include <atomic>
#include <candlelib.hpp>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>
#define private public
#include "comm/candle_motor.h"
#undef private

namespace {
unsigned reads = 0, writes = 0, fail_read = 0, fail_write = 0;
float limit_pan = .2F, limit_tilt = -.3F;
struct Write {
  unsigned node;
  float velocity;
};
std::vector<Write> commands;
std::vector<unsigned> transactions;
bool concurrent = false;
void require(bool value, const char *message) {
  if (!value)
    throw std::runtime_error(message);
}
void reset() {
  reads = writes = fail_read = fail_write = 0;
  limit_pan = .2F;
  limit_tilt = -.3F;
  commands.clear();
}
} // namespace

void mab::Pds::printModuleInfo() {
}
std::pair<float, mab::MD::Error_t> mab::MD::getPosition() {
  ++reads;
  if (concurrent) {
    transactions.push_back((m_canId - 1) / 2);
    std::this_thread::yield();
  }
  const float value =
    reads <= 2 || concurrent ? (m_canId % 2 ? .2F : -.3F) : (m_canId % 2 ? limit_pan : limit_tilt);
  return {value, reads == fail_read ? static_cast<Error_t>(3) : Error_t::OK};
}
std::pair<float, mab::MD::Error_t> mab::MD::getVelocity() {
  return getPosition();
}
mab::MD::Error_t mab::MD::setTargetVelocity(float velocity) {
  ++writes;
  commands.push_back({m_canId, velocity});
  if (concurrent) {
    transactions.push_back((m_canId - 1) / 2);
    std::this_thread::yield();
  }
  return writes == fail_write ? static_cast<Error_t>(3) : Error_t::OK;
}

int main() {
  using namespace patronus::comm;
  try {
    // Process-lifetime fixture: never initialize or shut down physical hardware.
    auto *motor = new CandleMotor(std::vector<patronus::config::GimbalConfig>(2));
    for (int g = 0; g < 2; ++g) {
      motor->pan_motors_.push_back(std::make_unique<mab::MD>(2 * g + 1, nullptr));
      motor->tilt_motors_.push_back(std::make_unique<mab::MD>(2 * g + 2, nullptr));
      motor->pan_limits_[g] = {true, -1, 1, 0};
      motor->tilt_limits_[g] = {true, -1, 1, 0};
    }
    auto drive = [&](double age = 60.0) {
      return motor->drive_to(0, .7F, -.8F, 1, .01F, 1, age);
    };
    auto expect_zero = [&] {
      require(commands.size() == 2 && commands[0].velocity == 0 && commands[1].velocity == 0,
              "invalid/stale feedback did not request paired hold");
    };
    reset();
    BusMetrics metrics;
    auto result = motor->drive_to(0, .7F, -.8F, 1, .01F, 1, 60, &metrics);
    require(result.command.ok() && !result.reached(), "normal move outcome");
    require(metrics.read_calls == 4 && metrics.write_calls == 2, "feedback reuse enabled");
    require(
      std::abs(commands[0].velocity - .5F) < 1e-6F && std::abs(commands[1].velocity + .5F) < 1e-6F,
      "position control changed");
    // New limit readings must override the earlier control readings. Test each
    // axis independently, on both sides of its exact outward boundary.
    for (int axis = 0; axis < 2; ++axis) {
      for (float magnitude : {std::nextafter(1.0F, 0.0F), 1.0F, 1.1F}) {
        reset();
        (axis == 0 ? limit_pan : limit_tilt) = axis == 0 ? magnitude : -magnitude;
        result = drive();
        require(reads == 4 && writes == 2, "missing fresh limit reads");
        require((commands[axis].velocity == 0) == (magnitude >= 1), "outward limit boundary");
        require(commands[1 - axis].velocity != 0, "other axis incorrectly limited");
        require(result.position.pan.value == .2F && result.position.tilt.value == -.3F,
                "control feedback overwritten by limit feedback");
      }
    }
    reset();
    limit_pan = -1.1F;
    limit_tilt = 1.1F;
    require(drive().command.ok() && commands[0].velocity > 0 && commands[1].velocity < 0,
            "inward recovery blocked");
    for (unsigned failure : {1U, 2U, 3U, 4U}) {
      reset();
      fail_read = failure;
      result = drive();
      expect_zero();
      require(!result.reached(), "failed read reported arrival");
      require(reads == failure, "read continued after unusable feedback");
      if (failure == 1)
        require(result.position.tilt.status == IoStatus::NotAttempted,
                "skipped tilt read reported as attempted");
      if (failure == 3)
        require(result.command.tilt.limit_position.status == IoStatus::NotAttempted,
                "skipped tilt limit read reported as attempted");
      if (failure > 2)
        require(!result.command.ok(), "limit error lost");
    }
    reset();
    result = drive(0);
    expect_zero();
    require(reads == 2 && !result.reached(), "expired feedback reused");
    reset();
    fail_write = 2;
    result = drive();
    require(!result.command.ok() && result.command.tilt.status == IoStatus::Error &&
              result.command.stop_pan.status == IoStatus::Ok &&
              result.command.stop_tilt.status == IoStatus::Ok,
            "partial-write outcomes lost");
    require(writes == 4 && commands[2].velocity == 0 && commands[3].velocity == 0,
            "missing fallback stops");
    reset();
    result = motor->drive_to(0, .2F, -.3F, 1, .01F, 1, 60);
    require(result.reached() && reads == 2, "arrival behavior changed");
    expect_zero();
    reset();
    result = motor->return_to_home(0, 1, .01F, 1, 60);
    require(result.command.ok() && !result.reached() && reads == 4 && writes == 2,
            "home transaction failed");
    // Home arrival requests a hold directly, not a corrective velocity followed
    // by another pair of stops. Ordinary follower drive_to retains its P-control.
    motor->pan_limits_[0].home_pos = .22F;
    motor->tilt_limits_[0].home_pos = -.31F;
    reset();
    result = motor->return_to_home(0, 1, .03F, 1, 60);
    require(result.reached() && reads == 2, "home arrival performed unnecessary limit reads");
    expect_zero();
    reset();
    result = motor->drive_to(0, .22F, -.31F, 1, .03F, 1, 60);
    require(result.reached() && reads == 4 && commands[0].velocity > 0 && commands[1].velocity < 0,
            "follower corrections were replaced by a home deadband");
    reset();
    result = motor->return_to_home(0, 1, .015F, 1, 60);
    require(!result.reached() && reads == 4 && commands[0].velocity > 0,
            "home motion stopped outside tolerance");
    for (unsigned failure : {1U, 2U}) {
      reset();
      fail_write = failure;
      result = motor->return_to_home(0, 1, .03F, 1, 60);
      require(!result.reached() && reads == 2, "failed home hold reported arrival");
      expect_zero();
    }
    reset();
    result = motor->drive_to(99, 0, 0, 1, .01F, 1, 60);
    require(!result.reached() && reads == 0 && writes == 0 &&
              result.command.pan.status == IoStatus::Unavailable,
            "invalid gimbal");

    // SDK access is serialized by the production mutex; all stub state above
    // is touched only under it. Different gimbals must not split a drive operation.
    reset();
    transactions.reserve(12000);
    commands.reserve(4000);
    concurrent = true;
    std::atomic<int> ready{0};
    auto worker = [&](int g) {
      ++ready;
      while (ready.load() != 2)
        std::this_thread::yield();
      for (int i = 0; i < 1000; ++i)
        motor->drive_to(g, .7F, -.8F, 1, .01F, 1, 60);
    };
    std::thread first(worker, 0), second(worker, 1);
    first.join();
    second.join();
    require(transactions.size() == 12000, "concurrent operation count");
    for (size_t i = 0; i < transactions.size(); i += 6)
      for (size_t j = 1; j < 6; ++j)
        require(transactions[i] == transactions[i + j], "drive transaction interleaved");
    std::cout << "PASS: fresh limit reads, boundaries, faults, stale holds, arrival and "
                 "transaction isolation\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
