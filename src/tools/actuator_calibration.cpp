// Standalone actuator identification. No camera, tracking, PDS or homing code.
// The replay build substitutes a simulated axis at compile time: it cannot open CAN.
#include <glib.h>
#ifndef PATRONUS_CALIBRATION_SIMULATION
  #include <candlelib.hpp>
#endif

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void interrupt(int) {
  interrupted = 1;
}
double clock_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
void require(bool condition, const std::string &message) {
  if (!condition)
    throw std::runtime_error(message);
}
void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}

struct Plan {
  int node, datarate, repetitions;
  double min_rad, max_rad, travel_rad, stop_margin_rad, max_speed, settle_speed;
  double sample_s, max_io_s, max_gap_s, pulse_s, settle_s, settle_timeout_s, max_run_s;
  double torque, kp, ki, kd, integral_max;
  std::vector<double> pulses;
};

Plan parse_plan(const std::string &text) {
  std::unique_ptr<GKeyFile, decltype(&g_key_file_free)> key(g_key_file_new(), g_key_file_free);
  GError *error = nullptr;
  const bool loaded =
    g_key_file_load_from_data(key.get(), text.data(), text.size(), G_KEY_FILE_NONE, &error);
  if (error != nullptr) {
    const std::string message = error->message;
    g_error_free(error);
    throw std::runtime_error(message);
  }
  require(loaded, "Cannot parse plan");
  const auto number = [&](const char *name) {
    GError *field_error = nullptr;
    const double value = g_key_file_get_double(key.get(), "calibration", name, &field_error);
    if (field_error != nullptr) {
      const std::string message = field_error->message;
      g_error_free(field_error);
      throw std::runtime_error(message);
    }
    require(std::isfinite(value), std::string("Nonfinite plan field: ") + name);
    return value;
  };
  const auto integer = [&](const char *name, int minimum, int maximum) {
    const double value = number(name);
    require(value >= minimum && value <= maximum && std::floor(value) == value,
            std::string("Invalid integer: ") + name);
    return static_cast<int>(value);
  };
  Plan p{};
  p.node = integer("node_id", 1, 2047);
  p.datarate = integer("datarate_mbps", 1, 8);
  require(p.datarate == 1 || p.datarate == 2 || p.datarate == 5 || p.datarate == 8,
          "Unsupported CAN datarate");
  p.repetitions = integer("repetitions", 1, 20);
  p.min_rad = number("min_position_rad");
  p.max_rad = number("max_position_rad");
  p.travel_rad = number("max_excursion_rad");
  p.stop_margin_rad = number("stop_margin_rad");
  p.max_speed = number("max_observed_velocity_rad_s");
  p.settle_speed = number("settle_velocity_rad_s");
  p.sample_s = number("sample_ms") / 1000;
  p.max_io_s = number("max_io_ms") / 1000;
  p.max_gap_s = number("max_sample_gap_ms") / 1000;
  p.pulse_s = number("pulse_ms") / 1000;
  p.settle_s = number("settle_ms") / 1000;
  p.settle_timeout_s = number("settle_timeout_ms") / 1000;
  p.max_run_s = number("max_run_ms") / 1000;
  p.torque = number("max_torque_nm");
  p.kp = number("velocity_kp");
  p.ki = number("velocity_ki");
  p.kd = number("velocity_kd");
  p.integral_max = number("velocity_integral_max");
  require(p.min_rad < p.max_rad && p.travel_rad > 0 && p.stop_margin_rad > 0,
          "Explicit ordered travel bounds and positive stopping margin required");
  require(p.max_speed > 0 && p.max_speed <= std::numeric_limits<float>::max() / 4 &&
            p.settle_speed > 0 && p.settle_speed < p.max_speed,
          "Invalid velocity thresholds");
  require(p.sample_s >= .002 && p.sample_s <= .02 && p.max_io_s > 0 && p.max_io_s <= .05 &&
            p.max_gap_s >= p.sample_s && p.max_gap_s <= .05,
          "Require sample 2..20 ms, I/O limit (0,50] ms and sample-gap limit [sample,50] ms");
  require(p.pulse_s >= p.sample_s && p.pulse_s <= 1 && p.settle_s >= p.sample_s &&
            p.settle_timeout_s > p.settle_s && p.settle_timeout_s <= 10 && p.max_run_s > 0 &&
            p.max_run_s <= 300,
          "Invalid pulse, settling or total timeout (maximum run 300 s)");
  require(p.torque > 0 && p.kp >= 0 && p.ki >= 0 && p.kd >= 0 && p.integral_max >= 0 &&
            p.torque <= 100 && p.kp <= 100000 && p.ki <= 100000 && p.kd <= 100000 &&
            p.integral_max <= 100000,
          "Invalid torque/PID settings; supply hardware-appropriate values");
  const double margin = p.stop_margin_rad + p.max_speed * (p.max_gap_s + 2 * p.max_io_s);
  require(2 * margin < p.max_rad - p.min_rad && margin < p.travel_rad,
          "Travel window too small for stopping and communication guard margins");
  const int count = integer("amplitude_count", 1, 8);
  for (int a = 1; a <= count; ++a) {
    const double speed = number(("amplitude_" + std::to_string(a) + "_rad_s").c_str());
    require(speed > 0 && speed < p.max_speed && speed * p.pulse_s + margin < p.travel_rad,
            "Pulse exceeds speed or excursion budget");
    for (int r = 0; r < p.repetitions; ++r) {
      p.pulses.push_back(speed);
      p.pulses.push_back(-speed);
    }
  }
  require(p.max_run_s > p.settle_s * (p.pulses.size() + 1) + p.pulse_s * p.pulses.size(),
          "Run timeout cannot accommodate the minimum pulse/settle schedule");
  return p;
}

struct Event {
  const char *operation;
  double begin, end, value;
  int status, pulse;
};
struct Trace {
  std::vector<Event> events;
  size_t limit;
  int pulse{-1};
  explicit Trace(const Plan &p)
    : limit(static_cast<size_t>(std::ceil(p.max_run_s / p.sample_s)) * 4 + 128) {
    events.reserve(limit);
  }
  void add(const char *operation, double begin, double value, int status) {
    require(events.size() < limit, "Trace capacity exhausted");
    events.push_back({operation, begin, clock_s(), value, status, pulse});
  }
  void save(const std::filesystem::path &directory) const {
    std::ofstream file(directory / "events.csv");
    file.exceptions(std::ios::badbit | std::ios::failbit);
    file << "operation,begin_monotonic_s,end_monotonic_s,value,sdk_status,pulse\n"
         << std::setprecision(17);
    for (const auto &e : events)
      file << e.operation << ',' << e.begin << ',' << e.end << ',' << e.value << ',' << e.status
           << ',' << e.pulse << '\n';
    file.close();
  }
};

// A distinct adapter, never CandleMotor::init(): only the explicitly selected node
// is addressed. The simulator is linked by compilation, not runtime hardware discovery.
class Axis {
public:
  Axis(const Plan &plan, Trace &trace, const std::string &fault)
    : plan_(plan), trace_(trace)
#ifdef PATRONUS_CALIBRATION_SIMULATION
      ,
      fault_(fault), position_((plan.min_rad + plan.max_rad) / 2)
#endif
  {
#ifndef PATRONUS_CALIBRATION_SIMULATION
    (void)fault;
#endif
  }
  ~Axis() {
    // Last-resort stop if an exception bypasses the explicit, recorded shutdown.
#ifndef PATRONUS_CALIBRATION_SIMULATION
    if (motor_) {
      if (!closed_) {
        motor_->setTargetVelocity(0);
        motor_->disable();
      }
      motor_.reset();
    }
    if (candle_ != nullptr)
      mab::detachCandle(candle_);
#endif
  }
  void open() {
#ifdef PATRONUS_CALIBRATION_SIMULATION
    trace_.add("disable", clock_s(), 0, 1);
#else
    const auto rate = plan_.datarate == 8   ? mab::CAN_DATARATE_8M
                      : plan_.datarate == 5 ? mab::CAN_DATARATE_5M
                      : plan_.datarate == 2 ? mab::CAN_DATARATE_2M
                                            : mab::CAN_DATARATE_1M;
    candle_ = mab::attachCandle(rate, mab::candleTypes::busTypes_t::USB);
    require(candle_ != nullptr, "Cannot attach CANdle");
    motor_ = std::make_unique<mab::MD>(static_cast<uint16_t>(plan_.node), candle_);
    checked("disable", 0, [&] {
      return motor_->disable();
    });
    checked("initialize", 0, [&] {
      return motor_->init();
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Disabled mode transition.
    checked("mode", 0, [&] {
      return motor_->setMotionMode(mab::MdMode_E::VELOCITY_PID);
    });
    checked("pid", 0, [&] {
      return motor_->setVelocityPIDparam(static_cast<float>(plan_.kp), static_cast<float>(plan_.ki),
                                         static_cast<float>(plan_.kd),
                                         static_cast<float>(plan_.integral_max));
    });
    checked("torque", plan_.torque, [&] {
      return motor_->setMaxTorque(static_cast<float>(plan_.torque));
    });
#endif
    write(0);
  }
  void enable() {
#ifdef PATRONUS_CALIBRATION_SIMULATION
    trace_.add("enable", clock_s(), 0, 1);
#else
    checked("enable", 0, [&] {
      return motor_->enable();
    });
    checked("verify_mode", 0, [&] {
      return motor_->readRegisters(motor_->m_mdRegisters.motionModeStatus);
    });
    require(motor_->m_mdRegisters.motionModeStatus.value == mab::MdMode_E::VELOCITY_PID,
            "Velocity mode did not take; aborting without a pulse");
#endif
  }
  double read(bool velocity) {
    const double begin = clock_s();
    double value = 0;
    int status = 1;
#ifdef PATRONUS_CALIBRATION_SIMULATION
    const double dt = begin - last_s_;
    last_s_ = begin;
    const bool inject = trace_.pulse >= 0;
    if (fault_ == "overspeed" && inject)
      velocity_ = plan_.max_speed * 2;
    else
      velocity_ += (command_ - velocity_) * (1 - std::exp(-dt / .015));
    position_ += velocity_ * dt;
    value = velocity ? velocity_ : position_;
    if (fault_ == "unsettled" && inject && velocity)
      value = plan_.settle_speed * 2;
    if (fault_ == "excursion" && inject && !velocity)
      value = (plan_.min_rad + plan_.max_rad) / 2 + plan_.travel_rad;
    if (fault_ == "read" && inject)
      status = 3;
    if (fault_ == "nonfinite" && inject)
      value = std::nan("");
    if ((fault_ == "limit" && inject) || fault_ == "initial-limit")
      value = velocity ? 0 : plan_.max_rad;
    if (fault_ == "lower-limit" && inject)
      value = velocity ? 0 : plan_.min_rad;
    if (fault_ == "stale" && inject)
      std::this_thread::sleep_for(std::chrono::duration<double>(plan_.max_io_s * 2));
#else
    const auto sample = velocity ? motor_->getVelocity() : motor_->getPosition();
    value = sample.first;
    status = static_cast<int>(sample.second);
#endif
    trace_.add(velocity ? "velocity" : "position", begin, value, status);
    require(status == 1 && std::isfinite(value), "Invalid feedback");
    require(clock_s() - begin <= plan_.max_io_s, "Feedback exchange exceeded I/O deadline");
    return value;
  }
  void write(double value) {
    require(value == 0 || !interrupted, "Interrupted before velocity write");
    // Record the float actually sent to the SDK, not a higher-precision request.
    value = static_cast<float>(value);
#ifdef PATRONUS_CALIBRATION_SIMULATION
    const int status =
      ((fault_ == "write" && value != 0) || (fault_ == "stop" && value == 0 && trace_.pulse >= 0))
        ? 3
        : 1;
    trace_.add("command", clock_s(), value, status);
    require(status == 1, "Velocity write failed");
    command_ = value;
#else
    checked("command", value, [&] {
      return motor_->setTargetVelocity(static_cast<float>(value));
    });
#endif
  }
  bool stop() {
    // Do not short circuit: attempt disable even if zeroing fails or is slow.
    bool ok = true;
    try {
      write(0);
    } catch (const std::exception &) {
      ok = false;
    }
#ifdef PATRONUS_CALIBRATION_SIMULATION
    trace_.add("disable", clock_s(), 0, 1);
#else
    if (motor_) {
      try {
        checked("disable", 0, [&] {
          return motor_->disable();
        });
      } catch (const std::exception &) {
        ok = false;
      }
    }
#endif
    closed_ = ok;
    return ok;
  }

private:
  const Plan &plan_;
  Trace &trace_;
  bool closed_{false};
#ifdef PATRONUS_CALIBRATION_SIMULATION
  std::string fault_;
  double position_, velocity_{0}, command_{0}, last_s_{clock_s()};
#else
  mab::Candle *candle_{nullptr};
  std::unique_ptr<mab::MD> motor_;
  template <typename Operation>
  void checked(const char *name, double value, Operation operation) {
    require(motor_ != nullptr, "Motor not connected");
    require(!interrupted || std::strcmp(name, "disable") == 0 ||
              (std::strcmp(name, "command") == 0 && value == 0),
            "Interrupted during setup");
    const double begin = clock_s();
    const auto status = operation();
    trace_.add(name, begin, value, static_cast<int>(status));
    if (status != mab::MD::Error_t::OK)
      throw std::runtime_error(std::string(name) + " failed");
    if (clock_s() - begin > plan_.max_io_s)
      throw std::runtime_error(std::string(name) + " exceeded I/O deadline");
  }
#endif
};

void check_feedback(const Plan &p, double position, double velocity, double origin) {
  const double margin = p.stop_margin_rad + p.max_speed * (p.max_gap_s + 2 * p.max_io_s);
  require(std::abs(velocity) <= p.max_speed, "Overspeed");
  require(position > p.min_rad + margin && position < p.max_rad - margin,
          "Absolute travel guard reached");
  require(std::abs(position - origin) < p.travel_rad - margin, "Excursion guard reached");
}

void run(const Plan &p, Axis &axis, Trace &trace) {
  require(!interrupted, "Interrupted before setup");
  axis.open();
  const double initial_begin = clock_s();
  const double origin = axis.read(false);
  const double initial_velocity = axis.read(true);
  require(clock_s() - initial_begin <= p.max_gap_s, "Initial feedback stale");
  check_feedback(p, origin, initial_velocity, origin);
  require(std::abs(initial_velocity) <= p.settle_speed, "Axis moving before enable");
  require(!interrupted, "Interrupted before enable");
  axis.enable();
  const double start = clock_s();
  double previous = start, phase_start = start, stable_since = start;
  bool pulsing = false;
  size_t next = 0;
  for (;;) {
    const double begin = clock_s();
    require(!interrupted, "Interrupted; no automatic restart");
    require(begin - start < p.max_run_s, "Total run deadline exceeded");
    require(begin - previous <= p.max_gap_s, "Sample gap exceeded");
    previous = begin;
    const double position = axis.read(false);
    const double velocity = axis.read(true);
    const double now = clock_s();
    require(now - begin <= p.max_gap_s, "Feedback stale before command");
    check_feedback(p, position, velocity, origin);
    if (pulsing && now - phase_start >= p.pulse_s) {
      axis.write(0);
      pulsing = false;
      phase_start = stable_since = clock_s();
    } else if (!pulsing) {
      require(now - phase_start < p.settle_timeout_s, "Axis did not settle");
      if (std::abs(velocity) > p.settle_speed)
        stable_since = now;
      if (now - stable_since >= p.settle_s) {
        if (next == p.pulses.size())
          return;
        trace.pulse = static_cast<int>(next);
        axis.write(p.pulses[next++]);
        pulsing = true;
        phase_start = clock_s();
      } else {
        axis.write(0); // Maintain zero command and watchdog traffic during settling.
      }
    } else {
      axis.write(p.pulses[next - 1]);
    }
    require(clock_s() - begin <= p.max_gap_s, "Cycle exceeded deadline");
    const auto deadline = std::chrono::steady_clock::time_point(
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(begin + p.sample_s)));
    std::this_thread::sleep_until(deadline); // No catch-up bursts.
  }
}

} // namespace

int main(int argc, char **argv) {
  try {
    std::string plan_path, output, fault;
    bool arm = false, simulate = false;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help") {
        std::cout
          << "actuator_calibration --plan FILE [--output NEW_DIRECTORY] [--arm | --simulate]\n"
             "Default: validate and print plan only; no hardware access.\n"
             "--arm additionally requires interactive terminal confirmation.\n";
        return 0;
      }
      if (arg == "--arm")
        arm = true;
      else if (arg == "--simulate")
        simulate = true;
      else {
        require(i + 1 < argc, "Missing option value");
        if (arg == "--plan")
          plan_path = argv[++i];
        else if (arg == "--output")
          output = argv[++i];
#ifdef PATRONUS_CALIBRATION_SIMULATION
        else if (arg == "--fault")
          fault = argv[++i];
#endif
        else
          throw std::runtime_error("Unknown option: " + arg);
      }
    }
    require(!plan_path.empty() && !(arm && simulate),
            "Supply --plan; --arm and --simulate are exclusive");
    std::ifstream source(plan_path);
    require(source.good(), "Cannot read plan");
    const std::string text((std::istreambuf_iterator<char>(source)),
                           std::istreambuf_iterator<char>());
    const Plan plan = parse_plan(text);
    std::cout << "Node " << plan.node << ": " << plan.pulses.size()
              << " alternating pulses; bounds [" << plan.min_rad << ", " << plan.max_rad
              << "] rad; maximum run " << plan.max_run_s
              << " s. No PDS, followers, cameras, homing or encoder zeroing.\n";
    if (!arm && !simulate) {
      std::cout << "DRY RUN: no hardware accessed, no output directory created.\n";
      return 0;
    }
#ifdef PATRONUS_CALIBRATION_SIMULATION
    require(!arm, "Simulation build cannot arm hardware");
    require(fault.empty() || fault == "read" || fault == "write" || fault == "nonfinite" ||
              fault == "limit" || fault == "stale" || fault == "overspeed" ||
              fault == "initial-limit" || fault == "stop" || fault == "unsettled" ||
              fault == "excursion" || fault == "lower-limit",
            "Unknown simulated fault");
#else
    require(!simulate, "Use replay-only build for simulation");
    require(isatty(STDIN_FILENO) != 0, "Arming requires an interactive terminal");
    const std::string confirmation = "ARM NODE " + std::to_string(plan.node);
    std::cout << "WARNING: selected axis will move and will be disabled afterward.\n"
                 "Verify external emergency stop, gravity support, known absolute encoder limits,\n"
                 "torque/PID values, exclusive bus ownership and all other axes disabled.\n"
              << "Type exactly '" << confirmation << "' to proceed: " << std::flush;
    std::string answer;
    std::getline(std::cin, answer);
    require(answer == confirmation, "Not armed");
#endif
    require(!output.empty(), "Execution requires --output NEW_DIRECTORY");
    require(std::filesystem::create_directory(output), "Output directory must not exist");
    const std::filesystem::path directory(output);
    {
      std::ofstream saved(directory / "plan.ini");
      saved.exceptions(std::ios::badbit | std::ios::failbit);
      saved << text;
      saved.close();
    }
    std::ofstream result(directory / "run.txt");
    result.exceptions(std::ios::badbit | std::ios::failbit);
    result << (simulate ? "mode=simulation\n" : "mode=hardware\n")
           << "state=incomplete\nclock=host_steady_clock\n"
           << std::flush;
    Trace trace(plan);
    Axis axis(plan, trace, fault);
    std::signal(SIGINT, interrupt);
    std::signal(SIGTERM, interrupt);
    bool success = false;
    std::string reason = "completed";
    try {
      run(plan, axis, trace);
      success = true;
    } catch (const std::exception &e) {
      reason = e.what();
    }
    const bool stopped = axis.stop();
    trace.save(directory); // Harness trace storage performs no disk I/O while moving.
    result << "state=" << (success && stopped ? "complete" : "aborted") << '\n'
           << "reason=" << reason << '\n'
           << "stop_confirmed_by_sdk=" << stopped << '\n'
           << "events=" << trace.events.size() << '\n';
    result.close();
    std::cout << (success && stopped ? "COMPLETE" : "ABORTED") << ": " << reason
              << "; stop/disable SDK success=" << stopped << '\n';
    return success && stopped ? 0 : 1;
  } catch (const std::exception &e) {
    std::cerr << "Calibration refused/failed: " << e.what() << '\n';
    return 1;
  }
}
