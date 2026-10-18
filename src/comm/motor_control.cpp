#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// MAB robotics libraries
#include "MD.hpp"
#include "MDStatus.hpp"
#include "candle.hpp"

// Config file reading
#include "json.hpp"

#include <fstream>
#include <optional>

using json = nlohmann::json;

// Define config file structure
struct PID_gains {
  mab::canId_t id;
  std::string role;
  float kp;
  float ki;
  float kd;
  float integral_max;
};

const std::string FILE_NAME = "pid_tuning.json";

// Function to read config
std::vector<PID_gains> loadPID_gains(const std::string &filename) {
  std::vector<PID_gains> PID_values;

  std::ifstream configFile(filename);
  if (!configFile.is_open()) {
    std::cerr << "No config file found" << std::endl;
    return PID_values;
  }

  try {
    json config = json::parse(configFile);

    for (auto &[key, value] : config.items()) {
      PID_gains gain;

      gain.id = static_cast<mab::canId_t>(std::stoi(key));
      gain.role = value.value("role", "unknown");
      gain.kp = value.at("kp").get<float>();
      gain.ki = value.at("ki").get<float>();
      gain.kd = value.at("kd").get<float>();
      gain.integral_max = value.at("integral_max").get<float>();

      PID_values.push_back(gain);
    }

  } catch (const std::exception &e) {
    std::cerr << "Failed to read " << filename << ": " << e.what() << std::endl;
    return PID_values;
  }

  return PID_values;
}

// Motor initiation
struct MotorHandle {
  mab::MD md;
  PID_gains gains;

  // Software limits, filled in by calibrate() (in radians, relative to the zero set during
  // calibration)
  bool calibrated = false;
  float min_pos = std::numeric_limits<float>::lowest();
  float max_pos = std::numeric_limits<float>::max();
};

struct MotorInit {
  mab::Candle *candle;
  std::vector<MotorHandle> motors;
};

MotorInit motor_init() {
  std::vector<PID_gains> gains_list = loadPID_gains(FILE_NAME);

  // Initiate Candle
  Logger log(Logger::ProgramLayer_E::TOP, "User Program");
  Logger::g_m_verbosity = Logger::Verbosity_E::VERBOSITY_1;

  mab::Candle *candle =
    mab::attachCandle(mab::CANdleDatarate_E::CAN_DATARATE_1M, mab::candleTypes::busTypes_t::USB);

  // Initiate motors
  std::vector<MotorHandle> motors;
  for (auto gains : gains_list) {
    mab::MD md(gains.id, candle);
    if (md.init() == mab::MD::Error_t::OK) {
      motors.push_back(MotorHandle{md, gains});
    }
  }

  if (motors.empty()) {
    std::cerr << "No motor drivers initiated!" << std::endl;
  }

  return MotorInit{candle, motors};
}

// Drive one motor in one direction (+1 or -1) until it stalls against a barrier.
// Returns true and writes the position where it stopped to 'barrier_pos'.
bool find_barrier(MotorHandle &cal_motor, float direction, float boot_step, float max_lead,
                  float &barrier_pos) {
  const int grace_samples = 30;   // ignore stalls while the motor gets going (~0.3 s)
  const int max_samples = 3000;   // give up after ~30 s
  const int max_read_errors = 20; // give up after this many failed position reads in a row

  // Start from where the motor actually is
  auto start = cal_motor.md.getPosition();
  if (start.second != mab::MD::Error_t::OK) {
    std::cerr << "Motor " << cal_motor.gains.id << ": could not read position" << std::endl;
    return false;
  }

  float currentPosition = start.first;
  float lastPosition = currentPosition;
  float target = currentPosition;
  int stallCount = 0;
  int readErrors = 0;

  for (int sample = 0; sample < max_samples; sample++) {
    // Move the target along, but never further than max_lead ahead of the motor.
    // (Otherwise the target runs away from the motor and can't be backed off from later)
    target += direction * boot_step;
    if (std::abs(target - currentPosition) > max_lead) {
      target = currentPosition + direction * max_lead;
    }
    cal_motor.md.setTargetPosition(target);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    auto pos = cal_motor.md.getPosition();
    if (pos.second != mab::MD::Error_t::OK) {
      if (++readErrors > max_read_errors) {
        std::cerr << "Motor " << cal_motor.gains.id << ": too many failed position reads"
                  << std::endl;
        return false;
      }
      continue; // A failed read returns 0, so don't use it
    }
    readErrors = 0;
    currentPosition = pos.first;

    // If motor isn't able to move for 20 samples, then know it has hit a barrier
    if (sample >= grace_samples && std::abs(currentPosition - lastPosition) < 0.01f)
      stallCount++;
    else
      stallCount = 0;

    lastPosition = currentPosition;

    if (stallCount > 20) {
      barrier_pos = currentPosition;
      return true;
    }
  }

  std::cerr << "Motor " << cal_motor.gains.id << ": no barrier found (timeout)" << std::endl;
  return false;
}

// Calibration function. Finds the barrier in both directions and stores software limits.
// 'backoff' = how far (rad) inside each barrier the software limit is placed.
void calibrate(std::vector<MotorHandle *> &motor_group, float kp, float kd, float torque,
               float backoff) {
  const float boot_step = 0.05f; // Calibration speed

  // Target may be ahead of the motor by at most this much (= enough to saturate the torque limit)
  const float max_lead = (kp > 0.0f) ? 2.0f * torque / kp : 1.0f;

  for (MotorHandle *cal_motor : motor_group) {
    cal_motor->calibrated = false;

    cal_motor->md.zero();
    cal_motor->md.setMotionMode(mab::MdMode_E::IMPEDANCE);
    cal_motor->md.setImpedanceParams(kp, kd);
    cal_motor->md.setMaxTorque(torque);
    cal_motor->md.enable();

    // Go up to one barrier, then down to the other, and record both
    float upper = 0.0f;
    float lower = 0.0f;
    bool ok = find_barrier(*cal_motor, +1.0f, boot_step, max_lead, upper) &&
              find_barrier(*cal_motor, -1.0f, boot_step, max_lead, lower);

    if (!ok) {
      std::cerr << "Motor " << cal_motor->gains.id << ": calibration FAILED" << std::endl;
      cal_motor->md.disable();
      continue;
    }

    if (upper - lower <= 2.0f * backoff) {
      std::cerr << "Motor " << cal_motor->gains.id << ": range (" << (upper - lower)
                << " rad) is too small for a backoff of " << backoff << " rad" << std::endl;
      cal_motor->md.disable();
      continue;
    }

    // Software limits sit 'backoff' inside the real barriers
    cal_motor->min_pos = lower + backoff;
    cal_motor->max_pos = upper - backoff;
    cal_motor->calibrated = true;

    // The motor is at the lower barrier now. Ramp up to the middle of the range
    float centre = (cal_motor->min_pos + cal_motor->max_pos) / 2.0f;
    float target = lower;
    for (int i = 0; i < 2000 && target < centre; i++) {
      target = std::min(target + boot_step, centre);
      cal_motor->md.setTargetPosition(target);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Hold ~1 s so the motor can get there (also feeds the watchdog)
    for (int i = 0; i < 100; i++) {
      cal_motor->md.setTargetPosition(centre);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    cal_motor->md.disable();
  }
}

// Boot sequence.
void boot_sequence(MotorInit &init) {
  // Pointers (not copies!) so the limits calibrate() stores end up in init.motors
  std::vector<MotorHandle *> pan_motors;
  std::vector<MotorHandle *> tilt_motors;

  // Sort motors into respective group
  for (auto &handle : init.motors) {
    if (handle.gains.role == "pan") {
      pan_motors.push_back(&handle);

    } else {
      tilt_motors.push_back(&handle);
    }
  }

  // Calibrate motors (temp values): group, kp, kd, max torque, software-limit margin from barrier
  // (rad)
  calibrate(pan_motors, 10, 0.5, 2, 0.3f);
  calibrate(tilt_motors, 1, 1, 1, 0.3f);
}

// Set speed
void set_velocityPID(MotorInit &init, const std::vector<float> &velocities) {
  if (init.motors.size() != velocities.size()) {
    std::cerr << "Invalid values! Expected " << init.motors.size() << " speeds, got "
              << velocities.size() << std::endl;
  } else {
    int i = 0;
    for (auto &handle : init.motors) {
      handle.md.setTargetVelocity(velocities[i]);
      i++;
    }
  }
}

// Stop sequence. This will disable motors! If you want a temporary stop, use "set_velocityPID" to
// set motor speed to zero.
void stop_motors(MotorInit &init) {
  for (auto &handle : init.motors) {
    handle.md.disable();
  }
  mab::detachCandle(init.candle);
}

// Shared between main (user input) and the control thread
std::vector<float> targets;      // Speeds the user asked for, same order as init.motors
std::mutex targets_mutex;        // Protects 'targets'
std::atomic<bool> running{true}; // Set to false to stop the control thread

// Software stop: take the requested speeds and zero any that would push a motor past its limit.
// Moving back away from a limit is always allowed.
std::vector<float> limit_speeds(MotorInit &init, const std::vector<float> &requested) {
  std::vector<float> allowed = requested;

  for (size_t i = 0; i < init.motors.size(); i++) {
    MotorHandle &m = init.motors[i];

    // No limits known = not safe to move
    if (!m.calibrated) {
      allowed[i] = 0.0f;
      continue;
    }

    auto pos = m.md.getPosition();
    if (pos.second != mab::MD::Error_t::OK) {
      allowed[i] = 0.0f; // Can't see where the motor is, so don't move it
      continue;
    }

    // Positive speed is assumed to increase position. Check with a small speed first!
    if (allowed[i] > 0.0f && pos.first >= m.max_pos)
      allowed[i] = 0.0f;
    if (allowed[i] < 0.0f && pos.first <= m.min_pos)
      allowed[i] = 0.0f;
  }

  return allowed;
}

// Runs in its own thread: keeps sending speeds (feeds the watchdog) and enforces the limits
void control_loop(MotorInit &init) {
  while (running) {
    std::vector<float> requested;
    {
      std::lock_guard<std::mutex> lock(targets_mutex);
      requested = targets;
    }

    set_velocityPID(init, limit_speeds(init, requested));

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

// Sloppa function (why does C++ not have any split function)
std::vector<std::string> split(const std::string &text, char delimiter) {
  std::vector<std::string> parts;
  std::istringstream in(text);
  std::string piece;
  while (std::getline(in, piece, delimiter)) {
    parts.push_back(piece);
  }
  return parts;
}

int main() {
  // Ask if you want to calibrate
  std::cout << "Initiate motors and run calibration? (Y/N)" << std::endl;
  std::string answer;
  std::getline(std::cin, answer);

  if (answer.empty() || std::tolower(static_cast<unsigned char>(answer[0])) != 'y') {
    return EXIT_SUCCESS;
  }

  // Initiate Candle and motors
  MotorInit init = motor_init(); // This object will hold candle + motors
  if (init.motors.empty()) {
    mab::detachCandle(init.candle);
    return EXIT_FAILURE;
  }

  boot_sequence(init); // Initiate Candle + motors first!!

  for (auto &motor : init.motors) {
    motor.md.setMotionMode(mab::MdMode_E::VELOCITY_PID);
    motor.md.setVelocityPIDparam(motor.gains.kp, motor.gains.ki, motor.gains.kd,
                                 motor.gains.integral_max); // Use valies from "pid_tuning.json"
    motor.md.enable();
  }

  // Start with all motors at zero speed, then start the control thread
  targets.assign(init.motors.size(), 0.0f);
  std::thread control(control_loop, std::ref(init));

  // Print out available motors
  std::cout << "Available motors:" << std::endl;

  for (auto &motor : init.motors) {
    std::cout << "Motor id: " << motor.gains.id << " (" << motor.gains.role << ") ";
    if (motor.calibrated) {
      std::cout << "limits: [" << motor.min_pos << ", " << motor.max_pos << "] rad" << std::endl;
    } else {
      std::cout << "NOT calibrated - will be held at zero speed" << std::endl;
    }
  }

  std::cout << "Please give each motor a speed. Format: '0.5, 0.3, 0.2, 0.5' or 'q' to quit. Make "
               "sure they are in order"
            << std::endl;

  // Ask for speeds. Apply if given, and wait for next set of speed
  bool quit = false;
  std::string speeds;

  while (!quit) {
    if (!std::getline(std::cin, speeds)) {
      break;
    } // Input closed

    // If user quits
    if (std::tolower(static_cast<unsigned char>(speeds[0])) == 'q') {
      quit = true;

      // If numbers are given
    } else {
      std::vector<float> speed_float;

      try {
        auto parts = split(speeds, ','); // Split answer line

        // Convert the split into floats
        for (const auto &p : parts) {
          speed_float.push_back(std::stof(p));
        }

        if (speed_float.size() != init.motors.size()) {
          std::cerr << "Expected " << init.motors.size() << " speeds, got " << speed_float.size()
                    << std::endl;
        } else {
          // Hand the new speeds to the control thread
          std::lock_guard<std::mutex> lock(targets_mutex);
          targets = speed_float;
        }
      } catch (const std::exception &e) {
        std::cerr << "Invalid format (skill issue)" << std::endl;
      }
    }
  }

  // Ramp down: zero speeds, give the motors time to stop, then stop the thread
  {
    std::lock_guard<std::mutex> lock(targets_mutex);
    targets.assign(init.motors.size(), 0.0f);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  running = false;
  control.join();

  stop_motors(init);
  return EXIT_SUCCESS;
}