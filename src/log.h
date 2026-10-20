// C++ program to implement a basic logging system.

#pragma once

#include <atomic>
#include <ctime>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>

// Enum to represent log levels
enum class Level { Debug, Info, Warning, Error, Critical, Count };

// Everything the sinks need about one level.
struct LevelInfo {
  const char *name;
  const char *color;
};

inline constexpr size_t log_level_count = static_cast<size_t>(Level::Count);

// Converts log level to a string for output
// Add colors to logging levels
inline constexpr LevelInfo log_levels[log_level_count] = {
  {"DEBUG", "\033[90m"},     // Gray
  {"INFO", "\033[32m"},      // Green
  {"WARNING", "\033[33m"},   // Yellow
  {"ERROR", "\033[31m"},     // Red
  {"CRITICAL", "\033[1;31m"} // Bold red
};

// All mutable logger state, in one place. No behaviour.
struct LogState {
  std::ofstream file_{"patronus.log", std::ios::app}; // File stream for the log file
  std::atomic<Level> min_level_{Level::Warning};
  std::mutex mutex_;
  char stamp_[20]{};
  time_t second_{0};
  bool warned_{false};
};

// Function-local static: the ofstream is not constructed until the first log
// call, so no global in another translation unit can log before it exists.
inline LogState &log_state() {
  static LogState s;
  return s;
}

inline void set_log_level(Level level) {
  log_state().min_level_.store(level, std::memory_order_relaxed);
}

// Logs a message with a given log level
inline void log_write(Level level, const std::string &message) {
  LogState &s = log_state();
  const std::scoped_lock lock(s.mutex_);

  // Get current timestamp
  const time_t now = std::time(nullptr);
  if (now != s.second_) {
    std::strftime(s.stamp_, sizeof(s.stamp_), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    s.second_ = now;
  }

  const LevelInfo &info = log_levels[static_cast<size_t>(level)];

  // Output to console
  std::cout << '[' << s.stamp_ << "] " << info.color << info.name << "\033[0m" // Reset
            << ": " << message << std::endl;

  if (!s.file_.is_open()) {
    if (!s.warned_) {
      s.warned_ = true;
      std::cerr << "Patronus: cannot open patronus.log, logging to the console only\n";
    }
    return;
  }

  // Output to log file
  s.file_ << '[' << s.stamp_ << "] " << info.name << ": " << message << '\n';
  s.file_.flush(); // Ensure immediate write to file
}

// Formats and emits one line. The level check runs before the buffer is built,
// so a filtered call neither allocates nor evaluates its arguments. Formatting
// stays outside the lock.
template <typename Format>
inline void log_line(Level level, const Format &format) {
  if (level < log_state().min_level_.load(std::memory_order_relaxed))
    return;

  std::ostringstream buf;
  format(buf);
  log_write(level, buf.str());
}

#define LOG(level, ...)                       \
  ::log_line(::Level::level, [&](auto &buf) { \
    buf << __VA_ARGS__;                       \
  })

inline std::optional<Level> parse_level(const std::string &text) {
  if (text == "debug")
    return Level::Debug;
  if (text == "info")
    return Level::Info;
  if (text == "warning" || text == "warn")
    return Level::Warning;
  if (text == "error")
    return Level::Error;
  if (text == "critical")
    return Level::Critical;
  return std::nullopt;
}