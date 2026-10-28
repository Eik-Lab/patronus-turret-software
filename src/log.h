#pragma once

#include <atomic>
#include <ctime>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>

enum class Level { Debug, Info, Warning, Error, Critical, Count };

struct LevelInfo {
  const char *name;
  const char *color;
};

inline constexpr size_t log_level_count = static_cast<size_t>(Level::Count);

inline constexpr LevelInfo log_levels[log_level_count] = {{"DEBUG", "\033[90m"},
                                                          {"INFO", "\033[32m"},
                                                          {"WARNING", "\033[33m"},
                                                          {"ERROR", "\033[31m"},
                                                          {"CRITICAL", "\033[1;31m"}};

struct LogState {
  std::ofstream file_{"patronus.log", std::ios::app};
  std::atomic<Level> min_level_{Level::Warning};
  std::mutex mutex_;
  char stamp_[20]{};
  time_t second_{0};
  bool warned_{false};
};

// Constructed on first use, so other globals can log during static init.
inline LogState &log_state() {
  static LogState s;
  return s;
}

inline void set_log_level(Level level) {
  log_state().min_level_.store(level, std::memory_order_relaxed);
}

inline void log_write(Level level, const std::string &message) {
  LogState &s = log_state();
  const std::scoped_lock lock(s.mutex_);

  const time_t now = std::time(nullptr);
  if (now != s.second_) {
    std::strftime(s.stamp_, sizeof(s.stamp_), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    s.second_ = now;
  }

  const LevelInfo &info = log_levels[static_cast<size_t>(level)];

  std::cout << '[' << s.stamp_ << "] " << info.color << info.name << "\033[0m" << ": " << message
            << std::endl;

  if (!s.file_.is_open()) {
    if (!s.warned_) {
      s.warned_ = true;
      std::cerr << "Patronus: cannot open patronus.log, logging to the console only\n";
    }
    return;
  }

  s.file_ << '[' << s.stamp_ << "] " << info.name << ": " << message << '\n';
  s.file_.flush(); // so the tail of the log survives a crash
}

// Filtered calls neither allocate nor evaluate their arguments.
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