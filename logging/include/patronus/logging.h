#pragma once

#include <atomic>
#include <ctime>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>

namespace patronus::logging {

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
  std::optional<std::string> file_path_{"patronus.log"};
  std::ofstream file_;
  bool file_initialized_{false};
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

// Append to path, or pass std::nullopt for console-only logging.
// A failed open leaves console logging enabled and returns false.
inline bool set_log_file(const std::optional<std::string> &path) {
  LogState &s = log_state();
  const std::scoped_lock lock(s.mutex_);
  if (s.file_.is_open())
    s.file_.close();
  s.file_.clear();
  s.file_path_ = path;
  s.file_initialized_ = true;
  s.warned_ = false;
  if (!path)
    return true;
  s.file_.open(*path, std::ios::app);
  return s.file_.is_open();
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

  if (!s.file_path_)
    return;

  if (!s.file_initialized_) {
    s.file_initialized_ = true;
    s.file_.open(*s.file_path_, std::ios::app);
  }
  if (!s.file_.is_open()) {
    if (!s.warned_) {
      s.warned_ = true;
      std::cerr << "Patronus: cannot open " << *s.file_path_ << ", logging to the console only\n";
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

#define LOG(level, ...)                                                             \
  ::patronus::logging::log_line(::patronus::logging::Level::level, [&](auto &buf) { \
    buf << __VA_ARGS__;                                                             \
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

} // namespace patronus::logging
