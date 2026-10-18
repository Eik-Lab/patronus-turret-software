// C++ program to implement a basic logging system.

#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>

// Enum to represent log levels
enum LogLevel { DEBUG, INFO, WARNING, ERROR, CRITICAL };

class Logger {
public:
  // Constructor: Opens the log file in append mode
  Logger(const std::string &filename, LogLevel minLevel = WARNING) : minLevel(minLevel) {
    logFile.open(filename, std::ios::app);
    if (!logFile.is_open()) {
      std::cerr << "Error opening log file." << std::endl;
    }
  }

  // Destructor: Closes the log file
  ~Logger() {
    logFile.close();
  }

  // Logs a message with a given log level
  void log(LogLevel level, const std::string &message) {
    if (level < minLevel) {
      return;
    };
    // Get current timestamp
    time_t now = time(0);
    tm *timeinfo = localtime(&now);
    char timestamp[20];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", timeinfo);

    // Create log entry
    std::ostringstream logEntry;
    logEntry << "[" << timestamp << "] " << levelToString(level) << ": " << message << std::endl;

    // Output to console
    std::cout << "[" << timestamp << "] " << colorForLevel(level) << levelToString(level)
              << "\033[0m"
              << ": " << message << std::endl;

    // Output to log file
    if (logFile.is_open()) {
      logFile << logEntry.str();
      logFile.flush(); // Ensure immediate write to file
    }
  }

private:
  std::ofstream logFile; // File stream for the log file
  LogLevel minLevel;

  // Add colors to logging levels
  const char *colorForLevel(LogLevel level) {
    switch (level) {
      case DEBUG:
        return "\033[90m"; // Gray
      case INFO:
        return "\033[32m"; // Green
      case WARNING:
        return "\033[33m"; // Yellow
      case ERROR:
        return "\033[31m"; // Red
      case CRITICAL:
        return "\033[1;31m"; // Bold red
      default:
        return "\033[0m"; // Reset
    }
  }

  // Converts log level to a string for output
  std::string levelToString(LogLevel level) {
    switch (level) {
      case DEBUG:
        return "DEBUG";
      case INFO:
        return "INFO";
      case WARNING:
        return "WARNING";
      case ERROR:
        return "ERROR";
      case CRITICAL:
        return "CRITICAL";
      default:
        return "UNKNOWN";
    }
  }
};
