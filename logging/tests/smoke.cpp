#include <patronus/logging.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace {
void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}

std::string read(const std::filesystem::path &path) {
  std::ifstream file(path);
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}
} // namespace

int main() {
  using namespace patronus::logging;
  const auto original = std::filesystem::current_path();
  const auto temp = std::filesystem::temp_directory_path() /
                    ("patronus-logging-" +
                     std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directory(temp);
  std::filesystem::current_path(temp);
  std::ostringstream output;
  std::ostringstream errors;
  auto *old_output = std::cout.rdbuf(output.rdbuf());
  auto *old_errors = std::cerr.rdbuf(errors.rdbuf());
  int result = 0;
  try {
    require(set_log_file(std::nullopt), "disable file output");
    int evaluated = 0;
    LOG(Info, ++evaluated);
    require(evaluated == 0 && output.str().empty(), "default threshold filters lazily");
    LOG(Warning, "console only");
    require(!std::filesystem::exists("patronus.log"), "console-only must not create default file");
    require(output.str().find("WARNING\033[0m: console only") != std::string::npos,
            "console warning output");

    std::ofstream("chosen.log") << "existing\n";
    require(set_log_file("chosen.log"), "custom file opens");
    set_log_level(Level::Error);
    LOG(Warning, ++evaluated);
    LOG(Error, "first error");
    require(evaluated == 0, "configured threshold filters lazily");
    require(read("chosen.log").find("existing\n") == 0, "append preserves existing content");
    require(read("chosen.log").find("ERROR: first error\n") != std::string::npos,
            "file output is flushed and uncolored");

    require(set_log_file("second.log"), "switch file");
    std::thread worker([] {
      LOG(Critical, "worker message");
    });
    worker.join();
    require(set_log_file(std::nullopt), "disable after writing");
    LOG(Error, "not in file");
    require(read("chosen.log").find("worker message") == std::string::npos, "old file is closed");
    require(read("second.log").find("CRITICAL: worker message\n") != std::string::npos,
            "worker shares destination");
    require(read("second.log").find("not in file") == std::string::npos,
            "disabled file stays unchanged");

    require(!set_log_file("missing/destination.log"), "open failure reported");
    LOG(Error, "fallback one");
    LOG(Error, "fallback two");
    require(output.str().find("fallback two") != std::string::npos,
            "console survives open failure");
    const auto warning = errors.str().find("cannot open missing/destination.log");
    require(warning != std::string::npos &&
              errors.str().find("cannot open", warning + 1) == std::string::npos,
            "open failure warns once");
    require(set_log_file("second.log"), "recover from open failure");
    LOG(Error, "recovered");
    require(read("second.log").find("ERROR: recovered\n") != std::string::npos, "recovered output");

    require(parse_level("debug") == Level::Debug && parse_level("info") == Level::Info &&
              parse_level("warning") == Level::Warning && parse_level("warn") == Level::Warning &&
              parse_level("error") == Level::Error && parse_level("critical") == Level::Critical &&
              !parse_level("unknown"),
            "level parsing");
  } catch (const std::exception &error) {
    result = 1;
    std::cerr.rdbuf(old_errors);
    std::cerr << error.what() << '\n';
  }
  set_log_file(std::nullopt);
  std::cout.rdbuf(old_output);
  std::cerr.rdbuf(old_errors);
  std::filesystem::current_path(original);
  std::filesystem::remove_all(temp);
  return result;
}
