#pragma once

#include <cstdint>
#include <iosfwd>
#include <map>
#include <string>
#include <vector>

namespace patronus::profiling {

struct Event {
  std::string kind, name, stream;
  long thread;
  int64_t start, duration;
  double value;
  uint64_t id;
  int64_t cpu;
};

using Groups = std::map<std::string, std::vector<double>>;

struct TreeRoot {
  std::string title, context;
};

// Optional application-owned report extensions. Events and root pointers are
// valid only during reporting, after producers have stopped. Null callbacks
// provide the generic report with every span included in the function tree.
using ReportExtension = void (*)(std::ostream &, const std::vector<Event> &,
                                 const std::vector<const Event *> &, double);
struct ReportOptions {
  TreeRoot (*tree_root)(const Event &) = nullptr;
  ReportExtension summary = nullptr;
  ReportExtension details = nullptr;
};

#ifdef PATRONUS_PROFILING
// Ranked samples are milliseconds; unranked samples retain their supplied units.
void print_table(std::ostream &out, const char *title, Groups groups, double seconds,
                 bool ranked = true, bool tree = false);

// Construct before worker threads, destroy after all workers have joined.
class Session {
public:
  explicit Session(ReportOptions options = {});
  ~Session();
  Session(const Session &) = delete;
  Session &operator=(const Session &) = delete;
};

int64_t now_ns();
// -1 means unavailable; never replace a failed CPU clock read with zero usage.
int64_t thread_cpu_ns();
void record(const char *kind, const char *name, const char *stream, int64_t start_ns,
            int64_t duration_ns = 0, double value = 0, uint64_t id = UINT64_MAX,
            int64_t cpu_ns = -1);

class Scope {
public:
  Scope(const char *name, const char *stream = "process", uint64_t id = UINT64_MAX)
    : name_(name), stream_(stream), id_(id), start_(now_ns()), cpu_start_(thread_cpu_ns()) {
  }
  ~Scope() {
    const auto cpu_end = thread_cpu_ns();
    record("span", name_, stream_, start_, now_ns() - start_, 0, id_,
           cpu_start_ >= 0 && cpu_end >= cpu_start_ ? cpu_end - cpu_start_ : -1);
  }
  Scope(const Scope &) = delete;
  Scope &operator=(const Scope &) = delete;

private:
  const char *name_;
  const char *stream_;
  uint64_t id_;
  int64_t start_;
  int64_t cpu_start_;
};
#else
class Session {
public:
  explicit Session(ReportOptions = {}) {
  }
};
#endif

} // namespace patronus::profiling

#ifdef PATRONUS_PROFILING
  #define PATRONUS_PROFILE(name, stream, id) \
    const patronus::profiling::Scope profile_scope_##name(#name, stream, id)
  #define PATRONUS_PROFILE_CALL(name, stream, id, ...) \
    [&]() {                                            \
      PATRONUS_PROFILE(name, stream, id);              \
      return (__VA_ARGS__);                            \
    }()
#else
  #define PATRONUS_PROFILE(name, stream, id) ((void)0)
  #define PATRONUS_PROFILE_CALL(name, stream, id, ...) (__VA_ARGS__)
#endif
