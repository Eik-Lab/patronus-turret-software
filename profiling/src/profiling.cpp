#include "patronus/profiling.h"

#include <sys/resource.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifdef PATRONUS_HAVE_NVML
  #include <dlfcn.h>
  #include <nvml.h>
#endif

namespace patronus::profiling {
void print_table(std::ostream &out, const char *title, Groups groups, double seconds, bool ranked,
                 bool tree) {
  std::vector<std::pair<std::string, std::vector<double>>> rows;
  std::map<std::string, std::string> last_child;
  double grand_total = 0;
  for (auto &[name, values] : groups) {
    if (tree) {
      const auto last = name.rfind('/');
      if (last != std::string::npos)
        last_child[name.substr(0, last)] = name;
    }
    grand_total += std::accumulate(values.begin(), values.end(), 0.0);
    rows.emplace_back(name, std::move(values));
  }
  if (ranked && !tree)
    std::sort(rows.begin(), rows.end(), [](const auto &a, const auto &b) {
      return std::accumulate(a.second.begin(), a.second.end(), 0.0) >
             std::accumulate(b.second.begin(), b.second.end(), 0.0);
    });
  const auto number = [](double value) {
    std::ostringstream text;
    text << std::fixed << std::setprecision(3) << value;
    return text.str();
  };
  const auto measurement = [&](double value) {
    if (!ranked)
      return number(value);
    if (value >= 1000)
      return number(value / 1000) + " s";
    if (value >= 1)
      return number(value) + " ms";
    if (value >= 0.001)
      return number(value * 1000) + " us";
    return number(value * 1000000) + " ns";
  };
  std::vector<std::vector<std::string>> cells{
    {"Metric", ranked ? "Calls" : "Samples", "Avg", "P50", "P95", "P99", "P999"}};
  if (ranked)
    cells.front().insert(cells.front().end(), {"Total", tree ? "Calls/s" : "% Total"});
  if (ranked && !tree)
    cells.front().emplace_back("Calls/s");
  for (auto &[name, values] : rows) {
    if (tree) {
      const auto last = name.rfind('/');
      if (last != std::string::npos) {
        std::string prefix;
        for (auto separator = name.find('/'); separator != std::string::npos;) {
          const auto next = name.find('/', separator + 1);
          const bool final_child = last_child.at(name.substr(0, separator)) == name.substr(0, next);
          prefix += next == std::string::npos ? (final_child ? "└─ " : "├─ ")
                                              : (final_child ? "   " : "│  ");
          separator = next;
        }
        prefix.append(name, last + 1);
        name = std::move(prefix);
      }
    }
    if (values.empty() && !tree)
      continue;
    if (values.empty()) {
      std::vector<std::string> row(cells.front().size(), "-");
      row[0] = name;
      row[2] = "unavailable";
      cells.push_back(std::move(row));
      continue;
    }
    std::sort(values.begin(), values.end());
    const double total = std::accumulate(values.begin(), values.end(), 0.0);
    std::vector<std::string> row{name, std::to_string(values.size()),
                                 measurement(total / values.size())};
    for (double p : {0.5, 0.95, 0.99, 0.999})
      row.push_back(measurement(values[static_cast<size_t>(std::ceil(p * values.size())) - 1]));
    if (ranked) {
      row.push_back(measurement(total));
      if (!tree)
        row.push_back(number(grand_total > 0 ? 100 * total / grand_total : 0) + "%");
      row.push_back(number(values.size() / seconds));
    }
    cells.push_back(std::move(row));
  }
  // The generated UTF-8 tree connectors occupy one terminal column each,
  // whereas ostream field widths count bytes. Keep numeric columns aligned.
  const auto columns = [](const std::string &text) {
    return static_cast<size_t>(std::count_if(text.begin(), text.end(), [](unsigned char byte) {
      return (byte & 0xc0) != 0x80;
    }));
  };
  std::vector<size_t> widths(cells.front().size());
  for (const auto &row : cells)
    for (size_t i = 0; i < row.size(); ++i)
      widths[i] = std::max(widths[i], columns(row[i]));
  const auto border = [&] {
    out << '+';
    for (const auto width : widths)
      out << std::string(width + 2, '-') << '+';
    out << '\n';
  };
  out << '\n' << title << '\n';
  border();
  for (size_t r = 0; r < cells.size(); ++r) {
    out << '|';
    for (size_t i = 0; i < cells[r].size(); ++i)
      out << ' ' << (i == 0 ? std::left : std::right)
          << std::setw(static_cast<int>(widths[i] + cells[r][i].size() - columns(cells[r][i])))
          << cells[r][i] << " |";
    out << '\n';
    if (r == 0)
      border();
  }
  if (cells.size() > 1)
    border();
}

namespace {
  class Recorder {
  public:
    explicit Recorder(ReportOptions options) : options_(options) {
      worker_ = std::thread([this] {
        sample_resources();
      });
    }

    ~Recorder() {
      {
        const std::lock_guard lock(mutex_);
        stopping_ = true;
      }
      cv_.notify_one();
      worker_.join();
      report();
    }

    void append(Event event) {
      const std::lock_guard lock(mutex_);
      if (events_.size() == capacity)
        ++dropped_;
      else
        events_.push_back(std::move(event));
    }

  private:
    void report() {
      const double seconds = std::max(1e-9, static_cast<double>(now_ns() - started) / 1e9);
      Groups wall, cpu, self_wall, self_cpu, estimates, resources;
      std::map<std::string, Groups> trees;
      std::vector<const Event *> roots;
      std::map<long, std::vector<const Event *>> threads;
      std::ostringstream out;
      out << std::fixed << std::setprecision(3) << "\n=== Patronus profiling (stdout only) ===\n"
          << "elapsed_s=" << seconds << " retained_events=" << events_.size()
          << " dropped_events=" << dropped_ << '\n'
          << "Durations carry ns/us/ms/s units. Percentiles: nearest rank; P999 = 99.9%.\n"
          << "Fewer than 1000 samples have insufficient p999 tail resolution.\n"
          << "Inclusive totals double-count nesting; shares are within each table, not process "
             "usage.\n";
      if (dropped_)
        out << "WARNING: event capacity reached; all distributions are partial, self times "
               "unavailable.\n";
      const auto label = [](const Event &event) {
        return event.stream + "/" + event.name + "@" + std::to_string(event.thread);
      };
      for (const auto &event : events_) {
        if (event.kind == "span") {
          wall[label(event)].push_back(event.duration / 1e6);
          if (event.cpu >= 0)
            cpu[label(event)].push_back(event.cpu / 1e6);
          threads[event.thread].push_back(&event);
        } else if (event.kind == "gauge") {
          resources[event.name].push_back(event.value);
        } else if (event.kind == "estimate") {
          estimates[event.stream + "/" + event.name].push_back(event.duration / 1e6);
        }
      }
      out << "\nInstrumented thread totals (wall union, CPU roots; milliseconds):\n";
      for (auto &[thread, spans] : threads) {
        std::sort(spans.begin(), spans.end(), [](const Event *a, const Event *b) {
          return std::make_pair(a->start, -a->duration) < std::make_pair(b->start, -b->duration);
        });
        std::vector<size_t> stack;
        std::vector<int64_t> own_wall, own_cpu;
        std::map<std::string, Groups> thread_trees;
        std::vector<const Event *> thread_roots;
        bool valid = dropped_ == 0;
        int64_t union_wall = 0, union_end = 0, root_cpu = 0;
        bool cpu_available = true;
        for (size_t i = 0; i < spans.size(); ++i) {
          const auto &event = *spans[i];
          const int64_t end = event.start + event.duration;
          union_wall += std::max<int64_t>(0, end - std::max(union_end, event.start));
          union_end = std::max(union_end, end);
          own_wall.push_back(event.duration);
          own_cpu.push_back(event.cpu);
          while (!stack.empty() &&
                 spans[stack.back()]->start + spans[stack.back()]->duration <= event.start)
            stack.pop_back();
          if (!stack.empty()) {
            const size_t parent = stack.back();
            const auto &ancestor = *spans[parent];
            if (end > ancestor.start + ancestor.duration ||
                (event.start == ancestor.start && event.duration == ancestor.duration))
              valid = false;
            own_wall[parent] -= event.duration;
            if (event.cpu < 0 || own_cpu[parent] < event.cpu)
              own_cpu[parent] = -1;
            else
              own_cpu[parent] -= event.cpu;
          } else if (event.cpu < 0) {
            cpu_available = false;
          } else {
            root_cpu += event.cpu;
          }
          stack.push_back(i);
          // The application may select category roots; the default includes
          // every span. Neither mode infers ancestry across threads.
          size_t root = 0;
          TreeRoot category;
          for (; root < stack.size(); ++root) {
            const auto &candidate = *spans[stack[root]];
            category = options_.tree_root ? options_.tree_root(candidate)
                                          : TreeRoot{"Function tree", candidate.stream};
            if (!category.title.empty())
              break;
          }
          if (root < stack.size() && valid) {
            const auto &ancestor = *spans[stack[root]];
            if (root == stack.size() - 1)
              thread_roots.push_back(&event);
            std::string path =
              ancestor.name + " [" + category.context + "@" + std::to_string(thread) + "]";
            for (size_t j = root + 1; j < stack.size(); ++j)
              path += "/" + spans[stack[j]]->name;
            thread_trees[category.title][path].push_back(event.duration / 1e6);
          }
        }
        out << "thread=" << thread << " wall_ms=" << union_wall / 1e6 << " cpu_ms=";
        if (valid && cpu_available)
          out << root_cpu / 1e6;
        else
          out << "unavailable";
        out << '\n';
        if (!valid) {
          out << "Self times unavailable for thread " << thread
              << ": loss or ambiguous intervals.\n";
          continue;
        }
        roots.insert(roots.end(), thread_roots.begin(), thread_roots.end());
        for (const auto &[title, groups] : thread_trees)
          for (const auto &[path, values] : groups)
            trees[title][path].insert(trees[title][path].end(), values.begin(), values.end());
        for (size_t i = 0; i < spans.size(); ++i) {
          self_wall[label(*spans[i])].push_back(own_wall[i] / 1e6);
          if (own_cpu[i] >= 0)
            self_cpu[label(*spans[i])].push_back(own_cpu[i] / 1e6);
        }
      }
      if (options_.summary)
        options_.summary(out, events_, roots, seconds);
      out << "\nFunction trees: recorded same-thread nesting; roots identify stream and thread. "
             "Only observed functions are shown, without synthetic stage nodes.\n";
      for (auto &[title, groups] : trees)
        print_table(out, title.c_str(), std::move(groups), seconds, true, true);
      out << "\nDetailed execution hotspots (same samples, not additional category work):\n";
      print_table(out, "Hotspots: inclusive wall", std::move(wall), seconds);
      print_table(out, "Hotspots: self wall", std::move(self_wall), seconds);
      print_table(out, "Hotspots: inclusive CPU (available measurements only)", std::move(cpu),
                  seconds);
      print_table(out, "Hotspots: self CPU (unavailable measurements omitted)", std::move(self_cpu),
                  seconds);
      print_table(out, "Estimated durations (not execution hotspots)", std::move(estimates),
                  seconds);
      if (options_.details)
        options_.details(out, events_, roots, seconds);
      print_table(out, "Resources (units in metric names; sampled every 250ms)", resources, seconds,
                  false);
      out << "\nResource trends (first / last / delta / peak):\n";
      for (const auto &[name, values] : resources)
        out << name << '\t' << values.front() << '\t' << values.back() << '\t'
            << values.back() - values.front() << '\t'
            << *std::max_element(values.begin(), values.end()) << '\n';
      for (const char *name :
           {"process_cpu_percent", "system_cpu_percent", "process_rss_bytes",
            "process_virtual_bytes", "system_memory_used_bytes", "gpu_util_percent",
            "gpu_memory_used_bytes", "gpu_memory_total_bytes"})
        if (!resources.count(name))
          out << name << ": unavailable\n";
      out << "=== End Patronus profiling ===\n";
      std::cout << out.str() << std::flush;
    }

    void gauge(const char *name, double value) {
      append({"gauge", name, "process", syscall(SYS_gettid), now_ns(), 0, value, UINT64_MAX, -1});
    }

    void sample() {
      struct rusage usage{};
      const int64_t now = now_ns();
      if (getrusage(RUSAGE_SELF, &usage) == 0) {
        const double cpu =
          static_cast<double>(usage.ru_utime.tv_sec + usage.ru_stime.tv_sec) +
          static_cast<double>(usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) / 1e6;
        if (previous_ns_ > 0)
          gauge("process_cpu_percent",
                100 * (cpu - previous_cpu_) * 1e9 / static_cast<double>(now - previous_ns_));
        previous_cpu_ = cpu;
        previous_ns_ = now;
      }
      uint64_t virtual_pages = 0, resident_pages = 0;
      if (std::ifstream("/proc/self/statm") >> virtual_pages >> resident_pages) {
        const auto page_size = sysconf(_SC_PAGESIZE);
        gauge("process_rss_bytes", static_cast<double>(resident_pages) * page_size);
        gauge("process_virtual_bytes", static_cast<double>(virtual_pages) * page_size);
      }
      std::ifstream mem("/proc/meminfo");
      std::string line;
      uint64_t total = 0, available = 0;
      while (std::getline(mem, line)) {
        std::istringstream fields(line);
        std::string key;
        uint64_t kb = 0;
        fields >> key >> kb;
        if (key == "MemTotal:")
          total = kb;
        if (key == "MemAvailable:")
          available = kb;
      }
      if (total > 0 && available <= total)
        gauge("system_memory_used_bytes", static_cast<double>(total - available) * 1024);
      std::ifstream cpu_file("/proc/stat");
      if (std::getline(cpu_file, line)) {
        std::istringstream fields(line);
        std::string label;
        uint64_t user = 0, nice = 0, system = 0, idle = 0, wait = 0, irq = 0, softirq = 0,
                 steal = 0;
        if (fields >> label >> user >> nice >> system >> idle >> wait >> irq >> softirq >> steal) {
          const uint64_t ticks = user + nice + system + idle + wait + irq + softirq + steal;
          const uint64_t idle_ticks = idle + wait;
          if (previous_ticks_ > 0 && ticks > previous_ticks_)
            gauge("system_cpu_percent",
                  100.0 * (1.0 - static_cast<double>(idle_ticks - previous_idle_) /
                                   static_cast<double>(ticks - previous_ticks_)));
          previous_ticks_ = ticks;
          previous_idle_ = idle_ticks;
        }
      }
#ifdef PATRONUS_HAVE_NVML
      if (device_) {
        nvmlUtilization_t utilization{};
        if (utilization_(device_, &utilization) == NVML_SUCCESS)
          gauge("gpu_util_percent", utilization.gpu);
        nvmlMemory_t memory{};
        if (memory_(device_, &memory) == NVML_SUCCESS) {
          gauge("gpu_memory_used_bytes", static_cast<double>(memory.used));
          gauge("gpu_memory_total_bytes", static_cast<double>(memory.total));
        }
      }
#endif
    }

    void sample_resources() {
#ifdef PATRONUS_HAVE_NVML
      // Optional at runtime: profiling remains usable without NVIDIA drivers.
      void *library = dlopen("libnvidia-ml.so.1", RTLD_NOW | RTLD_LOCAL);
      auto init =
        library ? reinterpret_cast<decltype(&nvmlInit_v2)>(dlsym(library, "nvmlInit_v2")) : nullptr;
      auto shutdown = library
                        ? reinterpret_cast<decltype(&nvmlShutdown)>(dlsym(library, "nvmlShutdown"))
                        : nullptr;
      auto device = library ? reinterpret_cast<decltype(&nvmlDeviceGetHandleByIndex_v2)>(
                                dlsym(library, "nvmlDeviceGetHandleByIndex_v2"))
                            : nullptr;
      utilization_ = library ? reinterpret_cast<decltype(utilization_)>(
                                 dlsym(library, "nvmlDeviceGetUtilizationRates"))
                             : nullptr;
      memory_ = library
                  ? reinterpret_cast<decltype(memory_)>(dlsym(library, "nvmlDeviceGetMemoryInfo"))
                  : nullptr;
      const bool initialized =
        init && shutdown && device && utilization_ && memory_ && init() == NVML_SUCCESS;
      if (initialized && device(0, &device_) != NVML_SUCCESS)
        device_ = nullptr;
#endif
      std::unique_lock lock(mutex_);
      for (;;) {
        const bool stopping = stopping_;
        lock.unlock();
        sample();
        if (stopping)
          break;
        lock.lock();
        cv_.wait_for(lock, std::chrono::milliseconds(250), [this] {
          return stopping_;
        });
      }
#ifdef PATRONUS_HAVE_NVML
      if (initialized)
        shutdown();
      if (library)
        dlclose(library);
#endif
    }

    // Bound profiling memory; report loss explicitly rather than silently sample.
    static constexpr size_t capacity = 1000000;
    ReportOptions options_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool stopping_{false};
    uint64_t dropped_{0};
    std::vector<Event> events_;
    const int64_t started = now_ns();
    std::thread worker_;
    int64_t previous_ns_{0};
    double previous_cpu_{0};
    uint64_t previous_ticks_{0}, previous_idle_{0};
#ifdef PATRONUS_HAVE_NVML
    nvmlDevice_t device_{nullptr};
    decltype(&nvmlDeviceGetUtilizationRates) utilization_{nullptr};
    decltype(&nvmlDeviceGetMemoryInfo) memory_{nullptr};
#endif
  };

  Recorder *recorder = nullptr;
} // namespace

int64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

Session::Session(ReportOptions options) {
  recorder = new Recorder(options);
}

Session::~Session() {
  delete recorder;
  recorder = nullptr;
}

int64_t thread_cpu_ns() {
  timespec time{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &time) != 0)
    return -1;
  return time.tv_sec * 1000000000LL + time.tv_nsec;
}

void record(const char *kind, const char *name, const char *stream, int64_t start_ns,
            int64_t duration_ns, double value, uint64_t id, int64_t cpu_ns) {
  if (recorder) {
    thread_local const long thread_id = syscall(SYS_gettid);
    recorder->append({kind, name, stream, thread_id, start_ns, duration_ns, value, id, cpu_ns});
  }
}
} // namespace patronus::profiling
