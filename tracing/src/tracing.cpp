#include "patronus/tracing.h"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace patronus::tracing {
namespace {

  struct Event {
    std::string stage;
    int64_t monotonic_ns;
    uint64_t pts;
    uint64_t offset;
    size_t bytes;
  };

  using File = std::unique_ptr<std::FILE, int (*)(std::FILE *)>;

  class Trace {
  public:
    explicit Trace(File file) : file_(std::move(file)) {
      pending_.reserve(capacity);
      batch_.reserve(capacity);
      std::fputs("stage,monotonic_ns,pts_ns,offset,bytes\n", file_.get());
      writer_ = std::thread([this]() {
        write();
      });
    }
    ~Trace() {
      {
        const std::lock_guard lock(mutex_);
        stopping_ = true;
      }
      cv_.notify_one();
      writer_.join();
    }
    Trace(const Trace &) = delete;
    Trace &operator=(const Trace &) = delete;

    void record(const std::string &stage, GstBuffer *buffer) {
      const auto now = std::chrono::steady_clock::now().time_since_epoch();
      Event event{stage, std::chrono::duration_cast<std::chrono::nanoseconds>(now).count(),
                  GST_BUFFER_PTS(buffer), GST_BUFFER_OFFSET(buffer), gst_buffer_get_size(buffer)};
      const std::lock_guard lock(mutex_);
      if (pending_.size() == capacity) {
        ++dropped_;
      } else {
        pending_.push_back(std::move(event));
      }
    }

  private:
    void write() {
      std::unique_lock lock(mutex_);
      for (;;) {
        cv_.wait_for(lock, std::chrono::milliseconds(250), [this]() {
          return stopping_;
        });
        pending_.swap(batch_);
        const bool stopping = stopping_;
        const auto dropped = dropped_;
        lock.unlock();
        for (const auto &event : batch_) {
          std::fprintf(file_.get(), "%s,%lld,%llu,%llu,%zu\n", event.stage.c_str(),
                       static_cast<long long>(event.monotonic_ns),
                       static_cast<unsigned long long>(event.pts),
                       static_cast<unsigned long long>(event.offset), event.bytes);
        }
        // Periodic counters survive a forced process stop; complete=1 requires teardown.
        std::fprintf(file_.get(), "# dropped=%llu complete=%d\n",
                     static_cast<unsigned long long>(dropped), stopping ? 1 : 0);
        if (std::fflush(file_.get()) != 0 || std::ferror(file_.get()) != 0) {
          std::fputs("Trace write failed; recording is incomplete\n", stderr);
          return;
        }
        batch_.clear();
        if (stopping)
          return;
        lock.lock();
      }
    }

    static constexpr size_t capacity = 8192;
    File file_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool stopping_{false};
    uint64_t dropped_{0};
    std::vector<Event> pending_;
    std::vector<Event> batch_;
    std::thread writer_;
  };

  struct Probe {
    std::shared_ptr<Trace> trace;
    std::string stage;
  };

  GstPadProbeReturn record_buffer(GstPad *, GstPadProbeInfo *info, gpointer data) {
    auto *probe = static_cast<Probe *>(data);
    if (auto *buffer = GST_PAD_PROBE_INFO_BUFFER(info); buffer != nullptr)
      probe->trace->record(probe->stage, buffer);
    return GST_PAD_PROBE_OK;
  }

} // namespace

bool install(GstElement *pipeline, const std::string &path,
             const std::vector<Location> &locations) {
  for (const auto &location : locations)
    if (location.stage.find_first_of(",\r\n\"") != std::string::npos) {
      std::fputs("Trace stage names cannot contain commas, quotes or newlines\n", stderr);
      return false;
    }
  // Never overwrite an earlier run, even if the caller reuses a directory.
  File file(std::fopen(path.c_str(), "wx"), &std::fclose);
  if (!file) {
    std::fprintf(stderr, "Cannot create requested trace: %s\n", path.c_str());
    return false;
  }
  auto trace = std::make_shared<Trace>(std::move(file));
  for (const auto &location : locations) {
    GstElement *element = gst_bin_get_by_name(GST_BIN(pipeline), location.element.c_str());
    GstPad *pad =
      element != nullptr ? gst_element_get_static_pad(element, location.pad.c_str()) : nullptr;
    if (element != nullptr)
      gst_object_unref(element);
    if (pad == nullptr) {
      std::fprintf(stderr, "Cannot trace missing pad: %s\n", location.stage.c_str());
      return false;
    }
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, record_buffer,
                      new Probe{trace, location.stage}, [](gpointer data) {
                        delete static_cast<Probe *>(data);
                      });
    gst_object_unref(pad);
  }
  return true;
}

} // namespace patronus::tracing
