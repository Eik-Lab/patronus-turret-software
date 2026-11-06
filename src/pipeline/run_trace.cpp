#include "pipeline/run_trace.h"

#include "log.h"

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace patronus::pipeline {
namespace {

  struct Event {
    const char *stage;
    int64_t monotonic_ns;
    uint64_t pts;
    uint64_t offset;
    size_t bytes;
  };

  class Trace {
  public:
    explicit Trace(std::FILE *file) : file_(file) {
      pending_.reserve(capacity);
      batch_.reserve(capacity);
      std::fputs("stage,monotonic_ns,pts_ns,offset,bytes\n", file_);
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
      std::fclose(file_);
    }
    Trace(const Trace &) = delete;
    Trace &operator=(const Trace &) = delete;

    void record(const char *stage, GstBuffer *buffer) {
      const auto now = std::chrono::steady_clock::now().time_since_epoch();
      const Event event{stage, std::chrono::duration_cast<std::chrono::nanoseconds>(now).count(),
                        GST_BUFFER_PTS(buffer), GST_BUFFER_OFFSET(buffer),
                        gst_buffer_get_size(buffer)};
      const std::lock_guard lock(mutex_);
      if (pending_.size() == capacity) {
        ++dropped_;
      } else {
        pending_.push_back(event);
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
          std::fprintf(file_, "%s,%lld,%llu,%llu,%zu\n", event.stage,
                       static_cast<long long>(event.monotonic_ns),
                       static_cast<unsigned long long>(event.pts),
                       static_cast<unsigned long long>(event.offset), event.bytes);
        }
        // Periodic counters survive a forced process stop; complete=1 requires teardown.
        std::fprintf(file_, "# dropped=%llu complete=%d\n",
                     static_cast<unsigned long long>(dropped), stopping ? 1 : 0);
        if (std::fflush(file_) != 0 || std::ferror(file_) != 0) {
          LOG(Error, "Live trace write failed; recording is incomplete");
          return;
        }
        batch_.clear();
        if (stopping)
          return;
        lock.lock();
      }
    }

    static constexpr size_t capacity = 8192;
    std::FILE *file_;
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
    const char *stage;
  };

  GstPadProbeReturn record_buffer(GstPad *, GstPadProbeInfo *info, gpointer data) {
    auto *probe = static_cast<Probe *>(data);
    if (auto *buffer = GST_PAD_PROBE_INFO_BUFFER(info); buffer != nullptr)
      probe->trace->record(probe->stage, buffer);
    return GST_PAD_PROBE_OK;
  }

} // namespace

bool install_run_trace(GstElement *pipeline, const char *camera) {
  const char *directory = std::getenv("PATRONUS_TRACE_DIR");
  if (directory == nullptr || *directory == '\0')
    return true;
  const std::string path = std::string(directory) + "/pipeline-" + camera + ".csv";
  // Never overwrite an earlier run, even if the caller reuses a directory.
  std::FILE *file = std::fopen(path.c_str(), "wx");
  if (file == nullptr) {
    LOG(Error, "Cannot create requested live trace: " << path);
    return false;
  }
  auto trace = std::make_shared<Trace>(file);
  struct Location {
    const char *element;
    const char *pad;
    const char *stage;
  };
  for (const auto &location : {Location{"pylon-source", "src", "source"},
                               Location{"nvvideo-converter-pre", "sink", "convert_in"},
                               Location{"nvvideo-converter-pre", "src", "convert_out"},
                               Location{"stream-muxer", "src", "mux_out"},
                               Location{"primary-nvinference-engine", "sink", "infer_in"},
                               Location{"primary-nvinference-engine", "src", "infer_out"},
                               Location{"nv-onscreendisplay", "sink", "osd_in"},
                               Location{"nv-onscreendisplay", "src", "osd_out"},
                               Location{"nvvideo-converter-post", "sink", "post_in"},
                               Location{"nvvideo-converter-post", "src", "post_out"},
                               Location{"video-encoder", "sink", "encode_in"},
                               Location{"video-encoder", "src", "encode_out"}}) {
    GstElement *element = gst_bin_get_by_name(GST_BIN(pipeline), location.element);
    GstPad *pad = element != nullptr ? gst_element_get_static_pad(element, location.pad) : nullptr;
    if (element != nullptr)
      gst_object_unref(element);
    if (pad == nullptr) {
      LOG(Error, "Cannot trace missing pad: " << location.stage);
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

} // namespace patronus::pipeline
