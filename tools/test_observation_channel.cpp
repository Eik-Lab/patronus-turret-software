#include "core/state.h"

#include <chrono>
#include <future>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

using namespace std::chrono_literals;
using patronus::core::LatestValue;

static void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}

int main() {
  try {
    LatestValue<std::string> rgb, mono;
    uint64_t fast = 0, slow = 0, mono_reader = 0;
    require(!rgb.try_read(fast, 0ms), "empty channel returned a value");
    rgb.push("frame one");
    require(rgb.try_read(fast, 0ms) == "frame one", "first leader missed frame");
    require(rgb.try_read(slow, 0ms) == "frame one", "second leader missed or received moved data");
    require(!rgb.try_read(fast, 0ms), "same frame processed twice");
    require(!mono.try_read(mono_reader, 0ms), "RGB publication leaked into mono");
    mono.push("mono frame");
    require(mono.try_read(mono_reader, 0ms) == "mono frame", "independent camera missed frame");
    rgb.push("frame two");
    require(rgb.try_read(fast, 0ms) == "frame two", "fast reader missed update");
    rgb.push("frame three");
    require(rgb.try_read(slow, 0ms) == "frame three", "slow reader did not skip superseded frame");
    require(rgb.try_read(fast, 0ms) == "frame three", "slow reader consumed fast reader's frame");
    require(!rgb.try_read(slow, 0ms), "slow reader repeated a frame");
    rgb.push("discarded");
    rgb.clear();
    require(!rgb.try_read(fast, 0ms), "clear retained an observation");
    rgb.push("after clear");
    require(rgb.try_read(fast, 0ms) == "after clear", "clear broke existing cursor");
    require(rgb.try_read(slow, 0ms) == "after clear", "clear broke second cursor");

    // Publication identity is independent of camera frame IDs and detection presence.
    LatestValue<patronus::core::FrameObservation> frames;
    uint64_t reader = 0;
    patronus::core::FrameObservation frame;
    frame.frame_id_ = 7;
    frames.push(frame);
    require(frames.try_read(reader, 0ms).has_value(), "no-detection frame was lost");
    frame.timestamp_s_ = 2.0;
    frames.push(frame);
    auto repeated_id = frames.try_read(reader, 0ms);
    require(repeated_id && repeated_id->timestamp_s_ == 2.0, "repeated frame ID hid publication");
    frame.frame_id_ = 0;
    frames.push(frame);
    auto reset_id = frames.try_read(reader, 0ms);
    require(reset_id && reset_id->frame_id_ == 0, "camera ID reset hid publication");

    // Both blocked readers should wake promptly, not wait until the timeout.
    LatestValue<int> broadcast;
    std::promise<void> ready_a, ready_b;
    auto read = [&](std::promise<void> &ready) {
      uint64_t cursor = 0;
      ready.set_value();
      return broadcast.try_read(cursor, 5s);
    };
    auto a = std::async(std::launch::async, read, std::ref(ready_a));
    auto b = std::async(std::launch::async, read, std::ref(ready_b));
    ready_a.get_future().wait();
    ready_b.get_future().wait();
    std::this_thread::sleep_for(50ms);
    broadcast.push(42);
    require(
      a.wait_for(1s) == std::future_status::ready && b.wait_for(1s) == std::future_status::ready,
      "publication did not wake both readers");
    require(a.get() == 42 && b.get() == 42, "blocked readers did not receive the same value");

    // A stopped producer leaves readers bounded by their existing timeout.
    LatestValue<int> stopped;
    auto waiting = std::async(std::launch::async, [&] {
      uint64_t cursor = 0;
      return stopped.try_read(cursor, 20ms);
    });
    require(waiting.wait_for(1s) == std::future_status::ready, "reader did not time out");
    require(!waiting.get(), "timeout fabricated an observation");
    std::cout << "Observation broadcast, independent cursors, skips, wakeups and timeouts passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
