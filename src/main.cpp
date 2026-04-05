#include "deepstream/nvdinfer/yolo_inference_mono.h"
#include "deepstream/nvdinfer/yolo_inference_rgb.h"
#include "state.hpp"
#include <cstdio>
#include <mutex>
#include <queue>
#include <thread>

struct Point {
  float cx;
  float cy;
};

std::mutex detection_rgb_mutex;
std::mutex detection_mono_mutex;

extern std::queue<NvDsObjectMeta> detection_rgb;
extern std::queue<NvDsObjectMeta> detection_mono;

Point compute_center(const NvDsObjectMeta& obj) {
  float left = obj.rect_params.left;
  float top = obj.rect_params.top;
  float width = obj.rect_params.width;
  float height = obj.rect_params.height;
  return {left + width / 2.0f, top + height / 2.0f};
}

bool try_pop_rgb(NvDsObjectMeta& out) {
  std::lock_guard<std::mutex> lock(detection_rgb_mutex);
  if (detection_rgb.empty()) return false;
  out = detection_rgb.front();
  detection_rgb.pop();
  return true;
}

bool try_pop_mono(NvDsObjectMeta& out) {
  std::lock_guard<std::mutex> lock(detection_mono_mutex);
  if (detection_mono.empty()) return false;
  out = detection_mono.front();
  detection_mono.pop();
  return true;
}

int main(int argc, char *argv[]) {
  if (argc != 3) {
    fprintf(stderr, "Usage: %s <rgb_config> <mono_config>\n", argv[0]);
    return -1;
  }

  char *rgb_argv[] = {argv[0], argv[1]};
  char *mono_argv[] = {argv[0], argv[2]};

  std::thread rgb_thread([&]() { run_pipeline_rgb(2, rgb_argv); });
  std::thread mono_thread([&]() { run_pipeline_mono(2, mono_argv); });

  // TODO: add loop for tracking, choosing detection logic, send to motors.
  // Later point Kalman filter
  std::thread tracking([&]() {
    NvDsObjectMeta obj{};
    while (true) {
      if (try_pop_rgb(obj)) {
        Point rgb_center = compute_center(obj);
        (void)rgb_center; // TODO: feed into tracking/motor control
      }
      if (try_pop_mono(obj)) {
        Point mono_center = compute_center(obj);
        (void)mono_center; // TODO: feed into tracking/motor control
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  rgb_thread.join();
  mono_thread.join();
  tracking.join();
  return 0;
}
