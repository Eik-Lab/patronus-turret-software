#include "deepstream/nvdinfer/yolo_inference_mono.h"
#include "deepstream/nvdinfer/yolo_inference_rgb.h"
#include "state.hpp"
#include <cstdio>
#include <thread>

struct Point {
  float cx;
  float cy;
};

ThreadSafeQueue<NvDsObjectMeta> detection_rgb(5);
ThreadSafeQueue<NvDsObjectMeta> detection_mono(5);

Point compute_center(const NvDsObjectMeta& obj) {
  float left = obj.rect_params.left;
  float top = obj.rect_params.top;
  float width = obj.rect_params.width;
  float height = obj.rect_params.height;
  return {left + width / 2.0f, top + height / 2.0f};
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
    while (true) {
      NvDsObjectMeta obj = detection_rgb.pop();
      Point rgb_center = compute_center(obj);
      (void)rgb_center; // TODO: feed into tracking/motor control
      NvDsObjectMeta obj_mono = detection_mono.pop();
      Point mono_center = compute_center(obj_mono);
      (void)mono_center; // TODO: feed into tracking/motor control
    }
  });

  rgb_thread.join();
  mono_thread.join();
  tracking.join();
  return 0;
}
