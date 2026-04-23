#include "deepstream/nvdinfer/yolo_inference_mono.h"
#include "deepstream/nvdinfer/yolo_inference_rgb.h"
#include "state.hpp"
#include "tracking.hpp"
#include "comm.hpp"
#include <gst/gst.h>
#include <cstdio>
#include <thread>

float KP = 0.2;

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
  gst_init(&argc, &argv);
  int serial_motor = serial_open("");
  int sensor_motor = serial_open("");
  float x;
  float y;
  char *rgb_config  = (char *)"deepstream/config/config_infer_primary_rgb.txt";
  char *mono_config = (char *)"deepstream/config/config_infer_primary_mono.txt";
  char *rgb_argv[]  = {argv[0], rgb_config};
  char *mono_argv[] = {argv[0], mono_config};

  std::thread rgb_thread([&]() { run_pipeline_rgb(2, rgb_argv); });
  std::thread mono_thread([&]() { run_pipeline_mono(2, mono_argv); });

  // TODO: choosing detection logic, send to motors.
  // Later point Kalman filter, implement in tracking.hpp
  std::thread tracking([&]() {
    while (true) {
      NvDsObjectMeta rgb_obj = detection_rgb.pop();
      Point rgb_center = compute_center(rgb_obj);
      NvDsObjectMeta mono_obj = detection_mono.pop();
      Point mono_center = compute_center(mono_obj);

      bool use_rgb = true; //TODO: choose which camera feed to use
      if (use_rgb) {
        compute_control(rgb_center, KP, x, y);
        auto positions = motor_read(serial_motor); //TODO: compute angle for shooter module
        motor_send(serial_motor, 0.0f, 0.0f, 0.0f, 0.0f);
      }
      else {
        compute_control(mono_center, KP, x, y);
        auto positions = motor_read(serial_motor); //TODO: compute angle for shooter module
        motor_send(serial_motor, 0.0f, 0.0f, 0.0f, 0.0f);
      }
      
    }
  });

  rgb_thread.join();
  mono_thread.join();
  tracking.join();
  close(serial_motor);
  return 0;
}
