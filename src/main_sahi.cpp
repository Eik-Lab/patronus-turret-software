#include "deepstream/nvdinfer/yolo_inference_rgb_sahi.h"
#include "deepstream/nvdinfer/yolo_inference_mono.h"
#include "state.hpp"
#include "tracking.hpp"
#include "comm.hpp"
#include <cstdio>
#include <thread>

struct Point
{
  float cx;
  float cy;
};

float KP = 0.2;

ThreadSafeQueue<NvDsObjectMeta> detection_rgb(5);
ThreadSafeQueue<NvDsObjectMeta> detection_mono(5);

Point compute_center(const NvDsObjectMeta &obj)
{
  float left = obj.rect_params.left;
  float top = obj.rect_params.top;
  float width = obj.rect_params.width;
  float height = obj.rect_params.height;
  return {left + width / 2.0f, top + height / 2.0f};
}

int main(int argc, char *argv[])
{
  int serial_motor = serial_open("");
  int sensor_motor = serial_open("");
  float x;
  float y;

  char *rgb_sahi_config = (char *)"deepstream/config/config_infer_primary_rgb_sahi.txt";
  char *preprocess_config = (char *)"deepstream/config/config_preprocess_rgb_sahi.txt";
  char *mono_config = (char *)"deepstream/config/config_infer_primary_rgb.txt";

  char *rgb_argv[] = {argv[0], rgb_sahi_config, preprocess_config};
  char *mono_argv[] = {argv[0], mono_config};

  std::thread rgb_thread([&]()
                         { run_pipeline_rgb_sahi(3, rgb_argv); });
  std::thread mono_thread([&]()
                          { run_pipeline_mono(2, mono_argv); });

  // TODO: choosing detection logic, send to motors.
  // Later point Kalman filter, implement in tracking.hpp
  std::thread tracking([&]()
                       {
    while (true) {
      NvDsObjectMeta obj = detection_rgb.pop();
      Point rgb_center = compute_center(obj);
      (void)rgb_center; // TODO: feed into tracking/motor control
      NvDsObjectMeta obj = detection_mono.pop();
      Point mono_center = compute_center(obj);
      (void)mono_center; // TODO: feed into tracking/motor control

      if () { //TODO:choose which camera feed to use
        compute_control(rgb_center, KP, x, y);
        auto positions = motor_read(fd); //TODO: compute angle for shooter module
        motor_send(fd, 0.0f, 0.0f, 0.0f, 0.0f);
      }
      else {
        compute_control(mono_center, KP, x, y);
        auto positions = motor_read(fd); //TODO: compute angle for shooter module
        motor_send(fd, 0.0f, 0.0f, 0.0f, 0.0f);
      }

    } });

  rgb_thread.join();
  mono_thread.join();
  tracking.join();
  close(serial_motor);
  return 0;
}
