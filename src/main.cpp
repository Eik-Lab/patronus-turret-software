#include "patronus/pipeline/inference_mono.hpp"
#include "patronus/pipeline/inference_rgb.hpp"
#include "patronus/comm/serial.hpp"
#include "patronus/tracking/aim_control.hpp"
#include "patronus/core/state.hpp"
#include <cstdio>
#include <thread>

LatestValue<NvDsObjectMeta> detection_rgb;
LatestValue<NvDsObjectMeta> detection_mono;

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
  if (argc != 3)
  {
    fprintf(stderr, "Usage: %s <rgb_config> <mono_config>\n", argv[0]);
    return -1;
  }

  int serial_motor = serial_open("");
  int sensor_motor = serial_open("");
  char *rgb_argv[] = {argv[0], argv[1]};
  char *mono_argv[] = {argv[0], argv[2]};

  std::thread rgb_thread([&]()
                         { run_pipeline_rgb(2, rgb_argv, detection_rgb); });
  std::thread mono_thread([&]()
                          { run_pipeline_mono(2, mono_argv, detection_mono); });

  std::thread tracking([&]()
                       {
    constexpr float Kp = 0.001f;
    while (true) {
      NvDsObjectMeta obj = detection_rgb.pop();
      Point rgb_center = compute_center(obj);

      auto positions = motor_read(serial_motor);

      AimAngles sensor = compute_control_sensor(rgb_center, Kp);
      AimAngles weapon = compute_control_weapon(positions[0], positions[1]);

      motor_send(serial_motor, sensor.pan, sensor.tilt, weapon.pan, weapon.tilt);
    } });

  rgb_thread.join();
  mono_thread.join();
  return 0;
}
