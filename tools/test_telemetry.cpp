// Writes a small public-API fixture and immediately shuts down. The Python
// regression checks its CSVs, including the final batch and optional fields.
#include "tracking/telemetry.h"

#include <iostream>
#include <limits>

int main(int argc, char **argv) {
  if (argc != 3)
    return 2;
  auto telemetry = patronus::tracking::Telemetry::open(argv[1], argv[2]);
  if (!telemetry)
    return 1;
  std::cout << telemetry->directory() << '\n';
  telemetry->note("gimbal 0 camera rgb home_tilt 0");
  patronus::tracking::TickRecord tick;
  tick.driving = true;
  tick.control_elapsed_ns = 499999999;
  tick.raw_detection = true;
  tick.estimate_available = true;
  tick.loop_start_s = 1.25;
  tick.loop_us = 12000;
  tick.pose_read_us = 1200;
  tick.observation_wait_us = 9000;
  tick.filter_us = 2.5;
  tick.aim_s = 1.26;
  tick.command_done_s = 1.2607;
  tick.aim = patronus::tracking::AimPoint{};
  tick.previous_submit_us = 3.25;
  tick.iteration_us = 13000;
  tick.control_us = .125;
  tick.aim_compute_us = 1.75;
  tick.follower_us = 420;
  tick.command_start_s = 1.26000175;
  tick.observation_received_s = 1.259;
  tick.skipped_observations = 7;
  tick.source_frame_id = 123;
  tick.source_capture_s = 1.23;
  tick.source_probe_s = 1.258;
  tick.source_publish_s = 1.2585;
  tick.bus = {5.25, 900.5, 320.75, 6, 8};
  tick.cmd_pan_rad_s = .25F;
  tick.cmd_tilt_rad_s = -.5F;
  tick.position.pan = {.1F, 1.25, patronus::comm::IoStatus::Ok, 1};
  tick.position.tilt = {-.2F, 1.251, patronus::comm::IoStatus::Ok, 1};
  tick.command.pan = {.5F, .25F, patronus::comm::IoStatus::Ok, 1, {}};
  tick.command.tilt = {-1.0F, -.5F, patronus::comm::IoStatus::Error, 3, {}};
  tick.command.stop_pan.status = patronus::comm::IoStatus::Ok;
  tick.command.stop_pan.sdk_error = 1;
  tick.command.stop_tilt.status = patronus::comm::IoStatus::Error;
  tick.command.stop_tilt.sdk_error = 3;
  telemetry->record(tick);
  tick.aim.reset();
  tick.cmd_pan_rad_s = std::numeric_limits<float>::quiet_NaN();
  tick.loop_start_s = 1.262;
  tick.control_elapsed_ns = 1000000000;
  tick.raw_detection = false;
  tick.estimate_available = false;
  tick.pose_usable = false;
  tick.observation_received_s = 0;
  tick.skipped_observations = 0;
  tick.bus = {};
  tick.position.tilt.status = patronus::comm::IoStatus::Error;
  tick.position.tilt.sdk_error = 3;
  telemetry->record(tick);
  patronus::tracking::FrameRecord frame;
  frame.obs.published_s_ = 1.2585;
  telemetry->record(frame);
}
