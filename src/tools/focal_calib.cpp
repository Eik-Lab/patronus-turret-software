// Measures focal_x_px/focal_y_px: steps gimbal 1 by known angles, saves a
// frame from each camera per pose, and tools/focal_calib_analyse.py fits the
// image shift per radian.
//
// Aim at a static, textured scene over 30 m away. The cameras sit off the
// rotation axes, so near scenes overstate the focal length (2x at 1 m).
//
//   ./build/focal_calib [out_dir] [config/system.ini]
//   python3 tools/focal_calib_analyse.py out_dir

#include "comm/candle_motor.h"
#include "core/config.h"

#include <pylon/PylonIncludes.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <regex>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace std::chrono_literals;

/// Abort if the gimbal wanders further than this from the start.
constexpr float k_abort_rad = 0.08F;
/// Before velocity_scale (0.05).
constexpr float k_gain = 100.0F;
constexpr float k_max_velocity = 4.0F;
constexpr std::array<float, 7> k_offsets{0.0F, -0.03F, -0.015F, 0.0F, 0.015F, 0.03F, 0.0F};
constexpr float k_still_rad = 0.0001F;
constexpr auto k_still_for = 300ms;
constexpr auto k_settle_timeout = 6s;
constexpr float k_near_rad = 0.001F;
constexpr auto k_give_up_after = 3s;
constexpr double k_target_luma = 100.0;
constexpr double k_max_exposure_us = 50000.0;

struct Pose {
  float pan{0.0F};
  float tilt{0.0F};
};

struct Shared {
  std::mutex mutex;
  Pose target;
  Pose pose;
  std::atomic<bool> run{true};
  std::atomic<bool> aborted{false};
};

struct Camera {
  std::string name;
  const patronus::config::PipelineConfig *pipe{nullptr};
  Pylon::CInstantCamera device;
  double original_exposure_us{0.0};
  int luma_stride{1}; ///< Bytes between luma samples: 2 for YCbCr422_8.
  int width{0};
  int height{0};
};

Pose read_pose(Shared &s) {
  const std::scoped_lock lock(s.mutex);
  return s.pose;
}

void set_target(Shared &s, Pose p) {
  const std::scoped_lock lock(s.mutex);
  s.target = p;
}

void motor_loop(patronus::comm::CandleMotor &motor, Shared &s, Pose start) {
  while (s.run) {
    Pose target;
    {
      const std::scoped_lock lock(s.mutex);
      target = s.target;
    }
    const auto [pan, tilt] = motor.get_position(0);
    {
      const std::scoped_lock lock(s.mutex);
      s.pose = Pose{pan, tilt};
    }
    if (std::abs(pan - start.pan) > k_abort_rad || std::abs(tilt - start.tilt) > k_abort_rad) {
      s.aborted = true;
      break;
    }
    motor.drive_to(0, target.pan, target.tilt, k_gain, 0.0F, k_max_velocity);
    std::this_thread::sleep_for(10ms);
  }
  motor.set_velocity(0, 0.0F, 0.0F);
}

/// An axis that sticks short is accepted once still: the pose is recorded
/// per frame, so only stillness matters.
bool wait_still(Shared &s) {
  const auto start = std::chrono::steady_clock::now();
  Pose ref = read_pose(s);
  auto ref_t = start;
  while (!s.aborted && std::chrono::steady_clock::now() - start < k_settle_timeout) {
    std::this_thread::sleep_for(20ms);
    Pose p;
    Pose target;
    {
      const std::scoped_lock lock(s.mutex);
      p = s.pose;
      target = s.target;
    }
    const auto now = std::chrono::steady_clock::now();
    const bool near =
      std::abs(p.pan - target.pan) < k_near_rad && std::abs(p.tilt - target.tilt) < k_near_rad;
    if (std::abs(p.pan - ref.pan) > k_still_rad || std::abs(p.tilt - ref.tilt) > k_still_rad) {
      ref = p;
      ref_t = now;
    } else if (now - ref_t > k_still_for && (near || now - start > k_give_up_after)) {
      return near;
    }
  }
  return false;
}

/// LatestImageOnly holds one older frame back, possibly taken mid-move.
Pylon::CGrabResultPtr grab_fresh(Camera &c) {
  Pylon::CGrabResultPtr r;
  for (int k = 0; k < 3; ++k)
    c.device.RetrieveResult(2000, r, Pylon::TimeoutHandling_ThrowException);
  return r;
}

uint8_t luma_at(const Pylon::CGrabResultPtr &r, size_t i, int stride) {
  return static_cast<const uint8_t *>(r->GetBuffer())[i * static_cast<size_t>(stride)];
}

double mean_luma(const Pylon::CGrabResultPtr &r, int stride) {
  const size_t n = static_cast<size_t>(r->GetWidth()) * r->GetHeight();
  double sum = 0.0;
  size_t count = 0;
  for (size_t i = 0; i < n; i += 97) {
    sum += luma_at(r, i, stride);
    ++count;
  }
  return count > 0 ? sum / static_cast<double>(count) : 0.0;
}

/// At tracking exposure, indoor scenes are black and phase correlation
/// locks onto fixed-pattern noise.
void auto_expose(Camera &c) {
  Pylon::CFloatParameter exposure(c.device.GetNodeMap(), "ExposureTime");
  c.original_exposure_us = exposure.GetValue();
  for (int i = 0; i < 5; ++i) {
    const double luma = std::max(mean_luma(grab_fresh(c), c.luma_stride), 1.0);
    if (std::abs(luma - k_target_luma) < 20.0)
      break;
    exposure.SetValue(
      std::clamp(exposure.GetValue() * k_target_luma / luma, exposure.GetMin(), k_max_exposure_us));
  }
  std::printf("%s: exposure %.0f us for calibration (was %.0f us)\n", c.name.c_str(),
              exposure.GetValue(), c.original_exposure_us);
}

void save_pgm(const std::string &path, const Pylon::CGrabResultPtr &r, int stride) {
  const auto w = static_cast<size_t>(r->GetWidth());
  const auto h = static_cast<size_t>(r->GetHeight());
  std::ofstream f(path, std::ios::binary);
  f << "P5\n" << w << ' ' << h << "\n255\n";
  std::vector<char> row(w);
  for (size_t y = 0; y < h; ++y) {
    for (size_t x = 0; x < w; ++x)
      row[x] = static_cast<char>(luma_at(r, (y * w) + x, stride));
    f.write(row.data(), static_cast<std::streamsize>(w));
  }
}

int caps_int(const std::string &caps, const char *key, int fallback) {
  const std::regex re(std::string(key) + R"(=(?:\(int\))?(\d+))");
  std::smatch m;
  return std::regex_search(caps, m, re) ? std::stoi(m[1].str()) : fallback;
}

/// Same size, centring and pixel format as the pipeline.
void open_camera(Camera &c) {
  Pylon::CDeviceInfo info;
  info.SetSerialNumber(c.pipe->camera_serial_.c_str());
  c.device.Attach(Pylon::CTlFactory::GetInstance().CreateFirstDevice(info));
  c.device.Open();
  auto &nm = c.device.GetNodeMap();

  const bool mono = c.name == "mono";
  Pylon::CEnumParameter(nm, "PixelFormat").SetValue(mono ? "Mono8" : "YCbCr422_8");
  c.luma_stride = mono ? 1 : 2;

  // Binning first: it changes the maximum size the caps are fitted to.
  const auto binning = static_cast<int64_t>(c.pipe->binning_);
  Pylon::CEnumParameter(nm, "BinningSelector").TrySetValue("Sensor");
  Pylon::CIntegerParameter(nm, "BinningHorizontal").TrySetValue(binning);
  Pylon::CIntegerParameter(nm, "BinningVertical").TrySetValue(binning);

  Pylon::CIntegerParameter width(nm, "Width");
  Pylon::CIntegerParameter height(nm, "Height");
  Pylon::CIntegerParameter offset_x(nm, "OffsetX");
  Pylon::CIntegerParameter offset_y(nm, "OffsetY");
  offset_x.SetValue(0);
  offset_y.SetValue(0);
  width.SetValue(caps_int(c.pipe->camera_caps_, "width", static_cast<int>(width.GetMax())),
                 Pylon::IntegerValueCorrection_Nearest);
  height.SetValue(caps_int(c.pipe->camera_caps_, "height", static_cast<int>(height.GetMax())),
                  Pylon::IntegerValueCorrection_Nearest);
  offset_x.SetValue((width.GetMax() - width.GetValue()) / 2, Pylon::IntegerValueCorrection_Nearest);
  offset_y.SetValue((height.GetMax() - height.GetValue()) / 2,
                    Pylon::IntegerValueCorrection_Nearest);
  c.width = static_cast<int>(width.GetValue());
  c.height = static_cast<int>(height.GetValue());

  c.device.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
}

void write_meta(const std::string &path, const std::vector<Camera *> &cams) {
  std::ofstream f(path);
  f << "camera,capture_w,capture_h,detection_w,detection_h,focal_x_px,focal_y_px\n";
  for (const Camera *c : cams)
    f << c->name << ',' << c->width << ',' << c->height << ',' << c->pipe->detection_width_ << ','
      << c->pipe->detection_height_ << ',' << c->pipe->focal_x_px_ << ',' << c->pipe->focal_y_px_
      << '\n';
}

void sweep(Shared &s, Pose start, std::vector<Camera *> &cams, const std::string &out) {
  std::ofstream log(out + "/poses.csv");
  log << "idx,axis,offset_rad,pan_rad,tilt_rad\n";
  int idx = 0;
  for (int axis = 0; axis < 2 && !s.aborted; ++axis) {
    for (const float off : k_offsets) {
      if (s.aborted)
        return;
      set_target(s,
                 Pose{start.pan + (axis == 0 ? off : 0.0F), start.tilt + (axis == 1 ? off : 0.0F)});
      const bool still = wait_still(s);

      Pose sum;
      for (Camera *c : cams) {
        const Pylon::CGrabResultPtr r = grab_fresh(*c);
        const Pose p = read_pose(s);
        sum.pan += p.pan;
        sum.tilt += p.tilt;
        save_pgm(out + "/" + c->name + "_" + std::to_string(idx) + ".pgm", r, c->luma_stride);
      }
      const auto n = static_cast<float>(cams.size());
      log << idx << ',' << (axis == 0 ? "pan" : "tilt") << ',' << off << ',' << (sum.pan / n) << ','
          << (sum.tilt / n) << '\n';
      std::printf("%2d %-4s offset %+.3f  pan %.5f  tilt %.5f%s\n", idx, axis == 0 ? "pan" : "tilt",
                  static_cast<double>(off), static_cast<double>(sum.pan / n),
                  static_cast<double>(sum.tilt / n),
                  still ? "" : "  (stopped short of the target)");
      std::fflush(stdout);
      ++idx;
    }
  }
}

} // namespace

int main(int argc, char **argv) {
  const std::vector<std::string> args(argv + 1, argv + argc);
  const std::string out = !args.empty() ? args[0] : "focal_calib_out";
  const std::string config_path = args.size() > 1 ? args[1] : "config/system.ini";
  if (std::system(("mkdir -p '" + out + "'").c_str()) != 0) {
    std::fprintf(stderr, "cannot create %s\n", out.c_str());
    return 1;
  }

  const auto cfg = patronus::config::load_config(config_path);
  if (cfg.gimbals_.empty()) {
    std::fprintf(stderr, "no gimbals in %s\n", config_path.c_str());
    return 1;
  }
  // Only gimbal 1 carries cameras.
  patronus::comm::CandleMotor motor({cfg.gimbals_[0]}, cfg.can_bus_);
  if (!motor.init()) {
    std::fprintf(stderr, "motor init failed\n");
    return 1;
  }
  const auto [start_pan, start_tilt] = motor.get_position(0);
  const Pose start{start_pan, start_tilt};
  std::printf("start pose pan %.5f tilt %.5f\n", static_cast<double>(start.pan),
              static_cast<double>(start.tilt));

  Shared s;
  s.target = start;
  s.pose = start;
  std::thread loop(motor_loop, std::ref(motor), std::ref(s), start);

  int rc = 0;
  Pylon::PylonInitialize();
  {
    Camera mono;
    mono.name = "mono";
    mono.pipe = &cfg.mono_;
    Camera rgb;
    rgb.name = "rgb";
    rgb.pipe = &cfg.rgb_;
    std::vector<Camera *> cams;
    try {
      for (Camera *c : {&mono, &rgb}) {
        open_camera(*c);
        auto_expose(*c);
        cams.push_back(c);
      }
      write_meta(out + "/cameras.csv", cams);
      sweep(s, start, cams, out);
    } catch (const Pylon::GenericException &e) {
      std::fprintf(stderr, "pylon: %s\n", e.GetDescription());
      rc = 1;
    }
    for (Camera *c : cams) {
      try {
        c->device.StopGrabbing();
        Pylon::CFloatParameter(c->device.GetNodeMap(), "ExposureTime")
          .SetValue(c->original_exposure_us);
        c->device.Close();
      } catch (const Pylon::GenericException &e) {
        std::fprintf(stderr, "pylon: %s: %s\n", c->name.c_str(), e.GetDescription());
      }
    }
  }

  if (!s.aborted) {
    set_target(s, start);
    wait_still(s);
  }
  s.run = false;
  loop.join();
  const auto [end_pan, end_tilt] = motor.get_position(0);
  motor.disable();
  std::printf("end pose pan %.5f tilt %.5f%s\n", static_cast<double>(end_pan),
              static_cast<double>(end_tilt), s.aborted ? "  ABORTED: left the safety window" : "");
  std::fflush(stdout);
  std::fflush(stderr);

  // pylon 8 segfaults in its own static teardown here; skip it.
  Pylon::PylonTerminate();
  std::_Exit(s.aborted ? 2 : rc);
}
