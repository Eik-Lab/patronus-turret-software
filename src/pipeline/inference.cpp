/*
 * SPDX-FileCopyrightText: Copyright (c) 2018-2024 NVIDIA CORPORATION & AFFILIATES. All rights
 * reserved. SPDX-License-Identifier: LicenseRef-NvidiaProprietary
 *
 * NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
 * property and proprietary rights in and to this material, related
 * documentation and any modifications thereto. Any use, reproduction,
 * disclosure or distribution of this material or related documentation
 * without an express license agreement and without an express license agreement
 * from NVIDIA CORPORATION is strictly prohibited.
 */

#include "core/config.h"
#include "core/state.h"
#include "pipeline/inference_mono.h"
#include "pipeline/inference_rgb.h"
#include "pipeline/video_encoder.h"
#include "tracking/prediction_overlay.h"

#include "gstnvdsmeta.h"
#include "log.h"
#include "nvds_yml_parser.h"

#include <glib.h>
#include <gst/gst.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <optional>
#include <string>

namespace patronus::pipeline {
namespace {

  constexpr int k_max_display_len = 64;
  constexpr int k_pgie_class_id_bird = 0;
  constexpr guint k_muxer_batch_timeout_usec = 40000;

  /// Only reached if a tracking thread dies; avoids a frozen marker.
  constexpr double k_prediction_max_age_s = 0.25;

  constexpr double k_max_extrapolation_s = 0.1;

  /// Frame-period snapping, see advance_capture_clock().
  constexpr double k_period_tolerance = 0.25;
  constexpr double k_period_smoothing = 0.05;
  constexpr int k_period_relearn_frames = 5;
  constexpr double k_max_frame_interval_s = 0.5;

  /// Older than this, the PTS is assumed to be on another clock base.
  constexpr double k_max_capture_age_s = 1.0;

  /// Usable USB 3 rate of the ace 2 cameras, for estimating readout time.
  constexpr double k_usb3_bytes_per_s = 360.0e6;

  /// GenICam reads are too slow for every frame.
  constexpr double k_exposure_refresh_s = 1.0;

  // Muxed pixels. Matches kalman-cpp's video_track for side-by-side comparison.
  constexpr unsigned int k_prediction_radius_px = 10U;
  constexpr unsigned int k_prediction_width_px = 3U;
  constexpr unsigned int k_detection_radius_px = 8U;
  constexpr unsigned int k_velocity_width_px = 2U;
  constexpr float k_ellipse_sigmas = 3.0F;
  constexpr int k_ellipse_segments = 24;
  constexpr float k_two_pi = 6.2831853F;

  enum class Camera { Rgb, Mono };

  /// Passed to the probe as u_data: source_id is 0 in both pipelines.
  struct CameraContext {
    const config::PipelineConfig *config{nullptr};
    core::LatestValue<core::FrameObservation> *output{nullptr};
    const tracking::PredictionChannel *predictions{nullptr};
    GMainLoop *loop{nullptr};

    /// For the command thread. Null while the pipeline is down.
    GstElement *source{nullptr};
    std::mutex source_mutex;

    // Streaming thread only.

    uint64_t frame_id{0};
    double last_arrival_s{0.0};
    GstClockTime last_pts{GST_CLOCK_TIME_NONE};
    double capture_s{0.0};
    double period_s{0.0};
    int off_period_frames{0};
    bool logged_capture_age{false};

    double readout_s{0.0}; ///< Set before PLAYING.
    double exposure_s{0.0};
    double next_exposure_refresh_s{0.0};
  };

  CameraContext g_ctx[2];

  void set_source(CameraContext &ctx, GstElement *source) {
    const std::scoped_lock lock(ctx.source_mutex);
    ctx.source = source;
  }

  /// A missing property is logged, not fatal.
  void set_child_arg(GstElement *source, const char *name, const std::string &value) {
    GObject *target = nullptr;
    GParamSpec *pspec = nullptr;
    if (gst_child_proxy_lookup(GST_CHILD_PROXY(source), name, &target, &pspec) == FALSE) {
      LOG(Warning, "pylonsrc has no property " << name << "; not set");
      return;
    }
    gst_util_set_object_arg(target, pspec->name, value.c_str());
    g_object_unref(target);
    LOG(Info, "pylonsrc " << name << " = " << value);
  }

  /// A GenICam read over USB: call rarely.
  double read_exposure_s(CameraContext &ctx) {
    const std::scoped_lock lock(ctx.source_mutex);
    if (ctx.source == nullptr)
      return 0.0;
    GObject *cam = gst_child_proxy_get_child_by_name(GST_CHILD_PROXY(ctx.source), "cam");
    if (cam == nullptr)
      return 0.0;
    gdouble exposure_us = 0.0;
    g_object_get(cam, "ExposureTime", &exposure_us, nullptr);
    g_object_unref(cam);
    return (exposure_us > 0.0) ? exposure_us * 1.0e-6 : 0.0;
  }

  double readout_time_s(const config::PipelineConfig &config) {
    if (config.readout_ms_ > 0.0F)
      return static_cast<double>(config.readout_ms_) * 1.0e-3;

    GstCaps *caps = gst_caps_from_string(config.camera_caps_.c_str());
    if (caps == nullptr)
      return 0.0;
    double bytes = 0.0;
    if (gst_caps_get_size(caps) > 0U) {
      const GstStructure *s = gst_caps_get_structure(caps, 0);
      gint width = 0;
      gint height = 0;
      const gchar *format = gst_structure_get_string(s, "format");
      if (gst_structure_get_int(s, "width", &width) &&
          gst_structure_get_int(s, "height", &height)) {
        double bpp = 2.0;
        if (format != nullptr && (g_str_equal(format, "GRAY8") != FALSE))
          bpp = 1.0;
        else if (format != nullptr &&
                 (g_str_equal(format, "RGB") != FALSE || g_str_equal(format, "BGR") != FALSE))
          bpp = 3.0;
        bytes = static_cast<double>(width) * static_cast<double>(height) * bpp;
      }
    }
    gst_caps_unref(caps);
    return bytes / k_usb3_bytes_per_s;
  }

  /// A second Pylon handle on an open device fails, so go through pylonsrc.
  void adjust_exposure(Camera camera, double factor, double min_us, double max_us) {
    CameraContext &ctx = g_ctx[static_cast<int>(camera)];
    const std::scoped_lock lock(ctx.source_mutex);
    if (ctx.source == nullptr)
      return;

    GObject *cam = gst_child_proxy_get_child_by_name(GST_CHILD_PROXY(ctx.source), "cam");
    if (cam == nullptr)
      return;

    // Must be a gdouble: varargs get/set silently corrupts any other type.
    gdouble current_us = 0.0;
    g_object_get(cam, "ExposureTime", &current_us, nullptr);
    const gdouble target_us = std::clamp(current_us * factor, min_us, max_us);
    g_object_set(cam, "ExposureTime", target_us, nullptr);
    g_object_unref(cam);

    LOG(Info, "Exposure " << (camera == Camera::Rgb ? "rgb" : "mono") << ": " << current_us
                          << " us -> " << target_us << " us");
  }

  constexpr NvOSD_ColorParams k_colour_prediction{1.0, 0.0, 0.0, 1.0};  // red
  constexpr NvOSD_ColorParams k_colour_velocity{0.0, 0.0, 1.0, 1.0};    // blue
  constexpr NvOSD_ColorParams k_colour_uncertainty{1.0, 1.0, 0.0, 1.0}; // yellow
  constexpr NvOSD_ColorParams k_colour_detection{0.0, 1.0, 0.0, 1.0};   // green
  constexpr NvOSD_ColorParams k_colour_white{1.0, 1.0, 1.0, 1.0};

  /// MAX_ELEMENTS_IN_DISPLAY_META is 16, fewer than one ellipse needs, so this
  /// rolls over to a fresh display meta when an array fills.
  class OsdWriter {
  public:
    OsdWriter(NvDsBatchMeta *batch, NvDsFrameMeta *frame) : batch_(batch), frame_(frame) {
    }

    void add_line(float x1, float y1, float x2, float y2, unsigned int thickness,
                  const NvOSD_ColorParams &colour) {
      NvDsDisplayMeta *meta =
        reserve(current_ == nullptr ? MAX_ELEMENTS_IN_DISPLAY_META : current_->num_lines);
      if (meta == nullptr)
        return;
      NvOSD_LineParams &line = meta->line_params[meta->num_lines++];
      line.x1 = static_cast<guint>(std::max(x1, 0.0F));
      line.y1 = static_cast<guint>(std::max(y1, 0.0F));
      line.x2 = static_cast<guint>(std::max(x2, 0.0F));
      line.y2 = static_cast<guint>(std::max(y2, 0.0F));
      line.line_width = thickness;
      line.line_color = colour;
    }

    void add_arrow(float x1, float y1, float x2, float y2, unsigned int thickness,
                   const NvOSD_ColorParams &colour) {
      NvDsDisplayMeta *meta =
        reserve(current_ == nullptr ? MAX_ELEMENTS_IN_DISPLAY_META : current_->num_arrows);
      if (meta == nullptr)
        return;
      NvOSD_ArrowParams &arrow = meta->arrow_params[meta->num_arrows++];
      arrow.x1 = static_cast<guint>(std::max(x1, 0.0F));
      arrow.y1 = static_cast<guint>(std::max(y1, 0.0F));
      arrow.x2 = static_cast<guint>(std::max(x2, 0.0F));
      arrow.y2 = static_cast<guint>(std::max(y2, 0.0F));
      arrow.arrow_width = thickness;
      arrow.arrow_head = END_HEAD;
      arrow.arrow_color = colour;
    }

    void add_circle(float cx, float cy, unsigned int radius, unsigned int thickness,
                    const NvOSD_ColorParams &colour, bool filled) {
      NvDsDisplayMeta *meta =
        reserve(current_ == nullptr ? MAX_ELEMENTS_IN_DISPLAY_META : current_->num_circles);
      if (meta == nullptr)
        return;
      NvOSD_CircleParams &circle = meta->circle_params[meta->num_circles++];
      circle.xc = static_cast<guint>(std::max(cx, 0.0F));
      circle.yc = static_cast<guint>(std::max(cy, 0.0F));
      circle.radius = radius;
      circle.circle_color = colour;
      circle.has_bg_color = filled ? 1U : 0U;
      circle.bg_color = colour;
      circle.circle_width = thickness;
    }

    /// DeepStream frees display_text, so it must be heap-allocated.
    void add_text(const char *text, float x, float y, unsigned int font_size,
                  const NvOSD_ColorParams &colour) {
      NvDsDisplayMeta *meta =
        reserve(current_ == nullptr ? MAX_ELEMENTS_IN_DISPLAY_META : current_->num_labels);
      if (meta == nullptr)
        return;
      NvOSD_TextParams &txt = meta->text_params[meta->num_labels++];
      txt.display_text = g_strdup(text);
      txt.x_offset = static_cast<guint>(std::max(x, 0.0F));
      txt.y_offset = static_cast<guint>(std::max(y, 0.0F));
      txt.font_params.font_name = const_cast<char *>("Serif");
      txt.font_params.font_size = font_size;
      txt.font_params.font_color = colour;
      txt.set_bg_clr = 1;
      txt.text_bg_clr = NvOSD_ColorParams{0.0, 0.0, 0.0, 1.0};
    }

  private:
    NvDsDisplayMeta *reserve(guint used) {
      if (current_ != nullptr && used < MAX_ELEMENTS_IN_DISPLAY_META)
        return current_;
      NvDsDisplayMeta *meta = nvds_acquire_display_meta_from_pool(batch_);
      if (meta == nullptr)
        return nullptr;
      // Recycled metas keep their old counts; trusting them corrupts the heap.
      meta->num_rects = 0;
      meta->num_labels = 0;
      meta->num_lines = 0;
      meta->num_arrows = 0;
      meta->num_circles = 0;
      nvds_add_display_meta_to_frame(frame_, meta);
      current_ = meta;
      return current_;
    }

    NvDsBatchMeta *batch_;
    NvDsFrameMeta *frame_;
    NvDsDisplayMeta *current_{nullptr};
  };

  struct Projected {
    tracking::PredictionSample sample;
    float u_norm{0.0F}; ///< Estimated centre at the drawn frame, normalised.
    float v_norm{0.0F};
  };

  /// The estimate is from the previous frame; move it by target plus camera
  /// velocity to this one.
  std::optional<Projected> project_prediction(const tracking::PredictionChannel &predictions,
                                              size_t gimbal, double now_s) {
    const auto sample = predictions.get(gimbal);
    if (!sample.has_value())
      return std::nullopt;
    const double age = now_s - sample->publish_t_s;
    if (age < 0.0 || age > k_prediction_max_age_s)
      return std::nullopt;

    const float ahead = static_cast<float>(std::min(age, k_max_extrapolation_s));
    Projected out;
    out.sample = *sample;
    out.u_norm = sample->u_norm + ((sample->vu_norm_s + sample->camera_vu_norm_s) * ahead);
    out.v_norm = sample->v_norm + ((sample->vv_norm_s + sample->camera_vv_norm_s) * ahead);
    return out;
  }

  /// Red ring: estimate. Blue arrow: velocity. Yellow: 3-sigma ellipse.
  /// Green dot: accepted detection.
  void draw_predictions(OsdWriter &osd, const CameraContext &ctx, double now_s) {
    if (ctx.predictions == nullptr || ctx.config == nullptr || !ctx.config->draw_predictions_)
      return;

    const auto &cfg = *ctx.config;
    const float width = static_cast<float>(cfg.muxer_width_);
    const float height = static_cast<float>(cfg.muxer_height_);
    if (width <= 1.0F || height <= 1.0F)
      return;

    // nvdsosd must only get coordinates inside the surface.
    const float max_x = width - 1.0F;
    const float max_y = height - 1.0F;

    for (size_t g = 0; g < tracking::PredictionChannel::k_max_gimbals; ++g) {
      const auto projected = project_prediction(*ctx.predictions, g, now_s);
      if (!projected.has_value())
        continue;
      const tracking::PredictionSample &sample = projected->sample;

      if (sample.measured) {
        const float det_x = sample.detection_u_norm * width;
        const float det_y = sample.detection_v_norm * height;
        if (det_x >= 0.0F && det_x <= max_x && det_y >= 0.0F && det_y <= max_y)
          osd.add_circle(det_x, det_y, k_detection_radius_px, 1U, k_colour_detection, true);
      }

      // Pinned to the edge rather than hidden, so a lost track still shows.
      const float cx = std::clamp(projected->u_norm * width, 0.0F, max_x);
      const float cy = std::clamp(projected->v_norm * height, 0.0F, max_y);

      osd.add_circle(cx, cy, k_prediction_radius_px, k_prediction_width_px, k_colour_prediction,
                     false);

      const float lead_s = cfg.prediction_lead_s_;
      const float tip_x = std::clamp(cx + (sample.vu_norm_s * lead_s * width), 0.0F, max_x);
      const float tip_y = std::clamp(cy + (sample.vv_norm_s * lead_s * height), 0.0F, max_y);
      if (std::abs(tip_x - cx) >= 1.0F || std::abs(tip_y - cy) >= 1.0F)
        osd.add_arrow(cx, cy, tip_x, tip_y, k_velocity_width_px, k_colour_velocity);

      const float radius_x = k_ellipse_sigmas * sample.sigma_u_norm * width;
      const float radius_y = k_ellipse_sigmas * sample.sigma_v_norm * height;
      if (radius_x >= 1.0F && radius_y >= 1.0F) {
        float prev_x = std::clamp(cx + radius_x, 0.0F, max_x);
        float prev_y = cy;
        for (int i = 1; i <= k_ellipse_segments; ++i) {
          const float angle =
            k_two_pi * static_cast<float>(i) / static_cast<float>(k_ellipse_segments);
          const float x = std::clamp(cx + (radius_x * std::cos(angle)), 0.0F, max_x);
          const float y = std::clamp(cy + (radius_y * std::sin(angle)), 0.0F, max_y);
          osd.add_line(prev_x, prev_y, x, y, 1U, k_colour_uncertainty);
          prev_x = x;
          prev_y = y;
        }
      }
    }
  }

  /// The interval comes from the PTS and is snapped to the running period:
  /// a free-running camera's interval is constant, and stamp jitter would
  /// become velocity noise in the filter.
  double advance_capture_clock(CameraContext &ctx, GstClockTime pts, double arrival_s) {
    double interval = 0.0;
    if (ctx.last_arrival_s > 0.0) {
      interval = arrival_s - ctx.last_arrival_s;
      if (GST_CLOCK_TIME_IS_VALID(pts) && GST_CLOCK_TIME_IS_VALID(ctx.last_pts) &&
          pts > ctx.last_pts) {
        const double by_pts = static_cast<double>(pts - ctx.last_pts) * 1.0e-9;
        if (by_pts < k_max_frame_interval_s)
          interval = by_pts;
      }
    }
    ctx.last_arrival_s = arrival_s;
    ctx.last_pts = pts;

    if (interval <= 0.0 || interval > k_max_frame_interval_s) {
      ctx.capture_s = arrival_s;
      ctx.period_s = 0.0;
      ctx.off_period_frames = 0;
      return 0.0;
    }

    if (ctx.period_s <= 0.0) {
      ctx.period_s = interval;
    } else if (std::abs(interval - ctx.period_s) <= k_period_tolerance * ctx.period_s) {
      ctx.period_s += k_period_smoothing * (interval - ctx.period_s);
      ctx.off_period_frames = 0;
      interval = ctx.period_s;
    } else if (++ctx.off_period_frames >= k_period_relearn_frames) {
      // A new frame rate, not a dropped frame.
      ctx.period_s = interval;
      ctx.off_period_frames = 0;
    }

    ctx.capture_s += interval;
    return interval;
  }

  /// Mid-exposure time in steady_now_s() seconds, or 0.0 if unknown. The
  /// frame's age is measured on the pipeline clock, so the two clocks never
  /// need a common origin.
  double captured_at(GstClockTime clock_now, GstClockTime base_time, GstClockTime pts,
                     double arrival_s, double before_retrieval_s) {
    if (!GST_CLOCK_TIME_IS_VALID(clock_now) || !GST_CLOCK_TIME_IS_VALID(base_time) ||
        !GST_CLOCK_TIME_IS_VALID(pts))
      return 0.0;
    const GstClockTime produced = base_time + pts;
    if (produced > clock_now)
      return 0.0;
    const double age_s = static_cast<double>(clock_now - produced) * 1.0e-9;
    if (age_s > k_max_capture_age_s || age_s + before_retrieval_s >= arrival_s)
      return 0.0;
    return arrival_s - age_s - before_retrieval_s;
  }

  /// Only the most confident detection is forwarded: the filter tracks one
  /// target.
  GstPadProbeReturn osd_sink_pad_buffer_probe(GstPad *pad, GstPadProbeInfo *info, gpointer u_data) {
    auto *ctx = static_cast<CameraContext *>(u_data);

    GstBuffer *buf = reinterpret_cast<GstBuffer *>(info->data);
    NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta(buf);
    if (batch_meta == nullptr)
      return GST_PAD_PROBE_OK;

    const double frame_t_s = core::steady_now_s();

    if (frame_t_s >= ctx->next_exposure_refresh_s) {
      ctx->next_exposure_refresh_s = frame_t_s + k_exposure_refresh_s;
      if (const double exposure_s = read_exposure_s(*ctx); exposure_s > 0.0)
        ctx->exposure_s = exposure_s;
    }
    const double before_retrieval_s = ctx->readout_s + (0.5 * ctx->exposure_s);

    GstClockTime clock_now = GST_CLOCK_TIME_NONE;
    GstClockTime base_time = GST_CLOCK_TIME_NONE;
    if (GstElement *element = gst_pad_get_parent_element(pad); element != nullptr) {
      if (GstClock *clock = gst_element_get_clock(element); clock != nullptr) {
        clock_now = gst_clock_get_time(clock);
        base_time = gst_element_get_base_time(element);
        gst_object_unref(clock);
      }
      gst_object_unref(element);
    }

    for (NvDsMetaList *l_frame = batch_meta->frame_meta_list; l_frame != nullptr;
         l_frame = l_frame->next) {
      auto *frame_meta = static_cast<NvDsFrameMeta *>(l_frame->data);

      core::FrameObservation obs;
      obs.timestamp_s_ = frame_t_s;
      obs.frame_dt_s_ = advance_capture_clock(*ctx, GST_BUFFER_PTS(buf), frame_t_s);
      obs.capture_s_ = ctx->capture_s;
      obs.captured_at_s_ =
        captured_at(clock_now, base_time, frame_meta->buf_pts, frame_t_s, before_retrieval_s);
      obs.readout_s_ = ctx->readout_s;
      obs.exposure_s_ = ctx->exposure_s;
      obs.frame_id_ = ++ctx->frame_id;
      if (!ctx->logged_capture_age && obs.captured_at_s_ > 0.0) {
        ctx->logged_capture_age = true;
        LOG(Info, "Frame age at the tracking hand-off: "
                    << ((frame_t_s - obs.captured_at_s_) * 1.0e3) << " ms (readout "
                    << (ctx->readout_s * 1.0e3) << " ms, exposure " << (ctx->exposure_s * 1.0e3)
                    << " ms)");
      }

      guint bird_count = 0;
      for (NvDsMetaList *l_obj = frame_meta->obj_meta_list; l_obj != nullptr; l_obj = l_obj->next) {
        const auto *obj_meta = static_cast<const NvDsObjectMeta *>(l_obj->data);
        if (obj_meta->class_id != k_pgie_class_id_bird)
          continue;
        ++bird_count;

        if (obs.detection_.has_value() && obj_meta->confidence <= obs.detection_->confidence_)
          continue;
        core::Detection det;
        det.left_ = obj_meta->rect_params.left;
        det.top_ = obj_meta->rect_params.top;
        det.width_ = obj_meta->rect_params.width;
        det.height_ = obj_meta->rect_params.height;
        det.class_id_ = obj_meta->class_id;
        det.confidence_ = obj_meta->confidence;
        det.timestamp_s_ = frame_t_s;
        det.frame_id_ = obs.frame_id_;
        obs.detection_ = det;
      }

      if (ctx->output != nullptr) {
        obs.published_s_ = core::steady_now_s();
        ctx->output->push(obs);
      }

      OsdWriter osd(batch_meta, frame_meta);

      char count_label[k_max_display_len];
      std::snprintf(count_label, sizeof(count_label), "Bird = %u ", bird_count);
      osd.add_text(count_label, 10.0F, 12.0F, 18U, k_colour_white);

      draw_predictions(osd, *ctx, frame_t_s);
    }

    return GST_PAD_PROBE_OK;
  }

  gboolean bus_call(GstBus * /*bus*/, GstMessage *msg, gpointer data) {
    GMainLoop *loop = static_cast<GMainLoop *>(data);
    switch (GST_MESSAGE_TYPE(msg)) {
      case GST_MESSAGE_EOS:
        LOG(Info, "End of stream");
        g_main_loop_quit(loop);
        break;
      case GST_MESSAGE_ERROR: {
        gchar *debug = nullptr;
        GError *error = nullptr;
        gst_message_parse_error(msg, &error, &debug);
        LOG(Error, "ERROR from element " << GST_OBJECT_NAME(msg->src) << ": " << error->message);
        if (debug != nullptr)
          LOG(Error, "Error details: " << debug);
        g_free(debug);
        g_error_free(error);
        g_main_loop_quit(loop);
        break;
      }
      default:
        break;
    }
    return TRUE;
  }

  /// Every failure returns through `fail`, which releases what was taken.
  GstElement *build_and_run(Camera camera, const config::PipelineConfig &config, GMainLoop *loop,
                            CameraContext *ctx) {
    gboolean yaml_config = FALSE;
    NvDsGieType pgie_type = NVDS_GIE_PLUGIN_INFER;
    GstBus *bus = nullptr;
    guint bus_watch_id = 0U;

    // Absolute, so DeepStream resolves the model paths relative to it.
    char *infer_config_abs = realpath(config.infer_config_.c_str(), nullptr);
    // Parenthesised: glib >= 2.76 also defines these as macros clang-tidy rejects.
    yaml_config = (infer_config_abs != nullptr) && ((g_str_has_suffix)(infer_config_abs, ".yml") ||
                                                    (g_str_has_suffix)(infer_config_abs, ".yaml"));

    auto fail = [&](GstElement *pipeline) {
      if (pipeline != nullptr) {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(GST_OBJECT(pipeline));
      }
      if (bus_watch_id != 0U)
        g_source_remove(bus_watch_id);
      g_free(infer_config_abs);
      return static_cast<GstElement *>(nullptr);
    };

    if (yaml_config && NVDS_YAML_PARSER_SUCCESS !=
                         nvds_parse_gie_type(&pgie_type, infer_config_abs, "primary-gie")) {
      LOG(Error, "Error in parsing configuration file.");
      return fail(nullptr);
    }

    GstElement *pipeline =
      gst_pipeline_new(camera == Camera::Rgb ? "patronus-rgb-pipeline" : "patronus-mono-pipeline");
    GstElement *encoder = make_video_encoder(config);
    if (encoder == nullptr)
      return fail(pipeline);
    gst_bin_add(GST_BIN(pipeline), encoder);

    GstElement *source = gst_element_factory_make("pylonsrc", "pylon-source");
    GstElement *capsfilter_src = gst_element_factory_make("capsfilter", "caps-src");
    GstElement *nvvidconv_pre = gst_element_factory_make("nvvideoconvert", "nvvideo-converter-pre");
    GstElement *streammux = gst_element_factory_make("nvstreammux", "stream-muxer");
    GstElement *nvvidconv = gst_element_factory_make("nvvideoconvert", "nvvideo-converter");
    GstElement *nvosd = gst_element_factory_make("nvdsosd", "nv-onscreendisplay");
    GstElement *nvvidconv_post =
      gst_element_factory_make("nvvideoconvert", "nvvideo-converter-post");
    GstElement *payload_encode = gst_element_factory_make("rtph264pay", "payload_encode");
    GstElement *udp_sink = gst_element_factory_make("udpsink", "udp-sink");

    if (pipeline == nullptr || source == nullptr || capsfilter_src == nullptr ||
        nvvidconv_pre == nullptr || streammux == nullptr || nvvidconv == nullptr ||
        nvosd == nullptr || nvvidconv_post == nullptr || payload_encode == nullptr || udp_sink == nullptr) {
      LOG(Error, "One element could not be created. Exiting.");
      return fail(pipeline);
    }

    g_object_set(G_OBJECT(source), "device-serial-number", config.camera_serial_.c_str(), nullptr);
    if (config.binning_ > 1U) {
      const std::string factor = std::to_string(config.binning_);
      set_child_arg(source, "cam::BinningHorizontal-Sensor", factor);
      set_child_arg(source, "cam::BinningVertical-Sensor", factor);
    }
    if (config.grab_buffers_ > 0U)
      set_child_arg(source, "stream::MaxNumBuffer", std::to_string(config.grab_buffers_));

    GstCaps *caps_src = gst_caps_from_string(config.camera_caps_.c_str());
    g_object_set(G_OBJECT(capsfilter_src), "caps", caps_src, nullptr);
    gst_caps_unref(caps_src);

    g_object_set(G_OBJECT(payload_encode), "config-interval", -1, nullptr);

    g_object_set(G_OBJECT(udp_sink), "host", config.udp_host_.c_str(), "port", config.udp_port_,
                 "sync", FALSE, "async", FALSE, nullptr);

    GstElement *pgie = (pgie_type == NVDS_GIE_PLUGIN_INFER_SERVER)
                         ? gst_element_factory_make("nvinferserver", "primary-nvinference-engine")
                         : gst_element_factory_make("nvinfer", "primary-nvinference-engine");
    if (pgie == nullptr) {
      LOG(Error, "Inference engine could not be created. Exiting.");
      return fail(pipeline);
    }

    g_object_set(G_OBJECT(streammux), "batch-size", static_cast<guint>(1), "width",
                 config.muxer_width_, "height", config.muxer_height_, "live-source", TRUE,
                 "batched-push-timeout", k_muxer_batch_timeout_usec, nullptr);

    if (yaml_config) {
      if (NVDS_YAML_PARSER_SUCCESS !=
            nvds_parse_streammux(streammux, infer_config_abs, "streammux") ||
          NVDS_YAML_PARSER_SUCCESS != nvds_parse_gie(pgie, infer_config_abs, "primary-gie")) {
        LOG(Error, "Error in parsing configuration file.");
        return fail(pipeline);
      }
    } else if (infer_config_abs != nullptr) {
      g_object_set(G_OBJECT(pgie), "config-file-path", infer_config_abs, nullptr);
    } else {
      g_object_set(G_OBJECT(pgie), "config-file-path", config.infer_config_.c_str(), nullptr);
    }
    g_free(infer_config_abs);
    infer_config_abs = nullptr;

    bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    bus_watch_id = gst_bus_add_watch(bus, bus_call, loop);
    gst_object_unref(bus);

    gst_bin_add_many(GST_BIN(pipeline), source, capsfilter_src, nvvidconv_pre, streammux, pgie,
                     nvvidconv, nvosd, nvvidconv_post, payload_encode,
                     udp_sink, nullptr);
    LOG(Debug, "Added elements to bin");

    GstPad *sinkpad = gst_element_request_pad_simple(streammux, "sink_0");
    if (sinkpad == nullptr) {
      LOG(Error, "Streammux request sink pad failed. Exiting.");
      return fail(pipeline);
    }
    GstPad *srcpad = gst_element_get_static_pad(nvvidconv_pre, "src");
    if (srcpad == nullptr) {
      LOG(Error, "nvvideoconvert-pre request src pad failed. Exiting.");
      gst_object_unref(sinkpad);
      return fail(pipeline);
    }
    const gboolean linked = (gst_pad_link(srcpad, sinkpad) == GST_PAD_LINK_OK);
    gst_object_unref(sinkpad);
    gst_object_unref(srcpad);
    if (!linked) {
      LOG(Error, "Failed to link source to stream muxer. Exiting.");
      return fail(pipeline);
    }

    if (!gst_element_link_many(source, capsfilter_src, nvvidconv_pre, nullptr) ||
        !gst_element_link_many(streammux, pgie, nvvidconv, nvosd, nvvidconv_post,
                               encoder, payload_encode, udp_sink, nullptr)) {
      LOG(Error, "Elements could not be linked. Exiting.");
      return fail(pipeline);
    }

    GstPad *osd_sink_pad = gst_element_get_static_pad(nvosd, "sink");
    if (osd_sink_pad == nullptr) {
      LOG(Error, "Unable to get OSD sink pad");
    } else {
      gst_pad_add_probe(osd_sink_pad, GST_PAD_PROBE_TYPE_BUFFER, osd_sink_pad_buffer_probe, ctx,
                        nullptr);
      gst_object_unref(osd_sink_pad);
    }

    LOG(Info, "Using pylonsrc (Basler camera serial " << config.camera_serial_ << ')');
    ctx->readout_s = readout_time_s(config);
    ctx->exposure_s = 0.0;
    ctx->next_exposure_refresh_s = 0.0;
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    set_source(*ctx, source);
    return pipeline;
  }

  int run_pipeline(Camera camera, const config::PipelineConfig &config,
                   core::LatestValue<core::FrameObservation> &output,
                   tracking::PredictionChannel &predictions) {
    CameraContext &ctx = g_ctx[static_cast<int>(camera)];
    ctx.config = &config;
    ctx.output = &output;
    ctx.predictions = &predictions;

    GMainLoop *loop = g_main_loop_new(nullptr, FALSE);
    ctx.loop = loop;

    GstElement *pipeline = build_and_run(camera, config, loop, &ctx);
    if (pipeline == nullptr) {
      g_main_loop_unref(loop);
      ctx.loop = nullptr;
      ctx.config = nullptr;
      ctx.output = nullptr;
      ctx.predictions = nullptr;
      return -1;
    }

    LOG(Info, "Running...");
    g_main_loop_run(loop);

    LOG(Info, "Returned, stopping playback");
    // Before the element is destroyed; the command thread may hold it.
    set_source(ctx, nullptr);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(GST_OBJECT(pipeline));
    g_main_loop_unref(loop);

    ctx.loop = nullptr;
    ctx.config = nullptr;
    ctx.output = nullptr;
    ctx.predictions = nullptr;
    return 0;
  }

} // namespace

int run_pipeline_rgb(const config::PipelineConfig &config,
                     core::LatestValue<core::FrameObservation> &output,
                     tracking::PredictionChannel &predictions) {
  return run_pipeline(Camera::Rgb, config, output, predictions);
}

int run_pipeline_mono(const config::PipelineConfig &config,
                      core::LatestValue<core::FrameObservation> &output,
                      tracking::PredictionChannel &predictions) {
  return run_pipeline(Camera::Mono, config, output, predictions);
}

void stop_pipeline_rgb() {
  if (g_ctx[static_cast<int>(Camera::Rgb)].loop != nullptr)
    g_main_loop_quit(g_ctx[static_cast<int>(Camera::Rgb)].loop);
}

void stop_pipeline_mono() {
  if (g_ctx[static_cast<int>(Camera::Mono)].loop != nullptr)
    g_main_loop_quit(g_ctx[static_cast<int>(Camera::Mono)].loop);
}

void adjust_exposure_rgb(double factor, double min_us, double max_us) {
  adjust_exposure(Camera::Rgb, factor, min_us, max_us);
}

void adjust_exposure_mono(double factor, double min_us, double max_us) {
  adjust_exposure(Camera::Mono, factor, min_us, max_us);
}

} // namespace patronus::pipeline
