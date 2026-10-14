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

#include "patronus/core/config.hpp"
#include "patronus/core/state.hpp"
#include "patronus/pipeline/inference_mono.hpp"
#include "patronus/pipeline/inference_rgb.hpp"
#include "patronus/tracking/prediction_overlay.hpp"

#include "gstnvdsmeta.h"
#include "nvds_yml_parser.h"

#include <glib.h>
#include <gst/gst.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>

namespace patronus::pipeline {
namespace {

  constexpr int k_max_display_len = 64;
  constexpr int k_pgie_class_id_drone = 0;
  constexpr guint k_muxer_batch_timeout_usec = 40000;

  /// A prediction older than this is not drawn.
  ///
  /// The tracking tick is ~10 ms and the frame rate is 30/40 fps, so a live sample
  /// is at most ~35 ms old. This bound covers the failure case: a tracking thread
  /// that has died, or a gimbal no longer aiming, must not leave a frozen marker on
  /// the stream implying the estimate is still being maintained.
  constexpr double k_prediction_max_age_s = 0.25;

  /// Furthest the overlay carries an estimate forward to the frame being drawn.
  /// Normally one frame interval; bounded so a stalled tracking thread cannot
  /// send the marker sliding across the screen.
  constexpr double k_max_extrapolation_s = 0.1;

  /// Frame intervals within this fraction of the running average are treated as
  /// the camera's nominal period, and snapped to it.
  constexpr double k_period_tolerance = 0.25;
  constexpr double k_period_smoothing = 0.05;
  /// Consecutive off-period intervals before the average is re-learned.
  constexpr int k_period_relearn_frames = 5;
  constexpr double k_max_frame_interval_s = 0.5;

  // Overlay geometry, in muxed pixels. Sizes and colours mirror kalman-cpp's
  // apps/video_track so the two renderings can be compared side by side.
  constexpr unsigned int k_prediction_radius_px = 10U;
  constexpr unsigned int k_prediction_width_px = 3U;
  constexpr unsigned int k_detection_radius_px = 8U;
  constexpr unsigned int k_velocity_width_px = 2U;
  constexpr float k_ellipse_sigmas = 3.0F;
  /// nvdsosd has no ellipse primitive, so the ellipse is a closed polyline.
  constexpr int k_ellipse_segments = 24;
  constexpr float k_two_pi = 6.2831853F;

  enum class Camera { Rgb, Mono };

  /// Per-camera state the OSD probe needs. Passed to the probe as `u_data` rather
  /// than recovered from `frame_meta->source_id`: each camera is its own pipeline
  /// with `batch-size 1`, so every source_id in both pipelines is 0.
  struct CameraContext {
    const config::PipelineConfig *config{nullptr};
    core::LatestValue<core::FrameObservation> *output{nullptr};
    const tracking::PredictionChannel *predictions{nullptr};
    GMainLoop *loop{nullptr};

    // --- Per-frame state, touched only by this camera's streaming thread ---

    uint64_t frame_id{0};
    double last_arrival_s{0.0};
    GstClockTime last_pts{GST_CLOCK_TIME_NONE};
    /// De-jittered capture clock and the frame period it advances by.
    double capture_s{0.0};
    double period_s{0.0};
    int off_period_frames{0};
  };

  CameraContext g_ctx[2];

  constexpr NvOSD_ColorParams k_colour_prediction{1.0, 0.0, 0.0, 1.0};  // red
  constexpr NvOSD_ColorParams k_colour_velocity{0.0, 0.0, 1.0, 1.0};    // blue
  constexpr NvOSD_ColorParams k_colour_uncertainty{1.0, 1.0, 0.0, 1.0}; // yellow
  constexpr NvOSD_ColorParams k_colour_detection{0.0, 1.0, 0.0, 1.0};   // green
  constexpr NvOSD_ColorParams k_colour_white{1.0, 1.0, 1.0, 1.0};

  /// Accumulates overlay primitives, rolling over into a fresh display meta
  /// whenever one of the fixed-size arrays fills up.
  ///
  /// MAX_ELEMENTS_IN_DISPLAY_META is small (16). A single gimbal's uncertainty
  /// ellipse alone is more line segments than one meta holds. The arrays are
  /// independent, so the rollover check is per primitive type.
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

    /// @param filled Fill the disc with `colour` instead of drawing only its outline.
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

    /// @param text Copied onto the heap. DeepStream owns `display_text` and frees it
    ///             when the meta is released, so it must never be a string literal or
    ///             a static buffer. `font_name` is the opposite case: it is only read,
    ///             so a literal is correct there.
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
    /// Return `current_` if it still has room at `used`, otherwise acquire a new
    /// meta. Returns nullptr when the pool is exhausted, in which case the caller
    /// drops the primitive rather than writing through a null pointer.
    NvDsDisplayMeta *reserve(guint used) {
      if (current_ != nullptr && used < MAX_ELEMENTS_IN_DISPLAY_META)
        return current_;
      NvDsDisplayMeta *meta = nvds_acquire_display_meta_from_pool(batch_);
      if (meta == nullptr)
        return nullptr;
      // The pool hands back *recycled* metas with their counts intact — DeepStream's
      // own plugins always assign num_rects/num_labels rather than incrementing
      // (gstnvdspreprocess.cpp does `display_meta->num_rects = 1`). Trusting a stale
      // count here indexes past the end of the fixed-size arrays and corrupts the
      // heap, so every count starts from zero.
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

  /// A published prediction carried forward to the frame being drawn.
  struct Projected {
    tracking::PredictionSample sample;
    float u_norm{0.0F}; ///< Estimated centre at the drawn frame, normalised.
    float v_norm{0.0F};
  };

  /// Read a gimbal's prediction and move it to `now_s`. The filter's estimate is
  /// for the frame it last processed, normally the one before this; the target
  /// has since moved through the image by its own velocity plus the camera's.
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

  /// Draw the target-filter predictions published by this camera's tracking thread, the way
  /// kalman-cpp's video_track draws them: a red ring on the estimate, a blue
  /// velocity arrow, a yellow 3-sigma uncertainty ellipse, and a green dot on the
  /// detection the filter accepted.
  ///
  /// Positions arrive normalised to the detection frame, so the only conversion
  /// here is normalised -> muxed pixels. That is correct only while the detection
  /// frame and the muxed frame cover the same field of view; if
  /// `detection_width`/`detection_height` describe a cropped or letterboxed
  /// sub-rectangle of the muxed frame, the overlay is offset by the same amount the
  /// aim error is. The two therefore cannot silently disagree.
  void draw_predictions(OsdWriter &osd, const CameraContext &ctx, double now_s) {
    if (ctx.predictions == nullptr || ctx.config == nullptr || !ctx.config->draw_predictions_)
      return;

    const auto &cfg = *ctx.config;
    const float width = static_cast<float>(cfg.muxer_width_);
    const float height = static_cast<float>(cfg.muxer_height_);
    if (width <= 1.0F || height <= 1.0F)
      return;

    // A coasting estimate can drift to the edge and its ellipse can outgrow the
    // frame; nvdsosd is handed only coordinates that lie inside the surface.
    const float max_x = width - 1.0F;
    const float max_y = height - 1.0F;

    for (size_t g = 0; g < tracking::PredictionChannel::k_max_gimbals; ++g) {
      const auto projected = project_prediction(*ctx.predictions, g, now_s);
      if (!projected.has_value())
        continue;
      const tracking::PredictionSample &sample = projected->sample;

      // Accepted detection. Drawn first so the prediction ring sits on top of it.
      if (sample.measured) {
        const float det_x = sample.detection_u_norm * width;
        const float det_y = sample.detection_v_norm * height;
        if (det_x >= 0.0F && det_x <= max_x && det_y >= 0.0F && det_y <= max_y)
          osd.add_circle(det_x, det_y, k_detection_radius_px, 1U, k_colour_detection, true);
      }

      // An estimate that has coasted out of view is pinned to the frame edge
      // rather than hidden, so a lost track still reads as one.
      const float cx = std::clamp(projected->u_norm * width, 0.0F, max_x);
      const float cy = std::clamp(projected->v_norm * height, 0.0F, max_y);

      osd.add_circle(cx, cy, k_prediction_radius_px, k_prediction_width_px, k_colour_prediction,
                     false);

      // Velocity arrow: the target's own motion over the look-ahead horizon, with
      // the camera's motion removed. This is the same constant-velocity prediction
      // the control law's lead term performs.
      const float lead_s = cfg.prediction_lead_s_;
      const float tip_x = std::clamp(cx + (sample.vu_norm_s * lead_s * width), 0.0F, max_x);
      const float tip_y = std::clamp(cy + (sample.vv_norm_s * lead_s * height), 0.0F, max_y);
      if (std::abs(tip_x - cx) >= 1.0F || std::abs(tip_y - cy) >= 1.0F)
        osd.add_arrow(cx, cy, tip_x, tip_y, k_velocity_width_px, k_colour_velocity);

      // Axis-aligned 3-sigma ellipse from the filter's own position covariance.
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

  /// Advance this camera's capture clock by one frame and return the interval.
  ///
  /// The interval comes from the buffer PTS where there is one, since that is the
  /// camera's cadence rather than when inference happened to finish. Either way
  /// it is then snapped to the running frame period: a free-running camera's true
  /// interval is constant, and whatever jitter is left in the stamps would
  /// otherwise be divided into the filter's velocity estimate.
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
      // First frame, or a gap long enough that the previous frame is unrelated.
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
      // Not a dropped frame but a new frame rate.
      ctx.period_s = interval;
      ctx.off_period_frames = 0;
    }

    ctx.capture_s += interval;
    return interval;
  }

  /// OSD probe: publish what this frame showed to the tracking loop and draw the
  /// filter's prediction. `u_data` is this pipeline's `CameraContext`.
  ///
  /// One `FrameObservation` goes out per frame, detection or not, because the
  /// filter steps once per frame. Of the frame's drone detections only the most
  /// confident is forwarded, as kalman-cpp's video_track does: the filter tracks
  /// one target, and handing it every box in turn has it chasing whichever came
  /// last.
  GstPadProbeReturn osd_sink_pad_buffer_probe(GstPad * /*pad*/, GstPadProbeInfo *info,
                                              gpointer u_data) {
    auto *ctx = static_cast<CameraContext *>(u_data);

    GstBuffer *buf = reinterpret_cast<GstBuffer *>(info->data);
    NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta(buf);
    if (batch_meta == nullptr)
      return GST_PAD_PROBE_OK;

    const double frame_t_s = core::steady_now_s();

    for (NvDsMetaList *l_frame = batch_meta->frame_meta_list; l_frame != nullptr;
         l_frame = l_frame->next) {
      auto *frame_meta = static_cast<NvDsFrameMeta *>(l_frame->data);

      core::FrameObservation obs;
      obs.timestamp_s_ = frame_t_s;
      obs.frame_dt_s_ = advance_capture_clock(*ctx, GST_BUFFER_PTS(buf), frame_t_s);
      obs.capture_s_ = ctx->capture_s;
      obs.frame_id_ = ++ctx->frame_id;

      guint drone_count = 0;
      for (NvDsMetaList *l_obj = frame_meta->obj_meta_list; l_obj != nullptr; l_obj = l_obj->next) {
        const auto *obj_meta = static_cast<const NvDsObjectMeta *>(l_obj->data);
        if (obj_meta->class_id != k_pgie_class_id_drone)
          continue;
        ++drone_count;

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

      if (ctx->output != nullptr)
        ctx->output->push(obs);

      OsdWriter osd(batch_meta, frame_meta);

      char count_label[k_max_display_len];
      std::snprintf(count_label, sizeof(count_label), "Drone = %u ", drone_count);
      osd.add_text(count_label, 10.0F, 12.0F, 18U, k_colour_white);

      draw_predictions(osd, *ctx, frame_t_s);
    }

    return GST_PAD_PROBE_OK;
  }

  gboolean bus_call(GstBus * /*bus*/, GstMessage *msg, gpointer data) {
    GMainLoop *loop = static_cast<GMainLoop *>(data);
    switch (GST_MESSAGE_TYPE(msg)) {
      case GST_MESSAGE_EOS:
        g_print("End of stream\n");
        g_main_loop_quit(loop);
        break;
      case GST_MESSAGE_ERROR: {
        gchar *debug = nullptr;
        GError *error = nullptr;
        gst_message_parse_error(msg, &error, &debug);
        g_printerr("ERROR from element %s: %s\n", GST_OBJECT_NAME(msg->src), error->message);
        if (debug != nullptr)
          g_printerr("Error details: %s\n", debug);
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

  /// Build the element graph, link it, and install the OSD probe.
  ///
  /// Split out of `run_pipeline` so every failure path is a plain `return` rather
  /// than a `goto`: the cleanup needed on the error paths (partially-constructed
  /// elements, the bus watch, the resolved config path) overlaps heavily with the
  /// success path, and `goto` across the initialisations that follow is exactly the
  /// pattern that skips a release and leaks the CAN bus wide open.
  ///
  /// @return The PLAYING pipeline, or nullptr on any failure. On failure nothing
  ///         is left running and every resource taken here is released.
  GstElement *build_and_run(Camera camera, const config::PipelineConfig &config, GMainLoop *loop,
                            CameraContext *ctx) {
    gboolean yaml_config = FALSE;
    NvDsGieType pgie_type = NVDS_GIE_PLUGIN_INFER;
    GstBus *bus = nullptr;
    guint bus_watch_id = 0U;

    // Resolve infer_config to an absolute path so DeepStream resolves
    // model-engine-file / onnx-file relative to the config file's directory.
    char *infer_config_abs = realpath(config.infer_config_.c_str(), nullptr);
    // Parenthesised to call the functions: glib >= 2.76 also defines these as
    // macros that assign inside an `if`, which clang-tidy rejects in our code.
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
      g_printerr("Error in parsing configuration file.\n");
      return fail(nullptr);
    }

    GstElement *pipeline =
      gst_pipeline_new(camera == Camera::Rgb ? "patronus-rgb-pipeline" : "patronus-mono-pipeline");

    GstElement *source = gst_element_factory_make("pylonsrc", "pylon-source");
    GstElement *capsfilter_src = gst_element_factory_make("capsfilter", "caps-src");
    GstElement *nvvidconv_pre = gst_element_factory_make("nvvideoconvert", "nvvideo-converter-pre");
    GstElement *streammux = gst_element_factory_make("nvstreammux", "stream-muxer");
    GstElement *nvvidconv = gst_element_factory_make("nvvideoconvert", "nvvideo-converter");
    GstElement *nvosd = gst_element_factory_make("nvdsosd", "nv-onscreendisplay");
    GstElement *nvvidconv_post =
      gst_element_factory_make("nvvideoconvert", "nvvideo-converter-post");
    GstElement *capsfilter_encoder = gst_element_factory_make("capsfilter", "caps-encoder");
    GstElement *encoder = gst_element_factory_make("x264enc", "encoder");
    GstElement *payload_encode = gst_element_factory_make("rtph264pay", "payload_encode");
    GstElement *udp_sink = gst_element_factory_make("udpsink", "udp-sink");

    if (pipeline == nullptr || source == nullptr || capsfilter_src == nullptr ||
        nvvidconv_pre == nullptr || streammux == nullptr || nvvidconv == nullptr ||
        nvosd == nullptr || nvvidconv_post == nullptr || capsfilter_encoder == nullptr ||
        encoder == nullptr || payload_encode == nullptr || udp_sink == nullptr) {
      g_printerr("One element could not be created. Exiting.\n");
      return fail(pipeline);
    }

    g_object_set(G_OBJECT(source), "device-serial-number", config.camera_serial_.c_str(), nullptr);

    GstCaps *caps_src = gst_caps_from_string(config.camera_caps_.c_str());
    g_object_set(G_OBJECT(capsfilter_src), "caps", caps_src, nullptr);
    gst_caps_unref(caps_src);

    GstCaps *caps_encoder = gst_caps_from_string("video/x-raw,format=I420");
    g_object_set(G_OBJECT(capsfilter_encoder), "caps", caps_encoder, nullptr);
    gst_caps_unref(caps_encoder);

    g_object_set(G_OBJECT(encoder), "bitrate", config.encoder_bitrate_, nullptr);
    gst_util_set_object_arg(G_OBJECT(encoder), "tune", "zerolatency");
    gst_util_set_object_arg(G_OBJECT(encoder), "speed-preset", "superfast");

    g_object_set(G_OBJECT(payload_encode), "config-interval", -1, nullptr);

    g_object_set(G_OBJECT(udp_sink), "host", config.udp_host_.c_str(), "port", config.udp_port_,
                 "sync", FALSE, "async", FALSE, nullptr);

    GstElement *pgie = (pgie_type == NVDS_GIE_PLUGIN_INFER_SERVER)
                         ? gst_element_factory_make("nvinferserver", "primary-nvinference-engine")
                         : gst_element_factory_make("nvinfer", "primary-nvinference-engine");
    if (pgie == nullptr) {
      g_printerr("Inference engine could not be created. Exiting.\n");
      return fail(pipeline);
    }

    // One camera per pipeline, so the muxer batch is a single source.
    g_object_set(G_OBJECT(streammux), "batch-size", static_cast<guint>(1), "width",
                 config.muxer_width_, "height", config.muxer_height_, "live-source", TRUE,
                 "batched-push-timeout", k_muxer_batch_timeout_usec, nullptr);

    if (yaml_config) {
      if (NVDS_YAML_PARSER_SUCCESS !=
            nvds_parse_streammux(streammux, infer_config_abs, "streammux") ||
          NVDS_YAML_PARSER_SUCCESS != nvds_parse_gie(pgie, infer_config_abs, "primary-gie")) {
        g_printerr("Error in parsing configuration file.\n");
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
                     nvvidconv, nvosd, nvvidconv_post, capsfilter_encoder, encoder, payload_encode,
                     udp_sink, nullptr);
    g_print("Added elements to bin\n");

    // The muxer sink pad is requested rather than statically linked: nvstreammux
    // only exposes it once the pipeline has a sink.
    GstPad *sinkpad = gst_element_request_pad_simple(streammux, "sink_0");
    if (sinkpad == nullptr) {
      g_printerr("Streammux request sink pad failed. Exiting.\n");
      return fail(pipeline);
    }
    GstPad *srcpad = gst_element_get_static_pad(nvvidconv_pre, "src");
    if (srcpad == nullptr) {
      g_printerr("nvvideoconvert-pre request src pad failed. Exiting.\n");
      gst_object_unref(sinkpad);
      return fail(pipeline);
    }
    const gboolean linked = (gst_pad_link(srcpad, sinkpad) == GST_PAD_LINK_OK);
    gst_object_unref(sinkpad);
    gst_object_unref(srcpad);
    if (!linked) {
      g_printerr("Failed to link source to stream muxer. Exiting.\n");
      return fail(pipeline);
    }

    if (!gst_element_link_many(source, capsfilter_src, nvvidconv_pre, nullptr) ||
        !gst_element_link_many(streammux, pgie, nvvidconv, nvosd, nvvidconv_post,
                               capsfilter_encoder, encoder, payload_encode, udp_sink, nullptr)) {
      g_printerr("Elements could not be linked. Exiting.\n");
      return fail(pipeline);
    }

    GstPad *osd_sink_pad = gst_element_get_static_pad(nvosd, "sink");
    if (osd_sink_pad == nullptr) {
      g_print("Unable to get OSD sink pad\n");
    } else {
      gst_pad_add_probe(osd_sink_pad, GST_PAD_PROBE_TYPE_BUFFER, osd_sink_pad_buffer_probe, ctx,
                        nullptr);
      gst_object_unref(osd_sink_pad);
    }

    g_print("Using pylonsrc (Basler camera serial %s)\n", config.camera_serial_.c_str());
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    return pipeline;
  }

  /// Shared pipeline body. Both cameras are byte-for-byte identical apart from
  /// their configuration and their slot in the file-scope context array, which is
  /// why the two public entry points are thin wrappers rather than the duplicated
  /// 300-line files this replaces.
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

    g_print("Running...\n");
    g_main_loop_run(loop);

    g_print("Returned, stopping playback\n");
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

} // namespace patronus::pipeline