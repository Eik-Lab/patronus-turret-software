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

  /// Persistent label buffers, cycled round-robin.
  ///
  /// nvdsosd renders on its own thread downstream and does not copy `display_text`,
  /// so the string has to outlive this probe — which rules out freeing it here, and
  /// equally rules out allocating per frame (the obvious alternative leaks one
  /// buffer per label per frame). A short ring gives each string a lifetime of
  /// several frame intervals, far longer than a render takes, and allocates nothing.
  constexpr int k_label_ring = 8;

  enum class Camera { Rgb, Mono };

  /// Per-camera state the OSD probe needs. Passed to the probe as `u_data` rather
  /// than recovered from `frame_meta->source_id`: each camera is its own pipeline
  /// with `batch-size 1`, so every source_id in both pipelines is 0.
  struct CameraContext {
    const config::PipelineConfig *config{nullptr};
    core::LatestValue<core::Detection> *output{nullptr};
    const tracking::PredictionChannel *predictions{nullptr};
    GMainLoop *loop{nullptr};
  };

  CameraContext g_ctx[2];

  // Prediction box colour: green while the filter is being corrected, amber while
  // it is coasting on its own prediction.
  constexpr NvOSD_ColorParams k_colour_live{0.0, 1.0, 0.0, 1.0};
  constexpr NvOSD_ColorParams k_colour_coast{1.0, 0.65, 0.0, 1.0};
  constexpr NvOSD_ColorParams k_colour_white{1.0, 1.0, 1.0, 1.0};

  char *next_label_buffer() {
    static char ring[k_label_ring][k_max_display_len] = {};
    static int next = 0;
    char *buf = ring[next];
    next = (next + 1) % k_label_ring;
    return buf;
  }

  /// Accumulates overlay primitives, rolling over into a fresh display meta
  /// whenever one of the fixed-size arrays fills up.
  ///
  /// MAX_ELEMENTS_IN_DISPLAY_META is small (16). A frame drawing up to eight gimbal
  /// predictions needs two rects, a line and a label each, which does not fit in a
  /// single meta's rect array. The three arrays are independent, so the rollover
  /// check is per primitive type.
  class OsdWriter {
  public:
    OsdWriter(NvDsBatchMeta *batch, NvDsFrameMeta *frame) : batch_(batch), frame_(frame) {
    }

    void add_box(float left, float top, float width, float height, unsigned int thickness,
                 const NvOSD_ColorParams &colour) {
      if (width <= 0.0F || height <= 0.0F)
        return;
      NvDsDisplayMeta *meta =
        reserve(current_ == nullptr ? MAX_ELEMENTS_IN_DISPLAY_META : current_->num_rects);
      if (meta == nullptr)
        return;
      NvOSD_RectParams &rect = meta->rect_params[meta->num_rects++];
      rect.left = left;
      rect.top = top;
      rect.width = width;
      rect.height = height;
      rect.border_width = thickness;
      rect.border_color = colour;
      rect.has_bg_color = 0;
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

    /// @param text Must outlive this call: pass a persistent ring buffer, never a
    ///             temporary.
    void add_text(const char *text, float x, float y, unsigned int font_size,
                  const NvOSD_ColorParams &colour) {
      NvDsDisplayMeta *meta =
        reserve(current_ == nullptr ? MAX_ELEMENTS_IN_DISPLAY_META : current_->num_labels);
      if (meta == nullptr)
        return;
      NvOSD_TextParams &txt = meta->text_params[meta->num_labels++];
      txt.display_text = const_cast<char *>(text);
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
      nvds_add_display_meta_to_frame(frame_, meta);
      current_ = meta;
      return current_;
    }

    NvDsBatchMeta *batch_;
    NvDsFrameMeta *frame_;
    NvDsDisplayMeta *current_{nullptr};
  };

  /// Draw the IMM predictions published by this camera's tracking thread.
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
    if (width <= 0.0F || height <= 0.0F)
      return;

    const float det_w = static_cast<float>(cfg.detection_width_);

    for (size_t g = 0; g < tracking::PredictionChannel::k_max_gimbals; ++g) {
      const auto sample = ctx.predictions->get(g);
      if (!sample.has_value())
        continue;

      const double age = now_s - sample->publish_t_s;
      if (age < 0.0 || age > k_prediction_max_age_s)
        continue;

      const bool coasting = !sample->measured;
      const NvOSD_ColorParams &colour = coasting ? k_colour_coast : k_colour_live;

      // Fall back to a fixed marker until a detection has been accepted, so a
      // freshly-seeded filter still draws something.
      float box_w = sample->box_w_norm * width;
      float box_h = sample->box_h_norm * height;
      if (box_w < 4.0F)
        box_w = 24.0F;
      if (box_h < 4.0F)
        box_h = 24.0F;

      const float cx = sample->u_norm * width;
      const float cy = sample->v_norm * height;
      const float left = cx - (box_w * 0.5F);
      const float top = cy - (box_h * 0.5F);

      osd.add_box(left, top, box_w, box_h, 3U, colour);

      // Lead vector: where the filter says the target will be at the horizon. This
      // is the same constant-velocity prediction the control law's lead term
      // performs, so the line shows what the gimbal is actually aiming at.
      osd.add_line(cx, cy, sample->lead_u_norm * width, sample->lead_v_norm * height, 3U, colour);

      // Uncertainty outline, scaled by the filter's own 1-sigma. A coasting
      // estimate's sigma grows, so the cue widens exactly when the filter is
      // extrapolating instead of correcting.
      const float sigma_px = sample->sigma_norm * width;
      if (sigma_px >= 4.0F) {
        const float r = std::min(sigma_px, std::min(width, height) * 0.5F);
        osd.add_box(cx - r, cy - r, r * 2.0F, r * 2.0F, 1U, colour);
      }

      // Velocity is reported in detection px/s, the unit the gains are tuned in.
      const float speed = std::sqrt((sample->vu_norm_s * sample->vu_norm_s) +
                                    (sample->vv_norm_s * sample->vv_norm_s)) *
                          det_w;

      char *label = next_label_buffer();
      std::snprintf(label, k_max_display_len, "G%zu %s %.0f px/s", g, coasting ? "COAST" : "LOCK",
                    static_cast<double>(speed));
      osd.add_text(label, left, top + box_h + 4.0F,
                   static_cast<unsigned int>(std::max(box_h * 0.4F, 12.0F)), colour);
    }
  }

  /// OSD probe: forward drone detections to the tracking loop and draw the filter's
  /// prediction. `u_data` is this pipeline's `CameraContext`.
  GstPadProbeReturn osd_sink_pad_buffer_probe(GstPad * /*pad*/, GstPadProbeInfo *info,
                                              gpointer u_data) {
    auto *ctx = static_cast<CameraContext *>(u_data);

    GstBuffer *buf = reinterpret_cast<GstBuffer *>(info->data);
    NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta(buf);
    if (batch_meta == nullptr)
      return GST_PAD_PROBE_OK;

    // One stamp for the whole buffer: every detection in it came from the same
    // exposure, and stamping per object would spread a single frame's measurements
    // over the time spent walking the metadata list.
    const double frame_t_s = core::steady_now_s();

    for (NvDsMetaList *l_frame = batch_meta->frame_meta_list; l_frame != nullptr;
         l_frame = l_frame->next) {
      auto *frame_meta = static_cast<NvDsFrameMeta *>(l_frame->data);

      guint drone_count = 0;
      for (NvDsMetaList *l_obj = frame_meta->obj_meta_list; l_obj != nullptr; l_obj = l_obj->next) {
        const auto *obj_meta = static_cast<const NvDsObjectMeta *>(l_obj->data);
        if (obj_meta->class_id != k_pgie_class_id_drone)
          continue;

        if (ctx->output != nullptr) {
          core::Detection det;
          det.left_ = obj_meta->rect_params.left;
          det.top_ = obj_meta->rect_params.top;
          det.width_ = obj_meta->rect_params.width;
          det.height_ = obj_meta->rect_params.height;
          det.class_id_ = obj_meta->class_id;
          det.confidence_ = obj_meta->confidence;
          det.timestamp_s_ = frame_t_s;
          ctx->output->push(det);
        }
        ++drone_count;
      }

      OsdWriter osd(batch_meta, frame_meta);

      char *count_label = next_label_buffer();
      std::snprintf(count_label, k_max_display_len, "Drone = %u ", drone_count);
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
    yaml_config = (infer_config_abs != nullptr) && (g_str_has_suffix(infer_config_abs, ".yml") ||
                                                    g_str_has_suffix(infer_config_abs, ".yaml"));

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
                   core::LatestValue<core::Detection> &output,
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
                     core::LatestValue<core::Detection> &output,
                     tracking::PredictionChannel &predictions) {
  return run_pipeline(Camera::Rgb, config, output, predictions);
}

int run_pipeline_mono(const config::PipelineConfig &config,
                      core::LatestValue<core::Detection> &output,
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