/*
 * SPDX-FileCopyrightText: Copyright (c) 2018-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: LicenseRef-NvidiaProprietary
 *
 * NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
 * property and proprietary rights in and to this material, related
 * documentation and any modifications thereto. Any use, reproduction,
 * disclosure or distribution of this material and related documentation
 * without an express license agreement from NVIDIA CORPORATION or
 * its affiliates is strictly prohibited.
 */

#include <gst/gst.h>
#include <glib.h>
#include <stdio.h>
#include <cstdlib>
#include "gstnvdsmeta.h"
#include "nvds_yml_parser.h"
#include "patronus/core/config.hpp"
#include "patronus/core/state.hpp"

#define MAX_DISPLAY_LEN 64

#define PGIE_CLASS_ID_DRONE 0

#define MUXER_BATCH_TIMEOUT_USEC 40000

#define RETURN_ON_PARSER_ERROR(parse_expr) \
  if (NVDS_YAML_PARSER_SUCCESS != parse_expr) { \
    g_printerr("Error in parsing configuration file.\n"); \
    return -1; \
  }

static LatestValue<Detection> *g_output_mono = nullptr;

static GstPadProbeReturn
osd_sink_pad_buffer_probe (GstPad * pad, GstPadProbeInfo * info,
    gpointer u_data)
{
    GstBuffer *buf = (GstBuffer *) info->data;
    NvDsObjectMeta *obj_meta = NULL;
    guint drone_count = 0;
    NvDsMetaList * l_frame = NULL;
    NvDsMetaList * l_obj = NULL;
    NvDsDisplayMeta *display_meta = NULL;

    NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta (buf);

    for (l_frame = batch_meta->frame_meta_list; l_frame != NULL;
      l_frame = l_frame->next) {
        NvDsFrameMeta *frame_meta = (NvDsFrameMeta *) (l_frame->data);
        for (l_obj = frame_meta->obj_meta_list; l_obj != NULL;
                l_obj = l_obj->next) {
            obj_meta = (NvDsObjectMeta *) (l_obj->data);
            if (obj_meta->class_id == PGIE_CLASS_ID_DRONE) {
                if (g_output_mono) {
                  Detection d;
                  d.left = obj_meta->rect_params.left;
                  d.top = obj_meta->rect_params.top;
                  d.width = obj_meta->rect_params.width;
                  d.height = obj_meta->rect_params.height;
                  d.class_id = obj_meta->class_id;
                  d.confidence = obj_meta->confidence;
                  g_output_mono->push(d);
                }
                drone_count++;
            }
        }
        display_meta = nvds_acquire_display_meta_from_pool(batch_meta);
        NvOSD_TextParams *txt_params  = &display_meta->text_params[0];
        display_meta->num_labels = 1;
        txt_params->display_text = (char*) g_malloc0 (MAX_DISPLAY_LEN);
        snprintf(txt_params->display_text, MAX_DISPLAY_LEN, "Drone = %d ", drone_count);

        txt_params->x_offset = 10;
        txt_params->y_offset = 12;

        txt_params->font_params.font_name = (char*) "Serif";
        txt_params->font_params.font_size = 10;
        txt_params->font_params.font_color.red = 1.0;
        txt_params->font_params.font_color.green = 1.0;
        txt_params->font_params.font_color.blue = 1.0;
        txt_params->font_params.font_color.alpha = 1.0;

        txt_params->set_bg_clr = 1;
        txt_params->text_bg_clr.red = 0.0;
        txt_params->text_bg_clr.green = 0.0;
        txt_params->text_bg_clr.blue = 0.0;
        txt_params->text_bg_clr.alpha = 1.0;

        nvds_add_display_meta_to_frame(frame_meta, display_meta);
    }

    return GST_PAD_PROBE_OK;
}

static gboolean
bus_call (GstBus * bus, GstMessage * msg, gpointer data)
{
  GMainLoop *loop = (GMainLoop *) data;
  switch (GST_MESSAGE_TYPE (msg)) {
    case GST_MESSAGE_EOS:
      g_print ("End of stream\n");
      g_main_loop_quit (loop);
      break;
    case GST_MESSAGE_ERROR:{
      gchar *debug = NULL;
      GError *error = NULL;
      gst_message_parse_error (msg, &error, &debug);
      g_printerr ("ERROR from element %s: %s\n",
          GST_OBJECT_NAME (msg->src), error->message);
      if (debug)
        g_printerr ("Error details: %s\n", debug);
      g_free (debug);
      g_error_free (error);
      g_main_loop_quit (loop);
      break;
    }
    default:
      break;
  }
  return TRUE;
}

int
run_pipeline_mono (const patronus::config::PipelineConfig &config,
                   LatestValue<Detection> &output)
{
  g_output_mono = &output;

  GMainLoop *loop = NULL;
  GstElement *pipeline = NULL, *source = NULL, *capsfilter_src = NULL,
      *nvvidconv_pre = NULL, *nvvidconv_post = NULL, *capsfilter_encoder = NULL,
      *streammux = NULL, *udp_sink = NULL, *pgie = NULL, *nvvidconv = NULL,
      *nvosd = NULL, *encoder = NULL, *payload_encode = NULL;
  GstCaps *caps_src = NULL, *caps_encoder = NULL;

  GstBus *bus = NULL;
  guint bus_watch_id;
  GstPad *osd_sink_pad = NULL;
  gboolean yaml_config = FALSE;
  NvDsGieType pgie_type = NVDS_GIE_PLUGIN_INFER;

  loop = g_main_loop_new (NULL, FALSE);

  // Resolve infer_config to absolute path so DeepStream resolves
  // model-engine-file / onnx-file relative to the config file's directory.
  char *infer_config_abs = realpath(config.infer_config.c_str(), NULL);

  yaml_config = infer_config_abs &&
      (g_str_has_suffix (infer_config_abs, ".yml") ||
       g_str_has_suffix (infer_config_abs, ".yaml"));

  if (yaml_config) {
    RETURN_ON_PARSER_ERROR(nvds_parse_gie_type(&pgie_type, infer_config_abs,
                "primary-gie"));
  }

  pipeline = gst_pipeline_new ("patronus-mono-pipeline");

  source = gst_element_factory_make ("pylonsrc", "pylon-source");
  g_object_set (G_OBJECT (source), "device-serial-number",
      config.camera_serial.c_str(), NULL);

  capsfilter_src = gst_element_factory_make ("capsfilter", "caps-src");
  caps_src = gst_caps_from_string (config.camera_caps.c_str());
  g_object_set (G_OBJECT (capsfilter_src), "caps", caps_src, NULL);
  gst_caps_unref (caps_src);

  nvvidconv_pre = gst_element_factory_make ("nvvideoconvert", "nvvideo-converter-pre");

  streammux = gst_element_factory_make ("nvstreammux", "stream-muxer");

  if (!pipeline || !streammux) {
    g_printerr ("One element could not be created. Exiting.\n");
    free(infer_config_abs);
    return -1;
  }

  if (pgie_type == NVDS_GIE_PLUGIN_INFER_SERVER) {
    pgie = gst_element_factory_make ("nvinferserver", "primary-nvinference-engine");
  } else {
    pgie = gst_element_factory_make ("nvinfer", "primary-nvinference-engine");
  }

  nvvidconv = gst_element_factory_make ("nvvideoconvert", "nvvideo-converter");

  nvosd = gst_element_factory_make ("nvdsosd", "nv-onscreendisplay");

  nvvidconv_post = gst_element_factory_make("nvvideoconvert", "nvvideo-converter-post");

  capsfilter_encoder = gst_element_factory_make("capsfilter", "caps-encoder");
  caps_encoder = gst_caps_from_string("video/x-raw,format=I420");
  g_object_set(G_OBJECT(capsfilter_encoder), "caps", caps_encoder, NULL);
  gst_caps_unref(caps_encoder);

  encoder = gst_element_factory_make("x264enc", "encoder");
  g_object_set(G_OBJECT(encoder), "bitrate", config.encoder_bitrate, NULL);
  gst_util_set_object_arg(G_OBJECT(encoder), "tune", "zerolatency");
  gst_util_set_object_arg(G_OBJECT(encoder), "speed-preset", "superfast");

  payload_encode = gst_element_factory_make ("rtph264pay", "payload_encode");
  g_object_set(G_OBJECT(payload_encode), "config-interval", -1, NULL);

  udp_sink = gst_element_factory_make("udpsink", "udp-sink");
  g_object_set(G_OBJECT(udp_sink),
    "host", config.udp_host.c_str(),
    "port", config.udp_port,
    "sync", FALSE,
    "async", FALSE,
    NULL);

  if (!source || !capsfilter_src || !nvvidconv_pre || !pgie || !nvvidconv || !nvosd || !nvvidconv_post || !capsfilter_encoder || !encoder || !payload_encode || !udp_sink) {
    g_printerr ("One element could not be created. Exiting.\n");
    free(infer_config_abs);
    return -1;
  }

  g_object_set (G_OBJECT (streammux), "batch-size", 1, NULL);
  g_object_set (G_OBJECT (streammux), "width", config.muxer_width, "height",
      config.muxer_height, "live-source", TRUE,
      "batched-push-timeout", MUXER_BATCH_TIMEOUT_USEC, NULL);

  if (yaml_config && infer_config_abs) {
    RETURN_ON_PARSER_ERROR(nvds_parse_streammux(streammux, infer_config_abs,
                           "streammux"));
    RETURN_ON_PARSER_ERROR(nvds_parse_gie(pgie, infer_config_abs,
                           "primary-gie"));
  } else if (infer_config_abs) {
    g_object_set (G_OBJECT (pgie), "config-file-path", infer_config_abs, NULL);
  } else {
    g_object_set (G_OBJECT (pgie), "config-file-path",
                  config.infer_config.c_str(), NULL);
  }
  free(infer_config_abs);

  bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
  bus_watch_id = gst_bus_add_watch (bus, bus_call, loop);
  gst_object_unref (bus);

  gst_bin_add_many (GST_BIN (pipeline),
      source, capsfilter_src, nvvidconv_pre, streammux, pgie,
      nvvidconv, nvosd, nvvidconv_post, capsfilter_encoder, encoder, payload_encode, udp_sink, NULL);
  g_print ("Added elements to bin\n");

  GstPad *sinkpad, *srcpad;
  gchar pad_name_sink[16] = "sink_0";
  gchar pad_name_src[16] = "src";

  sinkpad = gst_element_request_pad_simple (streammux, pad_name_sink);
  if (!sinkpad) {
    g_printerr ("Streammux request sink pad failed. Exiting.\n");
    return -1;
  }

  srcpad = gst_element_get_static_pad (nvvidconv_pre, pad_name_src);
  if (!srcpad) {
    g_printerr ("capsfilter_nvmm request src pad failed. Exiting.\n");
    return -1;
  }

  if (gst_pad_link (srcpad, sinkpad) != GST_PAD_LINK_OK) {
      g_printerr ("Failed to link decoder to stream muxer. Exiting.\n");
      return -1;
  }

  gst_object_unref (sinkpad);
  gst_object_unref (srcpad);

  if (!gst_element_link_many (source, capsfilter_src, nvvidconv_pre, NULL)) {
    g_printerr ("Elements could not be linked: 1. Exiting.\n");
    return -1;
  }

  if (!gst_element_link_many (streammux, pgie,
        nvvidconv, nvosd, nvvidconv_post, capsfilter_encoder, encoder, payload_encode, udp_sink, NULL)) {
      g_printerr ("Elements could not be linked: 2. Exiting.\n");
      return -1;
  }

  osd_sink_pad = gst_element_get_static_pad (nvosd, "sink");
  if (!osd_sink_pad)
    g_print ("Unable to get sink pad\n");
  else
    gst_pad_add_probe (osd_sink_pad, GST_PAD_PROBE_TYPE_BUFFER,
        osd_sink_pad_buffer_probe, NULL, NULL);
  gst_object_unref (osd_sink_pad);

  g_print ("Using pylonsrc (Basler camera serial %s)\n",
           config.camera_serial.c_str());
  gst_element_set_state (pipeline, GST_STATE_PLAYING);

  g_print ("Running...\n");
  g_main_loop_run (loop);

  g_print ("Returned, stopping playback\n");
  gst_element_set_state (pipeline, GST_STATE_NULL);
  g_print ("Deleting pipeline\n");
  gst_object_unref (GST_OBJECT (pipeline));
  g_source_remove (bus_watch_id);
  g_main_loop_unref (loop);
  return 0;
}
