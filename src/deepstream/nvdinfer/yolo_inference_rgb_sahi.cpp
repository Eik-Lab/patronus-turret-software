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
#include <cuda_runtime_api.h>
#include "gstnvdsmeta.h"
#include "nvds_yml_parser.h"
#include "../../state.hpp"
#include <vector>

#define MAX_DISPLAY_LEN 64

#define PGIE_CLASS_ID_DRONE 0

#define MUXER_OUTPUT_WIDTH 1920
#define MUXER_OUTPUT_HEIGHT 1088

#define MUXER_BATCH_TIMEOUT_USEC 40000

#define RETURN_ON_PARSER_ERROR(parse_expr)                \
  if (NVDS_YAML_PARSER_SUCCESS != parse_expr)             \
  {                                                       \
    g_printerr("Error in parsing configuration file.\n"); \
    return -1;                                            \
  }

gint frame_number_rgb_sahi = 0;
gchar pgie_classes_str_rgb_sahi[1][32] = {"Drone"};
extern ThreadSafeQueue<NvDsObjectMeta> detection_rgb;

static GstPadProbeReturn
osd_sink_pad_buffer_probe_sahi(GstPad *pad, GstPadProbeInfo *info,
                               gpointer u_data)
{
  GstBuffer *buf = (GstBuffer *)info->data;
  guint num_rects = 0;
  NvDsObjectMeta *obj_meta = NULL;
  guint drone_count = 0;
  NvDsMetaList *l_frame = NULL;
  NvDsMetaList *l_obj = NULL;
  NvDsDisplayMeta *display_meta = NULL;

  NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta(buf);

  for (l_frame = batch_meta->frame_meta_list; l_frame != NULL;
       l_frame = l_frame->next)
  {
    NvDsFrameMeta *frame_meta = (NvDsFrameMeta *)(l_frame->data);
    for (l_obj = frame_meta->obj_meta_list; l_obj != NULL;
         l_obj = l_obj->next)
    {
      obj_meta = (NvDsObjectMeta *)(l_obj->data);
      if (obj_meta->class_id == PGIE_CLASS_ID_DRONE)
      {
        detection_rgb.push(*obj_meta);
        drone_count++;
      }
    }
    num_rects = frame_meta->num_obj_meta;
    display_meta = nvds_acquire_display_meta_from_pool(batch_meta);
    NvOSD_TextParams *txt_params = &display_meta->text_params[0];
    display_meta->num_labels = 1;
    txt_params->display_text = (char *)g_malloc0(MAX_DISPLAY_LEN);
    snprintf(txt_params->display_text, MAX_DISPLAY_LEN, "Drone = %d ", drone_count);

    txt_params->x_offset = 10;
    txt_params->y_offset = 12;

    txt_params->font_params.font_name = (char *)"Serif";
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

  g_print("Frame Number = %d Number of objects = %d Drone Count = %d\n",
          frame_number_rgb_sahi, num_rects, drone_count);
  frame_number_rgb_sahi++;
  return GST_PAD_PROBE_OK;
}

static gboolean
bus_call_sahi(GstBus *bus, GstMessage *msg, gpointer data)
{
  GMainLoop *loop = (GMainLoop *)data;
  switch (GST_MESSAGE_TYPE(msg))
  {
  case GST_MESSAGE_EOS:
    g_print("End of stream\n");
    g_main_loop_quit(loop);
    break;
  case GST_MESSAGE_ERROR:
  {
    gchar *debug = NULL;
    GError *error = NULL;
    gst_message_parse_error(msg, &error, &debug);
    g_printerr("ERROR from element %s: %s\n",
               GST_OBJECT_NAME(msg->src), error->message);
    if (debug)
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

int run_pipeline_rgb_sahi(int argc, char *argv[])
{
  /* argv[1] = nvinfer config, argv[2] = nvdspreprocess config */
  GMainLoop *loop = NULL;
  GstElement *pipeline = NULL, *source = NULL, *capsfilter_src = NULL,
             *nvvidconv_pre = NULL,
             *streammux = NULL, *sink = NULL, *pgie = NULL, *nvvidconv = NULL,
             *nvosd = NULL, *preprocess = NULL;
  GstCaps *caps_src = NULL;

  GstBus *bus = NULL;
  guint bus_watch_id;
  GstPad *osd_sink_pad = NULL;

  int current_device = -1;
  cudaGetDevice(&current_device);
  struct cudaDeviceProp prop;
  cudaGetDeviceProperties(&prop, current_device);

  if (argc != 3)
  {
    g_printerr("Usage: %s <nvinfer config> <nvdspreprocess config>\n", argv[0]);
    return -1;
  }

  gst_init(&argc, &argv);
  GMainContext *context = g_main_context_new ();
  g_main_context_push_thread_default (context);
  loop = g_main_loop_new(context, FALSE);

  pipeline = gst_pipeline_new("patronus-rgb-sahi-pipeline");

  source = gst_element_factory_make("pylonsrc", "pylon-source");
  g_object_set(G_OBJECT(source), "device-serial-number", "41882813", NULL);

  capsfilter_src = gst_element_factory_make("capsfilter", "caps-src");
  caps_src = gst_caps_from_string("video/x-raw(memory:NVMM),format=YUY2,width=1920,height=1080");
  g_object_set(G_OBJECT(capsfilter_src), "caps", caps_src, NULL);
  gst_caps_unref(caps_src);

  nvvidconv_pre = gst_element_factory_make("nvvideoconvert", "nvvideo-converter-pre");

  streammux = gst_element_factory_make("nvstreammux", "stream-muxer");

  /* nvdspreprocess: slices each frame into tiles before nvinfer */
  preprocess = gst_element_factory_make("nvdspreprocess", "preprocess");
  g_object_set(G_OBJECT(preprocess), "config-file", argv[2], NULL);

  pgie = gst_element_factory_make("nvinfer", "primary-nvinference-engine");
  g_object_set(G_OBJECT(pgie), "input-tensor-meta", TRUE, NULL);

  nvvidconv = gst_element_factory_make("nvvideoconvert", "nvvideo-converter");
  nvosd = gst_element_factory_make("nvdsosd", "nv-onscreendisplay");

  if (prop.integrated)
  {
    sink = gst_element_factory_make("nv3dsink", "nv3d-sink");
  }
  else
  {
#ifdef __aarch64__
    sink = gst_element_factory_make("nv3dsink", "nvvideo-renderer");
#else
    sink = gst_element_factory_make("nveglglessink", "nvvideo-renderer");
#endif
  }

  if (!pipeline || !streammux || !source || !capsfilter_src || !nvvidconv_pre ||
      !preprocess || !pgie || !nvvidconv || !nvosd || !sink)
  {
    g_printerr("One element could not be created. Exiting.\n");
    return -1;
  }

  g_object_set(G_OBJECT(streammux), "batch-size", 1, NULL);
  g_object_set(G_OBJECT(streammux), "width", MUXER_OUTPUT_WIDTH, "height",
               MUXER_OUTPUT_HEIGHT, "live-source", TRUE,
               "batched-push-timeout", MUXER_BATCH_TIMEOUT_USEC, NULL);

  g_object_set(G_OBJECT(pgie), "config-file-path", argv[1], NULL);

  bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
  bus_watch_id = gst_bus_add_watch(bus, bus_call_sahi, loop);
  gst_object_unref(bus);

  gst_bin_add_many(GST_BIN(pipeline),
                   source, capsfilter_src, nvvidconv_pre, streammux,
                   preprocess, pgie, nvvidconv, nvosd, sink, NULL);

  GstPad *sinkpad, *srcpad;
  gchar pad_name_sink[16] = "sink_0";
  gchar pad_name_src[16] = "src";

  sinkpad = gst_element_request_pad_simple(streammux, pad_name_sink);
  if (!sinkpad)
  {
    g_printerr("Streammux request sink pad failed. Exiting.\n");
    return -1;
  }

  srcpad = gst_element_get_static_pad(nvvidconv_pre, pad_name_src);
  if (!srcpad)
  {
    g_printerr("nvvidconv_pre request src pad failed. Exiting.\n");
    return -1;
  }

  if (gst_pad_link(srcpad, sinkpad) != GST_PAD_LINK_OK)
  {
    g_printerr("Failed to link nvvidconv_pre to streammux. Exiting.\n");
    return -1;
  }

  gst_object_unref(sinkpad);
  gst_object_unref(srcpad);

  if (!gst_element_link_many(source, capsfilter_src, nvvidconv_pre, NULL))
  {
    g_printerr("Elements could not be linked: 1. Exiting.\n");
    return -1;
  }

  /* streammux -> preprocess -> pgie -> nvvidconv -> nvosd -> sink */
  if (!gst_element_link_many(streammux, preprocess, pgie,
                             nvvidconv, nvosd, sink, NULL))
  {
    g_printerr("Elements could not be linked: 2. Exiting.\n");
    return -1;
  }

  osd_sink_pad = gst_element_get_static_pad(nvosd, "sink");
  if (!osd_sink_pad)
    g_print("Unable to get sink pad\n");
  else
    gst_pad_add_probe(osd_sink_pad, GST_PAD_PROBE_TYPE_BUFFER,
                      osd_sink_pad_buffer_probe_sahi, NULL, NULL);
  gst_object_unref(osd_sink_pad);

  g_print("Using pylonsrc (Basler camera) with SAHI tiling\n");
  gst_element_set_state(pipeline, GST_STATE_PLAYING);

  g_print("Running...\n");
  g_main_loop_run(loop);

  g_print("Returned, stopping playback\n");
  gst_element_set_state(pipeline, GST_STATE_NULL);
  g_print("Deleting pipeline\n");
  gst_object_unref(GST_OBJECT(pipeline));
  g_source_remove(bus_watch_id);
  g_main_loop_unref(loop);
  g_main_context_pop_thread_default (context);
  g_main_context_unref (context);
  return 0;
}
