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

#define MAX_DISPLAY_LEN 64

#define PGIE_CLASS_ID_DRONE 0

/* The muxer output resolution must be set if the input streams will be of
 * different resolution. The muxer will scale all the input frames to this
 * resolution. */
#define MUXER_OUTPUT_WIDTH 960
#define MUXER_OUTPUT_HEIGHT 544

/* Muxer batch formation timeout, for e.g. 40 millisec. Should ideally be set
 * based on the fastest source's framerate. */
#define MUXER_BATCH_TIMEOUT_USEC 40000

/* Check for parsing error. */
#define RETURN_ON_PARSER_ERROR(parse_expr) \
  if (NVDS_YAML_PARSER_SUCCESS != parse_expr) { \
    g_printerr("Error in parsing configuration file.\n"); \
    return -1; \
  }

gint frame_number = 0;
gchar pgie_classes_str[1][32] = { "Drone" };

/* osd_sink_pad_buffer_probe  will extract metadata received on OSD sink pad
 * and update params for drawing rectangle, object information etc. */

static GstPadProbeReturn
osd_sink_pad_buffer_probe (GstPad * pad, GstPadProbeInfo * info,
    gpointer u_data)
{
    GstBuffer *buf = (GstBuffer *) info->data;
    guint num_rects = 0;
    NvDsObjectMeta *obj_meta = NULL;
    guint drone_count = 0;
    NvDsMetaList * l_frame = NULL;
    NvDsMetaList * l_obj = NULL;
    NvDsDisplayMeta *display_meta = NULL;

    NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta (buf);

    for (l_frame = batch_meta->frame_meta_list; l_frame != NULL;
      l_frame = l_frame->next) {
        NvDsFrameMeta *frame_meta = (NvDsFrameMeta *) (l_frame->data);
        int offset = 0;
        num_rects = 0;
        drone_count = 0;
        for (l_obj = frame_meta->obj_meta_list; l_obj != NULL;
                l_obj = l_obj->next) {
            obj_meta = (NvDsObjectMeta *) (l_obj->data);
            if (obj_meta->class_id == PGIE_CLASS_ID_DRONE) {
                drone_count++;
                num_rects++;
            }
        }
        display_meta = nvds_acquire_display_meta_from_pool(batch_meta);
        NvOSD_TextParams *txt_params  = &display_meta->text_params[0];
        display_meta->num_labels = 1;
        txt_params->display_text = g_malloc0 (MAX_DISPLAY_LEN);
        offset = snprintf(txt_params->display_text, MAX_DISPLAY_LEN, "Drone = %d ", drone_count);

        /* Now set the offsets where the string should appear */
        txt_params->x_offset = 10;
        txt_params->y_offset = 12;

        /* Font , font-color and font-size */
        txt_params->font_params.font_name = "Serif";
        txt_params->font_params.font_size = 10;
        txt_params->font_params.font_color.red = 1.0;
        txt_params->font_params.font_color.green = 1.0;
        txt_params->font_params.font_color.blue = 1.0;
        txt_params->font_params.font_color.alpha = 1.0;

        /* Text background color */
        txt_params->set_bg_clr = 1;
        txt_params->text_bg_clr.red = 0.0;
        txt_params->text_bg_clr.green = 0.0;
        txt_params->text_bg_clr.blue = 0.0;
        txt_params->text_bg_clr.alpha = 1.0;

        nvds_add_display_meta_to_frame(frame_meta, display_meta);

        g_print ("Frame %d | Quadrant %d | objects=%d drones=%d\n",
                frame_number, frame_meta->source_id, num_rects, drone_count);
    }
    frame_number++;
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
run_pipeline (int argc, char *argv[])
{
  GMainLoop *loop = NULL;
  GstElement *pipeline = NULL, *source = NULL, *capsfilter_src = NULL,
      *nvvidconv_pre = NULL, *tee = NULL,
      *streammux = NULL, *sink = NULL, *pgie = NULL, *nvvidconv = NULL,
      *nvosd = NULL;
  GstElement *crop[4];
  /* left:top:width:height for each quadrant of 1920x1080 */
  const gchar *crop_rects[4] = {
      "0:0:960:540",    /* top-left     */
      "960:0:960:540",  /* top-right    */
      "0:540:960:540",  /* bottom-left  */
      "960:540:960:540" /* bottom-right */
  };
  GstCaps *caps_src = NULL;

  GstBus *bus = NULL;
  guint bus_watch_id;
  GstPad *osd_sink_pad = NULL;
  gboolean yaml_config = FALSE;
  NvDsGieType pgie_type = NVDS_GIE_PLUGIN_INFER;

  int current_device = -1;
  cudaGetDevice(&current_device);
  struct cudaDeviceProp prop;
  cudaGetDeviceProperties(&prop, current_device);
  /* Check input arguments */
  if (argc != 2) {
    g_printerr ("Usage: %s <nvinfer config file or yml>\n", argv[0]);
    return -1;
  }

  /* Standard GStreamer initialization */
  gst_init (&argc, &argv);
  loop = g_main_loop_new (NULL, FALSE);

  /* Parse inference plugin type */
  yaml_config = (g_str_has_suffix (argv[1], ".yml") ||
          g_str_has_suffix (argv[1], ".yaml"));

  if (yaml_config) {
    RETURN_ON_PARSER_ERROR(nvds_parse_gie_type(&pgie_type, argv[1],
                "primary-gie"));
  }

  /* Create gstreamer elements */
  /* Create Pipeline element that will form a connection of other elements */
  pipeline = gst_pipeline_new ("patronus-mono-pipeline");

  /* Source element for Basler camera via pylonsrc */
  source = gst_element_factory_make ("pylonsrc", "pylon-source");

  /* Caps filter: YUY2 1920x1080 NVMM from pylonsrc */
  capsfilter_src = gst_element_factory_make ("capsfilter", "caps-src");
  caps_src = gst_caps_from_string ("video/x-raw(memory:NVMM),format=GRAY8,width=1920,height=1080");
  g_object_set (G_OBJECT (capsfilter_src), "caps", caps_src, NULL);
  gst_caps_unref (caps_src);

  /* Convert YUY2 NVMM -> NV12 NVMM for streammux (VIC handles YUY2->NV12) */
  nvvidconv_pre = gst_element_factory_make ("nvvideoconvert", "nvvideo-converter-pre");

  /* Tee to split the frame into 4 quadrant branches */
  tee = gst_element_factory_make ("tee", "frame-tee");

  /* One nvvideoconvert per quadrant, each crops a different region */
  for (int i = 0; i < 4; i++) {
    gchar name[32];
    g_snprintf (name, sizeof(name), "crop-conv-%d", i);
    crop[i] = gst_element_factory_make ("nvvideoconvert", name);
    if (!crop[i]) {
      g_printerr ("Failed to create crop converter %d. Exiting.\n", i);
      return -1;
    }
    g_object_set (G_OBJECT (crop[i]), "src-crop", crop_rects[i], NULL);
  }

  /* Create nvstreammux instance to form batches from one or more sources. */
  streammux = gst_element_factory_make ("nvstreammux", "stream-muxer");

  if (!pipeline || !streammux) {
    g_printerr ("One element could not be created. Exiting.\n");
    return -1;
  }

  /* Use nvinfer or nvinferserver to run inferencing on decoder's output,
   * behaviour of inferencing is set through config file */
  if (pgie_type == NVDS_GIE_PLUGIN_INFER_SERVER) {
    pgie = gst_element_factory_make ("nvinferserver", "primary-nvinference-engine");
  } else {
    pgie = gst_element_factory_make ("nvinfer", "primary-nvinference-engine");
  }

  /* Use convertor to convert from NV12 to RGBA as required by nvosd */
  nvvidconv = gst_element_factory_make ("nvvideoconvert", "nvvideo-converter");

  /* Create OSD to draw on the converted RGBA buffer */
  nvosd = gst_element_factory_make ("nvdsosd", "nv-onscreendisplay");

  /* Finally render the osd output */
  if(prop.integrated) {
    sink = gst_element_factory_make("nv3dsink", "nv3d-sink");
  } else {
#ifdef __aarch64__
    sink = gst_element_factory_make ("nv3dsink", "nvvideo-renderer");
#else
    sink = gst_element_factory_make ("nveglglessink", "nvvideo-renderer");
#endif
  }

  if (!source || !capsfilter_src || !nvvidconv_pre || !tee || !pgie || !nvvidconv || !nvosd || !sink) {
    g_printerr ("One element could not be created. Exiting.\n");
    return -1;
  }

  g_object_set (G_OBJECT (streammux), "batch-size", 4, NULL);
  g_object_set (G_OBJECT (streammux), "width", MUXER_OUTPUT_WIDTH, "height",
      MUXER_OUTPUT_HEIGHT, "live-source", TRUE,
      "batched-push-timeout", MUXER_BATCH_TIMEOUT_USEC, NULL);

  if (yaml_config) {
    RETURN_ON_PARSER_ERROR(nvds_parse_streammux(streammux, argv[1],"streammux"));

    /* Set all the necessary properties of the inference element */
    RETURN_ON_PARSER_ERROR(nvds_parse_gie(pgie, argv[1], "primary-gie"));
  } else {
    g_object_set (G_OBJECT (pgie), "config-file-path", argv[1], NULL);
  }

  /* we add a message handler */
  bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
  bus_watch_id = gst_bus_add_watch (bus, bus_call, loop);
  gst_object_unref (bus);

  /* Set up the pipeline */
  gst_bin_add_many (GST_BIN (pipeline),
      source, capsfilter_src, nvvidconv_pre, tee,
      crop[0], crop[1], crop[2], crop[3],
      streammux, pgie, nvvidconv, nvosd, sink, NULL);
  g_print ("Added elements to bin\n");

  /* source → capsfilter → nvvidconv_pre → tee */
  if (!gst_element_link_many (source, capsfilter_src, nvvidconv_pre, tee, NULL)) {
    g_printerr ("Elements could not be linked: 1. Exiting.\n");
    return -1;
  }

  /* tee → queue → crop[i] → streammux sink_i  for each quadrant */
  for (int i = 0; i < 4; i++) {
    gchar queue_name[32], sink_name[16];
    g_snprintf (queue_name, sizeof(queue_name), "queue-%d", i);
    GstElement *q = gst_element_factory_make ("queue", queue_name);
    gst_bin_add (GST_BIN (pipeline), q);

    GstPad *tee_src    = gst_element_request_pad_simple (tee, "src_%u");
    GstPad *queue_sink = gst_element_get_static_pad (q, "sink");
    if (gst_pad_link (tee_src, queue_sink) != GST_PAD_LINK_OK) {
      g_printerr ("Failed to link tee to queue[%d]. Exiting.\n", i);
      return -1;
    }
    gst_object_unref (tee_src);
    gst_object_unref (queue_sink);

    if (!gst_element_link (q, crop[i])) {
      g_printerr ("Failed to link queue[%d] to crop. Exiting.\n", i);
      return -1;
    }

    g_snprintf (sink_name, sizeof(sink_name), "sink_%d", i);
    GstPad *mux_sink  = gst_element_request_pad_simple (streammux, sink_name);
    GstPad *crop_src  = gst_element_get_static_pad (crop[i], "src");
    if (gst_pad_link (crop_src, mux_sink) != GST_PAD_LINK_OK) {
      g_printerr ("Failed to link crop[%d] to streammux. Exiting.\n", i);
      return -1;
    }
    gst_object_unref (mux_sink);
    gst_object_unref (crop_src);
  }

  if (!gst_element_link_many (streammux, pgie, nvvidconv, nvosd, sink, NULL)) {
    g_printerr ("Elements could not be linked: 2. Exiting.\n");
    return -1;
  }

  /* Lets add probe to get informed of the meta data generated, we add probe to
   * the sink pad of the osd element, since by that time, the buffer would have
   * had got all the metadata. */
  osd_sink_pad = gst_element_get_static_pad (nvosd, "sink");
  if (!osd_sink_pad)
    g_print ("Unable to get sink pad\n");
  else
    gst_pad_add_probe (osd_sink_pad, GST_PAD_PROBE_TYPE_BUFFER,
        osd_sink_pad_buffer_probe, NULL, NULL);
  gst_object_unref (osd_sink_pad);

  /* Set the pipeline to "playing" state */
  g_print ("Using pylonsrc (Basler camera)\n");
  gst_element_set_state (pipeline, GST_STATE_PLAYING);

  /* Wait till pipeline encounters an error or EOS */
  g_print ("Running...\n");
  g_main_loop_run (loop);

  /* Out of the main loop, clean up nicely */
  g_print ("Returned, stopping playback\n");
  gst_element_set_state (pipeline, GST_STATE_NULL);
  g_print ("Deleting pipeline\n");
  gst_object_unref (GST_OBJECT (pipeline));
  g_source_remove (bus_watch_id);
  g_main_loop_unref (loop);
  return 0;
}
