# Configuration Files

These are runtime configuration files for the DeepStream inference pipelines.

- `config_infer_primary_yolov8.txt` — Mono camera YOLOv8 config
- `config_infer_primary_rgb.txt` — RGB camera YOLOv8 config
- `labels.txt` — Class labels

## Custom bbox parser (.so)

The custom YOLOv8 bounding box parser (`libnvdsinfer_custom_impl_Yolo.so`) should be placed at:

```
libs/deepstream/libnvdsinfer_custom_impl_Yolo.so
```

```bash
CUDA_VER=13.0 make -C nvdsinfer_custom_impl_Yolo
```
