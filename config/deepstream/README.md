# Configuration Files

These are runtime configuration files for the DeepStream inference pipelines.

- `config_infer_primary_yolov8.txt` — Mono camera YOLOv8 config
- `config_infer_primary_rgb.txt` — RGB camera YOLOv8 config
- `config_infer_primary_deformable_detr.txt` — Deformable DETR config
- `config_infer_primary_rf_detr.txt` — RF-DETR drone config
- `labels.txt` — Class labels

## RF-DETR

Export the fine-tuned Hugging Face checkpoint to ONNX:

```bash
cd model_training
python export_rfdetr_onnx.py \
  --model-dir rf_detr_large \
  --output ./models/rf_detr_drone.onnx
```

The export is fixed to `1x3x1024x1024` RGB input with `pred_logits` (`[300, 1]`)
and `pred_boxes` (`[300, 4]`, normalized `cxcywh`). ImageNet normalization is
folded into the patch-embedding convolution, so the nvinfer config uses
`net-scale-factor=1/255` and `offsets=0`.

Build the parser on the target and point `config_infer_primary_rf_detr.txt` at
the ONNX file:

```bash
make -C deps/deepstream-deformable-detr
```

TensorRT engines are not portable across GPU architectures. Build the `.engine`
on the Jetson Thor (DeepStream does this on first run), not on the RTX 3090.

## Custom bbox parser (.so)

The custom YOLOv8 bounding box parser (`libnvdsinfer_custom_impl_Yolo.so`) should be placed at:

```
deps/deepstream/libnvdsinfer_custom_impl_Yolo.so
```

```bash
CUDA_VER=13.0 make -C nvdsinfer_custom_impl_Yolo
```
