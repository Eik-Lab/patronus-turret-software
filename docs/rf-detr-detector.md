# RF-DETR detector option

The detector model is selectable at runtime with `--detector`:

```bash
./run.py --detector rf-detr     # use RF-DETR
./run.py --detector yolov8      # use YOLOv8 (same as default)
./run.py                        # detector taken from config/system.ini
```

The flag is passed through to the `inference` binary (`--detector`, `-d`). When
set to `rf-detr` it overrides the `infer_config` path for both pipelines to
`config/deepstream/config_infer_primary_rfdetr.txt`. Model selection is purely
config-driven; no pipeline code changes per detector.

## Where the RF-DETR files live

The upstream RidgeRun "DeepStream RF-DETR" repo was relocated into the project
layout:

- Parser source + `Makefile` + `LICENSE` → `deps/deepstream-rfdetr/`
- Inference config (upstream example) → `config/deepstream/config_infer_primary_rfdetr.txt`
- Labels → `config/deepstream/coco91_labels.txt`
- ONNX / TensorRT engine → `models/`

## Before it will run

The plumbing is in place, but RF-DETR is not yet functional. To make it work:

1. **Build the parser lib.** `cd deps/deepstream-rfdetr && make` → produces
   `libdeepstream-rfdetr.so` (needs DeepStream headers).
2. **Get the weights.** Download `rfdetr-nano.onnx` into `models/` (see the
   upstream `download_weights.py`). TensorRT builds the `.engine` on first run.
3. **Retrain for drones.** Stock RF-DETR outputs 91 COCO classes (person, car,
   …), not drones. It must be retrained/fine-tuned on drones and the class count
   and `coco91_labels.txt` updated to match, or the turret will track the wrong
   objects.

Note: RF-DETR expects RGB input (`model-color-format=0`), so it suits the RGB
pipeline; the mono camera is GRAY8.
