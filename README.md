# Patronus Turret Software

Real-time drone detection and tracking system running on NVIDIA Jetson. Uses a Basler camera, NVIDIA DeepStream for YOLOv8 inference, and a serial-connected pan/tilt turret.

---

## How it works

### Pipeline overview

```
Basler Camera (pylonsrc)
        │
        ▼
capsfilter  ──  GRAY8 NVMM 1920x1080
        │
        ▼
nvvideoconvert  ──  GRAY8 → NV12 (for streammux)
        │
        ▼
nvstreammux  ──  batch size 1, 1920x1088
        │
        ▼
nvinfer  ──  YOLOv8 drone detector (FP16)
        │
        ▼
nvvideoconvert  ──  NV12 → RGBA (for OSD)
        │
        ▼
nvdsosd  ──  draws bounding boxes + drone count overlay
        │
        ▼
nv3dsink  ──  display output
```

The pipeline is built with GStreamer + DeepStream. At each frame, a pad probe on the OSD sink extracts `NvDsObjectMeta` to count detected drones and render the count on-screen.

### Inference

- Model: YOLOv8, single class (`drone`), ONNX input → TensorRT FP16 engine
- Config: `src/deepstream/nvdinfer/config_infer_primary_yolov8.txt`
- Custom bounding box parser: `NvDsInferParseYolo` from `libnvdsinfer_custom_impl_Yolo.so`
- The `.so` is a pre-built library from the [DeepStream-Yolo](https://github.com/marcoslucianops/DeepStream-Yolo) project — **source is included in `src/deepstream/DeepStream-Yolo/` for reference only**, the binary is what actually gets loaded at runtime

### Motor communication

`src/comms/motor_comm.cpp` opens a serial port at 115200 baud and sends pan/tilt commands as `X<float>Y<float>\n` strings to the turret controller.

---

## Project structure

```
src/
├── main.cpp                          # Entry point, calls run_pipeline()
├── comms/
│   ├── motor_comm.h                  # Communication class declaration
│   └── motor_comm.cpp                # Serial pan/tilt communication
└── deepstream/
    ├── nvdinfer/
    │   ├── yolo_inference.h          # run_pipeline() declaration
    │   ├── yolo_inference.c          # GStreamer/DeepStream pipeline
    │   ├── config_infer_primary_yolov8.txt  # nvinfer config (model, parser)
    │   ├── labels.txt                # Class names: ["drone"]
    │   ├── libnvdsinfer_custom_impl_Yolo.so # Pre-built YOLOv8 bbox parser
    │   └── Makefile
    ├── config/
    │   └── dstest1_pgie_config.txt   # Reference config (unused)
    └── DeepStream-Yolo/              # Reference source for the .so above
```

Models are not committed (`.gitignore`). Expected paths:

```
models/
├── yolov8_mono.onnx
└── yolov8_mono.onnx_b1_gpu0_fp16.engine   # auto-generated on first run
```

---

## Dependencies

| Dependency | Purpose |
|---|---|
| NVIDIA DeepStream 8.0 | Pipeline plugins (`nvinfer`, `nvstreammux`, `nvdsosd`, etc.) |
| GStreamer 1.0 | Pipeline framework |
| CUDA 13.0 | GPU runtime (`cudaGetDevice`, `cudaGetDeviceProperties`) |
| TensorRT | Engine file generation and inference (via DeepStream) |
| `pylonsrc` GStreamer plugin | Basler camera source |
| `libnvdsinfer_custom_impl_Yolo.so` | Custom YOLOv8 bounding box parser (pre-built, committed) |

### DeepStream-Yolo dependency

The pipeline config uses a custom bbox parser built from the [DeepStream-Yolo](https://github.com/marcoslucianops/DeepStream-Yolo) project. The pre-built `.so` is already committed at `src/deepstream/nvdinfer/libnvdsinfer_custom_impl_Yolo.so` — **you do not need to rebuild it** unless you upgrade DeepStream or TensorRT.

If a rebuild is needed:
1. Clone [DeepStream-Yolo](https://github.com/marcoslucianops/DeepStream-Yolo)
2. Build with `CUDA_VER=<version> make -C nvdsinfer_custom_impl_Yolo`
3. Replace the `.so` in `src/deepstream/nvdinfer/`

---

## Build

```bash
cd src/deepstream/nvdinfer
CUDA_VER=13.0 make
```

This compiles `yolo_inference.c` + `../../main.cpp` into the `deepstream-test1-app` binary.

---

## Run

```bash
./deepstream-test1-app src/deepstream/nvdinfer/config_infer_primary_yolov8.txt
```

On first run, TensorRT will generate the `.engine` file from the `.onnx` model (takes a few minutes). Subsequent runs load the cached engine directly.

The app expects a Basler camera accessible via `pylonsrc`. If no camera is connected, the pipeline will fail to enter `PLAYING` state.
