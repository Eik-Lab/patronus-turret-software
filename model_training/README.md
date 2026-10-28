# Patronus Training

Train detection models (YOLO / RF-DETR / D-FINE) from COCO or YOLO datasets with
one command. The trainer detects the dataset format, merges several folders,
creates a validation split when one is missing, and hands the model a canonical
dataset that contains both COCO annotations and YOLO labels.

## Install

```bash
uv sync
```

## Quick start

```bash
uv run patronus --check
uv run patronus
```

Training reads `train_configs/default.yaml` and uses values in `src/train.py`. All training options go in code or the config.

Behaviour worth knowing:

- **Nothing is overwritten.** If the output directory already has content, the
  run goes to `<name>-<timestamp>` and prints where. Pass `--force` to reuse it.
- **CPU is opt-in.** `--device auto` resolves CUDA → MPS → CPU, but refuses to
  start on CPU; pass `--device cpu` explicitly (useful for smoke tests).
- **`--check` writes nothing** and prints dataset counts, classes, model,
  device, output directory, and every problem it found.
- **Datasets are converted once** when the model needs the other format: an
  HF model given a YOLO folder (or vice versa) gets a canonical copy in
  `<run>/dataset/` that contains both formats.
- **Small boxes are dropped and size buckets are balanced.** The filter removes
  boxes below `dataset.min_box_size` (20 px by default), then downsamples the
  over-represented tiny/small/medium/large buckets to the smallest non-empty
  bucket. The source dataset is never modified; the result goes to
  `<run>/filtered/` or `<run>/filtered_yolo/`. Balancing applies to `train`
  only, so `val` keeps the real distribution. Buckets are bbox area as a share
  of the image: tiny <0.5%, small 0.5–2%, medium 2–6%, large ≥6%.
- **Balancing is code-controlled.** It is on by default. To keep every box above
  `dataset.min_box_size`, call the training entry point from Python with
  `balance_sizes=False`:

  ```python
  from src.config import load_config
  from src.train import train

  train(load_config("train_configs/yolo11_p2.yaml"), balance_sizes=False)
  ```

## Pre-training tests

Before a single training step, `train` runs tests on the filtered train/val
data, prints the result, and writes it to `<run>/tests.json`. Every stat also
gets a plot under `<run>/stats/` (for both splits), plus a cross-split
`class_balance.png`.

The tests report:

- image/annotation counts, empty images and objects per image
- image resolutions (mixed cameras/ crops), bbox area and aspect ratio
- class distribution per split and train→val coverage gaps
- tiny/small/medium/large object counts after balancing
- warnings for train/val filename overlap, classes missing from a split,
  extreme class imbalance, rare classes, zero/negative boxes, boxes outside the
  image, extreme aspect ratios, crowded images and empty size buckets

`patronus train --check` runs the same tests without writing anything.

## Dataset layouts

Anything in one of these shapes is detected automatically. Several roots can be
merged in one run; class names are unified across them.

```
my_dataset/
  train/images/*.jpg
  train/annotations.json
  val/images/*.jpg
  val/annotations.json

my_dataset/
  data.yaml
  images/train/*.jpg
  labels/train/*.txt
```

A folder with only a train split gets a deterministic val split (`--val-fraction`,
`--seed`). `patronus prepare --data ... --config-out my.yaml` writes a starting
config that works for either backend.

## Outputs

```
runs/<run>/
  config.yaml
  results.json
  tests.json
  dataset/
  filtered/
  original/
  stats/
  model.onnx / model_fp16.onnx, labels.txt, inference configs
  <model checkpoints, unless a run was forced>
```

TensorBoard logs are collected in `runs/tensorboard/<run>/` so one
`tensorboard --logdir runs/tensorboard` overlays all experiments. `--force`
reuses an existing directory and its logs; otherwise every run is separate.

## YAML configs

```yaml
dataset:
  root: ./data/drone_yolo
  data_yaml: ./data/drone_yolo/data.yaml
  min_box_size: 20
model:
  type: yolo
  name: yolo11s.pt
training:
  output_dir: ./runs/my_run
  epochs: 100
  batch_size: 16
  image_size: 640
  num_workers: 8
  seed: 0
```

The rules for paths:

- `dataset.root`, `dataset.data_yaml`, `training.output_dir` and
  `training.tensorboard_dir` are relative to **`model_training/`**, regardless
  of where you run the command.
- `*_images` / `*_annotations` are relative to `dataset.root` (absolute paths
  are used as-is).
- `model.architecture` is also anchored on `model_training/`.

`patronus prepare --config-out` is the quickest way to get a correct config for
a dataset on disk; `--check` tells you if an existing config still points at
files that exist.

## Development

`uv sync` installs the runtime dependencies. Type checking is available with
`uv run pyright`; there is no Python test or lint tooling in this project.
