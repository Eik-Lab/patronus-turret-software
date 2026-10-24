# Patronus Training

Train detection models from COCO or YOLO datasets with one command. The trainer auto-detects the format, merges multiple folders, creates a validation split if needed, and picks sensible defaults.

## Quick start

```bash
cd model_training
uv sync  # or pip install -e .

# From one or more dataset folders (COCO or YOLO)
python train.py --data /path/to/dataset

# Merge multiple datasets
python train.py --data /path/to/dataset1 /path/to/dataset2

# Pick a model
python train.py --data /path/to/dataset --model yolo11m.pt
python train.py --data /path/to/dataset --model Roboflow/rf-detr-medium
```

## What it does

- **Auto-detects** COCO (`annotations.json`) or YOLO (`data.yaml` or `images/` + `labels/` folders)
- **Finds splits** by name (`train/`, `val/`, `test/`)
- **Auto-splits** if only a train split exists (`--val-fraction 0.2`)
- **Merges** multiple folders into one canonical dataset under `runs/<timestamp>/dataset/`
- **Writes** COCO JSON, matching YOLO labels, and `data.yaml`
- **Trains** with sensible defaults, filters small boxes, runs stats, exports ONNX

## Options

```bash
python train.py --data <folder> [<folder> ...] \
  --model auto \
  --epochs 100 \
  --batch-size 16 \
  --image-size 640 \
  --val-fraction 0.2 \
  --seed 0 \
  --output runs/my_run
```

## Advanced (YAML configs)

You can still use the old config files:

```bash
python train.py --config train_configs/yolo11_p2.yaml
python train_multiple.py
```

## Outputs

Each run creates `runs/<timestamp>/` with `config.yaml`, `results.json`, `dataset/`, filtered data, weights/ONNX, TensorBoard logs, and evaluation artifacts.
