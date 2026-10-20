#!/usr/bin/env python3
"""
Patronus — dataset stats orchestrator.

Runs every stat registered in dataset/ALL_STATS over the splits named in a
training config.  Works with both COCO-JSON and YOLO-txt datasets.

Run standalone to inspect a dataset before training:

  python stats.py --config train_configs/rf_detr.yaml
"""

import argparse
import json
import os
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml

from src.config import load_config, resolve
from src.stats import ALL_STATS
from src.stats.yolo_loader import yolo_split_to_coco


def _load_coco_split(annotations_path):
    with open(annotations_path, "r") as f:
        return json.load(f)


def _load_yolo_split(data_yaml, split):
    """Load a YOLO split into a COCO-like dict via data.yaml."""
    with open(data_yaml, "r") as f:
        data = yaml.safe_load(f)

    base = Path(data.get("path", os.path.dirname(data_yaml)))
    if not base.is_absolute():
        base = (Path(os.path.dirname(data_yaml)) / base).resolve()

    class_names = data.get("names", {})
    if isinstance(class_names, list):
        class_names = {i: name for i, name in enumerate(class_names)}

    image_dir = base / data[split]
    label_dir = Path(str(image_dir).replace(f"{os.sep}images{os.sep}", f"{os.sep}labels{os.sep}"))

    return yolo_split_to_coco(image_dir, label_dir, class_names)


def run_stats(coco, split_name):
    """Run every registered stat over one split.

    Returns {stat_name: result} so callers can reuse the raw numbers.
    """
    print(f"\n=== Dataset stats: {split_name} ===")
    combined = {}
    for stat in ALL_STATS:
        result = stat.compute(coco)
        stat.report(result)
        combined[stat.name] = result
    return combined


def save_plots(combined, output_dir, split_name):
    """Render every stat's plot into <output_dir>/stats/<split>_<stat>.png."""
    stats_dir = os.path.join(output_dir, "stats")
    os.makedirs(stats_dir, exist_ok=True)

    for stat in ALL_STATS:
        fig, ax = plt.subplots(figsize=(8, 5))
        if stat.plot(combined[stat.name], ax):
            ax.set_title(f"{split_name} — {stat.name}")
            fig.tight_layout()
            path = os.path.join(stats_dir, f"{split_name}_{stat.name}.png")
            fig.savefig(path, dpi=120)
            print(f"  saved {path}")
        plt.close(fig)


def check_overlap(train_coco, val_coco):
    """Warn if any image filenames appear in both train and val."""
    train_names = {img["file_name"] for img in train_coco["images"]}
    val_names = {img["file_name"] for img in val_coco["images"]}
    overlap = train_names & val_names
    if overlap:
        print(f"\n  WARNING: {len(overlap)} images appear in both train and val:")
        for name in sorted(overlap)[:10]:
            print(f"    {name}")
        if len(overlap) > 10:
            print(f"    ... and {len(overlap) - 10} more")
    else:
        print("\n  train/val overlap check: OK (no shared images)")


def explore_dataset(config, output_dir=None):
    """Run stats on the train and val splits; return the train-split results."""
    ds_cfg = config["dataset"]
    model_type = config["model"].get("type", "")
    is_yolo = model_type == "yolo"

    if is_yolo:
        data_yaml = ds_cfg.get("data_yaml") or os.path.join(ds_cfg["root"], "data.yaml")
        train_coco = _load_yolo_split(data_yaml, "train")
        val_coco = _load_yolo_split(data_yaml, "val")
    else:
        root = ds_cfg["root"]
        train_coco = _load_coco_split(resolve(root, ds_cfg["train_annotations"]))
        val_coco = _load_coco_split(resolve(root, ds_cfg["val_annotations"]))

    train_stats = run_stats(train_coco, "train")
    run_stats(val_coco, "val")
    check_overlap(train_coco, val_coco)

    if output_dir:
        save_plots(train_stats, output_dir, "train")

    return train_stats


def main():
    parser = argparse.ArgumentParser(description="Patronus dataset stats")
    parser.add_argument(
        "--config", required=True, help="path to a YAML config in train_configs/"
    )
    cli_args = parser.parse_args()
    config = load_config(cli_args.config)
    explore_dataset(config, config["training"]["output_dir"])


if __name__ == "__main__":
    main()
