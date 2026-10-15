#!/usr/bin/env python3
import argparse
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml

from dataset import ALL_STATS


def load_config(path):
    with open(path, "r") as f:
        config = yaml.safe_load(f)
    for section in ("dataset", "model", "training"):
        if section not in config:
            raise ValueError(f"config '{path}' is missing the '{section}' section")
    return config


def resolve(root, relative):
    """Join a dataset-relative path onto the dataset root."""
    return os.path.join(root, relative)


def run_stats(annotations_path, split_name):
    """Run every registered stat over one COCO split.

    Returns {stat_name: result} so callers can reuse the raw numbers.
    """
    with open(annotations_path, "r") as f:
        coco = json.load(f)

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


def explore_dataset(config, output_dir=None):
    """Run stats on the train and val splits; return the train-split results.

    When output_dir is given, plots are written to <output_dir>/stats/.
    """
    ds_cfg = config["dataset"]
    root = ds_cfg["root"]
    train_stats = run_stats(resolve(root, ds_cfg["train_annotations"]), "train")
    val_stats = run_stats(resolve(root, ds_cfg["val_annotations"]), "val")
    if output_dir:
        save_plots(train_stats, output_dir, "train")
        save_plots(val_stats, output_dir, "val")
    return train_stats


def main():
    parser = argparse.ArgumentParser(description="Patronus COCO dataset stats")
    parser.add_argument(
        "--config", required=True, help="path to a YAML config in train_configs/"
    )
    cli_args = parser.parse_args()
    config = load_config(cli_args.config)
    explore_dataset(config, config["training"]["output_dir"])


if __name__ == "__main__":
    main()
