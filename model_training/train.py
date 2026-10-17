#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

from src.model import get_format
from stats import explore_dataset, load_config


def run(config, stats_only=False):
    output_dir = config["training"]["output_dir"]
    model_format = get_format(config["model"]["type"])
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    # filter -> stats -> train. Each format filters its own dataset layout; the
    # COCO stats checker only applies to COCO-JSON formats (YOLO has its own).
    model_format.filter(config, output_dir)

    train_stats = None
    if model_format.coco_dataset:
        train_stats = explore_dataset(config, output_dir)
    elif stats_only:
        print(f"model.type '{model_format.name}' does not use the COCO stats checker")

    if stats_only:
        return {}

    metrics = model_format.train(config, train_stats)
    with open(Path(output_dir) / "results.json", "w") as f:
        json.dump(metrics, f, indent=2)
    return metrics


def main():
    parser = argparse.ArgumentParser(description="Patronus detection trainer")
    parser.add_argument(
        "--config", required=True, help="path to a YAML config in train_configs/"
    )
    parser.add_argument(
        "--stats-only",
        action="store_true",
        help="run the dataset stats checker and exit without training",
    )
    cli_args = parser.parse_args()

    run(load_config(cli_args.config), cli_args.stats_only)


if __name__ == "__main__":
    main()
