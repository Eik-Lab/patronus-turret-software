#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import yaml

from src.config import load_config
from src.model import get_format
from src.stats import explore_dataset


def run(config):
    output_dir = config["training"]["output_dir"]
    model_format = get_format(config["model"]["type"])
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    with open(Path(output_dir) / "config.yaml", "w") as f:
        yaml.safe_dump(config, f, sort_keys=False)

    # filter -> stats -> train
    try:
        model_format.filter(config, output_dir)
    except Exception as e:
        print(f"Warning: filtering failed: {e}")
    try:
        train_stats = explore_dataset(config, output_dir)
    except Exception as e:
        print(f"Warning: stats exploration failed: {e}")
        train_stats = {"class_distribution": {"categories": {}}}

    metrics = model_format.train(config, train_stats)
    try:
        with open(Path(output_dir) / "results.json", "w") as f:
            json.dump(metrics, f, indent=2)
    except Exception as e:
        print(f"Warning: failed to save results.json: {e}")
    return metrics


def main():
    parser = argparse.ArgumentParser(description="Patronus detection trainer")
    parser.add_argument(
        "--config", required=True, help="path to a YAML config in train_configs/"
    )
    cli_args = parser.parse_args()

    run(load_config(cli_args.config))


if __name__ == "__main__":
    main()
