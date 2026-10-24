#!/usr/bin/env python3
import argparse
import json
import os
from pathlib import Path

os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")

import torch
import yaml

from src.config import load_config
from src.model import get_format
from src.stats import explore_dataset


def require_gpu():
    """Abort unless PyTorch can see a CUDA GPU."""
    if not torch.cuda.is_available():
        raise SystemExit(
            "error: no CUDA GPU visible to PyTorch (torch.cuda.is_available() is False); "
            "training requires a GPU."
        )


def run(config):
    require_gpu()
    output_dir = config["training"]["output_dir"]
    model_format = get_format(config["model"]["type"])
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    with open(Path(output_dir) / "config.yaml", "w") as f:
        yaml.safe_dump(config, f, sort_keys=False)

    # filter -> stats -> train
    model_format.filter(config, output_dir)
    train_stats = explore_dataset(config, output_dir)

    metrics = model_format.train(config, train_stats)
    with open(Path(output_dir) / "results.json", "w") as f:
        json.dump(metrics, f, indent=2)
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
