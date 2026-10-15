#!/usr/bin/env python3
import argparse
from pathlib import Path

from stats import load_config
from train import run

DEFAULT_CONFIGS = [
    "train_configs/rf_detr_large.yaml",
    "train_configs/dfine_medium.yaml",
    "train_configs/yolov8_p2.yaml",
]


def main():
    parser = argparse.ArgumentParser(
        description="Train several Patronus configs in sequence"
    )
    parser.add_argument(
        "configs",
        nargs="*",
        default=DEFAULT_CONFIGS,
        help="paths to YAML configs in train_configs/ (default: rf-detr large, D-FINE medium, YOLOv8-P2)",
    )
    parser.add_argument(
        "--runs-dir",
        default="./runs",
        help="parent directory for all experiment outputs",
    )
    cli_args = parser.parse_args()

    # Each config becomes an experiment named after its file (e.g. dfine_large),
    # with its outputs grouped under <runs-dir>/<name> so runs never collide.
    results = {}
    for config_path in cli_args.configs:
        name = Path(config_path).stem
        config = load_config(config_path)
        config["training"]["output_dir"] = str(Path(cli_args.runs_dir) / name)
        config["training"]["tensorboard_dir"] = str(Path(cli_args.runs_dir) / name / "tb")

        print(f"\n########## experiment: {name} ##########")
        try:
            run(config)
            results[name] = "ok"
        except Exception as exc:
            results[name] = f"failed: {exc}"
            print(f"  experiment {name} failed: {exc}")

    print("\n=== summary ===")
    for name, status in results.items():
        print(f"  {name}: {status}")


if __name__ == "__main__":
    main()
