#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

from src.config import load_config
from train import require_gpu, run

DEFAULT_CONFIGS = [
    "train_configs/yolov8_p2.yaml",
    "train_configs/yolo11_p2.yaml",
    "train_configs/yolo11_p2_1024.yaml",
]


def main():
    parser = argparse.ArgumentParser(
        description="Train several Patronus configs in sequence"
    )
    parser.add_argument(
        "configs",
        nargs="*",
        default=DEFAULT_CONFIGS,
        help="paths to YAML configs in train_configs/",
    )
    parser.add_argument(
        "--runs-dir",
        default="./runs",
        help="parent directory for all experiment outputs",
    )
    cli_args = parser.parse_args()

    # fail once up front instead of once per experiment
    require_gpu()

    # Each config becomes an experiment named after its file (e.g. dfine_large).
    # Artifacts (checkpoints, stats, samples) go under <runs-dir>/<name>, while
    # every experiment logs TensorBoard into the shared <runs-dir>/tensorboard/
    # root under its own name, so one `tensorboard --logdir` overlays them all.
    runs_dir = Path(cli_args.runs_dir)
    tb_root = runs_dir / "tensorboard"
    results = {}
    for config_path in cli_args.configs:
        name = Path(config_path).stem
        config = load_config(config_path)
        config["training"]["output_dir"] = str(runs_dir / name)
        config["training"]["tensorboard_dir"] = str(tb_root / name)

        print(f"\n########## experiment: {name} ##########")
        try:
            metrics = run(config)
            results[name] = {"status": "ok", "metrics": metrics}
        except Exception as exc:
            results[name] = {"status": f"failed: {exc}", "metrics": {}}
            print(f"  experiment {name} failed: {exc}")

    runs_dir.mkdir(parents=True, exist_ok=True)
    with open(runs_dir / "results.json", "w") as f:
        json.dump(results, f, indent=2)

    print("\n=== summary ===")
    for name, result in results.items():
        map_score = result["metrics"].get("map")
        suffix = f" (mAP {map_score:.4f})" if map_score is not None else ""
        print(f"  {name}: {result['status']}{suffix}")
    print(f"\ncompare all experiments: tensorboard --logdir {tb_root}")
    print(f"metrics summary: {runs_dir / 'results.json'}")


if __name__ == "__main__":
    main()
