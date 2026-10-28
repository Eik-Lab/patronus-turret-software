from __future__ import annotations

import json
import os
import sys
from pathlib import Path

from src.config import (
    TRAINING_DEFAULTS,
    ConfigError,
    apply_overrides,
    dataset_problems,
    dataset_report,
    load_config,
)
from src.data.discovery import ensure_format, inspect_dataset, needs_conversion
from src.model import get_format, resolve_model
from src.paths import RUNS_DIR, resolve_path, resolve_run_dir
from src.tests import run_tests

os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")

SPLIT_WIDTH = 5

CONFIG_PATH = "train_configs/default.yaml"
DATASET_PATHS = []
MODEL_SPEC = {"type": "yolo", "name": "yolo11s.pt"}
EPOCHS = 100
BATCH_SIZE = 16
IMAGE_SIZE = 640
VAL_FRACTION = 0.2
SEED = 0
OUTPUT_DIR = "runs/my_run"
FORCE = False
CHECK = False
DEVICE = "cuda"


def _print_dataset_report(report):
    print("Dataset:")
    for key, path in report.get("paths", []):
        print(f"key: path")


def _print_inspection(info):
    if not info:
        return
    if isinstance(info, dict):
        if "datasets" in info:
            for ds in info["datasets"]:
                print("  inspected:")
                if isinstance(ds, list):
                    for item in ds:
                        if isinstance(item, dict):
                            for k, v in item.items():
                                print(f"{k}: {v}")
                        else:
                            print(f"item")
                elif isinstance(ds, dict):
                    for k, v in ds.items():
                        print(f"{k}: {v}")
            return
        for k, v in info.items():
            print(f"{k}: {v}")
        return
    if isinstance(info, list):
        for item in info:
            if isinstance(item, dict):
                for k, v in item.items():
                    print(f"{k}: {v}")
            elif isinstance(item, list):
                for sub in item:
                    if isinstance(sub, dict):
                        for k, v in sub.items():
                            print(f"{k}: {v}")
                    else:
                        print(f"sub")
            else:
                print(f"item")


def _select_device(requested: str = "cuda") -> str:
    try:
        import torch
    except ImportError as exc:
        raise SystemExit("error: PyTorch is not installed; run `uv sync` first") from exc
    if not torch.cuda.is_available():
        raise SystemExit("error: CUDA device required but torch.cuda.is_available() is False")
    return "cuda"


def _require_trainable(device: str, requested: str) -> None:
    if device != "cuda":
        raise SystemExit("error: only CUDA training is supported")


def require_gpu() -> None:
    _select_device("cuda")


def prepare_run(config):
    problems = dataset_problems(config)
    if problems:
        for problem in problems:
            print(f"error: {problem}", file=sys.stderr)
        raise SystemExit(1)
    if needs_conversion(config):
        root = config["dataset"]["root"]
        print(f"[prepare] Converting dataset at {root}")
        ensure_format(config, output_dir=root, val_fraction=config["training"].get("val_fraction", 0.2), seed=config["training"].get("seed", 0))
    report = dataset_report(config)
    _print_dataset_report(report)
    info = inspect_dataset([config["dataset"].get("root")] if config["dataset"].get("root") else [], config["training"].get("val_fraction", 0.2), config["training"].get("seed", 0))
    _print_inspection(info)
    return config


def ensure_dataset(config, force=False):
    return prepare_run(config)


def run(config):
    config = json.loads(json.dumps(config))
    device = _select_device(config["training"].get("device") or "cuda")
    config["training"]["device"] = device
    _require_trainable(device, config["training"].get("device") or "cuda")

    run_dir = resolve_run_dir(config["training"]["output_dir"], force=FORCE)
    config["training"]["output_dir"] = str(run_dir)
    config["training"]["tensorboard_dir"] = str(resolve_path(config["training"].get("tensorboard_dir") or Path(RUNS_DIR) / "tensorboard"))

    print(f"[train] device: {device}")
    print(f"[train] run dir: {run_dir}")

    report = run_tests(config)
    train_stats = report["splits"]["train"]["stats"]

    model_spec = resolve_model(config["model"].get("name"), config["model"].get("architecture"))
    config["model"].update(model_spec)
    model = get_format(config["model"]["type"])
    if hasattr(model, "filter"):
        model.filter(config, str(run_dir), balance_sizes=TRAINING_DEFAULTS.get("balance_sizes", True))
    metrics = model.train(config, train_stats)

    (run_dir / "results.json").write_text(json.dumps(metrics, indent=2))
    return metrics


def finalize_run(metrics, config=None):
    return metrics


def main(argv=None):
    config = load_config(CONFIG_PATH)
    config = prepare_run(config)
    return run(config)


if __name__ == "__main__":
    raise SystemExit(main())
