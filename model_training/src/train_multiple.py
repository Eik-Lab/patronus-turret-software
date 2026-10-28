from __future__ import annotations

import json
import os
import sys
from pathlib import Path

from src.config import ConfigError, load_config
from src.data.discovery import ensure_format, inspect_dataset, needs_conversion
from src.model import get_format, resolve_model
from src.paths import RUNS_DIR, resolve_path, resolve_run_dir
from src.tests import run_tests

os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")


def _select_device(requested: str = "cuda") -> str:
    try:
        import torch
    except ImportError as exc:
        raise SystemExit("error: PyTorch is not installed; run `uv sync` first") from exc
    if not torch.cuda.is_available():
        raise SystemExit("error: CUDA device required but torch.cuda.is_available() is False")
    return "cuda"


def train_one(config, force=False):
    config = json.loads(json.dumps(config))
    device = _select_device(config["training"].get("device") or "cuda")
    config["training"]["device"] = device

    run_dir = resolve_run_dir(config["training"]["output_dir"], force=force)
    config["training"]["output_dir"] = str(run_dir)
    config["training"]["tensorboard_dir"] = str(resolve_path(config["training"].get("tensorboard_dir") or Path(RUNS_DIR) / "tensorboard"))

    print(f"[train] device: {device}")
    print(f"[train] run dir: {run_dir}")

    if needs_conversion(config):
        root = config["dataset"]["root"]
        ensure_format(config, output_dir=root, val_fraction=config["training"].get("val_fraction", 0.2), seed=config["training"].get("seed", 0))

    roots = [config["dataset"].get("root")] if config["dataset"].get("root") else []
    info = inspect_dataset(roots, config["training"].get("val_fraction", 0.2), config["training"].get("seed", 0))
    if isinstance(info, dict):
        datasets = info.get("datasets")
        if isinstance(datasets, list):
            for ds in datasets:
                if isinstance(ds, list):
                    _ds = ds
                    for item in _ds:
                        if isinstance(item, dict):
                            for k, v in item.items():
                                print(f"{k}: {v}")
                elif isinstance(ds, dict):
                    for k, v in ds.items():
                        print(f"{k}: {v}")
            return
        for k, v in info.items():
            if k != "datasets":
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

    report = run_tests(config)
    train_stats = report["splits"]["train"]["stats"]

    model_spec = resolve_model(config["model"].get("name"), config["model"].get("architecture"))
    config["model"].update(model_spec)
    model = get_format(config["model"]["type"])
    if hasattr(model, "filter"):
        model.filter(config, str(run_dir))
    metrics = model.train(config, train_stats)
    (run_dir / "results.json").write_text(json.dumps(metrics, indent=2))
    return metrics


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]
    if not argv:
        raise SystemExit("usage: train_multiple.py <config1.yaml> <config2.yaml> ...")
    for arg in argv:
        try:
            cfg = load_config(arg)
            train_one(cfg)
        except Exception as e:
            print(f"error training {arg}: {e}")
            raise
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
