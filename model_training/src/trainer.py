import json
import os
from pathlib import Path

from src.data.discovery import ensure_format, inspect_dataset, needs_conversion
from src.model import get_format, resolve_model
from src.paths import resolve_run_dir
from src.data.audit import run_tests

os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")


class GPUUnavailableError(RuntimeError):
    """No usable GPU is available for training."""


def select_device() -> str:
    """Require CUDA; never fall back to another device."""
    try:
        import torch
    except ImportError as exc:
        raise GPUUnavailableError("PyTorch is not installed; run `uv sync` first") from exc

    if torch.cuda.is_available():
        return "cuda"
    raise GPUUnavailableError("CUDA is required but no CUDA device is available")


def train(config: dict, *, force: bool = False, balance_sizes: bool = True) -> dict:
    """Prepare the dataset and run one model using a validated configuration."""
    config = json.loads(json.dumps(config))
    device = select_device()
    config["training"]["device"] = device

    run_dir = resolve_run_dir(config["training"]["output_dir"], force=force)
    config["training"]["output_dir"] = str(run_dir)
    print(f"[train] device: {device}")
    print(f"[train] run dir: {run_dir}")

    if needs_conversion(config):
        root = config["dataset"]["root"]
        print(f"[prepare] Converting dataset at {root}")
        ensure_format(
            config,
            output_dir=root,
            val_fraction=config["training"].get("val_fraction", 0.2),
            seed=config["training"].get("seed", 0),
        )

    roots = [config["dataset"]["root"]] if config["dataset"].get("root") else []
    inspection = inspect_dataset(
        roots, config["training"].get("val_fraction", 0.2), config["training"].get("seed", 0)
    )
    print(f"[train] dataset: {inspection}")

    model_spec = resolve_model(config["model"].get("name"), config["model"].get("architecture"))
    config["model"].update(model_spec)
    model = get_format(config["model"]["type"])
    if hasattr(model, "filter"):
        model.filter(config, str(run_dir), balance_sizes=balance_sizes)

    report = run_tests(config, output_dir=run_dir, save=True, balance_sizes=balance_sizes)
    train_stats = report["splits"]["train"]["stats"]
    metrics = model.train(config, train_stats)
    (run_dir / "results.json").write_text(json.dumps(metrics, indent=2))
    return metrics
