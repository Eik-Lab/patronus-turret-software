from pathlib import Path

from src.config.defaults import TRAINING_DEFAULTS
from src.config.load import (
    ConfigError,
    apply_overrides,
    dataset_problems,
    dataset_report,
    load_config,
    load_defaults,
)

__all__ = [
    "TRAINING_DEFAULTS",
    "ConfigError",
    "apply_overrides",
    "dataset_problems",
    "dataset_report",
    "load_config",
    "load_defaults",
    "resolve",
]


def resolve(root: str, relative: str) -> str:
    path = Path(relative).expanduser()
    return str(path if path.is_absolute() else (Path(root) / path).resolve())
