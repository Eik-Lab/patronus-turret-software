import json
import os
from pathlib import Path
from typing import Any

import yaml

from src.config.defaults import TRAINING_DEFAULTS
from src.paths import PROJECT_ROOT, resolve_path


class ConfigError(ValueError):
    pass


def _deep_merge(base: dict, override: dict) -> dict:
    result = dict(base)
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def _load_yaml(path: Path) -> dict:
    with path.open() as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ConfigError(f"config must be a mapping: {path}")
    return data


def load_defaults() -> dict:
    return json.loads(json.dumps(TRAINING_DEFAULTS))


def load_config(path: str | Path | None = None) -> dict:
    if path is None:
        return load_defaults()
    resolved = resolve_path(path)
    if not resolved.is_file():
        raise ConfigError(f"config not found: {resolved}")
    if resolved.suffix in (".yaml", ".yml"):
        data = _load_yaml(resolved)
    elif resolved.suffix == ".json":
        with resolved.open() as f:
            data = json.load(f)
        if not isinstance(data, dict):
            raise ConfigError(f"config must be a mapping: {resolved}")
    else:
        raise ConfigError(f"unsupported config format: {resolved.suffix}")
    config = _deep_merge(load_defaults(), data)
    _validate(config, resolved)
    return config


def _validate(config: dict, source: Path) -> None:
    model = config.get("model") or {}
    if not model.get("type"):
        raise ConfigError("model.type is required")
    if not model.get("name"):
        raise ConfigError("model.name is required")
    training = config.get("training") or {}
    if "device" in training:
        raise ConfigError("training.device is not supported; GPU selection is automatic")
    for key in ("epochs", "batch_size", "image_size"):
        value = training.get(key)
        if not isinstance(value, int) or value <= 0:
            raise ConfigError(f"training.{key} must be a positive int")
    if training.get("val_fraction") is not None:
        vf = training.get("val_fraction")
        if not isinstance(vf, (int, float)) or not 0 < float(vf) < 1:
            raise ConfigError("training.val_fraction must be between 0 and 1")
    dataset = config.get("dataset") or {}
    if not dataset.get("root") and not (dataset.get("train") and dataset.get("val")):
        raise ConfigError("dataset.root or dataset.train+dataset.val required")


def apply_overrides(config: dict, overrides: dict) -> dict:
    if not overrides:
        return config
    result = json.loads(json.dumps(config))
    for key, value in overrides.items():
        if "." in key:
            parts = key.split(".")
            target = result
            for part in parts[:-1]:
                if part not in target or not isinstance(target[part], dict):
                    target[part] = {}
                target = target[part]
            target[parts[-1]] = value
        else:
            result[key] = value
    _validate(result, Path("<overrides>"))
    return result


def dataset_report(config: dict) -> dict:
    dataset = config.get("dataset") or {}
    root = dataset.get("root")
    paths = []
    for key in ("train", "val", "test"):
        if dataset.get(key):
            paths.append((key, dataset[key]))
    if root:
        paths.append(("root", root))
    return {"paths": paths}


def dataset_problems(config: dict) -> list[str]:
    problems = []
    dataset = config.get("dataset") or {}
    root = dataset.get("root")
    if root:
        resolved = resolve_path(root)
        if not resolved.exists():
            problems.append(f"dataset.root does not exist: {resolved}")
    for key in ("train", "val", "test"):
        p = dataset.get(key)
        if p:
            resolved = resolve_path(p)
            if not resolved.exists():
                problems.append(f"dataset.{key} does not exist: {resolved}")
    return problems
