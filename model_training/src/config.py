'Config loading, path normalisation and dataset validation.'


from __future__ import annotations

import json
import os
from pathlib import Path

import yaml

from src.paths import IMAGE_EXTENSIONS, RUNS_DIR, resolve_path

MODEL_TYPES = frozenset({"yolo", "hf", "rfdetr", "dfine"})
COCO_MODEL_TYPES = frozenset({"hf", "rfdetr", "dfine"})

TRAINING_DEFAULTS = {
    "epochs": 100,
    "batch_size": 16,
    "image_size": 640,
    "learning_rate": 1.0e-4,
    "weight_decay": 5.0e-4,
    "num_workers": 4,
    "seed": 0,
    "logging_steps": 10,
}

DATASET_DEFAULTS = {"min_box_size": 20}


class ConfigError(ValueError):
    pass


def load_config(path) -> dict:
    config_path = Path(path)
    try:
        raw = yaml.safe_load(config_path.read_text())
    except OSError as exc:
        raise ConfigError(f"cannot read config '{config_path}': {exc}") from exc
    except yaml.YAMLError as exc:
        raise ConfigError(f"config '{config_path}' is not valid YAML: {exc}") from exc

    if not isinstance(raw, dict):
        raise ConfigError(f"config '{config_path}' must contain a YAML mapping")

    config = normalize_config(raw, source=str(config_path))
    validate_config(config, source=str(config_path))
    return config


def new_config(dataset: dict, model: dict, training: dict) -> dict:
    config = normalize_config(
        {"dataset": dataset, "model": model, "training": training},
        source="<cli>",
    )
    validate_config(config, source="<cli>")
    return config


def normalize_config(raw: dict, source: str = "<config>") -> dict:
    config = {
        "dataset": dict(raw.get("dataset") or {}),
        "model": dict(raw.get("model") or {}),
        "training": dict(raw.get("training") or {}),
    }
    for section in ("dataset", "model", "training"):
        if not isinstance(raw.get(section), dict):
            raise ConfigError(f"config '{source}': missing the '{section}' section")

    dataset = config["dataset"]
    for key, value in DATASET_DEFAULTS.items():
        dataset.setdefault(key, value)
    training = config["training"]
    for key, value in TRAINING_DEFAULTS.items():
        training.setdefault(key, value)

    stem = Path(source).stem if source not in ("<cli>", "") else "run"
    training.setdefault("output_dir", str(RUNS_DIR / stem))
    training.setdefault("tensorboard_dir", str(RUNS_DIR / "tensorboard" / stem))

    root_value = dataset.get("root")
    if not root_value:
        raise ConfigError(f"config '{source}': dataset.root is required")
    root = resolve_path(root_value)
    dataset["root"] = str(root)

    if dataset.get("data_yaml"):
        data_yaml = resolve_path(dataset["data_yaml"])
        if not data_yaml.is_file():
            fallback = root / str(dataset["data_yaml"])
            if fallback.is_file():
                data_yaml = fallback.resolve()
        dataset["data_yaml"] = str(data_yaml)

    for split in ("train", "val", "test"):
        for suffix in ("images", "annotations"):
            key = f"{split}_{suffix}"
            if dataset.get(key):
                dataset[key] = str(_relative_to(root, dataset[key]))

    training["output_dir"] = str(resolve_path(training["output_dir"]))
    training["tensorboard_dir"] = str(resolve_path(training["tensorboard_dir"]))

    if config["model"].get("architecture"):
        config["model"]["architecture"] = str(resolve_path(config["model"]["architecture"]))
    return config


def apply_overrides(config: dict, overrides: dict) -> dict:
    training_keys = (
        "epochs",
        "batch_size",
        "image_size",
        "learning_rate",
        "seed",
        "num_workers",
        "output_dir",
        "tensorboard_dir",
        "device",
    )
    if overrides.get("dataset_root") is not None:
        _rebase_dataset(config["dataset"], resolve_path(overrides["dataset_root"]))
    if overrides.get("dataset_data_yaml") is not None:
        config["dataset"]["data_yaml"] = str(resolve_path(overrides["dataset_data_yaml"]))
    for key in training_keys:
        if overrides.get(key) is not None:
            config["training"][key] = overrides[key]
    if overrides.get("model") is not None:
        from src.model import resolve_model

        resolved = resolve_model(overrides["model"], overrides.get("architecture"))
        config["model"].update(resolved)
    return config


def _rebase_dataset(dataset: dict, new_root: Path) -> None:
    old_root = Path(dataset["root"])
    for key in (
        "data_yaml",
        "train_images",
        "train_annotations",
        "val_images",
        "val_annotations",
        "test_images",
        "test_annotations",
    ):
        value = dataset.get(key)
        if not value:
            continue
        try:
            relative = Path(value).relative_to(old_root)
        except ValueError:
            continue
        dataset[key] = str((new_root / relative).resolve())
    dataset["root"] = str(new_root)


def _relative_to(root: Path, value) -> Path:
    path = Path(os.path.expandvars(str(value))).expanduser()
    return path if path.is_absolute() else (root / path).resolve()


def resolve(root, relative) -> str:
    return str(_relative_to(Path(root), relative))


def validate_config(config: dict, source: str = "<config>") -> None:
    model = config["model"]
    model_type = model.get("type")
    if model_type not in MODEL_TYPES:
        known = ", ".join(sorted(MODEL_TYPES))
        raise ConfigError(
            f"config '{source}': model.type '{model_type}' is unknown (known: {known})"
        )
    if not str(model.get("name") or "").strip():
        raise ConfigError(f"config '{source}': model.name is required")

    training = config["training"]
    positive_ints = ("epochs", "batch_size", "image_size")
    for key in positive_ints:
        _require_positive_int(training.get(key), f"training.{key}", source)
    _require_int(training.get("seed"), "training.seed", source)
    _require_int(training.get("logging_steps"), "training.logging_steps", source, minimum=1)
    _require_int(training.get("num_workers"), "training.num_workers", source, minimum=0)
    _require_positive_number(training.get("learning_rate"), "training.learning_rate", source)

    _require_positive_int(config["dataset"].get("min_box_size"), "dataset.min_box_size", source)

    if model_type == "yolo":
        if not config["dataset"].get("data_yaml"):
            raise ConfigError(
                f"config '{source}': a yolo model needs dataset.data_yaml "
                "(or use --data to build a canonical dataset)"
            )
    else:
        for key in ("train_annotations", "val_annotations"):
            if not config["dataset"].get(key):
                raise ConfigError(
                    f"config '{source}': a {model_type} model needs dataset.{key} "
                    "(or use --data to build a canonical dataset)"
                )


def _require_int(value, field: str, source: str, minimum: int | None = None) -> None:
    ok = isinstance(value, int) and not isinstance(value, bool)
    if ok and minimum is not None:
        ok = value >= minimum
    if not ok:
        hint = f" >= {minimum}" if minimum is not None else ""
        raise ConfigError(f"config '{source}': {field} must be an integer{hint} (got {value!r})")


def _require_positive_int(value, field: str, source: str) -> None:
    _require_int(value, field, source, minimum=1)


def _require_positive_number(value, field: str, source: str) -> None:
    ok = isinstance(value, (int, float)) and not isinstance(value, bool) and value > 0
    if not ok:
        raise ConfigError(f"config '{source}': {field} must be a positive number (got {value!r})")


def dataset_report(config: dict) -> dict:
    dataset = config["dataset"]
    model_type = config["model"]["type"]
    root = Path(dataset["root"])
    if not root.is_dir():
        raise ConfigError(f"dataset.root does not exist: {root}")

    if model_type == "yolo":
        return _yolo_report(dataset, root)
    return _coco_report(dataset, root)


def dataset_problems(config: dict) -> list[str]:
    try:
        report = dataset_report(config)
    except ConfigError as exc:
        return [str(exc)]

    problems = []
    for split in ("train", "val"):
        stats = report["splits"].get(split)
        if stats is None:
            problems.append(f"missing '{split}' split")
            continue
        if stats["images"] == 0:
            problems.append(f"split '{split}' has no images")
        if stats["boxes"] == 0:
            problems.append(f"split '{split}' has no annotated boxes")
    if not report["classes"]:
        problems.append("no class names found")
    return problems


def _yolo_report(dataset: dict, root: Path) -> dict:
    data_yaml = Path(dataset["data_yaml"])
    if not data_yaml.is_file():
        raise ConfigError(f"dataset.data_yaml does not exist: {data_yaml}")
    try:
        data = yaml.safe_load(data_yaml.read_text()) or {}
    except yaml.YAMLError as exc:
        raise ConfigError(f"dataset.data_yaml is not valid YAML: {data_yaml}: {exc}") from exc

    base = Path(data.get("path") or data_yaml.parent)
    if not base.is_absolute():
        base = (data_yaml.parent / base).resolve()

    names = _names(data.get("names"), data.get("nc"))
    splits = {}
    for split in ("train", "val", "test"):
        if not data.get(split):
            continue
        values = data[split] if isinstance(data[split], (list, tuple)) else [data[split]]
        images = boxes = 0
        for value in values:
            image_dir = Path(value) if Path(str(value)).is_absolute() else base / str(value)
            if not image_dir.is_dir():
                raise ConfigError(f"split '{split}' image directory does not exist: {image_dir}")
            label_dir = _labels_dir(image_dir)
            for image_path in image_dir.iterdir():
                if image_path.suffix.lower() not in IMAGE_EXTENSIONS:
                    continue
                images += 1
                label_path = label_dir / f"{image_path.stem}.txt"
                if label_path.is_file():
                    boxes += len(
                        [line for line in label_path.read_text().splitlines() if line.strip()]
                    )
        splits[split] = {"images": images, "boxes": boxes}

    return {"format": "yolo", "root": str(root), "splits": splits, "classes": names}


def _coco_report(dataset: dict, root: Path) -> dict:
    splits = {}
    classes: list[str] = []
    for split in ("train", "val", "test"):
        key = f"{split}_annotations"
        if not dataset.get(key):
            continue
        annotations = Path(dataset[key])
        if not annotations.is_file():
            raise ConfigError(f"dataset.{key} does not exist: {annotations}")
        try:
            coco = json.loads(annotations.read_text())
        except (OSError, json.JSONDecodeError) as exc:
            raise ConfigError(
                f"dataset.{key} is not valid COCO JSON: {annotations}: {exc}"
            ) from exc
        splits[split] = {
            "images": len(coco.get("images", [])),
            "boxes": len(coco.get("annotations", [])),
        }
        if not classes:
            classes = [
                str(category.get("name", category.get("id")))
                for category in sorted(coco.get("categories", []), key=lambda c: c.get("id", 0))
            ]
    return {"format": "coco", "root": str(root), "splits": splits, "classes": classes}


def _names(names, nc) -> list[str]:
    if isinstance(names, dict):
        return [
            str(value) for _, value in sorted((int(key), value) for key, value in names.items())
        ]
    if isinstance(names, (list, tuple)):
        return [str(value) for value in names]
    if nc:
        return [f"class_{index}" for index in range(int(nc))]
    return []


def _labels_dir(image_dir: Path) -> Path:
    parts = list(image_dir.parts)
    if "images" in parts:
        parts[len(parts) - 1 - parts[::-1].index("images")] = "labels"
        return Path(*parts)
    return image_dir.parent / "labels"
