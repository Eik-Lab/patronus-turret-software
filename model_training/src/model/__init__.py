'Model format registry and model-spec resolution.'


from __future__ import annotations

import json
from pathlib import Path

from src.paths import PROJECT_ROOT

MODEL_TYPES = frozenset({"rfdetr", "dfine", "hf", "yolo"})

MODEL_ALIASES = {
    "auto": "yolo11s.pt",
    "yolo11s": "yolo11s.pt",
    "yolo11m": "yolo11m.pt",
    "yolo11l": "yolo11l.pt",
    "yolo11x": "yolo11x.pt",
    "yolov8s": "yolov8s.pt",
    "yolov8m": "yolov8m.pt",
    "yolov8l": "yolov8l.pt",
    "yolov8x": "yolov8x.pt",
    "rfdetr": "Roboflow/rf-detr-medium",
    "rfdetr-medium": "Roboflow/rf-detr-medium",
    "rfdetr-large": "Roboflow/rf-detr-large",
    "dfine": "ustc-community/dfine-medium-coco",
    "dfine-medium": "ustc-community/dfine-medium-coco",
    "dfine-large": "ustc-community/dfine-large-coco",
}


def get_format(type_name: str):
    if type_name == "yolo":
        from src.model.yolo import YOLOModel

        return YOLOModel()
    if type_name in MODEL_TYPES:
        from src.model.base import HFDetModel

        return HFDetModel()
    known = ", ".join(sorted(MODEL_TYPES))
    raise ValueError(f"unknown model.type '{type_name}' (known: {known})")


def _detect_hf_type(path: Path) -> str:
    config_path = path / "config.json"
    if not config_path.is_file():
        return "hf"
    try:
        data = json.loads(config_path.read_text())
    except (OSError, json.JSONDecodeError):
        return "hf"

    text = str(data).lower()
    for architecture in data.get("architectures") or []:
        arch_lower = str(architecture).lower()
        if "rfdetr" in arch_lower or ("rt-detr" in arch_lower and "rfdetr" in text):
            return "rfdetr"
        if "dfine" in arch_lower:
            return "dfine"
    if "rfdetr" in text:
        return "rfdetr"
    if "dfine" in text:
        return "dfine"
    return "hf"


def resolve_model(spec: str | None = None, architecture: str | None = None) -> dict:
    raw = (spec or "auto").strip()
    name = MODEL_ALIASES.get(raw.lower(), raw)
    path = Path(name).expanduser()
    if not path.is_absolute() and (PROJECT_ROOT / path).exists():
        name = str((PROJECT_ROOT / path).resolve())
        path = Path(name)
    lower = name.lower()

    if lower.endswith(".pt"):
        model_type = "yolo"
    elif "rfdetr" in lower or "rf-detr" in lower:
        model_type = "rfdetr"
    elif "dfine" in lower:
        model_type = "dfine"
    else:
        path = Path(name).expanduser()
        if path.is_dir() and (path / "config.json").is_file():
            model_type = _detect_hf_type(path)
        else:
            model_type = "hf"

    config = {"type": model_type, "name": name}
    if model_type == "yolo" and architecture:
        config["architecture"] = architecture
    return config
