import json
from pathlib import Path

from src.model.base import HFDetModel
from src.model.yolo import Yolo

_HF = HFDetModel()
_YOLO = Yolo()

MODEL_FORMATS = {
    "rfdetr": _HF,
    "dfine": _HF,
    "hf": _HF,
    "yolo": _YOLO,
}

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


def get_format(type_name):
    try:
        return MODEL_FORMATS[type_name]
    except KeyError:
        known = ", ".join(sorted(MODEL_FORMATS))
        raise ValueError(f"unknown model.type '{type_name}' (known: {known})")


def _detect_hf_type(path):
    config_path = path / "config.json"
    if not config_path.is_file():
        return "hf"
    try:
        data = json.loads(config_path.read_text())
    except (OSError, json.JSONDecodeError):
        return "hf"
    architectures = data.get("architectures") or []
    for arch in architectures:
        arch_lower = str(arch).lower()
        if "rfdetr" in arch_lower or "rt-detr" in arch_lower and "rfdetr" in str(data).lower():
            return "rfdetr"
        if "dfine" in arch_lower:
            return "dfine"
    if "rfdetr" in str(data).lower():
        return "rfdetr"
    if "dfine" in str(data).lower():
        return "dfine"
    return "hf"


def resolve_model(spec=None, architecture=None):
    """Resolve a model spec (alias, HF id, .pt, or local dir) to a model config."""
    raw = (spec or "auto").strip()
    name = MODEL_ALIASES.get(raw.lower(), raw)
    lower = str(name).lower()

    if lower.endswith(".pt"):
        model_type = "yolo"
    elif "rfdetr" in lower or "rf-detr" in lower:
        model_type = "rfdetr"
    elif "dfine" in lower:
        model_type = "dfine"
    else:
        path = Path(name)
        if path.is_dir() and (path / "config.json").is_file():
            model_type = _detect_hf_type(path)
        elif "/" in name or "-" in name:
            # HuggingFace repo id or local path - default to hf unless matched above
            model_type = "hf"
        else:
            model_type = "hf"

    config = {"type": model_type, "name": name}
    if model_type == "yolo" and architecture:
        config["architecture"] = architecture
    return config
