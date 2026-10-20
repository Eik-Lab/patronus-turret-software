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


def get_format(type_name):
    try:
        return MODEL_FORMATS[type_name]
    except KeyError:
        known = ", ".join(sorted(MODEL_FORMATS))
        raise ValueError(f"unknown model.type '{type_name}' (known: {known})")
