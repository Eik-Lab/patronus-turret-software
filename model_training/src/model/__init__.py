from src.model.hf_models import DeformableDetr, Detr, DFine, RfDetr, RtDetr
from src.model.yolo import Yolo

MODEL_FORMATS = {
    fmt.name: fmt
    for fmt in [DFine(), Detr(), RtDetr(), DeformableDetr(), RfDetr(), Yolo()]
}


def get_format(type_name):
    try:
        return MODEL_FORMATS[type_name]
    except KeyError:
        known = ", ".join(sorted(MODEL_FORMATS))
        raise ValueError(f"unknown model.type '{type_name}' (known: {known})")
