from transformers import (
    DeformableDetrForObjectDetection,
    DetrForObjectDetection,
    DFineForObjectDetection,
    RfDetrForObjectDetection,
    RTDetrForObjectDetection,
)

from src.model.base import HFDetModel


class Detr(HFDetModel):
    name = "detr"
    model_class = DetrForObjectDetection


class DeformableDetr(HFDetModel):
    name = "deformable_detr"
    model_class = DeformableDetrForObjectDetection


class RtDetr(HFDetModel):
    name = "rtdetr"
    model_class = RTDetrForObjectDetection


class DFine(HFDetModel):
    name = "dfine"
    model_class = DFineForObjectDetection


class RfDetr(HFDetModel):
    name = "rfdetr"
    model_class = RfDetrForObjectDetection
