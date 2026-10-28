'Dataset helpers.'


__all__ = ["CocoDetectionDataset"]


def __getattr__(name):
    if name == "CocoDetectionDataset":
        from src.data.coco import CocoDetectionDataset

        return CocoDetectionDataset
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
