'Dataset statistics plugins and the shared ``explore_dataset`` entry point.'


class Stat:
    name = "stat"

    def compute(self, coco):
        raise NotImplementedError

    def report(self, result):
        pass

    def plot(self, result, ax):
        return False


from .aspect_ratio import AspectRatio
from .bbox_area import BboxArea
from .class_distribution import ClassDistribution
from .image_size import ImageSize
from .object_size import ObjectSize
from .objects_per_image import ObjectsPerImage
from .overview import Overview
from .stats import ALL_STATS, explore_dataset

__all__ = [
    "ALL_STATS",
    "Stat",
    "AspectRatio",
    "BboxArea",
    "ClassDistribution",
    "ImageSize",
    "ObjectSize",
    "ObjectsPerImage",
    "Overview",
    "explore_dataset",
]
