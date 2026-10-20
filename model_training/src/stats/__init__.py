from .aspect_ratio import AspectRatio
from .bbox_area import BboxArea
from .class_distribution import ClassDistribution
from .object_size import ObjectSize
from .overview import Overview
from .stats import explore_dataset

ALL_STATS = [
    Overview(),
    ClassDistribution(),
    ObjectSize(),
    AspectRatio(),
    BboxArea(),
]
