from .class_distribution import ClassDistribution
from .object_size import ObjectSize
from .overview import Overview

ALL_STATS = [
    Overview(),
    ClassDistribution(),
    ObjectSize(),
]
