#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path

from src.config import load_config
from src.data.load import load_split

from .aspect_ratio import AspectRatio
from .bbox_area import BBoxArea
from .class_distribution import ClassDistribution
from .image_size import ImageSize
from .object_size import ObjectSize
from .objects_per_image import ObjectsPerImage
from .overview import Overview

ALL_STATS = [Overview, ImageSize, ObjectSize, ObjectsPerImage, BBoxArea, AspectRatio, ClassDistribution]


def run_stats(coco, split_name):
    results = {}
    for stat in ALL_STATS:
        instance = stat()
        instance.compute(coco)
        results[stat.name] = instance.results()
    return results


def save_plots(stats, output_dir, split_name):
    output = Path(output_dir)
    for stat in ALL_STATS:
        instance = stat()
        instance.load(stats)
        instance.plot(output, split_name)


def check_overlap(train_coco, val_coco):
    train_names = {img["file_name"] for img in train_coco["images"]}
    val_names = {img["file_name"] for img in val_coco["images"]}
    return sorted(train_names & val_names)


def explore_dataset(config, output_dir=None):
    from src.tests import run_tests

    report = run_tests(config, output_dir, save=bool(output_dir))
    train = report["splits"].get("train")
    return train["stats"] if train else {}
