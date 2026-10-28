#!/usr/bin/env python3
import json
from pathlib import Path

from src.config import load_config
from src.data.load import load_split

from .aspect_ratio import AspectRatio
from .bbox_area import BboxArea
from .class_distribution import ClassDistribution
from .image_size import ImageSize
from .object_size import ObjectSize
from .objects_per_image import ObjectsPerImage
from .overview import Overview

ALL_STATS = [Overview, ImageSize, ObjectSize, ObjectsPerImage, BboxArea, AspectRatio, ClassDistribution]


def run_stats(coco, split_name):
    results = {}
    for stat in ALL_STATS:
        instance = stat()
        results[stat.name] = instance.compute(coco)
    return results


def save_plots(stats, output_dir, split_name):
    import matplotlib.pyplot as plt

    output = Path(output_dir)
    plots = output / "stats"
    plots.mkdir(parents=True, exist_ok=True)
    for stat in ALL_STATS:
        instance = stat()
        fig, ax = plt.subplots()
        if instance.plot(stats[stat.name], ax):
            fig.tight_layout()
            fig.savefig(plots / f"{split_name}_{stat.name}.png")
        plt.close(fig)


def check_overlap(train_coco, val_coco):
    train_names = {img["file_name"] for img in train_coco["images"]}
    val_names = {img["file_name"] for img in val_coco["images"]}
    return sorted(train_names & val_names)


def explore_dataset(config, output_dir=None):
    from src.data.audit import run_tests

    report = run_tests(config, output_dir, save=bool(output_dir))
    train = report["splits"].get("train")
    return train["stats"] if train else {}
