import numpy as np

from src.stats.palette import PRIMARY

from .base import Stat


class BboxArea(Stat):
    name = "bbox_area"

    def compute(self, coco):
        areas = []
        for ann in coco["annotations"]:
            area = ann.get("area")
            if area is None:
                _, _, w, h = ann["bbox"]
                area = w * h
            if area > 0:
                areas.append(area)
        areas = np.array(areas)
        return {
            "areas": areas,
            "median": float(np.median(areas)) if len(areas) else 0.0,
            "min": float(np.min(areas)) if len(areas) else 0.0,
            "max": float(np.max(areas)) if len(areas) else 0.0,
        }

    def report(self, result):
        print(
            f"  bbox area (px²): min {result['min']:.0f}, "
            f"median {result['median']:.0f}, max {result['max']:.0f}"
        )

    def plot(self, result, ax):
        areas = result["areas"]
        if len(areas) == 0:
            return False
        log_areas = np.log10(areas)
        ax.hist(log_areas, bins=50, color=PRIMARY, edgecolor="white", linewidth=0.3)
        ax.set_xlabel("log₁₀(area px²)")
        ax.set_ylabel("count")
        return True
