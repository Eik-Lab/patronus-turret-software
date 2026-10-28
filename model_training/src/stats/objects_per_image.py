from collections import Counter

import numpy as np

from src.stats.palette import PRIMARY

from . import Stat


class ObjectsPerImage(Stat):
    name = "objects_per_image"

    def compute(self, coco):
        counts = Counter()
        for ann in coco["annotations"]:
            counts[ann["image_id"]] += 1
        per_image = [counts.get(img["id"], 0) for img in coco["images"]]
        values = np.array(per_image)
        return {
            "counts": {str(count): per_image.count(count) for count in sorted(set(per_image))},
            "median": float(np.median(values)) if len(values) else 0.0,
            "max": int(np.max(values)) if len(values) else 0,
            "empty": sum(1 for count in per_image if count == 0),
        }

    def report(self, result):
        print(
            f"  objects/image: median {result['median']:.1f}, "
            f"max {result['max']}, empty {result['empty']}"
        )

    def plot(self, result, ax):
        counts = result["counts"]
        if not counts:
            return False
        labels = [int(value) for value in counts]
        values = [counts[str(value)] for value in labels]
        ax.bar(labels, values, color=PRIMARY)
        ax.set_xlabel("annotations per image")
        ax.set_ylabel("images")
        return True
