from collections import Counter

from src.stats.palette import EIK_20, EIK_50, EIK_80, STORM_40
from src.data.size_buckets import SIZE_BUCKETS, empty_counts, size_bucket, size_label

from .base import Stat


class ObjectSize(Stat):
    name = "object_size"

    def compute(self, coco):
        image_area = {img["id"]: img["width"] * img["height"] for img in coco["images"]}
        buckets = Counter()
        for ann in coco["annotations"]:
            _, _, width, height = ann["bbox"]
            bucket = size_bucket(width * height, image_area.get(ann["image_id"], 0))
            if bucket is not None:
                buckets[bucket] += 1
        return {"size_buckets": {**empty_counts(), **dict(buckets)}}

    def report(self, result):
        buckets = result["size_buckets"]
        total = sum(buckets.values())
        print("  object sizes (% of image area):")
        for name in SIZE_BUCKETS:
            count = buckets.get(name, 0)
            share = round(100.0 * count / total, 1) if total else 0.0
            print(f"    {size_label(name):<22} {count:>8}  ({share:5.1f}%)")

    def plot(self, result, ax):
        buckets = result["size_buckets"]
        counts = [buckets.get(name, 0) for name in SIZE_BUCKETS]
        colors = [STORM_40, EIK_20, EIK_50, EIK_80]
        ax.bar(list(SIZE_BUCKETS), counts, color=colors)
        ax.set_ylabel("annotations")
        for index, count in enumerate(counts):
            ax.text(index, count, str(count), ha="center", va="bottom")
        return True
