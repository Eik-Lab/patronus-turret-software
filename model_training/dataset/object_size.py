from collections import Counter
from palette import EIK_30, EIK_50, EIK_80
from .base import Stat

# Object area as a percentage of the image area.
SMALL_MAX_PCT = 0.5
MEDIUM_MAX_PCT = 3.0


class ObjectSize(Stat):
    name = "object_size"

    def compute(self, coco):
        image_area = {
            img["id"]: img["width"] * img["height"] for img in coco["images"]
        }
        buckets = Counter()
        for ann in coco["annotations"]:
            # Prefer the stored area; fall back to width*height of the bbox.
            area = ann.get("area")
            if area is None:
                _, _, w, h = ann["bbox"]
                area = w * h
            total = image_area.get(ann["image_id"], 0)
            if not total:
                continue
            pct = 100.0 * area / total
            if pct < SMALL_MAX_PCT:
                buckets["small"] += 1
            elif pct < MEDIUM_MAX_PCT:
                buckets["medium"] += 1
            else:
                buckets["large"] += 1
        return {"size_buckets": dict(buckets)}

    def report(self, result):
        buckets = result["size_buckets"]
        total = sum(buckets.values())
        labels = {
            "small": f"small (<{SMALL_MAX_PCT}%)",
            "medium": f"medium ({SMALL_MAX_PCT}-{MEDIUM_MAX_PCT}%)",
            "large": f"large (>={MEDIUM_MAX_PCT}%)",
        }
        print("  object sizes (% of image area):")
        for name in ("small", "medium", "large"):
            count = buckets.get(name, 0)
            share = round(100.0 * count / total, 1) if total else 0.0
            print(f"    {labels[name]}: {count} ({share}%)")

    def plot(self, result, ax):
        buckets = result["size_buckets"]
        names = ["small", "medium", "large"]
        counts = [buckets.get(name, 0) for name in names]
        ax.bar(names, counts, color=[EIK_80, EIK_50, EIK_30])
        ax.set_ylabel("annotations")
        return True
