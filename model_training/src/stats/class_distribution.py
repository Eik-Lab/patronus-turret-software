from collections import Counter

from src.palette import SEQUENCE

from .base import Stat


class ClassDistribution(Stat):
    name = "class_distribution"

    def compute(self, coco):
        id_to_name = {c["id"]: c["name"] for c in coco["categories"]}
        counts = Counter()
        for ann in coco["annotations"]:
            counts[id_to_name.get(ann["category_id"], ann["category_id"])] += 1
        return {"categories": id_to_name, "per_class": dict(counts)}

    def report(self, result):
        per_class = result["per_class"]
        total = sum(per_class.values())
        print("  class distribution:")
        for name, count in Counter(per_class).most_common():
            share = 100.0 * count / total if total else 0.0
            print(f"    {str(name):<18} {count:>8}  ({share:5.1f}%)")

    def plot(self, result, ax):
        per_class = result["per_class"]
        if not per_class:
            return False
        items = sorted(per_class.items(), key=lambda kv: kv[1], reverse=True)
        names = [str(name) for name, _ in items]
        counts = [count for _, count in items]
        colors = [SEQUENCE[i % len(SEQUENCE)] for i in range(len(names))]
        ax.bar(names, counts, color=colors)
        ax.set_ylabel("annotations")
        ax.tick_params(axis="x", rotation=45)
        return True
