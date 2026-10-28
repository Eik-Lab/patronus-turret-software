import numpy as np

from src.stats.palette import PRIMARY

from . import Stat


class AspectRatio(Stat):
    name = "aspect_ratio"

    def compute(self, coco):
        ratios = []
        for ann in coco["annotations"]:
            _, _, w, h = ann["bbox"]
            if h > 0 and w > 0:
                ratios.append(w / h)
        ratios = np.array(ratios)
        return {
            "ratios": ratios,
            "median": float(np.median(ratios)) if len(ratios) else 0.0,
            "mean": float(np.mean(ratios)) if len(ratios) else 0.0,
            "std": float(np.std(ratios)) if len(ratios) else 0.0,
        }

    def report(self, result):
        print(
            f"  aspect ratio (w/h): median {result['median']:.2f}, "
            f"mean {result['mean']:.2f}, std {result['std']:.2f}"
        )

    def plot(self, result, ax):
        ratios = result["ratios"]
        if len(ratios) == 0:
            return False
        ax.hist(ratios, bins=50, color=PRIMARY, edgecolor="white", linewidth=0.3)
        ax.axvline(1.0, color="red", linestyle="--", linewidth=0.8, label="square")
        ax.set_xlabel("width / height")
        ax.set_ylabel("count")
        ax.legend()
        return True
