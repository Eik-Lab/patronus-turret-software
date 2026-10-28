from collections import Counter

from src.stats.palette import PRIMARY

from . import Stat


class ImageSize(Stat):
    name = "image_size"

    def compute(self, coco):
        sizes = Counter((img["width"], img["height"]) for img in coco["images"])
        widths = [width for width, _ in sizes]
        heights = [height for _, height in sizes]
        return {
            "sizes": {f"{width}x{height}": count for (width, height), count in sizes.items()},
            "unique": len(sizes),
            "min_width": min(widths) if widths else 0,
            "max_width": max(widths) if widths else 0,
            "min_height": min(heights) if heights else 0,
            "max_height": max(heights) if heights else 0,
        }

    def report(self, result):
        sizes = sorted(result["sizes"].items(), key=lambda item: item[1], reverse=True)
        summary = ", ".join(f"{name} x{count}" for name, count in sizes[:4])
        more = f" (+{len(sizes) - 4} more)" if len(sizes) > 4 else ""
        print(f"  image sizes: {result['unique']} unique — {summary}{more}")

    def plot(self, result, ax):
        sizes = sorted(result["sizes"].items(), key=lambda item: item[1], reverse=True)
        if not sizes:
            return False
        names = [name for name, _ in sizes[:12]]
        counts = [count for _, count in sizes[:12]]
        ax.bar(names, counts, color=PRIMARY)
        ax.set_ylabel("images")
        ax.tick_params(axis="x", rotation=45)
        return True
