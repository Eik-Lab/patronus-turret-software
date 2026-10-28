from collections import defaultdict

from src.stats.palette import PRIMARY

from . import Stat


class Overview(Stat):
    name = "overview"

    def compute(self, coco):
        num_images = len(coco["images"])
        annotations = coco["annotations"]
        per_image = defaultdict(int)
        for ann in annotations:
            per_image[ann["image_id"]] += 1
        return {
            "num_images": num_images,
            "num_annotations": len(annotations),
            "empty_images": num_images - len(per_image),
            "objects_per_image": len(annotations) / num_images if num_images else 0.0,
        }

    def report(self, result):
        print(f"  images: {result['num_images']}")
        print(f"  annotations: {result['num_annotations']}")
        print(f"  images without labels: {result['empty_images']}")
        print(f"  objects per image: {round(result['objects_per_image'], 2)}")

    def plot(self, result, ax):
        labels = ["images", "annotations", "empty images"]
        values = [
            result["num_images"],
            result["num_annotations"],
            result["empty_images"],
        ]
        ax.bar(labels, values, color=PRIMARY)
        ax.set_ylabel("count")
        for i, value in enumerate(values):
            ax.text(i, value, str(value), ha="center", va="bottom")
        return True
