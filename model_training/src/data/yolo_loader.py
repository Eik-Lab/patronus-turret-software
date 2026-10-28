import os
from pathlib import Path

from PIL import Image

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def yolo_split_to_coco(image_dir, label_dir, class_names):
    image_dir = Path(image_dir)
    label_dir = Path(label_dir)

    categories = [{"id": idx, "name": name} for idx, name in sorted(class_names.items())]

    images = []
    annotations = []
    ann_id = 0

    for img_path in sorted(image_dir.iterdir()):
        if img_path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue

        with Image.open(img_path) as im:
            w, h = im.size
        image_id = len(images)
        images.append({"id": image_id, "file_name": img_path.name, "width": w, "height": h})

        label_path = label_dir / f"{img_path.stem}.txt"
        if not label_path.is_file():
            continue

        with open(label_path) as f:
            for line in f:
                parts = line.split()
                if len(parts) < 5:
                    continue
                cls = int(parts[0])
                cx, cy, bw, bh = float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])

                abs_w = bw * w
                abs_h = bh * h
                abs_x = (cx - bw / 2) * w
                abs_y = (cy - bh / 2) * h
                annotations.append({
                    "id": ann_id,
                    "image_id": image_id,
                    "category_id": cls,
                    "bbox": [abs_x, abs_y, abs_w, abs_h],
                    "area": abs_w * abs_h,
                    "iscrowd": 0,
                })
                ann_id += 1

    return {"images": images, "annotations": annotations, "categories": categories}
