#!/usr/bin/env python3
import argparse
import json
import os
from pathlib import Path

import yaml
from PIL import Image

IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


def filter_coco(coco, min_size=20):
    # Drop annotations whose box is smaller than min_size px on either side,
    # then drop images that are left without any annotation.
    kept_annotations = []
    for ann in coco["annotations"]:
        _, _, w, h = ann["bbox"]
        if w < min_size or h < min_size:
            continue
        kept_annotations.append(ann)

    image_ids_with_objects = {ann["image_id"] for ann in kept_annotations}
    kept_images = [
        img for img in coco["images"] if img["id"] in image_ids_with_objects
    ]

    filtered = dict(coco)
    filtered["images"] = kept_images
    filtered["annotations"] = kept_annotations

    stats = {
        "images_before": len(coco["images"]),
        "images_after": len(kept_images),
        "annotations_before": len(coco["annotations"]),
        "annotations_after": len(kept_annotations),
    }
    return filtered, stats


def filter_split(annotations_path, output_path, min_size):
    with open(annotations_path, "r") as f:
        coco = json.load(f)
    filtered, stats = filter_coco(coco, min_size)
    with open(output_path, "w") as f:
        json.dump(filtered, f)
    return stats


def filter_coco_dataset(config, output_dir, min_size=None):
    # Filter each COCO split into <output_dir>/filtered/ and repoint the config
    # at the filtered files so stats and training both use them.
    ds_cfg = config["dataset"]
    root = ds_cfg["root"]
    if min_size is None:
        min_size = ds_cfg.get("min_box_size", 20)

    filtered_dir = Path(output_dir) / "filtered"
    filtered_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n=== Filtering dataset (min box size {min_size}px) ===")
    for split in ("train", "val"):
        key = f"{split}_annotations"
        src = os.path.join(root, ds_cfg[key])
        dst = filtered_dir / f"{split}_annotations.coco.json"
        stats = filter_split(src, dst, min_size)
        # Absolute, so joining it onto the dataset root later is a no-op.
        ds_cfg[key] = str(dst.resolve())

        dropped_images = stats["images_before"] - stats["images_after"]
        dropped_annotations = stats["annotations_before"] - stats["annotations_after"]
        print(
            f"  {split}: {stats['annotations_before']} -> "
            f"{stats['annotations_after']} annotations "
            f"(dropped {dropped_annotations}), "
            f"{stats['images_before']} -> {stats['images_after']} images "
            f"(dropped {dropped_images} without detections)"
        )
    return config


def _label_path_for(image_path):
    # Ultralytics maps an image path to its label by swapping the images/ dir
    # for labels/ and the extension for .txt.
    stem = os.path.splitext(image_path)[0]
    parts = stem.rsplit(f"{os.sep}images{os.sep}", 1)
    if len(parts) == 2:
        stem = f"{os.sep}labels{os.sep}".join(parts)
    return stem + ".txt"


def filter_yolo_split(image_dir, out_image_dir, out_label_dir, min_size):
    out_image_dir.mkdir(parents=True, exist_ok=True)
    out_label_dir.mkdir(parents=True, exist_ok=True)

    images_before = images_after = boxes_before = boxes_after = 0
    for image_path in sorted(Path(image_dir).iterdir()):
        if image_path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue
        images_before += 1

        label_path = _label_path_for(str(image_path))
        if not os.path.exists(label_path):
            continue  # no detections -> drop the image
        with open(label_path, "r") as f:
            lines = [line.split() for line in f if line.strip()]
        boxes_before += len(lines)

        with Image.open(image_path) as im:
            img_w, img_h = im.size

        kept = []
        for parts in lines:
            # YOLO txt: class cx cy w h, all normalised to [0, 1].
            _, _, _, w, h = parts[:5]
            if float(w) * img_w < min_size or float(h) * img_h < min_size:
                continue
            kept.append(" ".join(parts))
        if not kept:
            continue  # every box was too small -> drop the image

        boxes_after += len(kept)
        images_after += 1
        link = out_image_dir / image_path.name
        if not link.exists():
            os.symlink(image_path.resolve(), link)
        with open(out_label_dir / f"{image_path.stem}.txt", "w") as f:
            f.write("\n".join(kept) + "\n")

    return {
        "images_before": images_before,
        "images_after": images_after,
        "boxes_before": boxes_before,
        "boxes_after": boxes_after,
    }


def filter_yolo_dataset(config, output_dir, min_size=None):
    # Build a filtered YOLO dataset under <output_dir>/filtered_yolo/ (images
    # symlinked, labels rewritten) and repoint the config's data.yaml at it.
    ds_cfg = config["dataset"]
    if min_size is None:
        min_size = ds_cfg.get("min_box_size", 20)

    data_yaml = ds_cfg.get("data_yaml") or os.path.join(ds_cfg["root"], "data.yaml")
    with open(data_yaml, "r") as f:
        data = yaml.safe_load(f)

    base = Path(data.get("path", os.path.dirname(data_yaml)))
    if not base.is_absolute():
        base = (Path(os.path.dirname(data_yaml)) / base).resolve()

    filtered_root = (Path(output_dir) / "filtered_yolo").resolve()
    new_data = dict(data)
    new_data["path"] = str(filtered_root)

    print(f"\n=== Filtering dataset (min box size {min_size}px) ===")
    for split in ("train", "val"):
        rel = data.get(split)
        if not rel:
            continue
        stats = filter_yolo_split(
            base / rel,
            filtered_root / "images" / split,
            filtered_root / "labels" / split,
            min_size,
        )
        new_data[split] = str(Path("images") / split)

        dropped_images = stats["images_before"] - stats["images_after"]
        dropped_boxes = stats["boxes_before"] - stats["boxes_after"]
        print(
            f"  {split}: {stats['boxes_before']} -> {stats['boxes_after']} boxes "
            f"(dropped {dropped_boxes}), "
            f"{stats['images_before']} -> {stats['images_after']} images "
            f"(dropped {dropped_images} without detections)"
        )

    filtered_root.mkdir(parents=True, exist_ok=True)
    new_data_yaml = filtered_root / "data.yaml"
    with open(new_data_yaml, "w") as f:
        yaml.safe_dump(new_data, f)

    ds_cfg["root"] = str(filtered_root)
    ds_cfg["data_yaml"] = str(new_data_yaml)
    return config


def main():
    parser = argparse.ArgumentParser(
        description="Filter a COCO dataset before running stats"
    )
    parser.add_argument("--input", required=True, help="path to a COCO JSON file")
    parser.add_argument(
        "--output",
        help="where to write the filtered COCO JSON (default: overwrite input)",
    )
    parser.add_argument(
        "--min-size",
        type=int,
        default=20,
        help="drop boxes smaller than this many pixels on either side (default: 20)",
    )
    args = parser.parse_args()

    with open(args.input, "r") as f:
        coco = json.load(f)

    filtered, stats = filter_coco(coco, args.min_size)

    output = Path(args.output) if args.output else Path(args.input)
    with open(output, "w") as f:
        json.dump(filtered, f)

    dropped_images = stats["images_before"] - stats["images_after"]
    dropped_annotations = stats["annotations_before"] - stats["annotations_after"]
    print(f"filtered {args.input} (min box size {args.min_size}px)")
    print(
        f"  annotations: {stats['annotations_before']} -> "
        f"{stats['annotations_after']} (dropped {dropped_annotations})"
    )
    print(
        f"  images: {stats['images_before']} -> "
        f"{stats['images_after']} (dropped {dropped_images} without detections)"
    )
    print(f"  wrote {output}")


if __name__ == "__main__":
    main()
