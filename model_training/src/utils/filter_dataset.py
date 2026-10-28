#!/usr/bin/env python3
'Dataset filtering: minimum box size and object-size balancing.'


import argparse
import json
import os
import random
import shutil
from collections import defaultdict
from pathlib import Path

import yaml
from PIL import Image

from src.data.size_buckets import SIZE_BUCKETS, empty_counts, size_bucket

IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


def _image_areas(coco):
    return {img["id"]: img["width"] * img["height"] for img in coco["images"]}


def _annotation_bucket(annotation, image_areas):
    _, _, width, height = annotation["bbox"]
    return size_bucket(width * height, image_areas.get(annotation["image_id"], 0))


def balance_coco(coco, seed=0, target_per_bucket=None):
    image_areas = _image_areas(coco)
    buckets = defaultdict(list)
    unclassified = []
    for annotation in coco["annotations"]:
        bucket = _annotation_bucket(annotation, image_areas)
        if bucket is None:
            unclassified.append(annotation)
        else:
            buckets[bucket].append(annotation)

    before = {bucket: len(buckets[bucket]) for bucket in SIZE_BUCKETS}
    non_empty = [count for count in before.values() if count > 0]
    if target_per_bucket is None:
        target = min(non_empty) if non_empty else 0
    else:
        target = max(0, int(target_per_bucket))

    rng = random.Random(seed)
    kept = list(unclassified)
    after = empty_counts()
    for bucket in SIZE_BUCKETS:
        annotations = sorted(buckets[bucket], key=lambda item: item.get("id", 0))
        if len(annotations) > target:
            annotations = rng.sample(annotations, target)
        after[bucket] = len(annotations)
        kept.extend(annotations)

    kept.sort(key=lambda item: item.get("id", 0))
    return kept, {
        "before": before,
        "after": after,
        "target": target,
        "unclassified": len(unclassified),
    }


def filter_coco(coco, min_size=20, balance_sizes=True, seed=0, target_per_bucket=None):
    kept_annotations = []
    for ann in coco["annotations"]:
        _, _, w, h = ann["bbox"]
        if w < min_size or h < min_size:
            continue
        kept_annotations.append(ann)

    balance = None
    if balance_sizes and kept_annotations:
        kept_annotations, balance = balance_coco(
            {**coco, "annotations": kept_annotations},
            seed=seed,
            target_per_bucket=target_per_bucket,
        )

    image_ids_with_objects = {ann["image_id"] for ann in kept_annotations}
    kept_images = [img for img in coco["images"] if img["id"] in image_ids_with_objects]

    filtered = dict(coco)
    filtered["images"] = kept_images
    filtered["annotations"] = kept_annotations

    stats = {
        "images_before": len(coco["images"]),
        "images_after": len(kept_images),
        "annotations_before": len(coco["annotations"]),
        "annotations_after": len(kept_annotations),
        "balance_sizes": bool(balance_sizes),
        "balance": balance,
    }
    return filtered, stats


def filter_split(
    annotations_path,
    output_path,
    min_size,
    balance_sizes=True,
    seed=0,
    target_per_bucket=None,
):
    with open(annotations_path) as f:
        coco = json.load(f)
    filtered, stats = filter_coco(
        coco,
        min_size,
        balance_sizes=balance_sizes,
        seed=seed,
        target_per_bucket=target_per_bucket,
    )
    with open(output_path, "w") as f:
        json.dump(filtered, f)
    return stats


def filter_coco_dataset(
    config,
    output_dir,
    min_size=None,
    balance_sizes=True,
    seed=None,
):
    ds_cfg = config["dataset"]
    root = ds_cfg["root"]
    if min_size is None:
        min_size = ds_cfg.get("min_box_size", 20)
    if seed is None:
        seed = config.get("training", {}).get("seed", 0)

    filtered_dir = Path(output_dir) / "filtered"
    staging_dir = filtered_dir.with_name("filtered.tmp")
    if staging_dir.exists():
        shutil.rmtree(staging_dir)
    staging_dir.mkdir(parents=True, exist_ok=True)


    original_annotations = {
        f"{split}_annotations": ds_cfg[f"{split}_annotations"]
        for split in ("train", "val", "test")
        if f"{split}_annotations" in ds_cfg
    }

    mode = _filter_mode(balance_sizes)
    print(f"\nFiltering dataset (min box size {min_size}px, {mode})", "-" * 50)
    summary = {
        "min_box_size": min_size,
        "balance_sizes": bool(balance_sizes),
        "seed": seed,
        "splits": {},
    }
    for split in ("train", "val"):
        key = f"{split}_annotations"
        if key not in original_annotations:
            continue
        src = os.path.join(root, original_annotations[key])
        if not os.path.exists(src):
            continue
        dst = staging_dir / f"{split}_annotations.coco.json"
        split_balance = balance_sizes and split == "train"
        stats = filter_split(
            src,
            dst,
            min_size,
            balance_sizes=split_balance,
            seed=seed,
        )

        ds_cfg[key] = str(dst.resolve())
        summary["splits"][split] = stats

        dropped_images = stats["images_before"] - stats["images_after"]
        dropped_annotations = stats["annotations_before"] - stats["annotations_after"]
        print(
            f"  {split}: {stats['annotations_before']} -> "
            f"{stats['annotations_after']} annotations "
            f"(dropped {dropped_annotations}), "
            f"{stats['images_before']} -> {stats['images_after']} images "
            f"(dropped {dropped_images} without detections)"
        )
        _print_balance(split, stats.get("balance"))


    try:
        orig_dir = Path(output_dir) / "original"
        orig_dir.mkdir(parents=True, exist_ok=True)
        for key, path in original_annotations.items():
            src = path if os.path.isabs(path) else os.path.join(root, path)
            if os.path.exists(src):
                dst = orig_dir / f"{key}.coco.json"
                if not dst.exists():
                    shutil.copy(src, dst)
    except OSError as e:
        print(f"  Warning: could not preserve original annotations: {e}")


    if filtered_dir.exists():
        shutil.rmtree(filtered_dir)
    staging_dir.rename(filtered_dir)
    for split in summary["splits"]:
        ds_cfg[f"{split}_annotations"] = str(filtered_dir / f"{split}_annotations.coco.json")
    return summary


def _filter_mode(balance_sizes):
    if not balance_sizes:
        return "no size balancing"
    return "balancing train sizes"


def _print_balance(split, balance):
    if not balance:
        return
    before = balance["before"]
    after = balance["after"]
    print(f"    {split} sizes before: {_format_counts(before)}")
    target = balance.get("target")
    target_text = f" (target {target} per bucket)" if target is not None else ""
    print(f"    {split} sizes after:  {_format_counts(after)}{target_text}")
    empty = [bucket for bucket in SIZE_BUCKETS if before[bucket] == 0]
    if empty and balance.get("target") is not None:
        print(
            f"    warning: {split} has no {', '.join(empty)} objects; "
            "those buckets cannot be balanced"
        )
    if balance["unclassified"]:
        print(
            f"    warning: {balance['unclassified']} {split} box(es) had no image size "
            "and were kept unclassified"
        )
    target = balance.get("target")
    before_total = sum(before.values())
    after_total = sum(after.values()) + balance["unclassified"]
    if target is not None and before_total and after_total / before_total < 0.5:
        print(
            f"    warning: size balancing kept only {100.0 * after_total / before_total:.0f}% "
            f"of the {split} boxes ({before_total} -> {after_total}); "
            "check the bucket distribution before trusting this run"
        )


def _format_counts(counts):
    return ", ".join(f"{bucket}={counts[bucket]}" for bucket in SIZE_BUCKETS)


def _label_path_for(image_path):
    stem = os.path.splitext(image_path)[0]
    parts = stem.rsplit(f"{os.sep}images{os.sep}", 1)
    if len(parts) == 2:
        stem = f"{os.sep}labels{os.sep}".join(parts)
    return stem + ".txt"


def filter_yolo_split(
    image_dir,
    out_image_dir,
    out_label_dir,
    min_size,
    balance_sizes=True,
    seed=0,
    target_per_bucket=None,
):
    out_image_dir.mkdir(parents=True, exist_ok=True)
    out_label_dir.mkdir(parents=True, exist_ok=True)

    images_before = boxes_before = 0
    records = []
    buckets = defaultdict(list)
    unclassified = []


    for image_path in sorted(Path(image_dir).iterdir()):
        if image_path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue
        images_before += 1

        label_path = _label_path_for(str(image_path))
        if not os.path.exists(label_path):
            continue
        with open(label_path) as f:
            lines = [line.split() for line in f if line.strip()]
        boxes_before += len(lines)

        with Image.open(image_path) as im:
            img_w, img_h = im.size

        image_index = len(records)
        kept_lines = []
        for parts in lines:
            if len(parts) < 5:
                continue

            _, _, _, w, h = parts[:5]
            abs_w = float(w) * img_w
            abs_h = float(h) * img_h
            if abs_w < min_size or abs_h < min_size:
                continue
            bucket = size_bucket(abs_w * abs_h, img_w * img_h)
            box_index = len(kept_lines)
            kept_lines.append(" ".join(parts))
            if bucket is None:
                unclassified.append((image_index, box_index))
            else:
                buckets[bucket].append((image_index, box_index))
        records.append({"image": image_path, "lines": kept_lines})

    before = {bucket: len(buckets[bucket]) for bucket in SIZE_BUCKETS}
    non_empty = [count for count in before.values() if count > 0]
    if not balance_sizes:
        target = None
        keep = set(unclassified)
        for bucket in SIZE_BUCKETS:
            keep.update(buckets[bucket])
    else:
        if target_per_bucket is None:
            target = min(non_empty) if non_empty else 0
        else:
            target = max(0, int(target_per_bucket))
        keep = set(unclassified)
        rng = random.Random(seed)
        for bucket in SIZE_BUCKETS:
            refs = sorted(buckets[bucket])
            if len(refs) > target:
                refs = rng.sample(refs, target)
            keep.update(refs)

    after = {bucket: sum(1 for ref in buckets[bucket] if ref in keep) for bucket in SIZE_BUCKETS}

    images_after = boxes_after = 0
    for image_index, record in enumerate(records):
        lines = [
            line
            for box_index, line in enumerate(record["lines"])
            if (image_index, box_index) in keep
        ]
        if not lines:
            continue
        boxes_after += len(lines)
        images_after += 1
        image_path = record["image"]
        link = out_image_dir / image_path.name
        if not link.exists():
            os.symlink(image_path.resolve(), link)
        with open(out_label_dir / f"{image_path.stem}.txt", "w") as f:
            f.write("\n".join(lines) + "\n")

    return {
        "images_before": images_before,
        "images_after": images_after,
        "boxes_before": boxes_before,
        "boxes_after": boxes_after,
        "balance_sizes": bool(balance_sizes),
        "balance": {
            "before": before,
            "after": after,
            "target": target,
            "unclassified": len(unclassified),
        },
    }


def filter_yolo_dataset(config, output_dir, min_size=None, balance_sizes=True, seed=None):
    ds_cfg = config["dataset"]
    if min_size is None:
        min_size = ds_cfg.get("min_box_size", 20)
    if seed is None:
        seed = config.get("training", {}).get("seed", 0)

    data_yaml = ds_cfg.get("data_yaml") or os.path.join(ds_cfg["root"], "data.yaml")
    with open(data_yaml) as f:
        data = yaml.safe_load(f)

    base = Path(data.get("path", os.path.dirname(data_yaml)))
    if not base.is_absolute():
        base = (Path(os.path.dirname(data_yaml)) / base).resolve()

    filtered_root = (Path(output_dir) / "filtered_yolo").resolve()
    staging_root = filtered_root.with_name("filtered_yolo.tmp")
    if staging_root.exists():
        shutil.rmtree(staging_root)
    new_data = dict(data)
    new_data["path"] = str(filtered_root)

    mode = _filter_mode(balance_sizes)
    print(f"\nFiltering dataset (min box size {min_size}px, {mode})", "-" * 50)
    summary = {
        "min_box_size": min_size,
        "balance_sizes": bool(balance_sizes),
        "seed": seed,
        "splits": {},
    }
    for split in ("train", "val"):
        rel = data.get(split)
        if not rel:
            continue
        split_balance = balance_sizes and split == "train"
        stats = filter_yolo_split(
            base / rel,
            staging_root / "images" / split,
            staging_root / "labels" / split,
            min_size,
            balance_sizes=split_balance,
            seed=seed,
        )
        new_data[split] = str(Path("images") / split)
        summary["splits"][split] = stats
        dropped_images = stats["images_before"] - stats["images_after"]
        dropped_boxes = stats["boxes_before"] - stats["boxes_after"]
        print(
            f"  {split}: {stats['boxes_before']} -> {stats['boxes_after']} boxes "
            f"(dropped {dropped_boxes}), "
            f"{stats['images_before']} -> {stats['images_after']} images "
            f"(dropped {dropped_images} without detections)"
        )
        _print_balance(split, stats.get("balance"))


    for split in ("test", "valid", "validation"):
        if not data.get(split):
            continue
        value = data[split]
        if isinstance(value, (list, tuple)):
            new_data[split] = [str(base / str(item)) for item in value]
        else:
            new_data[split] = str(base / str(value))

    staging_root.mkdir(parents=True, exist_ok=True)
    new_data_yaml = staging_root / "data.yaml"
    with open(new_data_yaml, "w") as f:
        yaml.safe_dump(new_data, f)

    if filtered_root.exists():
        shutil.rmtree(filtered_root)
    staging_root.rename(filtered_root)

    ds_cfg["root"] = str(filtered_root)
    ds_cfg["data_yaml"] = str(filtered_root / "data.yaml")
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Filter one COCO JSON and optionally balance object sizes"
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
    parser.add_argument(
        "--balance-sizes",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="even out the tiny/small/medium/large buckets (default: on)",
    )
    parser.add_argument("--seed", type=int, default=0, help="sampling seed (default: 0)")
    args = parser.parse_args(argv)

    with open(args.input) as f:
        coco = json.load(f)

    filtered, stats = filter_coco(
        coco,
        args.min_size,
        balance_sizes=args.balance_sizes,
        seed=args.seed,
    )

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
    if stats["balance"]:
        balance = stats["balance"]
        print(f"  sizes before: {_format_counts(balance['before'])}")
        print(
            f"  sizes after:  {_format_counts(balance['after'])} "
            f"(target {balance['target']} per bucket)"
        )
    print(f"  wrote {output}")
    return 0


if __name__ == "__main__":
    main()
