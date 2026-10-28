'Pre-training dataset tests.'


from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from src.config import ConfigError
from src.data.load import load_split
from src.palette import SEQUENCE
from src.size_buckets import SIZE_BUCKETS

SPLITS = ("train", "val", "test")
EPSILON = 1.0e-6


def run_tests(
    config: dict,
    output_dir=None,
    save: bool = False,
    filter_summary: dict | None = None,
    balance_sizes: bool = False,
) -> dict:
    from src.stats.stats import run_stats

    report = {
        "dataset": {
            "root": config["dataset"]["root"],
            "format": "yolo" if config["model"]["type"] == "yolo" else "coco",
            "balance_sizes": bool(balance_sizes),
            "min_box_size": config["dataset"].get("min_box_size"),
        },
        "filter": filter_summary,
        "splits": {},
        "warnings": [],
    }

    loaded: dict[str, dict] = {}
    full_stats: dict[str, dict] = {}
    for split in SPLITS:
        coco = load_split(config, split)
        if coco is None:
            continue
        loaded[split] = coco
        stats = run_stats(coco, split)
        full_stats[split] = stats
        report["splits"][split] = {
            "images": len(coco.get("images", [])),
            "annotations": len(coco.get("annotations", [])),
            "stats": _compact(stats),
            "warnings": [],
        }

    if "train" not in loaded:
        raise ConfigError("the dataset has no train split to audit")
    if not loaded["train"].get("images"):
        raise ConfigError("the train split has no images after filtering; check min_box_size")
    if not loaded["train"].get("annotations"):
        raise ConfigError(
            "the train split has no annotated boxes after filtering; "
            "lower min_box_size or disable size balancing"
        )
    if "val" not in loaded:
        report["warnings"].append(
            "no validation split: the model cannot be evaluated during training"
        )
    elif not loaded["val"].get("images") or not loaded["val"].get("annotations"):
        report["warnings"].append(
            "the validation split is empty; training metrics will be unavailable"
        )

    for split, coco in loaded.items():
        issues = _split_issues(split, coco, report["splits"][split]["stats"], config, balance_sizes)
        report["splits"][split]["warnings"] = issues
        report["warnings"].extend(f"{split}: {issue}" for issue in issues)

    report["warnings"].extend(_cross_split_issues(loaded))
    _print_warnings(report["warnings"])

    if save and output_dir:
        _save_artifacts(report, loaded, full_stats, output_dir)

    return report


def _compact(stats: dict) -> dict:
    compact = {}
    for name, result in stats.items():
        compact[name] = {
            key: _jsonable(value)
            for key, value in result.items()
            if not isinstance(value, np.ndarray)
        }
    return compact


def _jsonable(value):
    if isinstance(value, dict):
        return {key: _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def _split_issues(
    split: str, coco: dict, stats: dict, config: dict, balance_sizes: bool
) -> list[str]:
    issues: list[str] = []
    images = {img["id"]: img for img in coco.get("images", [])}
    annotations = coco.get("annotations", [])
    categories = {
        category["id"]: str(category.get("name", category["id"]))
        for category in coco.get("categories", [])
    }
    min_size = float(config["dataset"].get("min_box_size") or 0)

    if not images:
        issues.append("no images")
        return issues

    names = [img.get("file_name", "") for img in images.values()]
    duplicates = len(names) - len(set(names))
    if duplicates:
        issues.append(f"{duplicates} duplicate image file name(s)")

    missing_categories = {
        ann["category_id"] for ann in annotations if ann["category_id"] not in categories
    }
    if missing_categories:
        issues.append(
            f"{len(missing_categories)} category id(s) missing from categories: "
            f"{sorted(missing_categories)[:5]}"
        )

    degenerate = out_of_bounds = below_min = non_finite = extreme_aspect = 0
    per_image: dict[int, int] = {}
    for ann in annotations:
        bbox = ann.get("bbox") or []
        if len(bbox) != 4:
            degenerate += 1
            continue
        x, y, width, height = (float(value) for value in bbox)
        if not all(math.isfinite(value) for value in bbox):
            non_finite += 1
            continue
        if width <= 0 or height <= 0:
            degenerate += 1
            continue
        if width < min_size or height < min_size:
            below_min += 1
        image = images.get(ann["image_id"])
        if image is not None:
            image_width = float(image.get("width") or 0)
            image_height = float(image.get("height") or 0)
            if (
                image_width > 0
                and image_height > 0
                and (
                    x < -EPSILON
                    or y < -EPSILON
                    or x + width > image_width + EPSILON
                    or y + height > image_height + EPSILON
                )
            ):
                out_of_bounds += 1
        if height > 0 and (width / height > 20 or width / height < 1 / 20):
            extreme_aspect += 1
        per_image[ann["image_id"]] = per_image.get(ann["image_id"], 0) + 1

    if non_finite:
        issues.append(f"{non_finite} annotation(s) contain non-finite bbox values")
    if degenerate:
        issues.append(f"{degenerate} annotation(s) have a zero/negative/invalid bbox")
    if below_min:
        issues.append(
            f"{below_min} box(es) are still below min_box_size={min_size:g}px after filtering"
        )
    if out_of_bounds:
        issues.append(f"{out_of_bounds} box(es) extend outside their image")
    if extreme_aspect:
        issues.append(f"{extreme_aspect} box(es) have an extreme aspect ratio (>20:1)")

    empty = sum(1 for image_id in images if per_image.get(image_id, 0) == 0)
    if empty:
        issues.append(f"{empty} image(s) have no annotations")

    crowded = [count for count in per_image.values() if count > 50]
    if crowded:
        issues.append(f"{len(crowded)} image(s) contain more than 50 objects")

    image_size = stats.get("image_size", {})
    if image_size.get("unique", 0) > 1:
        issues.append(
            f"{image_size['unique']} different image resolutions "
            f"({image_size['min_width']}x{image_size['min_height']}.."
            f"{image_size['max_width']}x{image_size['max_height']})"
        )

    class_counts = _class_counts(coco)
    if len(class_counts) > 1:
        most = max(class_counts.values())
        least = min(class_counts.values())
        if least and most / least >= 10:
            issues.append(
                f"class imbalance {most / least:.1f}:1 "
                f"({_top_class(class_counts, most)} has {most}, "
                f"{_top_class(class_counts, least)} has {least})"
            )
    rare = [f"{name} ({count})" for name, count in sorted(class_counts.items()) if count < 10]
    if rare:
        issues.append(f"{len(rare)} class(es) have fewer than 10 boxes: {', '.join(rare[:5])}")

    buckets = stats.get("object_size", {}).get("size_buckets", {})
    empty_buckets = [name for name in SIZE_BUCKETS if buckets.get(name, 0) == 0]
    if empty_buckets:
        issues.append(
            f"no objects in size bucket(s): {', '.join(empty_buckets)} "
            "(the filter cannot make these even)"
        )
    counts = [buckets.get(name, 0) for name in SIZE_BUCKETS if buckets.get(name, 0) > 0]
    balance_this_split = balance_sizes and split == "train"
    if not balance_this_split and len(counts) > 1:
        smallest = min(counts)
        largest = max(counts)
        if smallest and largest / smallest >= 4:
            issues.append(
                f"size buckets are {largest / smallest:.1f}:1 imbalanced and "
                "size balancing is off for this split"
            )
    total_boxes = sum(buckets.get(name, 0) for name in SIZE_BUCKETS)
    tiny = buckets.get("tiny", 0)
    if total_boxes and tiny / total_boxes >= 0.5:
        issues.append(
            f"{100.0 * tiny / total_boxes:.0f}% of boxes are tiny (<0.5% of the image); "
            "consider a smaller --image-size or larger images"
        )

    if not annotations:
        issues.append("no annotated boxes")
    return issues


def _cross_split_issues(loaded: dict[str, dict]) -> list[str]:
    issues: list[str] = []
    if "train" not in loaded or "val" not in loaded:
        return issues

    train_coco = loaded["train"]
    val_coco = loaded["val"]
    train_names = {img["file_name"] for img in train_coco.get("images", [])}
    val_names = {img["file_name"] for img in val_coco.get("images", [])}
    overlap = sorted(train_names & val_names)
    if overlap:
        issues.append(
            f"{len(overlap)} image name(s) appear in both train and val; "
            f"first: {', '.join(overlap[:3])}"
        )

    train_classes = _class_counts(train_coco)
    val_classes = _class_counts(val_coco)
    only_train = sorted(set(train_classes) - set(val_classes))
    only_val = sorted(set(val_classes) - set(train_classes))
    if only_train:
        issues.append(f"class(es) only in train (never validated): {', '.join(only_train[:5])}")
    if only_val:
        issues.append(f"class(es) only in val (never trained): {', '.join(only_val[:5])}")

    rare_val = [
        f"{name} ({val_classes[name]})" for name in sorted(val_classes) if val_classes[name] < 5
    ]
    if rare_val:
        issues.append(
            f"{len(rare_val)} class(es) have fewer than 5 boxes in val: {', '.join(rare_val[:5])}"
        )
    return issues


def _class_counts(coco: dict) -> dict[str, int]:
    names = {
        category["id"]: str(category.get("name", category["id"]))
        for category in coco.get("categories", [])
    }
    counts: dict[str, int] = {}
    for ann in coco.get("annotations", []):
        name = names.get(ann["category_id"], f"class_{ann['category_id']}")
        counts[name] = counts.get(name, 0) + 1
    return counts


def _top_class(counts: dict[str, int], target: int) -> str:
    for name, count in sorted(counts.items()):
        if count == target:
            return name
    return "?"


def _print_warnings(warnings: list[str]) -> None:
    if not warnings:
        print("\n[tests] no issues found")
        return
    print(f"\n[tests] {len(warnings)} issue(s):")
    for warning in warnings:
        print(f"  ! {warning}")


def _save_artifacts(
    report: dict, loaded: dict[str, dict], full_stats: dict[str, dict], output_dir
) -> None:
    from src.stats.stats import save_plots

    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)

    for split, stats in full_stats.items():
        save_plots(stats, output, split)

    class_counts = {split: _class_counts(coco) for split, coco in loaded.items()}
    _save_class_balance(class_counts, output)

    path = output / "tests.json"
    path.write_text(json.dumps(report, indent=2))
    print(f"  saved {path}")


def _save_class_balance(class_counts: dict[str, dict[str, int]], output_dir: Path) -> None:
    names = sorted({name for counts in class_counts.values() for name in counts})
    if not names:
        return

    splits = list(class_counts)
    width = 0.8 / len(splits)
    fig, ax = plt.subplots(figsize=(max(8, len(names) * 1.1), 5))
    for index, split in enumerate(splits):
        offsets = [position + index * width for position in range(len(names))]
        values = [class_counts[split].get(name, 0) for name in names]
        ax.bar(offsets, values, width, label=split, color=SEQUENCE[index % len(SEQUENCE)])
    positions = [position + width * (len(splits) - 1) / 2 for position in range(len(names))]
    ax.set_xticks(positions)
    ax.set_xticklabels(names, rotation=45, ha="right")
    ax.set_ylabel("annotations")
    ax.set_title("class counts per split")
    ax.legend()
    fig.tight_layout()
    path = output_dir / "stats" / "class_balance.png"
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=120)
    plt.close(fig)
    print(f"  saved {path}")


__all__ = ["run_tests", "SPLITS"]
