import argparse
import csv
import json
import os
import time
from pathlib import Path

from pycocotools.coco import COCO

from src.eval.evaluators.hf import HFModelEvaluator
from src.eval.evaluators.yolo import YOLOEvaluator
from src.eval.coco import evaluate_dataset


def run_eval(model_path, model_type="auto", resolution=640, datasets=None, conf_thres=0.001, log_file=None):
    args = argparse.Namespace(
        model_path=model_path,
        model_type=model_type,
        model_name=None,
        resolution=resolution,
        datasets=datasets or [],
        conf_thres=conf_thres,
        log_file=log_file,
    )
    return _run(args)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Evaluate model on COCO datasets")
    parser.add_argument("--model-path", required=True)
    parser.add_argument("--model-type", choices=["auto", "yolo", "hf", "rfdetr", "dfine", "ultralytics"], default="auto")
    parser.add_argument("--model-name")
    parser.add_argument("--resolution", type=int, default=640)
    parser.add_argument("--datasets", nargs="+", required=True)
    parser.add_argument("--conf-thres", type=float, default=0.001)
    parser.add_argument("--log-file")
    args = parser.parse_args(argv)
    return _run(args)


def _run(args):
    model_path = Path(args.model_path)
    model_type = args.model_type
    if model_type == "auto":
        if model_path.suffix in (".pt", ".onnx"):
            model_type = "yolo"
        elif model_path.is_dir() and (model_path / "config.json").exists():
            try:
                cfg = json.loads((model_path / "config.json").read_text())
                if "rfdetr" in str(cfg).lower():
                    model_type = "rfdetr"
                else:
                    model_type = "hf"
            except Exception:
                model_type = "hf"
        else:
            model_type = "hf"

    model_name = args.model_name or model_path.name
    log_file = args.log_file or str(Path(model_path.parent if model_path.is_file() else model_path) / f"evaluation_results_{model_name}.csv")

    if model_type in ("hf", "rfdetr", "dfine"):
        evaluator = HFModelEvaluator(model_path, args.resolution)
    elif model_type in ("yolo", "ultralytics"):
        evaluator = YOLOEvaluator(model_path, args.resolution)
    else:
        raise ValueError(f"Unknown model type: {model_type}")

    file_exists = os.path.exists(log_file)
    f = open(log_file, "a", newline="")
    fieldnames = ["timestamp", "model_name", "model_path", "model_type", "resolution", "dataset_name", "dataset_path", "num_images", "num_annotations", "map", "map_50", "map_75", "map_95", "map_small", "map_medium", "map_large", "mar_1", "mar_10", "mar_100", "mar_small", "mar_medium", "mar_large"]
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    if not file_exists:
        writer.writeheader()

    for dpath in args.datasets:
        dpath = Path(dpath)
        try:
            ann_file = None
            images_dir = None
            if (dpath / "annotations" / "instances_default.json").exists():
                ann_file = dpath / "annotations" / "instances_default.json"
                images_dir = dpath / "images"
            elif (dpath / "annotations.json").exists():
                ann_file = dpath / "annotations.json"
                images_dir = dpath / "images"
            elif (dpath / "_annotations.coco.json").exists():
                ann_file = dpath / "_annotations.coco.json"
                images_dir = dpath
            else:
                for cand in dpath.rglob("*annotations*.json"):
                    ann_file = cand
                    images_dir = cand.parent.parent / "images" if cand.parent.name == "annotations" else dpath
                    break
            if ann_file is None or not ann_file.exists():
                print(f"Skipping {dpath}: no annotations found")
                continue
            if images_dir is None or not images_dir.exists():
                images_dir = dpath
            coco = COCO(str(ann_file))
            num_images = len(coco.getImgIds())
            num_annotations = len(coco.getAnnIds())
            dataset_name = dpath.name
            results = evaluate_dataset(evaluator, coco, images_dir, args.conf_thres)
            row = {
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "model_name": model_name,
                "model_path": str(model_path),
                "model_type": model_type,
                "resolution": args.resolution,
                "dataset_name": dataset_name,
                "dataset_path": str(dpath),
                "num_images": num_images,
                "num_annotations": num_annotations,
                "map": results.get("map", 0.0),
                "map_50": results.get("map_50", 0.0),
                "map_75": results.get("map_75", 0.0),
                "map_95": results.get("map_95", 0.0),
                "map_small": results.get("map_small", 0.0),
                "map_medium": results.get("map_medium", 0.0),
                "map_large": results.get("map_large", 0.0),
                "mar_1": results.get("mar_1", 0.0),
                "mar_10": results.get("mar_10", 0.0),
                "mar_100": results.get("mar_100", 0.0),
                "mar_small": results.get("mar_small", 0.0),
                "mar_medium": results.get("mar_medium", 0.0),
                "mar_large": results.get("mar_large", 0.0),
            }
            writer.writerow(row)
            f.flush()
            print(f"\nResults for {dataset_name}: mAP@.50:.95={row['map']:.4f}")
        except Exception as e:
            print(f"ERROR evaluating {dpath}: {e}")
            continue
    f.close()
    print(f"\n{'='*60}\nResults appended to: {log_file}\n{'='*60}")
    return 0


if __name__ == "__main__":
    main()
