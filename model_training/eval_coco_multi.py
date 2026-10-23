#!/usr/bin/env python3
"""
Evaluate a detection model on multiple COCO datasets and log results to a common CSV
for easy comparison across models.

Supports:
- HuggingFace Transformers models (e.g., RF-DETR, D-FINE, Deformable-DETR, RT-DETR)
- Ultralytics YOLO models (.pt)

Logs per dataset: model name, resolution, dataset name, image count, instance count,
mAP (COCO @ .50:.95), mAP50, mAP75, mAP_small/medium/large, AR metrics.
Appends to a shared CSV file so you can compare across runs/models.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import torch
from PIL import Image
from pycocotools.coco import COCO
from tqdm import tqdm

try:
    from torchmetrics.detection.mean_ap import MeanAveragePrecision
except Exception:
    MeanAveragePrecision = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--model-path",
        type=str,
        required=True,
        help="Path to model: HF checkpoint dir, or .pt file for YOLO/Ultralytics",
    )
    parser.add_argument(
        "--model-type",
        type=str,
        choices=["auto", "hf", "rfdetr", "yolo", "ultralytics"],
        default="auto",
        help="Model type (auto tries to infer)",
    )
    parser.add_argument(
        "--model-name",
        type=str,
        default=None,
        help="Name to log in results (defaults to model-path basename)",
    )
    parser.add_argument(
        "--resolution",
        type=int,
        default=1024,
        help="Input resolution for evaluation (square). Default: 1024",
    )
    parser.add_argument(
        "--datasets",
        type=str,
        nargs="+",
        required=True,
        help="Paths to dataset folders. Each folder should contain images/ and annotations (or coco json). "
        "We look for common names: annotations.json, instances_val.json, val.json, *_annotations.coco.json",
    )
    parser.add_argument(
        "--images-subdir",
        type=str,
        default=None,
        help="Override images subdirectory name (e.g., 'images', 'val', 'train')",
    )
    parser.add_argument(
        "--ann-file",
        type=str,
        default=None,
        help="Override annotation filename (e.g., 'annotations.coco.json')",
    )
    parser.add_argument(
        "--log-file",
        type=str,
        default="evaluation_results.csv",
        help="Common CSV file to append results to (for comparing models). Default: evaluation_results.csv",
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=1,
        help="Batch size for inference (default: 1)",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda" if torch.cuda.is_available() else "cpu",
        help="Device to run on",
    )
    parser.add_argument(
        "--conf-thres",
        type=float,
        default=0.001,
        help="Confidence threshold for predictions (default: 0.001)",
    )
    parser.add_argument(
        "--iou-thres",
        type=float,
        default=0.6,
        help="IoU threshold for NMS (YOLO only, default: 0.6)",
    )
    parser.add_argument(
        "--max-det",
        type=int,
        default=300,
        help="Maximum detections per image (default: 300)",
    )
    parser.add_argument(
        "--use-torchmetrics",
        action="store_true",
        default=True,
        help="Use torchmetrics MeanAveragePrecision (COCO style). Default: true",
    )
    return parser.parse_args()


def find_coco_ann(dataset_dir: Path, ann_file: Optional[str] = None) -> Path:
    if ann_file:
        p = dataset_dir / ann_file
        if p.exists():
            return p
        # also try relative
    candidates = [
        "annotations.coco.json",
        "instances_val2017.json",
        "instances_val.json",
        "val_annotations.coco.json",
        "annotations.json",
        "val.json",
        "coco_annotations.json",
    ]
    for c in candidates:
        p = dataset_dir / c
        if p.exists():
            return p
    # search recursively
    for p in dataset_dir.rglob("*.json"):
        name = p.name.lower()
        if "coco" in name or ("instances" in name and "val" in name):
            return p
    raise FileNotFoundError(f"Could not find COCO annotation file in {dataset_dir}")


def find_images_dir(dataset_dir: Path, images_subdir: Optional[str] = None) -> Path:
    if images_subdir:
        p = dataset_dir / images_subdir
        if p.exists():
            return p
    candidates = ["images", "val", "test", "data", "JPEGImages"]
    for c in candidates:
        p = dataset_dir / c
        if p.exists() and p.is_dir():
            return p
    # maybe images are directly in dataset_dir
    # check if there are image files
    for ext in [".jpg", ".jpeg", ".png", ".JPG", ".JPEG", ".PNG"]:
        if any(dataset_dir.glob(f"*{ext}")):
            return dataset_dir
    raise FileNotFoundError(f"Could not find images directory in {dataset_dir}")


class HFModelEvaluator:
    def __init__(self, model_path: str, resolution: int, device: str):
        self.model_path = model_path
        self.resolution = resolution
        self.device = device
        self.model = None
        self.processor = None
        self.load()

    def load(self):
        try:
            from transformers import AutoImageProcessor, AutoModelForObjectDetection

            print(f"[HF] Loading model from {self.model_path}")
            self.processor = AutoImageProcessor.from_pretrained(self.model_path, local_files_only=True)
            # Try to resize processor if resolution differs
            try:
                if hasattr(self.processor, 'size'):
                    self.processor.size = {"height": self.resolution, "width": self.resolution}
            except Exception:
                pass
            self.model = AutoModelForObjectDetection.from_pretrained(self.model_path, local_files_only=True)
            self.model.to(self.device)
            self.model.eval()
            print(f"[HF] Model loaded. Classes: {getattr(self.model.config, 'id2label', {})}")
        except Exception as e:
            # Try RF-DETR specific
            try:
                from transformers import RfDetrForObjectDetection, RfDetrImageProcessor

                print(f"[RF-DETR] Loading model from {self.model_path}")
                self.processor = RfDetrImageProcessor.from_pretrained(self.model_path, local_files_only=True)
                try:
                    if hasattr(self.processor, 'size'):
                        self.processor.size = {"height": self.resolution, "width": self.resolution}
                except Exception:
                    pass
                self.model = RfDetrForObjectDetection.from_pretrained(self.model_path, local_files_only=True)
                self.model.to(self.device)
                self.model.eval()
            except Exception as e2:
                raise RuntimeError(f"Failed to load HF model: {e} / {e2}")

    def predict(self, image: Image.Image):
        inputs = self.processor(images=image, return_tensors="pt")
        inputs = {k: v.to(self.device) for k, v in inputs.items()}
        with torch.no_grad():
            outputs = self.model(**inputs)
        # Post-process
        target_sizes = torch.tensor([image.size[::-1]])  # (height, width)
        results = self.processor.post_process_object_detection(
            outputs, target_sizes=target_sizes, threshold=0.0
        )[0]
        return results


class YOLOEvaluator:
    def __init__(self, model_path: str, resolution: int, device: str, conf_thres: float, iou_thres: float):
        self.model_path = model_path
        self.resolution = resolution
        self.device = device
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.model = None
        self.load()

    def load(self):
        try:
            from ultralytics import YOLO

            print(f"[YOLO] Loading model from {self.model_path}")
            self.model = YOLO(self.model_path)
            self.model.to(self.device)
        except Exception as e:
            raise RuntimeError(f"Failed to load YOLO model: {e}")

    def predict(self, image: Image.Image):
        results = self.model.predict(
            image,
            imgsz=self.resolution,
            conf=self.conf_thres,
            iou=self.iou_thres,
            verbose=False,
            device=self.device,
        )[0]
        # Convert to standard format
        boxes = []
        scores = []
        labels = []
        if results.boxes is not None:
            for box in results.boxes:
                xyxy = box.xyxy.cpu().numpy()[0]
                boxes.append(xyxy)
                scores.append(float(box.conf.cpu().numpy()[0]))
                labels.append(int(box.cls.cpu().numpy()[0]))
        return {"boxes": np.array(boxes) if boxes else np.array([]), 
                "scores": np.array(scores) if scores else np.array([]),
                "labels": np.array(labels) if labels else np.array([])}


def evaluate_dataset(evaluator, coco: COCO, images_dir: Path, conf_thres: float) -> Dict:
    if MeanAveragePrecision is None:
        raise RuntimeError("torchmetrics is required for evaluation")
    metric = MeanAveragePrecision(box_format="xyxy", iou_type="bbox")
    # Also compute mAP@0.95 separately if possible
    try:
        metric95 = MeanAveragePrecision(box_format="xyxy", iou_type="bbox", iou_thresholds=[0.95])
    except Exception:
        metric95 = None
    preds = []
    targets = []
    preds95 = []
    targets95 = []
    
    img_ids = coco.getImgIds()
    cat_ids = coco.getCatIds()
    # Build category mapping
    cats = coco.loadCats(cat_ids)
    cat_id_to_idx = {cat['id']: i for i, cat in enumerate(sorted(cats, key=lambda x: x['id']))}
    
    missing_images = 0
    for img_id in tqdm(img_ids, desc="Evaluating"):
        img_info = coco.loadImgs(img_id)[0]
        img_path = images_dir / img_info["file_name"]
        if not img_path.exists():
            # try without subdir structure
            img_path = images_dir / Path(img_info["file_name"]).name
        if not img_path.exists():
            # try in images subfolder
            for sub in ["images", "val", "train", "test"]:
                p = images_dir / sub / Path(img_info["file_name"]).name
                if p.exists():
                    img_path = p
                    break
        if not img_path.exists():
            missing_images += 1
            continue
        try:
            image = Image.open(img_path).convert("RGB")
        except Exception:
            missing_images += 1
            continue
        
        # Get predictions
        if isinstance(evaluator, HFModelEvaluator):
            results = evaluator.predict(image)
            pred_boxes = results["boxes"].cpu().numpy() if len(results["boxes"]) > 0 else np.array([])
            pred_scores = results["scores"].cpu().numpy() if len(results["scores"]) > 0 else np.array([])
            pred_labels = results["labels"].cpu().numpy() if len(results["labels"]) > 0 else np.array([])
        elif isinstance(evaluator, YOLOEvaluator):
            res = evaluator.predict(image)
            pred_boxes = res["boxes"]
            pred_scores = res["scores"]
            pred_labels = res["labels"]
        else:
            continue
        
        # Get targets
        ann_ids = coco.getAnnIds(imgIds=img_id)
        anns = coco.loadAnns(ann_ids)
        target_boxes = []
        target_labels = []
        for ann in anns:
            if ann.get("iscrowd", 0):
                continue
            x, y, w, h = ann["bbox"]
            target_boxes.append([x, y, x + w, y + h])
            cat_id = ann["category_id"]
            # Map to contiguous if needed
            if cat_id in cat_id_to_idx:
                target_labels.append(cat_id_to_idx[cat_id])
            else:
                target_labels.append(0)
        target_boxes = np.array(target_boxes) if target_boxes else np.array([])
        target_labels = np.array(target_labels) if target_labels else np.array([])
        
        # Format for torchmetrics
        if len(pred_boxes) > 0 and pred_scores is not None and len(pred_scores) > 0:
            preds.append({
                "boxes": torch.tensor(pred_boxes, dtype=torch.float32),
                "scores": torch.tensor(pred_scores, dtype=torch.float32),
                "labels": torch.tensor(pred_labels, dtype=torch.int64),
            })
            preds95.append({
                "boxes": torch.tensor(pred_boxes, dtype=torch.float32),
                "scores": torch.tensor(pred_scores, dtype=torch.float32),
                "labels": torch.tensor(pred_labels, dtype=torch.int64),
            })
        else:
            preds.append({
                "boxes": torch.tensor([], dtype=torch.float32).reshape(0, 4),
                "scores": torch.tensor([], dtype=torch.float32),
                "labels": torch.tensor([], dtype=torch.int64),
            })
            preds95.append({
                "boxes": torch.tensor([], dtype=torch.float32).reshape(0, 4),
                "scores": torch.tensor([], dtype=torch.float32),
                "labels": torch.tensor([], dtype=torch.int64),
            })
        
        if len(target_boxes) > 0:
            targets.append({
                "boxes": torch.tensor(target_boxes, dtype=torch.float32),
                "labels": torch.tensor(target_labels, dtype=torch.int64),
            })
            targets95.append({
                "boxes": torch.tensor(target_boxes, dtype=torch.float32),
                "labels": torch.tensor(target_labels, dtype=torch.int64),
            })
        else:
            targets.append({
                "boxes": torch.tensor([], dtype=torch.float32).reshape(0, 4),
                "labels": torch.tensor([], dtype=torch.int64),
            })
            targets95.append({
                "boxes": torch.tensor([], dtype=torch.float32).reshape(0, 4),
                "labels": torch.tensor([], dtype=torch.int64),
            })
    
    if missing_images > 0:
        print(f"Warning: {missing_images} images not found/skipped")
    
    metric.update(preds, targets)
    results = metric.compute()
    
    # Flatten results
    out = {}
    for k, v in results.items():
        if isinstance(v, torch.Tensor):
            if v.numel() == 1:
                out[k] = float(v.item())
            else:
                pass  # skip arrays
        else:
            out[k] = v
    
    if metric95 is not None:
        try:
            metric95.update(preds95, targets95)
            results95 = metric95.compute()
            if 'map' in results95 and isinstance(results95['map'], torch.Tensor) and results95['map'].numel() == 1:
                out['map_95'] = float(results95['map'].item())
        except Exception:
            pass
    
    return out


def main():
    args = parse_args()
    model_path = Path(args.model_path).expanduser().resolve()
    model_name = args.model_name or model_path.name
    
    # Determine model type
    model_type = args.model_type
    if model_type == "auto":
        if model_path.suffix == ".pt":
            model_type = "yolo"
        elif model_path.is_dir():
            # Check for config
            if (model_path / "config.json").exists():
                try:
                    with open(model_path / "config.json") as f:
                        cfg = json.load(f)
                    arch = cfg.get("architectures", [""])[0].lower()
                    if "rfdetr" in arch:
                        model_type = "rfdetr"
                    else:
                        model_type = "hf"
                except:
                    model_type = "hf"
            else:
                model_type = "hf"
        else:
            model_type = "hf"
    
    print(f"Model: {model_name} ({model_type}) from {model_path}, resolution={args.resolution}, device={args.device}")
    
    # Load evaluator
    if model_type in ["yolo", "ultralytics"]:
        evaluator = YOLOEvaluator(str(model_path), args.resolution, args.device, args.conf_thres, args.iou_thres)
    else:
        evaluator = HFModelEvaluator(str(model_path), args.resolution, args.device)
    
    # Prepare log file
    log_file = Path(args.log_file)
    log_file.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "timestamp",
        "model_name",
        "model_path",
        "model_type",
        "resolution",
        "dataset_name",
        "dataset_path",
        "num_images",
        "num_annotations",
        "map",
        "map_50",
        "map_75",
        "map_95",
        "map_small",
        "map_medium",
        "map_large",
        "mar_1",
        "mar_10",
        "mar_100",
        "mar_small",
        "mar_medium",
        "mar_large",
    ]
    
    write_header = not log_file.exists()
    f = open(log_file, "a", newline="")
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    if write_header:
        writer.writeheader()
    
    # Evaluate each dataset
    for dataset_path in args.datasets:
        dpath = Path(dataset_path).expanduser().resolve()
        print(f"\n{'='*60}")
        print(f"Evaluating on dataset: {dpath}")
        print(f"{'='*60}")
        
        try:
            ann_file = find_coco_ann(dpath, args.ann_file)
            images_dir = find_images_dir(dpath, args.images_subdir)
            print(f"Annotations: {ann_file}")
            print(f"Images dir: {images_dir}")
            
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
            
            print(f"\nResults for {dataset_name}:")
            print(f"  mAP@.50:.95: {row['map']:.4f}")
            print(f"  mAP@.50:     {row['map_50']:.4f}")
            print(f"  mAP@.75:     {row['map_75']:.4f}")
            print(f"  mAP@.95:     {row['map_95']:.4f}")
            print(f"  mAP_small:   {row['map_small']:.4f}")
            print(f"  mAP_medium:  {row['map_medium']:.4f}")
            print(f"  mAP_large:   {row['map_large']:.4f}")
            
        except Exception as e:
            print(f"ERROR evaluating {dpath}: {e}")
            import traceback
            traceback.print_exc()
            continue
    
    f.close()
    print(f"\n{'='*60}")
    print(f"Results appended to: {log_file}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
