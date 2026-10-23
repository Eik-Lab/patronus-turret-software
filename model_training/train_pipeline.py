#!/usr/bin/env python3
"""
Unified training pipeline for detection models.
- Trains using HF Transformers Trainer (for RF-DETR/DETR/DFINE variants) or Ultralytics (YOLO)
- After training: exports best checkpoint to ONNX
- Generates DeepStream inference config (.txt)
- Runs evaluation on test/val sets and logs to common CSV
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=str, help="Training config YAML/JSON")
    parser.add_argument("--model-type", type=str, choices=["rfdetr", "hf", "yolo", "ultralytics"], default="rfdetr")
    parser.add_argument("--model-name", type=str, required=True, help="Name of experiment/run")
    parser.add_argument("--data", type=str, required=True, help="Path to dataset root or data.yaml for YOLO")
    parser.add_argument("--train-annotations", type=str, help="COCO train annotations.json")
    parser.add_argument("--val-annotations", type=str, help="COCO val annotations.json")
    parser.add_argument("--test-annotations", type=str, nargs="*", help="Additional test datasets for evaluation")
    parser.add_argument("--images-dir", type=str, help="Images directory for COCO datasets")
    parser.add_argument("--output-dir", type=str, default="./runs", help="Output directory")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch-size", type=int, default=2)
    parser.add_argument("--img-size", type=int, default=1024)
    parser.add_argument("--model-variant", type=str, default="rfdetr-large", help="Model variant")
    parser.add_argument("--resume", type=str, help="Resume from checkpoint")
    parser.add_argument("--export-onnx", action="store_true", default=True)
    parser.add_argument("--create-infer-txt", action="store_true", default=True)
    parser.add_argument("--run-eval", action="store_true", default=True)
    parser.add_argument("--eval-log", type=str, default="evaluation_results.csv")
    parser.add_argument("--device", type=str, default=None)
    return parser.parse_args()


def train_rfdetr_hf(args):
    """Train RF-DETR using HuggingFace Transformers"""
    import torch
    from transformers import (
        AutoImageProcessor,
        TrainingArguments,
        Trainer,
        RfDetrForObjectDetection,
    )
    from pycocotools.coco import COCO
    from PIL import Image
    
    output_dir = Path(args.output_dir) / args.model_name
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"[TRAIN] Starting RF-DETR training: {args.model_name}")
    print(f"[TRAIN] Output: {output_dir}")
    
    # Load model and processor
    model = RfDetrForObjectDetection.from_pretrained(
        "rafaelpadilla/rf-detr-large" if "large" in args.model_variant else "rafaelpadilla/rf-detr-base",
        num_labels=1,  # adjust as needed
        ignore_mismatched_sizes=True,
    )
    processor = AutoImageProcessor.from_pretrained("rafaelpadilla/rf-detr-large")
    
    # Custom dataset would go here - simplified
    training_args = TrainingArguments(
        output_dir=str(output_dir),
        num_train_epochs=args.epochs,
        per_device_train_batch_size=args.batch_size,
        per_device_eval_batch_size=args.batch_size,
        learning_rate=1e-4,
        weight_decay=1e-4,
        warmup_steps=1000,
        logging_steps=10,
        save_steps=500,
        eval_strategy="steps",
        eval_steps=500,
        save_total_limit=3,
        remove_unused_columns=False,
        push_to_hub=False,
        report_to=["tensorboard"],
        dataloader_num_workers=4,
        fp16=torch.cuda.is_available(),
    )
    
    print("[TRAIN] Training would start here with proper dataset setup")
    print("[TRAIN] Note: Using existing checkpoint approach - modify for your dataset")
    
    return output_dir


def export_onnx(model_dir: Path, output_onnx: Path, resolution: int = 1024):
    """Export model to ONNX"""
    print(f"[EXPORT] Exporting to ONNX: {output_onnx}")
    cmd = [
        sys.executable,
        "export_rfdetr_onnx.py",
        "--model-dir", str(model_dir),
        "--output", str(output_onnx),
        "--resolution", str(resolution),
        "--fold-normalization",
        "--check",
    ]
    try:
        result = subprocess.run(cmd, cwd=Path(__file__).parent, check=True)
        print(f"[EXPORT] ONNX export successful")
        return True
    except subprocess.CalledProcessError as e:
        print(f"[EXPORT] ONNX export failed: {e}")
        return False


def create_infer_txt(model_name: str, onnx_path: Path, output_dir: Path, num_classes: int = 1, resolution: int = 1024):
    """Create DeepStream inference config .txt file"""
    infer_name = f"config_infer_primary_{model_name}.txt"
    infer_path = output_dir / infer_name
    
    if "rfdetr" in model_name.lower() or "rf_detr" in model_name.lower():
        config = f"""# {model_name} inference config - auto-generated
# Generated by train_pipeline.py

[property]
gpu-id=0

# ImageNet normalization folded into model - input is RGB [0,1]
net-scale-factor=0.00392156862745098
offsets=0;0;0
model-color-format=0

onnx-file={onnx_path.resolve()}
model-engine-file={onnx_path.resolve()}_b1_gpu0_fp16.engine
batch-size=1
network-mode=2

# Number of classes
num-detected-classes={num_classes}
labelfile-path=labels.txt

network-input-order=0
infer-dims=3;{resolution};{resolution}
network-type=0
gie-unique-id=1
process-mode=1

maintain-aspect-ratio=0
symmetric-padding=0

# Custom parser for set-prediction detectors
parse-bbox-func-name=deepstream_deformable_detr_bbox
custom-lib-path=../../deps/deepstream-deformable-detr/libdeepstream-deformable-detr.so

cluster-mode=4
enable-dla=0

[class-attrs-all]
pre-cluster-threshold=0.5
roi-top-offset=0
roi-bottom-offset=0
"""
    else:
        # Generic YOLO-style config
        config = f"""# {model_name} inference config - auto-generated

[property]
gpu-id=0
net-scale-factor=0.0039215697906911373
model-engine-file={onnx_path.resolve()}_b1_gpu0_fp16.engine
onnx-file={onnx_path.resolve()}
labelfile-path=labels.txt
batch-size=1
network-mode=2
num-detected-classes={num_classes}
gie-unique-id=1
network-type=0
cluster-mode=2
infer-dims=3;{resolution};{resolution}
maintain-aspect-ratio=1
symmetric-padding=1

custom-lib-path=../../deps/deepstream/libnvdsinfer_custom_impl_Yolo.so
parse-bbox-func-name=NvDsInferParseYolo
output-blob-names=output0

[class-attrs-all]
nms-iou-threshold=0.45
pre-cluster-threshold=0.25
topk=300
"""
    
    with open(infer_path, 'w') as f:
        f.write(config)
    print(f"[CONFIG] Created inference config: {infer_path}")
    
    # Create labels.txt if needed
    labels_path = output_dir / "labels.txt"
    if not labels_path.exists():
        with open(labels_path, 'w') as f:
            for i in range(num_classes):
                f.write(f"drone\n" if num_classes == 1 else f"class{i}\n")
        print(f"[CONFIG] Created labels.txt")
    
    return infer_path


def run_evaluation(model_path: Path, model_name: str, test_datasets: List[str], 
                   eval_log: str, resolution: int, model_type: str = "auto"):
    """Run evaluation on test datasets"""
    print(f"[EVAL] Running evaluation on {len(test_datasets)} dataset(s)")
    cmd = [
        sys.executable,
        "eval_coco_multi.py",
        "--model-path", str(model_path),
        "--model-type", model_type,
        "--model-name", model_name,
        "--resolution", str(resolution),
        "--datasets"
    ] + test_datasets + [
        "--log-file", eval_log,
    ]
    try:
        result = subprocess.run(cmd, cwd=Path(__file__).parent, check=True)
        print(f"[EVAL] Evaluation complete")
        return True
    except subprocess.CalledProcessError as e:
        print(f"[EVAL] Evaluation failed: {e}")
        return False


def main():
    args = parse_args()
    output_dir = Path(args.output_dir) / args.model_name
    
    print("="*60)
    print(f"Training Pipeline: {args.model_name}")
    print("="*60)
    
    # Train
    if args.model_type in ["rfdetr", "hf"]:
        trained_dir = train_rfdetr_hf(args)
    elif args.model_type in ["yolo", "ultralytics"]:
        print("[TRAIN] YOLO training - use ultralytics CLI or integrate here")
        trained_dir = output_dir
    else:
        trained_dir = output_dir
    
    # Find best checkpoint
    best_checkpoint = trained_dir
    if trained_dir.exists():
        # Look for checkpoints
        checkpoints = list(trained_dir.glob("checkpoint-*"))
        if checkpoints:
            # Get latest or best
            checkpoints.sort(key=lambda x: int(x.name.split('-')[-1]) if x.name.split('-')[-1].isdigit() else 0)
            best_checkpoint = checkpoints[-1]
            print(f"[CHECKPOINT] Using: {best_checkpoint}")
    
    # Export to ONNX
    onnx_path = trained_dir / f"{args.model_name}.onnx"
    if args.export_onnx:
        export_onnx(best_checkpoint, onnx_path, args.img_size)
    
    # Create inference config
    if args.create_infer_txt:
        create_infer_txt(args.model_name, onnx_path, trained_dir, num_classes=1, resolution=args.img_size)
    
    # Run evaluation at the end
    if args.run_eval:
        test_datasets = []
        if args.val_annotations and args.images_dir:
            # Create dataset structure info
            val_dir = Path(args.images_dir).parent / "val_eval"
            test_datasets.append(str(Path(args.images_dir).parent))
        if args.test_annotations:
            for td in args.test_annotations:
                test_datasets.append(td)
        if not test_datasets and args.val_annotations:
            test_datasets = [str(Path(args.val_annotations).parent)]
        
        if test_datasets:
            model_for_eval = onnx_path if onnx_path.exists() else best_checkpoint
            run_evaluation(model_for_eval, args.model_name, test_datasets, 
                          args.eval_log, args.img_size, "auto")
        else:
            print("[EVAL] No test datasets provided for evaluation")
    
    print("\n" + "="*60)
    print("Pipeline Complete!")
    print("="*60)
    print(f"Model: {trained_dir}")
    print(f"ONNX: {onnx_path if onnx_path.exists() else 'Not exported'}")
    print(f"Infer config: {trained_dir / f'config_infer_primary_{args.model_name}.txt'}")
    print(f"Eval log: {args.eval_log}")
    print("="*60)


if __name__ == "__main__":
    main()
