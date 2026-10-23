import os
import shutil
from functools import partial
from pathlib import Path

import onnx
import torch
from onnxslim import slim
from transformers import (
    AutoModelForObjectDetection,
    EarlyStoppingCallback,
    Trainer,
    TrainingArguments,
)

from src.data.dataset import build_datasets
from src.export import export_fp16
from src.metrics import compute_metrics
from src.samples import SampleGridCallback
from src.utils.filter_dataset import filter_coco_dataset


class Model:
    name = "model"
    coco_dataset = True

    def filter(self, config, output_dir):
        pass

    def train(self, config, train_stats):
        raise NotImplementedError


def _export_onnx(model, output_dir, image_size, model_name=None, config=None):
    """Export the detection model to ONNX with onnxslim optimisation."""
    model_name = model_name or ""
    model_name_lower = str(model_name).lower()
    config = config or {}
    model_cfg = config.get("model", {}) if isinstance(config, dict) else {}
    training_cfg = config.get("training", {}) if isinstance(config, dict) else {}

    # Use RF-DETR specific export if it's an RF-DETR model
    if "rfdetr" in model_name_lower or "rf-detr" in model_name_lower or model_cfg.get("type") == "rfdetr":
        try:
            import subprocess
            import sys
            # Try to use the dedicated export script
            export_script = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "export_rfdetr_onnx.py")
            if not os.path.exists(export_script):
                export_script = os.path.join(os.getcwd(), "export_rfdetr_onnx.py")
            if os.path.exists(export_script):
                onnx_path = os.path.join(output_dir, f"{os.path.basename(output_dir) or 'model'}.onnx")
                cmd = [
                    sys.executable,
                    export_script,
                    "--model-dir", str(output_dir),
                    "--output", str(onnx_path),
                    "--resolution", str(image_size),
                    "--fold-normalization",
                    "--check",
                ]
                print(f"  Exporting with RF-DETR specific exporter: {' '.join(cmd)}")
                result = subprocess.run(cmd, cwd=os.path.dirname(export_script), check=True)
                # Validate ONNX
                try:
                    import onnx as onnx_lib
                    onnx_model = onnx_lib.load(str(onnx_path))
                    onnx_lib.checker.check_model(onnx_model)
                    print(f"  ONNX validation passed")
                except Exception as ve:
                    print(f"  Warning: ONNX validation failed: {ve}")
                size_mb = os.path.getsize(onnx_path) / (1024 * 1024) if os.path.exists(onnx_path) else 0
                print(f"  ONNX exported to {onnx_path} ({size_mb:.1f} MB)")
                return onnx_path
        except Exception as e:
            print(f"  Warning: RF-DETR specific export failed, falling back to generic export: {e}")

    class _Wrapper(torch.nn.Module):
        def __init__(self, inner):
            super().__init__()
            self.inner = inner

        def forward(self, pixel_values, pixel_mask):
            out = self.inner(pixel_values=pixel_values, pixel_mask=pixel_mask)
            # Try to return in format compatible with DeepStream
            if hasattr(out, 'logits') and hasattr(out, 'pred_boxes'):
                return out.logits, out.pred_boxes
            elif isinstance(out, dict):
                logits = out.get('logits') or out.get('pred_logits')
                boxes = out.get('pred_boxes') or out.get('boxes')
                return logits, boxes
            return out

    wrapper = _Wrapper(model).eval()
    device = next(model.parameters()).device
    dummy_pixels = torch.randn(1, 3, image_size, image_size, device=device)
    dummy_mask = torch.ones(1, image_size, image_size, dtype=torch.long, device=device)

    onnx_path = os.path.join(output_dir, "model.onnx")
    try:
        torch.onnx.export(
            wrapper,
            (dummy_pixels, dummy_mask),
            onnx_path,
            input_names=["pixel_values", "pixel_mask"],
            output_names=["pred_logits", "pred_boxes"],
            dynamic_axes={
                "pixel_values": {0: "batch"},
                "pixel_mask": {0: "batch"},
                "pred_logits": {0: "batch"},
                "pred_boxes": {0: "batch"},
            },
            opset_version=17,
            dynamo=False,
        )
    except Exception:
        # Fallback to old format
        torch.onnx.export(
            wrapper,
            (dummy_pixels, dummy_mask),
            onnx_path,
            input_names=["pixel_values", "pixel_mask"],
            output_names=["logits", "pred_boxes"],
            dynamic_axes={
                "pixel_values": {0: "batch"},
                "pixel_mask": {0: "batch"},
                "logits": {0: "batch"},
                "pred_boxes": {0: "batch"},
            },
            opset_version=17,
        )

    try:
        slimmed = slim(onnx_path)
        onnx.save(slimmed, onnx_path)
    except Exception as e:
        print(f"  Warning: onnxslim failed: {e}")

    size_mb = os.path.getsize(onnx_path) / (1024 * 1024)
    print(f"  ONNX exported to {onnx_path} ({size_mb:.1f} MB)")
    return onnx_path


def _remove_checkpoints(output_dir):
    """Delete all checkpoint-* directories, keeping only the final model."""
    for item in Path(output_dir).iterdir():
        if item.is_dir() and item.name.startswith("checkpoint-"):
            shutil.rmtree(item)


def _print_summary(metrics, id2label):
    print("\nEvaluation results", "-"*50)
    for key in ("map", "map_50", "map_75", "mar_100"):
        if key in metrics:
            print(f"  {key}: {metrics[key]:.4f}")
    for size in ("small", "medium", "large"):
        if f"map_{size}" in metrics:
            print(f"  {size}: mAP {metrics[f'map_{size}']:.4f}")
    for name in sorted(n for n in id2label.values() if f"map_{n}" in metrics):
        print(f"  {name}: mAP {metrics[f'map_{name}']:.4f}  mAR {metrics.get(f'mar_100_{name}', 0):.4f}")


class HFDetModel(Model):
    name = "hf"

    def filter(self, config, output_dir):
        filter_coco_dataset(config, output_dir)

    def train(self, config, train_stats):
        model_cfg = config["model"]
        train_cfg = config["training"]
        output_dir = train_cfg["output_dir"]
        image_size = train_cfg["image_size"]
        Path(train_cfg["tensorboard_dir"]).mkdir(parents=True, exist_ok=True)

        image_processor, train_dataset, val_dataset = build_datasets(config)

        categories = train_stats["class_distribution"]["categories"]
        id2label = {i: name for i, (_, name) in enumerate(sorted(categories.items()))}
        label2id = {name: i for i, name in id2label.items()}

        model = AutoModelForObjectDetection.from_pretrained(
            model_cfg["name"],
            id2label=id2label,
            label2id=label2id,
            ignore_mismatched_sizes=True,
        )

        def collate_fn(batch):
            pixel_values = torch.stack([item["pixel_values"] for item in batch])
            pixel_mask = torch.ones(
                pixel_values.shape[0],
                pixel_values.shape[2],
                pixel_values.shape[3],
                dtype=torch.long,
                device=pixel_values.device,
            )
            return {
                "pixel_values": pixel_values,
                "pixel_mask": pixel_mask,
                "labels": [item["labels"] for item in batch],
            }

        use_bf16 = torch.cuda.is_available() and torch.cuda.is_bf16_supported()
        gc_requested = train_cfg.get("gradient_checkpointing", False)
        gc_supported = getattr(model, "supports_gradient_checkpointing", False)

        os.environ["TENSORBOARD_LOGGING_DIR"] = train_cfg["tensorboard_dir"]

        args = TrainingArguments(
            output_dir=output_dir,
            report_to=["tensorboard"],
            num_train_epochs=train_cfg["epochs"],
            per_device_train_batch_size=train_cfg["batch_size"],
            per_device_eval_batch_size=train_cfg["batch_size"],
            learning_rate=float(train_cfg["learning_rate"]),
            weight_decay=float(train_cfg["weight_decay"]),
            gradient_accumulation_steps=train_cfg.get(
                "gradient_accumulation_steps", 1
            ),
            warmup_ratio=train_cfg.get("warmup_ratio", 0.0),
            lr_scheduler_type=train_cfg.get("lr_scheduler_type", "linear"),
            max_grad_norm=train_cfg.get("max_grad_norm", 1.0),
            optim=train_cfg.get("optimizer", "adamw_torch"),
            dataloader_num_workers=train_cfg["num_workers"],
            dataloader_persistent_workers=train_cfg["num_workers"] > 0,
            logging_steps=train_cfg["logging_steps"],
            save_total_limit=train_cfg.get("save_total_limit", 1),
            seed=train_cfg["seed"],
            data_seed=train_cfg["seed"],
            full_determinism=train_cfg.get("deterministic", False),
            eval_strategy="epoch",
            save_strategy="epoch",
            eval_do_concat_batches=False,
            bf16=use_bf16,
            fp16=torch.cuda.is_available() and not use_bf16,
            gradient_checkpointing=gc_requested and gc_supported,
            gradient_checkpointing_kwargs={"use_reentrant": False},
            load_best_model_at_end=False,
            metric_for_best_model="map",
            greater_is_better=True,
            remove_unused_columns=False,
        )

        trainer = Trainer(
            model=model,
            args=args,
            train_dataset=train_dataset,
            eval_dataset=val_dataset,
            data_collator=collate_fn,
            processing_class=image_processor,
            compute_metrics=partial(
                compute_metrics,
                image_processor=image_processor,
                id2label=id2label,
            ),
        )

        trainer.add_callback(
            SampleGridCallback(
                image_processor=image_processor,
                val_dataset=val_dataset,
                output_dir=output_dir,
                logging_dir=train_cfg["tensorboard_dir"],
                id2label=id2label,
            )
        )
        early_stopping_patience = train_cfg.get("early_stopping_patience", 0)
        if early_stopping_patience > 0:
            trainer.add_callback(
                EarlyStoppingCallback(early_stopping_patience=early_stopping_patience)
            )

        print(f"\nTraining {model_cfg['name']} ({self.name})", "-"*50)
        print(f"  TensorBoard: tensorboard --logdir {train_cfg['tensorboard_dir']}")
        trainer.train()

        # Reload the best checkpoint — from_pretrained applies the weight-layout
        # conversion that Trainer._load_best_model skips in transformers v5.
        best_ckpt = trainer.state.best_model_checkpoint
        if best_ckpt and os.path.isfile(
            os.path.join(best_ckpt, "model.safetensors")
        ):
            best_model = AutoModelForObjectDetection.from_pretrained(best_ckpt)
            trainer.accelerator.unwrap_model(trainer.model).load_state_dict(
                best_model.state_dict()
            )
            del best_model
            print(f"  best checkpoint: {best_ckpt}")

        trainer.save_model(output_dir)
        image_processor.save_pretrained(output_dir)

        onnx_path = _export_onnx(
            trainer.accelerator.unwrap_model(trainer.model), 
            output_dir, 
            image_size,
            model_name=model_cfg.get("name", ""),
            config=config
        )
        if onnx_path and os.path.exists(onnx_path):
            export_fp16(onnx_path)
            # Also create fp16 version with standard name
            fp16_path = os.path.splitext(onnx_path)[0] + "_fp16.onnx"
            if os.path.exists(fp16_path):
                pass
        _remove_checkpoints(output_dir)
        print(f"  model saved to {output_dir}")

        # Generate labels.txt and inference config
        try:
            _generate_inference_artifacts(output_dir, id2label, image_size, model_cfg.get("name", "model"), config)
        except Exception as e:
            print(f"  Warning: Failed to generate inference artifacts: {e}")

        # Run evaluation on test/val splits
        try:
            _run_post_training_eval(output_dir, config, model_cfg.get("name", "model"), image_size)
        except Exception as e:
            print(f"  Warning: Post-training evaluation failed: {e}")

        metrics = trainer.evaluate()
        metrics = {key.removeprefix("eval_"): value for key, value in metrics.items()}
        _print_summary(metrics, id2label)

        # Save metrics to results.json
        try:
            results_path = os.path.join(output_dir, "results.json")
            with open(results_path, "w") as f:
                import json
                json.dump(metrics, f, indent=2)
        except Exception as e:
            print(f"  Warning: Failed to save results.json: {e}")

        return metrics


def _generate_inference_artifacts(output_dir, id2label, image_size, model_name, config):
    """Generate labels.txt and DeepStream inference config files."""
    # Generate labels.txt
    labels_path = os.path.join(output_dir, "labels.txt")
    # Sort by label id
    sorted_labels = []
    for k in sorted(id2label.keys()):
        sorted_labels.append(id2label[k])
    with open(labels_path, 'w') as f:
        for label in sorted_labels:
            f.write(f"{label}\n")
    print(f"  Generated {labels_path} with {len(sorted_labels)} classes")
    
    # Copy labels.txt to config/deepstream for reference
    try:
        repo_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        ds_config_dir = os.path.join(repo_root, "config", "deepstream")
        if os.path.exists(ds_config_dir):
            import shutil
            shutil.copy(labels_path, os.path.join(ds_config_dir, f"labels_{os.path.basename(output_dir)}.txt"))
    except Exception:
        pass
    
    # Find ONNX model
    onnx_path = None
    for name in os.listdir(output_dir):
        if name.endswith('.onnx') and not name.endswith('_fp16.onnx'):
            onnx_path = os.path.join(output_dir, name)
            break
    if not onnx_path:
        onnx_path = os.path.join(output_dir, "model.onnx")
    
    num_classes = len(sorted_labels)
    model_name_clean = os.path.basename(output_dir)
    infer_name = f"config_infer_primary_{model_name_clean}.txt"
    infer_path = os.path.join(output_dir, infer_name)
    
    model_name_lower = str(model_name).lower()
    if "rfdetr" in model_name_lower or "rf-detr" in model_name_lower or config.get("model", {}).get("type") == "rfdetr":
        parser_lib = os.path.abspath(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), '..', 'deps', 'deepstream-deformable-detr', 'libdeepstream-deformable-detr.so'))
        config_content = f"""# {model_name_clean} inference config - auto-generated
# Generated by training pipeline

[property]
gpu-id=0

# ImageNet normalization folded into model - input is RGB [0,1]
net-scale-factor=0.00392156862745098
offsets=0;0;0
model-color-format=0

onnx-file={os.path.abspath(onnx_path)}
model-engine-file={os.path.abspath(onnx_path)}_b1_gpu0_fp16.engine
batch-size=1
network-mode=2

# Number of classes
num-detected-classes={num_classes}
labelfile-path={os.path.abspath(labels_path)}

network-input-order=0
infer-dims=3;{image_size};{image_size}
network-type=0
gie-unique-id=1
process-mode=1

maintain-aspect-ratio=0
symmetric-padding=0

# Custom parser for set-prediction detectors
parse-bbox-func-name=deepstream_deformable_detr_bbox
custom-lib-path={parser_lib}

cluster-mode=4
enable-dla=0

[class-attrs-all]
pre-cluster-threshold=0.5
roi-top-offset=0
roi-bottom-offset=0
"""
    else:
        parser_lib = os.path.abspath(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), '..', 'deps', 'deepstream', 'libnvdsinfer_custom_impl_Yolo.so'))
        config_content = f"""# {model_name_clean} inference config - auto-generated

[property]
gpu-id=0
net-scale-factor=0.0039215697906911373
model-engine-file={os.path.abspath(onnx_path)}_b1_gpu0_fp16.engine
onnx-file={os.path.abspath(onnx_path)}
labelfile-path={os.path.abspath(labels_path)}
batch-size=1
network-mode=2
num-detected-classes={num_classes}
gie-unique-id=1
network-type=0
cluster-mode=2
infer-dims=3;{image_size};{image_size}
maintain-aspect-ratio=1
symmetric-padding=1

custom-lib-path={parser_lib}
parse-bbox-func-name=NvDsInferParseYolo
output-blob-names=output0

[class-attrs-all]
nms-iou-threshold=0.45
pre-cluster-threshold=0.25
topk=300
"""
    
    with open(infer_path, 'w') as f:
        f.write(config_content)
    print(f"  Generated {infer_path}")


def _run_post_training_eval(output_dir, config, model_name, image_size):
    """Run evaluation on test/val datasets after training."""
    try:
        import subprocess
        import sys
    except:
        return
    
    ds_cfg = config.get("dataset", {}) if isinstance(config, dict) else {}
    test_datasets = []
    
    # Look for test/val datasets
    root = ds_cfg.get("root", "")
    if root and os.path.exists(root):
        # Check common test dataset locations
        for candidate in ["test", "val", "val_original", "test_original", "validation"]:
            cand_path = os.path.join(root, candidate)
            if os.path.exists(cand_path):
                test_datasets.append(cand_path)
        # Also check if there are multiple dataset folders
        parent = os.path.dirname(root)
        if parent and os.path.exists(parent):
            for item in os.listdir(parent):
                item_path = os.path.join(parent, item)
                if os.path.isdir(item_path) and item not in [os.path.basename(root), "runs", "models"]:
                    # Check if it looks like a dataset
                    if any(os.path.exists(os.path.join(item_path, f)) for f in ["annotations.json", "val_annotations.coco.json", "instances_val.json", "annotations.coco.json"]):
                        test_datasets.append(item_path)
    
    # Also check for test_annotations in config
    if ds_cfg.get("test_annotations"):
        test_ann = ds_cfg["test_annotations"]
        if isinstance(test_ann, str):
            if os.path.isabs(test_ann):
                test_datasets.append(os.path.dirname(test_ann))
            elif root:
                test_datasets.append(os.path.join(root, os.path.dirname(test_ann)))
        elif isinstance(test_ann, list):
            for ta in test_ann:
                if os.path.isabs(ta):
                    test_datasets.append(os.path.dirname(ta))
                elif root:
                    test_datasets.append(os.path.join(root, os.path.dirname(ta)))
    
    # Remove duplicates while preserving order
    seen = set()
    unique_datasets = []
    for td in test_datasets:
        td_abs = os.path.abspath(td)
        if td_abs not in seen:
            seen.add(td_abs)
            unique_datasets.append(td)
    
    if not unique_datasets:
        print("  No test datasets found for post-training evaluation")
        return
    
    # Find model to evaluate
    model_path = output_dir
    for name in os.listdir(output_dir):
        if name.endswith('.onnx') and not name.endswith('_fp16.onnx'):
            model_path = os.path.join(output_dir, name)
            break
    
    eval_script = os.path.join(os.path.dirname(os.path.dirname(__file__)), "eval_coco_multi.py")
    if not os.path.exists(eval_script):
        eval_script = os.path.join(os.getcwd(), "eval_coco_multi.py")
    
    if not os.path.exists(eval_script):
        return
    
    log_file = os.path.join(os.path.dirname(output_dir), "evaluation_results.csv")
    
    cmd = [
        sys.executable,
        eval_script,
        "--model-path", str(model_path),
        "--model-type", "auto",
        "--model-name", os.path.basename(output_dir),
        "--resolution", str(image_size),
        "--datasets"
    ] + unique_datasets + [
        "--log-file", log_file,
    ]
    
    try:
        print(f"  Running post-training evaluation on {len(unique_datasets)} dataset(s)...")
        subprocess.run(cmd, cwd=os.path.dirname(eval_script), check=False)
    except Exception as e:
        print(f"  Warning: Evaluation failed: {e}")
