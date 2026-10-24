import math
import os
import shutil
from functools import partial
from pathlib import Path
from inspect import signature

import onnx
import torch
from onnxslim import slim
from transformers import (
    AutoModelForObjectDetection,
    EarlyStoppingCallback,
    Trainer,
    TrainingArguments,
)

if torch.cuda.is_available():
    torch.backends.cudnn.benchmark = True
    torch.backends.cuda.matmul.allow_tf32 = True
    torch.backends.cudnn.allow_tf32 = True

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


def _export_onnx(model, output_dir, image_size):
    """Export the detection model to ONNX with onnxslim optimisation."""

    class _Wrapper(torch.nn.Module):
        def __init__(self, inner):
            super().__init__()
            self.inner = inner

        def forward(self, pixel_values, pixel_mask):
            out = self.inner(pixel_values=pixel_values, pixel_mask=pixel_mask)
            return out.logits, out.pred_boxes

    wrapper = _Wrapper(model).eval()
    device = next(model.parameters()).device
    dummy_pixels = torch.randn(1, 3, image_size, image_size, device=device)
    dummy_mask = torch.ones(1, image_size, image_size, dtype=torch.long, device=device)

    onnx_path = os.path.join(output_dir, "model.onnx")
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

    slimmed = slim(onnx_path)
    onnx.save(slimmed, onnx_path)

    size_mb = os.path.getsize(onnx_path) / (1024 * 1024)
    print(f"  ONNX exported to {onnx_path} ({size_mb:.1f} MB)")


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

        from transformers import AutoConfig
        model_config = AutoConfig.from_pretrained(model_cfg["name"])
        model_config.num_labels = len(id2label)
        num_denoising = model_cfg.get("num_denoising")
        if num_denoising is not None:
            model_config.num_denoising = num_denoising
        model_config.id2label = id2label
        model_config.label2id = label2id
        model = AutoModelForObjectDetection.from_pretrained(
            model_cfg["name"],
            config=model_config,
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

        batch_size = train_cfg["batch_size"]
        grad_accum = train_cfg.get("gradient_accumulation_steps", 1)
        warmup_ratio = float(train_cfg.get("warmup_ratio", 0.0))
        warmup_steps = int(train_cfg.get("warmup_steps", 0))
        if warmup_steps == 0 and warmup_ratio > 0:
            steps_per_epoch = math.ceil(len(train_dataset) / (batch_size * grad_accum))
            warmup_steps = int(steps_per_epoch * train_cfg["epochs"] * warmup_ratio)

        ta_kwargs = {
            "output_dir": output_dir,
            "report_to": ["tensorboard"],
            "num_train_epochs": train_cfg["epochs"],
            "per_device_train_batch_size": batch_size,
            "per_device_eval_batch_size": train_cfg.get("eval_batch_size", batch_size),
            "learning_rate": float(train_cfg["learning_rate"]),
            "weight_decay": float(train_cfg["weight_decay"]),
            "gradient_accumulation_steps": grad_accum,
            "lr_scheduler_type": train_cfg.get("lr_scheduler_type", "linear"),
            "max_grad_norm": train_cfg.get("max_grad_norm", 1.0),
            "optim": train_cfg.get("optimizer", "adamw_torch"),
            "dataloader_num_workers": train_cfg["num_workers"],
            "dataloader_prefetch_factor": train_cfg.get("dataloader_prefetch_factor"),
            "logging_steps": train_cfg["logging_steps"],
            "save_total_limit": train_cfg.get("save_total_limit", 1),
            "seed": train_cfg["seed"],
            "data_seed": train_cfg["seed"],
            "full_determinism": train_cfg.get("deterministic", False),
            "eval_strategy": "epoch",
            "save_strategy": "epoch",
            "bf16": use_bf16,
            "fp16": torch.cuda.is_available() and not use_bf16,
            "gradient_checkpointing": gc_requested and gc_supported,
            "load_best_model_at_end": False,
            "metric_for_best_model": "map",
            "greater_is_better": True,
            "remove_unused_columns": False,
        }
        ta_sig = signature(TrainingArguments.__init__)
        if "warmup_ratio" in ta_sig.parameters and "warmup_ratio" in train_cfg:
            ta_kwargs["warmup_ratio"] = warmup_ratio
        elif "warmup_steps" in ta_sig.parameters:
            ta_kwargs["warmup_steps"] = warmup_steps
        if "dataloader_persistent_workers" in ta_sig.parameters:
            ta_kwargs["dataloader_persistent_workers"] = train_cfg["num_workers"] > 0
        if "eval_do_concat_batches" in ta_sig.parameters:
            ta_kwargs["eval_do_concat_batches"] = False
        if "gradient_checkpointing_kwargs" in ta_sig.parameters and ta_kwargs.get("gradient_checkpointing"):
            ta_kwargs["gradient_checkpointing_kwargs"] = {"use_reentrant": False}
        args = TrainingArguments(**ta_kwargs)
        if "LOCAL_RANK" not in os.environ:
            args._n_gpu = 1

        if train_cfg.get("compile", False):
            try:
                model = torch.compile(model)
            except Exception as exc:
                print(f"  torch.compile skipped: {exc}")

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

        _export_onnx(
            trainer.accelerator.unwrap_model(trainer.model), output_dir, image_size
        )
        export_fp16(os.path.join(output_dir, "model.onnx"))
        _remove_checkpoints(output_dir)
        print(f"  model saved to {output_dir}")

        metrics = trainer.evaluate()
        metrics = {key.removeprefix("eval_"): value for key, value in metrics.items()}
        _print_summary(metrics, id2label)

        return metrics
