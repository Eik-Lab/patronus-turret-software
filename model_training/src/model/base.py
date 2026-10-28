import os
from pathlib import Path

import torch

from src.data.dataset import build_datasets
from src.model.export import export_fp16
from src.model.metrics import compute_metrics
from src.model.samples import SampleGridCallback
from src.utils.filter_dataset import filter_coco_dataset


class Model:
    name = "model"
    coco_dataset = True

    def filter(self, config, output_dir, balance_sizes=True):
        pass

    def train(self, config, train_stats):
        raise NotImplementedError


class HFDetModel(Model):
    name = "hf"

    def filter(self, config, output_dir, balance_sizes=True):
        return filter_coco_dataset(config, output_dir, balance_sizes=balance_sizes)

    def train(self, config, train_stats):
        from transformers import AutoModelForObjectDetection, EarlyStoppingCallback, Trainer, TrainingArguments

        model_cfg = config["model"]
        train_cfg = config["training"]
        output_dir = train_cfg["output_dir"]
        image_size = train_cfg["image_size"]
        Path(train_cfg["tensorboard_dir"]).mkdir(parents=True, exist_ok=True)
        Path(output_dir).mkdir(parents=True, exist_ok=True)

        train_dataset, val_dataset, id2label, label2id = build_datasets(config)

        model = AutoModelForObjectDetection.from_pretrained(
            model_cfg["name"],
            id2label=id2label,
            label2id=label2id,
            ignore_mismatched_sizes=True,
        )

        training_args = TrainingArguments(
            output_dir=output_dir,
            per_device_train_batch_size=train_cfg["batch_size"],
            per_device_eval_batch_size=train_cfg["batch_size"],
            num_train_epochs=train_cfg["epochs"],
            learning_rate=train_cfg["learning_rate"],
            weight_decay=train_cfg["weight_decay"],
            warmup_ratio=0.1,
            lr_scheduler_type="cosine",
            logging_steps=train_cfg["logging_steps"],
            eval_strategy="steps",
            eval_steps=train_cfg["eval_steps"],
            save_steps=train_cfg["save_steps"],
            save_total_limit=train_cfg["save_total_limit"],
            dataloader_num_workers=train_cfg["num_workers"],
            remove_unused_columns=False,
            report_to=["tensorboard"],
            seed=train_cfg["seed"],
            load_best_model_at_end=True,
            metric_for_best_model="eval_map",
            greater_is_better=True,
            fp16=train_cfg["device"] == "cuda",
            use_cpu=False,
            dataloader_pin_memory=True,
        )

        trainer = Trainer(
            model=model,
            args=training_args,
            train_dataset=train_dataset,
            eval_dataset=val_dataset,
            processing_class=train_dataset.processor,
            data_collator=val_dataset.collate_fn,
            compute_metrics=compute_metrics,
            callbacks=[EarlyStoppingCallback(early_stopping_patience=15), SampleGridCallback()],
        )

        trainer.train()
        trainer.save_model(output_dir)
        trainer.save_state()

        metrics = trainer.evaluate(eval_dataset=val_dataset)
        _generate_inference_artifacts(output_dir, id2label, image_size, model_cfg["name"], config)
        onnx_path = _export_onnx(model, output_dir, image_size, model_cfg["name"], config)
        export_fp16(onnx_path)
        _remove_checkpoints(output_dir)
        _run_post_training_eval(output_dir, config, image_size)
        return metrics


def _export_onnx(model, output_dir, image_size, model_name=None, config=None):
    from transformers import AutoProcessor

    model_name = model_name or ""
    model_name_lower = str(model_name).lower()
    config = config or {}
    model_cfg = config.get("model", {}) if isinstance(config, dict) else {}
    model_type = str(model_cfg.get("type", "")).lower()

    if "rfdetr" in model_name_lower or "rf-detr" in model_name_lower or model_type == "rfdetr":
        try:
            from src.export_rfdetr_onnx import export_onnx

            onnx_path = os.path.join(output_dir, f"{os.path.basename(output_dir) or 'model'}.onnx")
            export_onnx(output_dir, onnx_path, resolution=image_size, fold_normalization=True, check=True)
            if os.path.isfile(onnx_path):
                return onnx_path
        except Exception:
            pass

    if "dfine" in model_name_lower or model_type == "dfine":
        try:
            class _DFineWrapper(torch.nn.Module):
                def __init__(self, inner):
                    super().__init__()
                    self.inner = inner

                def forward(self, pixel_values, pixel_mask=None):
                    out = self.inner(pixel_values=pixel_values, pixel_mask=pixel_mask)
                    if hasattr(out, "logits") and hasattr(out, "pred_boxes"):
                        return out.logits, out.pred_boxes
                    if isinstance(out, dict):
                        logits = out.get("logits") or out.get("pred_logits") or out.get("class_logits")
                        boxes = out.get("pred_boxes") or out.get("boxes") or out.get("pred_bbox")
                        if logits is not None and boxes is not None:
                            return logits, boxes
                    # Fallback to tuple access
                    try:
                        if isinstance(out, (tuple, list)) and len(out) >= 2:
                            return out[0], out[1]
                    except Exception:
                        pass
                    raise RuntimeError("Could not extract logits and boxes from DFINE model output")

            wrapper = _DFineWrapper(model).eval()
            device = next(model.parameters()).device
            dummy_pixels = torch.randn(1, 3, image_size, image_size, device=device)
            try:
                dummy_mask = torch.ones(1, image_size, image_size, dtype=torch.long, device=device)
            except Exception:
                dummy_mask = None
            onnx_path = os.path.join(output_dir, "model.onnx")
            try:
                if dummy_mask is not None:
                    torch.onnx.export(
                        wrapper,
                        (dummy_pixels, dummy_mask),
                        onnx_path,
                        input_names=["pixel_values", "pixel_mask"],
                        output_names=["pred_logits", "pred_boxes"],
                        dynamic_axes={"pixel_values": {0: "batch"}, "pixel_mask": {0: "batch"}, "pred_logits": {0: "batch"}, "pred_boxes": {0: "batch"}},
                        opset_version=17,
                        dynamo=False,
                    )
                else:
                    torch.onnx.export(
                        wrapper,
                        (dummy_pixels,),
                        onnx_path,
                        input_names=["pixel_values"],
                        output_names=["pred_logits", "pred_boxes"],
                        dynamic_axes={"pixel_values": {0: "batch"}, "pred_logits": {0: "batch"}, "pred_boxes": {0: "batch"}},
                        opset_version=17,
                        dynamo=False,
                    )
            except Exception:
                if dummy_mask is not None:
                    torch.onnx.export(
                        wrapper,
                        (dummy_pixels, dummy_mask),
                        onnx_path,
                        input_names=["pixel_values", "pixel_mask"],
                        output_names=["logits", "pred_boxes"],
                        dynamic_axes={"pixel_values": {0: "batch"}, "pixel_mask": {0: "batch"}, "logits": {0: "batch"}, "pred_boxes": {0: "batch"}},
                        opset_version=17,
                        dynamo=False,
                    )
                else:
                    torch.onnx.export(
                        wrapper,
                        (dummy_pixels,),
                        onnx_path,
                        input_names=["pixel_values"],
                        output_names=["logits", "pred_boxes"],
                        dynamic_axes={"pixel_values": {0: "batch"}, "logits": {0: "batch"}, "pred_boxes": {0: "batch"}},
                        opset_version=17,
                        dynamo=False,
                    )
            try:
                from onnxslim import slim
                import onnx

                onnx.save(slim(onnx_path), onnx_path)
            except Exception:
                pass
            if os.path.isfile(onnx_path) and os.path.getsize(onnx_path) > 0:
                return onnx_path
        except Exception:
            pass

    class _Wrapper(torch.nn.Module):
        def __init__(self, inner):
            super().__init__()
            self.inner = inner

        def forward(self, pixel_values, pixel_mask):
            out = self.inner(pixel_values=pixel_values, pixel_mask=pixel_mask)
            if hasattr(out, "logits") and hasattr(out, "pred_boxes"):
                return out.logits, out.pred_boxes
            if isinstance(out, dict):
                return out.get("logits") or out.get("pred_logits"), out.get("pred_boxes") or out.get("boxes")
            return out

    wrapper = _Wrapper(model).eval()
    device = next(model.parameters()).device
    dummy_pixels = torch.randn(1, 3, image_size, image_size, device=device)
    try:
        dummy_mask = torch.ones(1, image_size, image_size, dtype=torch.long, device=device)
    except Exception:
        dummy_mask = None
    onnx_path = os.path.join(output_dir, "model.onnx")
    try:
        if dummy_mask is not None:
            torch.onnx.export(
                wrapper,
                (dummy_pixels, dummy_mask),
                onnx_path,
                input_names=["pixel_values", "pixel_mask"],
                output_names=["pred_logits", "pred_boxes"],
                dynamic_axes={"pixel_values": {0: "batch"}, "pixel_mask": {0: "batch"}, "pred_logits": {0: "batch"}, "pred_boxes": {0: "batch"}},
                opset_version=17,
                dynamo=False,
            )
        else:
            torch.onnx.export(
                wrapper,
                (dummy_pixels,),
                onnx_path,
                input_names=["pixel_values"],
                output_names=["pred_logits", "pred_boxes"],
                dynamic_axes={"pixel_values": {0: "batch"}, "pred_logits": {0: "batch"}, "pred_boxes": {0: "batch"}},
                opset_version=17,
                dynamo=False,
            )
    except Exception:
        if dummy_mask is not None:
            torch.onnx.export(
                wrapper,
                (dummy_pixels, dummy_mask),
                onnx_path,
                input_names=["pixel_values", "pixel_mask"],
                output_names=["logits", "pred_boxes"],
                dynamic_axes={"pixel_values": {0: "batch"}, "pixel_mask": {0: "batch"}, "logits": {0: "batch"}, "pred_boxes": {0: "batch"}},
                opset_version=17,
                dynamo=False,
            )
        else:
            torch.onnx.export(
                wrapper,
                (dummy_pixels,),
                onnx_path,
                input_names=["pixel_values"],
                output_names=["logits", "pred_boxes"],
                dynamic_axes={"pixel_values": {0: "batch"}, "logits": {0: "batch"}, "pred_boxes": {0: "batch"}},
                opset_version=17,
                dynamo=False,
            )
    try:
        from onnxslim import slim
        import onnx

        onnx.save(slim(onnx_path), onnx_path)
    except Exception:
        pass
    if not os.path.isfile(onnx_path) or os.path.getsize(onnx_path) == 0:
        return None
    return onnx_path


def _remove_checkpoints(output_dir):
    for item in Path(output_dir).iterdir():
        if item.is_dir() and item.name.startswith("checkpoint-"):
            import shutil

            shutil.rmtree(item)


def _generate_inference_artifacts(output_dir, id2label, image_size, model_name, config):
    from src.model.inference_artifacts import generate_inference_artifacts

    generate_inference_artifacts(output_dir, id2label, image_size, model_name, config)


def _run_post_training_eval(output_dir, config, image_size):
    ds_cfg = config.get("dataset", {}) if isinstance(config, dict) else {}
    test_datasets = []
    root = ds_cfg.get("root", "")
    if root and os.path.exists(root):
        for candidate in ["test", "val", "val_original", "test_original", "validation"]:
            cand_path = os.path.join(root, candidate)
            if os.path.exists(cand_path):
                test_datasets.append(cand_path)
    seen = set()
    unique_datasets = []
    for td in test_datasets:
        td_abs = os.path.abspath(td)
        if td_abs not in seen:
            seen.add(td_abs)
            unique_datasets.append(td)
    if not unique_datasets:
        return
    model_path = output_dir
    eval_model_type = "auto"
    config_model_type = str(config.get("model", {}).get("type", "")).lower()
    if config_model_type in ("yolo", "ultralytics"):
        eval_model_type = "yolo"
        for name in os.listdir(output_dir):
            if name.endswith(".onnx") and not name.endswith("_fp16.onnx"):
                model_path = os.path.join(output_dir, name)
                break
    log_file = os.path.join(os.path.dirname(output_dir), "evaluation_results.csv")
    try:
        from src.eval import run_eval
        run_eval(model_path=str(model_path), model_type=eval_model_type, resolution=image_size, datasets=unique_datasets, log_file=log_file)
    except Exception:
        pass
