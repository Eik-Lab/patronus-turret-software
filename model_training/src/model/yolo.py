from __future__ import annotations

import json
import os
import shutil
from pathlib import Path

from src.model.base import Model, _generate_inference_artifacts, _run_post_training_eval
from src.utils.filter_dataset import filter_yolo_dataset


class YOLOModel(Model):
    name = "yolo"
    coco_dataset = False

    def filter(self, config, output_dir, balance_sizes=True):
        return filter_yolo_dataset(config, output_dir, balance_sizes=balance_sizes)

    def train(self, config, train_stats=None):
        from ultralytics import YOLO

        train_cfg = config["training"]
        model_cfg = config["model"]
        output_dir = Path(train_cfg["output_dir"])
        image_size = train_cfg["image_size"]
        output_dir.mkdir(parents=True, exist_ok=True)

        data_yaml = config["dataset"].get("data_yaml") or os.path.join(config["dataset"]["root"], "data.yaml")
        model = YOLO(model_cfg.get("name") or "yolo11s.pt")

        overrides = {}
        if config["model"].get("architecture"):
            overrides["cfg"] = config["model"]["architecture"]

        results = model.train(
            data=data_yaml,
            epochs=train_cfg["epochs"],
            batch=train_cfg["batch_size"],
            imgsz=image_size,
            project=str(output_dir.parent),
            name=output_dir.name,
            device=0,
            patience=15,
            save=True,
            plots=True,
            **overrides,
        )

        weights_dir = output_dir / "weights"
        best = weights_dir / "best.pt"
        last = weights_dir / "last.pt"
        if best.exists():
            shutil.copy(best, output_dir / "best.pt")
        elif last.exists():
            shutil.copy(last, output_dir / "last.pt")

        run_name = output_dir.name
        try:
            model = YOLO(str(output_dir / "best.pt" if (output_dir / "best.pt").exists() else weights_dir / "best.pt"))
            onnx_path = model.export(format="onnx", imgsz=image_size, simplify=True)
            if onnx_path and os.path.exists(onnx_path):
                target = output_dir / "model.onnx"
                if str(onnx_path) != str(target):
                    shutil.copy(onnx_path, target)
                shutil.copy(target, output_dir / f"{run_name}.onnx")
        except Exception:
            pass

        labels = []
        if os.path.exists(data_yaml):
            try:
                import yaml

                with open(data_yaml) as f:
                    data = yaml.safe_load(f)
                if data and "names" in data:
                    if isinstance(data["names"], dict):
                        labels = [data["names"][i] for i in sorted(data["names"].keys())]
                    elif isinstance(data["names"], list):
                        labels = data["names"]
            except Exception:
                pass
        if labels:
            with open(output_dir / "labels.txt", "w") as f:
                for label in labels:
                    f.write(f"{label}\n")
            id2label = {i: label for i, label in enumerate(labels)}
            _generate_inference_artifacts(output_dir, id2label, image_size, model_cfg.get("name"), config)

        if (output_dir / "model.onnx").exists():
            try:
                from src.model.export import export_fp16
                export_fp16(str(output_dir / "model.onnx"))
            except Exception:
                pass

        _run_post_training_eval(output_dir, config, image_size)
        metrics = {}
        if hasattr(results, "results_dict"):
            metrics = dict(results.results_dict)
        elif isinstance(results, dict):
            metrics = results
        return metrics
