import os
from pathlib import Path

from ultralytics import YOLO

from src.export import export_fp16
from src.model.base import Model
from src.utils.filter_dataset import filter_yolo_dataset


class Yolo(Model):
    name = "yolo"
    coco_dataset = False

    def filter(self, config, output_dir):
        filter_yolo_dataset(config, output_dir)

    def train(self, config, train_stats=None):
        ds_cfg = config["dataset"]
        model_cfg = config["model"]
        train_cfg = config["training"]

        data_yaml = ds_cfg.get("data_yaml") or os.path.join(
            ds_cfg["root"], "data.yaml"
        )
        architecture = model_cfg.get("architecture")
        weights = model_cfg.get("name")
        if architecture:
            model = YOLO(architecture)
            if weights:
                model.load(weights)
        else:
            model = YOLO(weights)

        tb_dir = train_cfg.get("tensorboard_dir") or os.path.join(
            train_cfg["output_dir"], "train"
        )
        project = os.path.dirname(tb_dir) or "."
        name = os.path.basename(tb_dir)

        print(f"\n=== Training {model_cfg['name']} ({self.name}) ===")
        print(f"  TensorBoard: tensorboard --logdir {project}")
        results = model.train(
            data=data_yaml,
            epochs=train_cfg["epochs"],
            imgsz=train_cfg["image_size"],
            batch=train_cfg["batch_size"],
            lr0=float(train_cfg["learning_rate"]),
            lrf=float(train_cfg.get("lr_final_ratio", 0.01)),
            weight_decay=float(train_cfg["weight_decay"]),
            momentum=float(train_cfg.get("momentum", 0.937)),
            warmup_epochs=float(train_cfg.get("warmup_epochs", 3.0)),
            optimizer=train_cfg.get("optimizer", "auto"),
            cos_lr=train_cfg.get("cos_lr", False),
            freeze=train_cfg.get("freeze"),
            copy_paste=float(train_cfg.get("copy_paste", 0.0)),
            workers=train_cfg["num_workers"],
            seed=train_cfg["seed"],
            patience=train_cfg.get("early_stopping_patience", 0),
            deterministic=train_cfg.get("deterministic", True),
            project=project,
            name=name,
            exist_ok=True,
        )

        # Export only best.pt to ONNX, then remove all .pt files.
        weights_dir = Path(model.trainer.save_dir) / "weights"
        best_pt = weights_dir / "best.pt"
        if best_pt.is_file():
            YOLO(str(best_pt)).export(format="onnx", imgsz=train_cfg["image_size"])
        for pt in weights_dir.glob("*.pt"):
            pt.unlink()

        onnx_path = weights_dir / "best.onnx"
        if onnx_path.is_file():
            size_mb = onnx_path.stat().st_size / (1024 * 1024)
            print(f"  ONNX exported to {onnx_path} ({size_mb:.1f} MB)")
            export_fp16(str(onnx_path))

        rd = getattr(results, "results_dict", {})
        metrics = {
            "map": rd.get("metrics/mAP50-95(B)"),
            "map_50": rd.get("metrics/mAP50(B)"),
            "precision": rd.get("metrics/precision(B)"),
            "recall": rd.get("metrics/recall(B)"),
        }

        print("\n=== Evaluation Results ===")
        for label, value in metrics.items():
            if value is not None:
                print(f"  {label:<16} {value:.4f}")

        return metrics
