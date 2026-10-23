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

        print(f"\nTraining {model_cfg['name']} ({self.name})", "-"*50)
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
            try:
                YOLO(str(best_pt)).export(format="onnx", imgsz=train_cfg["image_size"], simplify=True)
            except Exception as e:
                print(f"  Warning: ONNX export failed: {e}")
        for pt in weights_dir.glob("*.pt"):
            pt.unlink()

        onnx_path = weights_dir / "best.onnx"
        if onnx_path.is_file():
            size_mb = onnx_path.stat().st_size / (1024 * 1024)
            print(f"  ONNX exported to {onnx_path} ({size_mb:.1f} MB)")
            try:
                export_fp16(str(onnx_path))
            except Exception as e:
                print(f"  Warning: FP16 export failed: {e}")
            # Copy to main output dir and generate artifacts
            import shutil
            main_onnx = os.path.join(train_cfg["output_dir"], "model.onnx")
            try:
                shutil.copy(onnx_path, main_onnx)
                fp16_out = os.path.join(train_cfg["output_dir"], "model_fp16.onnx")
                fp16_w = weights_dir / "best_fp16.onnx"
                if fp16_w.exists():
                    shutil.copy(fp16_w, fp16_out)
            except Exception as e:
                print(f"  Warning: Failed to copy ONNX: {e}")
            # Generate inference artifacts
            try:
                from src.model.base import _generate_inference_artifacts, _run_post_training_eval
                id2label = {}
                # Try to get from data.yaml
                try:
                    import yaml
                    ds_cfg = config["dataset"]
                    data_yaml = ds_cfg.get("data_yaml") or os.path.join(ds_cfg["root"], "data.yaml")
                    with open(data_yaml) as f:
                        data = yaml.safe_load(f)
                    names = data.get("names", {})
                    if isinstance(names, list):
                        id2label = {i: n for i, n in enumerate(names)}
                    elif isinstance(names, dict):
                        id2label = names
                except:
                    pass
                if not id2label:
                    id2label = {0: "drone"}
                _generate_inference_artifacts(train_cfg["output_dir"], id2label, train_cfg["image_size"], model_cfg.get("name", "yolo"), config)
                _run_post_training_eval(train_cfg["output_dir"], config, model_cfg.get("name", "yolo"), train_cfg["image_size"])
            except Exception as e:
                print(f"  Warning: Failed to generate artifacts: {e}")

        rd = getattr(results, "results_dict", {})
        metrics = {
            "map": rd.get("metrics/mAP50-95(B)"),
            "map_50": rd.get("metrics/mAP50(B)"),
            "precision": rd.get("metrics/precision(B)"),
            "recall": rd.get("metrics/recall(B)"),
        }

        print("\nEvaluation results", "-"*50)
        for label, value in metrics.items():
            if value is not None:
                print(f"  {label:<16} {value:.4f}")

        return metrics
