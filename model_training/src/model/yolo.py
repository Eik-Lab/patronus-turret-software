import glob
import os

from ultralytics import YOLO

from src.model.base import Model


class Yolo(Model):
    name = "yolo"
    coco_dataset = False

    def train(self, config, train_stats=None):
        ds_cfg = config["dataset"]
        model_cfg = config["model"]
        train_cfg = config["training"]

        # Ultralytics trains from a YOLO-txt dataset described by a data.yaml.
        data_yaml = ds_cfg.get("data_yaml") or os.path.join(ds_cfg["root"], "data.yaml")
        architecture = model_cfg.get("architecture")
        weights = model_cfg.get("name")
        if architecture:
            model = YOLO(architecture)
            if weights:
                model.load(weights)
        else:
            model = YOLO(weights)

        print(f"\n=== Training {model_cfg['name']} ({self.name}) ===")
        print(f"  TensorBoard: tensorboard --logdir {train_cfg['output_dir']}")
        model.train(
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
            workers=train_cfg["num_workers"],
            seed=train_cfg["seed"],
            project=train_cfg["output_dir"],
            name="train",
            save_period=1,  # checkpoint every epoch
        )

        weights_dir = os.path.join(model.trainer.save_dir, "weights")
        for pt in sorted(glob.glob(os.path.join(weights_dir, "*.pt"))):
            YOLO(pt).export(format="onnx", imgsz=train_cfg["image_size"])
            os.remove(pt)
        print(f"  ONNX models saved to {weights_dir}")
