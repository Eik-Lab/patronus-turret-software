import glob
import os

from ultralytics import YOLO

from src.model.base import Model
from utils.filter_dataset import filter_yolo_dataset


class Yolo(Model):
    name = "yolo"
    coco_dataset = False

    def filter(self, config, output_dir):
        filter_yolo_dataset(config, output_dir)

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

        # Ultralytics ties its TensorBoard writer to the run's save_dir, so we
        # point the whole run at training.tensorboard_dir. That keeps YOLO runs
        # under the same shared TensorBoard root as the HF experiments, letting
        # one `tensorboard --logdir` overlay and compare every model.
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
            workers=train_cfg["num_workers"],
            seed=train_cfg["seed"],
            deterministic=train_cfg.get("deterministic", True),
            project=project,
            name=name,
            exist_ok=True,
            save_period=1,  # checkpoint every epoch
        )

        weights_dir = os.path.join(model.trainer.save_dir, "weights")
        for pt in sorted(glob.glob(os.path.join(weights_dir, "*.pt"))):
            YOLO(pt).export(format="onnx", imgsz=train_cfg["image_size"])
            os.remove(pt)
        print(f"  ONNX models saved to {weights_dir}")

        # Normalise Ultralytics' metric keys to match the HF experiments.
        rd = getattr(results, "results_dict", {})
        return {
            "map": rd.get("metrics/mAP50-95(B)"),
            "map_50": rd.get("metrics/mAP50(B)"),
            "map_75": rd.get("metrics/mAP75(B)"),
        }
