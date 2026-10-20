from transformers import AutoImageProcessor

from src.config import resolve
from src.data.augment import build_train_transform
from src.data.coco import CocoDetectionDataset


def build_datasets(config):
    """Create the image processor and train/val COCO datasets from config.

    The train split gets augment.py transforms when training.augment is set
    (defaults to on); the val split is never augmented. Returns
    (image_processor, train_dataset, val_dataset).
    """
    ds_cfg = config["dataset"]
    model_cfg = config["model"]
    train_cfg = config["training"]
    root = ds_cfg["root"]
    image_size = train_cfg["image_size"]

    image_processor = AutoImageProcessor.from_pretrained(
        model_cfg["name"],
        do_resize=True,
        size={"width": image_size, "height": image_size},
    )

    train_transform = build_train_transform() if train_cfg.get("augment", True) else None

    train_dataset = CocoDetectionDataset(
        resolve(root, ds_cfg["train_images"]),
        resolve(root, ds_cfg["train_annotations"]),
        image_processor,
        transform=train_transform,
    )
    val_dataset = CocoDetectionDataset(
        resolve(root, ds_cfg["val_images"]),
        resolve(root, ds_cfg["val_annotations"]),
        image_processor,
    )
    return image_processor, train_dataset, val_dataset
