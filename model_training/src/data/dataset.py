from transformers import AutoImageProcessor

from src.config import ConfigError, resolve
from src.data.augment import build_train_transform
from src.data.coco import CocoDetectionDataset


def build_datasets(config):
    ds_cfg = config["dataset"]
    model_cfg = config["model"]
    train_cfg = config["training"]

    required = ("train_images", "train_annotations", "val_images", "val_annotations")
    missing = [key for key in required if not ds_cfg.get(key)]
    if missing:
        raise ConfigError(
            "the %s backend needs COCO annotations; missing dataset.%s. "
            "Use --data to build a canonical dataset or run `patronus prepare`."
            % (model_cfg.get("type", "hf"), ", dataset.".join(missing))
        )

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
