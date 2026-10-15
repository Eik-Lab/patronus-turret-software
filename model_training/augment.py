import albumentations as A


def build_train_transform():
    return A.Compose(
        [
            A.HorizontalFlip(p=0.5),
            A.RandomBrightnessContrast(p=0.3),
            A.HueSaturationValue(p=0.2),
            A.Affine(
                scale=(0.9, 1.1),
                translate_percent=(0.0, 0.1),
                rotate=(-10, 10),
                p=0.3,
            ),
        ],
        bbox_params=A.BboxParams(
            format="coco",
            label_fields=["category_ids"],
            min_visibility=0.3,
        ),
    )
