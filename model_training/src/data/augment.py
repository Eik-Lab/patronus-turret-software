import albumentations as A


def build_train_transform():
    return A.Compose(
        [
            A.HorizontalFlip(p=0.5),
            A.RandomBrightnessContrast(
                brightness_limit=0.2, contrast_limit=0.2, p=0.3
            ),
            A.HueSaturationValue(
                hue_shift_limit=10, sat_shift_limit=30, val_shift_limit=20, p=0.2
            ),
            A.CLAHE(clip_limit=4.0, p=0.2),
            A.ToGray(p=0.1),
            A.Affine(
                scale=(0.5, 1.5),
                translate_percent=(0.0, 0.1),
                rotate=(-30, 30),
                p=0.4,
            ),
            A.MotionBlur(blur_limit=7, p=0.15),
            # Outdoor / sky-specific augmentations
            A.RandomFog(fog_coef_lower=0.1, fog_coef_upper=0.3, p=0.1),
            A.RandomSunFlare(
                src_radius=80, num_flare_circles_lower=1,
                num_flare_circles_upper=3, p=0.1,
            ),
            A.Downscale(scale_range=(0.5, 0.75), p=0.15),
        ],
        bbox_params=A.BboxParams(
            format="coco",
            label_fields=["category_ids"],
            min_visibility=0.3,
        ),
    )
