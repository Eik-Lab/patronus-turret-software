import numpy as np
import torchvision


class CocoDetectionDataset(torchvision.datasets.CocoDetection):
    def __init__(self, img_folder, ann_file, processor, transform=None):
        super().__init__(img_folder, ann_file)
        self.processor = processor
        self.transform = transform


        self._category_map = {
            cat_id: idx for idx, cat_id in enumerate(sorted(self.coco.getCatIds()))
        }

    def __getitem__(self, idx):
        img, target = super().__getitem__(idx)
        image_id = self.ids[idx]

        target = [
            {**ann, "category_id": self._category_map.get(ann["category_id"],
                                                          ann["category_id"])}
            for ann in target
        ]

        if self.transform is not None:
            augmented = self.transform(
                image=np.array(img.convert("RGB")),
                bboxes=[ann["bbox"] for ann in target],
                category_ids=[ann["category_id"] for ann in target],
            )
            image = augmented["image"]
            target = [
                {
                    "image_id": image_id,
                    "category_id": category_id,
                    "bbox": list(bbox),
                    "area": bbox[2] * bbox[3],
                    "iscrowd": 0,
                }
                for bbox, category_id in zip(
                    augmented["bboxes"], augmented["category_ids"]
                )
            ]
        else:
            image = img

        coco_target = {"image_id": image_id, "annotations": target}
        encoding = self.processor(
            images=image, annotations=coco_target, return_tensors="pt"
        )
        return {
            "pixel_values": encoding["pixel_values"].squeeze(0),
            "labels": encoding["labels"][0],
        }
