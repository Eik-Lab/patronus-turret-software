from __future__ import annotations

import os
from pathlib import Path


class YOLOEvaluator:
    def __init__(self, model_path, resolution=640):
        from ultralytics import YOLO

        self.model_path = Path(model_path)
        self.resolution = resolution
        self.model = YOLO(str(self.model_path))

    def evaluate(self, coco, images_dir, conf_thres=0.001):
        results = []
        for img_id in coco.getImgIds():
            img_info = coco.loadImgs(img_id)[0]
            file_name = img_info["file_name"]
            img_path = os.path.join(images_dir, file_name)
            if not os.path.exists(img_path):
                img_path = os.path.join(images_dir, os.path.basename(file_name))
            if not os.path.exists(img_path):
                continue
            preds = self.model.predict(source=img_path, imgsz=self.resolution, conf=conf_thres, verbose=False)[0]
            if preds.boxes is None:
                continue
            for box in preds.boxes:
                xyxy = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = xyxy
                results.append({"image_id": int(img_id), "category_id": int(box.cls[0].cpu().item()) + 1, "bbox": [float(x1), float(y1), float(x2 - x1), float(y2 - y1)], "score": float(box.conf[0].cpu().item())})
        return results
