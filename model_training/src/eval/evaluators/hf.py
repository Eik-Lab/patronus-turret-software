from __future__ import annotations

import os
from pathlib import Path

import torch
from PIL import Image
from transformers import AutoImageProcessor, AutoModelForObjectDetection


class HFModelEvaluator:
    def __init__(self, model_path, resolution=640):
        self.model_path = Path(model_path)
        self.resolution = resolution
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.processor = AutoImageProcessor.from_pretrained(str(self.model_path))
        self.model = AutoModelForObjectDetection.from_pretrained(str(self.model_path)).to(self.device)
        self.model.eval()

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
            image = Image.open(img_path).convert("RGB")
            inputs = self.processor(images=image, return_tensors="pt")
            inputs = {k: v.to(self.device) for k, v in inputs.items()}
            with torch.no_grad():
                outputs = self.model(**inputs)
            target_sizes = torch.tensor([image.size[::-1]]).to(self.device)
            post = self.processor.post_process_object_detection(outputs, threshold=conf_thres, target_sizes=target_sizes)[0]
            for score, label, box in zip(post["scores"], post["labels"], post["boxes"]):
                x1, y1, x2, y2 = box.cpu().numpy()
                results.append({"image_id": int(img_id), "category_id": int(label) + 1, "bbox": [float(x1), float(y1), float(x2 - x1), float(y2 - y1)], "score": float(score)})
        return results
