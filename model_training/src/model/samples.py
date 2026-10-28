import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import torch
from PIL import Image
from torch.utils.tensorboard import SummaryWriter
from transformers import TrainerCallback

from src.stats.palette import HIGHLIGHT

GRID = 4
NUM_SAMPLES = GRID * GRID


class SampleGridCallback(TrainerCallback):
    def __init__(
        self,
        image_processor,
        val_dataset,
        output_dir,
        logging_dir,
        id2label,
        score_threshold=0.3,
        batch_size=4,
    ):
        self.image_processor = image_processor
        self.id2label = id2label
        self.score_threshold = score_threshold
        self.batch_size = batch_size
        self.samples_dir = os.path.join(output_dir, "samples")
        os.makedirs(self.samples_dir, exist_ok=True)
        self.logging_dir = logging_dir
        self._tb_writer = None


        coco = val_dataset.coco
        self.images = []
        for image_id in val_dataset.ids[:NUM_SAMPLES]:
            file_name = coco.loadImgs(image_id)[0]["file_name"]
            self.images.append(Image.open(os.path.join(val_dataset.root, file_name)).convert("RGB"))

    def _predict(self, model):
        was_training = model.training
        model.eval()
        device = next(model.parameters()).device
        results = []
        with torch.no_grad():
            for start in range(0, len(self.images), self.batch_size):
                images = self.images[start : start + self.batch_size]
                inputs = self.image_processor(images=images, return_tensors="pt").to(device)
                outputs = model(**inputs)
                target_sizes = torch.tensor([img.size[::-1] for img in images], device=device)
                results.extend(
                    self.image_processor.post_process_object_detection(
                        outputs, target_sizes=target_sizes, threshold=self.score_threshold
                    )
                )
        if was_training:
            model.train()
        return results

    def _render(self, results):
        fig, axes = plt.subplots(GRID, GRID, figsize=(16, 16))
        for ax, img, result in zip(axes.flat, self.images, results, strict=False):
            ax.imshow(img)
            ax.axis("off")
            for score, label, box in zip(
                result["scores"], result["labels"], result["boxes"], strict=True
            ):
                x0, y0, x1, y1 = box.tolist()
                ax.add_patch(
                    patches.Rectangle(
                        (x0, y0),
                        x1 - x0,
                        y1 - y0,
                        linewidth=2,
                        edgecolor=HIGHLIGHT,
                        facecolor="none",
                    )
                )
                name = self.id2label.get(int(label), int(label))
                ax.text(
                    x0,
                    y0,
                    f"{name} {score:.2f}",
                    color="white",
                    fontsize=8,
                    backgroundcolor=HIGHLIGHT,
                )
        fig.tight_layout()
        return fig

    def on_evaluate(self, args, state, control, model=None, **kwargs):
        if model is None:
            return
        results = self._predict(model)
        fig = self._render(results)

        path = os.path.join(self.samples_dir, f"step_{state.global_step:06d}.png")
        fig.savefig(path, dpi=100)

        if self._tb_writer is None:
            self._tb_writer = SummaryWriter(log_dir=self.logging_dir)
        self._tb_writer.add_figure("samples/predictions", fig, global_step=state.global_step)

        plt.close(fig)
        print(f"  saved sample grid {path}")

    def on_train_end(self, args, state, control, **kwargs):
        if self._tb_writer is not None:
            self._tb_writer.close()
