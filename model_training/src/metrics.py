from dataclasses import dataclass

import torch
from torchmetrics.detection.mean_ap import MeanAveragePrecision
from transformers.image_transforms import center_to_corners_format


@dataclass
class DetectionOutput:
    logits: torch.Tensor
    pred_boxes: torch.Tensor


def _to_xyxy(boxes, image_size):
    boxes = center_to_corners_format(boxes)
    height, width = float(image_size[0]), float(image_size[1])
    scale = torch.tensor([width, height, width, height], dtype=boxes.dtype)
    return boxes * scale


@torch.no_grad()
def compute_metrics(evaluation_results, image_processor, id2label, threshold=0.0):
    predictions, targets = (
        evaluation_results.predictions,
        evaluation_results.label_ids,
    )

    image_sizes = []
    post_processed_targets = []
    post_processed_predictions = []

    for batch in targets:
        image_sizes.append(torch.stack([torch.as_tensor(t["orig_size"]) for t in batch]))
        for target in batch:
            boxes = _to_xyxy(torch.as_tensor(target["boxes"]), target["orig_size"])
            post_processed_targets.append(
                {"boxes": boxes, "labels": torch.as_tensor(target["class_labels"])}
            )

    for batch, target_sizes in zip(predictions, image_sizes, strict=True):
        output = DetectionOutput(logits=torch.tensor(batch[1]), pred_boxes=torch.tensor(batch[2]))
        post_processed_predictions.extend(
            image_processor.post_process_object_detection(
                output, threshold=threshold, target_sizes=target_sizes
            )
        )

    metric = MeanAveragePrecision(box_format="xyxy", class_metrics=True)
    metric.update(post_processed_predictions, post_processed_targets)
    computed = metric.compute()

    classes = computed.pop("classes", None)
    map_per_class = computed.pop("map_per_class", None)
    mar_100_per_class = computed.pop("mar_100_per_class", None)


    metrics = {key: round(value.item(), 4) for key, value in computed.items() if value.numel() == 1}

    if classes is not None and map_per_class is not None:
        classes = classes.reshape(-1)
        map_per_class = map_per_class.reshape(-1)
        mar_100_per_class = mar_100_per_class.reshape(-1)
        for class_id, class_map, class_mar in zip(
            classes, map_per_class, mar_100_per_class, strict=True
        ):
            name = id2label.get(class_id.item(), class_id.item())
            metrics[f"map_{name}"] = round(class_map.item(), 4)
            metrics[f"mar_100_{name}"] = round(class_mar.item(), 4)

    return metrics
