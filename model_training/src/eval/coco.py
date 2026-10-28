from __future__ import annotations

from pycocotools.coco import COCO
from pycocotools.cocoeval import COCOeval


def evaluate_dataset(evaluator, coco: COCO, images_dir, conf_thres: float) -> dict:
    results = evaluator.evaluate(coco, images_dir, conf_thres)
    if not results:
        return {k: 0.0 for k in ["map", "map_50", "map_75", "map_95", "map_small", "map_medium", "map_large", "mar_1", "mar_10", "mar_100", "mar_small", "mar_medium", "mar_large"]}
    coco_dt = coco.loadRes(results)
    coco_eval = COCOeval(coco, coco_dt, "bbox")
    coco_eval.evaluate()
    coco_eval.accumulate()
    coco_eval.summarize()
    stats = coco_eval.stats
    return {
        "map": float(stats[0]),
        "map_50": float(stats[1]),
        "map_75": float(stats[2]),
        "map_95": float(stats[3]),
        "map_small": float(stats[4]),
        "map_medium": float(stats[5]),
        "map_large": float(stats[6]),
        "mar_1": float(stats[7]),
        "mar_10": float(stats[8]),
        "mar_100": float(stats[9]),
        "mar_small": float(stats[12]),
        "mar_medium": float(stats[13]),
        "mar_large": float(stats[14]),
    }
