#!/usr/bin/env python3
"""Export a fine-tuned Hugging Face RF-DETR checkpoint to ONNX.

The exported graph is static (batch 1) and is ready for TensorRT / DeepStream
on Jetson Thor:

    input   pixel_values : float32 [1, 3, H, W]  (RGB, [0, 1])
    outputs pred_logits  : float32 [num_queries, num_classes]  (raw logits)
            pred_boxes   : float32 [num_queries, 4]            (normalized cxcywh)

By default the ImageNet normalization is folded into the first patch-embedding
convolution, so the ONNX input is simply RGB / 255.  This makes the DeepStream
preprocessing exact (``net-scale-factor=1/255``, ``offsets=0``) and adds no
extra runtime operations.

The checkpoint's ``preprocessor_config.json`` resizes images to a 1024x1024
square (``do_pad`` is enabled, but the image is already square, so no padding
is applied).  The export is therefore fixed to 1024x1024.

Examples:
    # From model_training/
    python export_rfdetr_onnx.py --model-dir rf_detr_large

    # Export for a runtime that already feeds ImageNet-normalized tensors
    python export_rfdetr_onnx.py --model-dir rf_detr_large --no-fold-normalization

    # Different resolution (must be validated against the checkpoint!)
    python export_rfdetr_onnx.py --model-dir rf_detr_large --resolution 640

Requirements:
    torch, transformers, onnx, onnxruntime, onnxslim (optional but recommended)
"""

from __future__ import annotations

import argparse
import copy
import json
import os
from pathlib import Path

import torch


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--model-dir",
        type=Path,
        default=Path("."),
        help="Directory containing config.json, model.safetensors and preprocessor_config.json (default: .)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("rf_detr_drone.onnx"),
        help="Output ONNX path (default: rf_detr_drone.onnx)",
    )
    parser.add_argument(
        "--resolution",
        type=int,
        default=None,
        help="Square input resolution. Defaults to the height/width in preprocessor_config.json.",
    )
    parser.add_argument(
        "--opset",
        type=int,
        default=17,
        help="ONNX opset version (default: 17; TensorRT 10 on Thor supports this).",
    )
    parser.add_argument(
        "--fold-normalization",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Fold ImageNet mean/std into the first convolution so input is RGB [0,1] (default: true).",
    )
    parser.add_argument(
        "--scores",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Also emit a sigmoid 'pred_scores' output (default: false).",
    )
    parser.add_argument(
        "--slim",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Run onnxslim to constant-fold and simplify the graph (default: false). "
            "onnxslim can change TopK tie-breaking, which reorders the 300 queries; "
            "validate the output against PyTorch before using a slimmed file."
        ),
    )
    parser.add_argument(
        "--check",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Compare ONNX Runtime against PyTorch (default: true).",
    )
    return parser.parse_args()


def read_preprocessor(model_dir: Path) -> dict:
    path = model_dir / "preprocessor_config.json"
    if not path.exists():
        return {}
    with path.open() as handle:
        return json.load(handle)


def infer_resolution(model_dir: Path, override: int | None) -> int:
    if override is not None:
        return override
    config = read_preprocessor(model_dir)
    size = config.get("size", {})
    height = size.get("height")
    width = size.get("width")
    if height and width and height == width:
        return int(height)
    raise SystemExit(
        "Could not infer a square resolution from preprocessor_config.json; pass --resolution explicitly."
    )


def get_normalization(model_dir: Path) -> tuple[torch.Tensor, torch.Tensor]:
    config = read_preprocessor(model_dir)
    mean = config.get("image_mean", [0.485, 0.456, 0.406])
    std = config.get("image_std", [0.229, 0.224, 0.225])
    return (
        torch.tensor(mean, dtype=torch.float32).view(1, 3, 1, 1),
        torch.tensor(std, dtype=torch.float32).view(1, 3, 1, 1),
    )


def patch_static_position_embedding(model: torch.nn.Module, height: int, width: int) -> None:
    """Precompute the interpolated DINOv2 position embedding for the static input.

    The model interpolates its 44x44 (704/16) position grid to the input grid
    with ``antialias=True`` bicubic interpolation.  ONNX has no antialias
    bicubic Resize, so we compute it once in eager mode and return the constant
    during tracing.  This is exact for the fixed input resolution we export.
    """
    embeddings = model.model.backbone.backbone.embeddings
    patch_size = embeddings.patch_size
    num_patches = (height // patch_size) * (width // patch_size)
    hidden_size = embeddings.position_embeddings.shape[-1]

    with torch.no_grad():
        dummy = torch.zeros(1, 1 + num_patches, hidden_size, dtype=embeddings.position_embeddings.dtype)
        position = embeddings.interpolate_pos_encoding(dummy, height, width).detach().clone()

    def static_position_embedding(emb, height: int, width: int, _position=position):  # noqa: ARG001
        return _position.to(device=emb.device, dtype=emb.dtype)

    embeddings.interpolate_pos_encoding = static_position_embedding


def fold_imagenet_normalization(model: torch.nn.Module, mean: torch.Tensor, std: torch.Tensor) -> None:
    """Fold ``(x - mean) / std`` into the first Conv2d (patch embedding).

    After this the model consumes RGB in [0, 1] instead of ImageNet-normalized
    input.  The result is mathematically identical for the original pipeline.
    """
    projection = model.model.backbone.backbone.embeddings.patch_embeddings.projection
    if not isinstance(projection, torch.nn.Conv2d):
        raise SystemExit("Expected the RF-DETR patch embedding to be a Conv2d; cannot fold normalization.")

    with torch.no_grad():
        weight = projection.weight.detach().clone()
        bias = projection.bias.detach().clone() if projection.bias is not None else torch.zeros(
            weight.shape[0], dtype=weight.dtype
        )
        mean_c = mean.view(1, 3, 1, 1)
        std_c = std.view(1, 3, 1, 1)
        projection.weight.copy_(weight / std_c)
        projection.bias.copy_(bias - (weight * (mean_c / std_c)).sum(dim=(1, 2, 3)))


class ExportWrapper(torch.nn.Module):
    def __init__(self, model: torch.nn.Module, emit_scores: bool) -> None:
        super().__init__()
        self.model = model
        self.emit_scores = emit_scores

    def forward(self, pixel_values: torch.Tensor):
        outputs = self.model(pixel_values=pixel_values)
        # DeepStream custom parsers expect 2-D [queries, classes] / [queries, 4].
        logits = outputs.logits[0]
        boxes = outputs.pred_boxes[0]
        if self.emit_scores:
            return logits, logits.sigmoid(), boxes
        return logits, boxes


def simplify(path: Path, output: Path) -> None:
    try:
        import onnxslim
    except ImportError:
        print("[warn] onnxslim is not installed; skipping graph simplification.")
        return
    onnxslim.slim(str(path), str(output))


def pairwise_iou(boxes_a: torch.Tensor, boxes_b: torch.Tensor) -> torch.Tensor:
    """IoU between normalized cxcywh boxes."""

    def to_xyxy(boxes: torch.Tensor) -> torch.Tensor:
        half = boxes[:, 2:] / 2
        return torch.cat((boxes[:, :2] - half, boxes[:, :2] + half), dim=1)

    a = to_xyxy(boxes_a)
    b = to_xyxy(boxes_b)
    top_left = torch.maximum(a[:, None, :2], b[None, :, :2])
    bottom_right = torch.minimum(a[:, None, 2:], b[None, :, 2:])
    intersection = (bottom_right - top_left).clamp(min=0).prod(dim=-1)
    area_a = (a[:, 2:] - a[:, :2]).prod(dim=-1)
    area_b = (b[:, 2:] - b[:, :2]).prod(dim=-1)
    return intersection / (area_a[:, None] + area_b[None, :] - intersection + 1e-9)


def verify(
    reference: torch.nn.Module,
    onnx_path: Path,
    resolution: int,
    mean: torch.Tensor,
    std: torch.Tensor,
    folded: bool,
) -> None:
    import onnxruntime as ort

    torch.manual_seed(0)
    image_u8 = torch.randint(0, 256, (1, 3, resolution, resolution), dtype=torch.uint8)
    image = image_u8.float() / 255.0

    with torch.no_grad():
        normalized = (image - mean) / std
        expected = reference(pixel_values=normalized)
        expected_logits = expected.logits[0]
        expected_boxes = expected.pred_boxes[0]

    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    onnx_input = image.numpy() if folded else normalized.numpy()
    outputs = session.run(None, {session.get_inputs()[0].name: onnx_input})

    logits = torch.from_numpy(outputs[0])
    boxes = torch.from_numpy(outputs[-1])

    # Set-prediction models can permute low-confidence queries when tiny numeric
    # differences change TopK tie-breaking.  Compare the detection *set* (the
    # top-k scores/boxes), not query slot i against query slot i.
    expected_scores = expected_logits.sigmoid().reshape(-1)
    actual_scores = logits.sigmoid().reshape(-1)
    num_classes = expected_logits.shape[-1]
    k = min(100, expected_scores.numel())

    expected_top = torch.topk(expected_scores, k).indices
    actual_top = torch.topk(actual_scores, k).indices
    expected_boxes_top = expected_boxes[expected_top // num_classes]
    actual_boxes_top = boxes[actual_top // num_classes]

    score_error = float((expected_scores[expected_top] - actual_scores[actual_top]).abs().max())
    ious = pairwise_iou(expected_boxes_top, actual_boxes_top).max(dim=1).values
    mean_iou = float(ious.mean())
    matched_fraction = float((ious > 0.5).float().mean())
    slot_error = float((expected_logits.reshape(-1) - logits.reshape(-1)).abs().max())

    print(f"[check] top-{k} score max abs error: {score_error:.3e}")
    print(f"[check] top-{k} detection IoU: mean {mean_iou:.5f}, matched {matched_fraction:.1%}")
    print(f"[check] per-query logit max abs error: {slot_error:.3e} (informational)")

    if score_error > 0.02 or mean_iou < 0.95 or matched_fraction < 0.95:
        raise SystemExit("ONNX detections do not match PyTorch; refusing to call the export valid.")


def main() -> None:
    args = parse_args()
    model_dir = args.model_dir.expanduser().resolve()
    resolution = infer_resolution(model_dir, args.resolution)
    mean, std = get_normalization(model_dir)

    from transformers import RfDetrForObjectDetection

    print(f"[load] {model_dir}")
    model = RfDetrForObjectDetection.from_pretrained(str(model_dir), local_files_only=True).eval()
    reference = copy.deepcopy(model).eval() if args.check else None

    print(f"[model] {sum(p.numel() for p in model.parameters()) / 1e6:.2f} M parameters")
    print(
        f"[model] {model.config.num_queries} queries, {model.config.num_labels} classes, "
        f"input {resolution}x{resolution}"
    )

    if args.fold_normalization:
        print("[prepare] folding ImageNet normalization into patch embedding")
        fold_imagenet_normalization(model, mean, std)

    # Must happen after loading and folding: replaces the antialias bicubic
    # interpolation with a constant positional embedding for this resolution.
    print("[prepare] precomputing static DINOv2 position embedding")
    patch_static_position_embedding(model, resolution, resolution)

    wrapper = ExportWrapper(model, emit_scores=args.scores).eval()
    dummy = torch.randn(1, 3, resolution, resolution)

    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    raw_output = output
    if args.slim:
        raw_output = output.with_name(output.stem + ".raw.onnx")

    print(f"[export] writing {raw_output}")
    output_names = ["pred_logits", "pred_boxes"]
    if args.scores:
        output_names = ["pred_logits", "pred_scores", "pred_boxes"]

    with torch.no_grad():
        torch.onnx.export(
            wrapper,
            (dummy,),
            str(raw_output),
            input_names=["pixel_values"],
            output_names=output_names,
            opset_version=args.opset,
            dynamo=False,  # TorchScript exporter: stable with the DETR grid_sample ops
            do_constant_folding=True,
            dynamic_axes=None,
            export_params=True,
        )

    if args.slim:
        print(f"[slim] writing {output}")
        simplify(raw_output, output)
        raw_output.unlink(missing_ok=True)

    import onnx

    onnx_model = onnx.load(str(output))
    onnx.checker.check_model(onnx_model)
    print(f"[export] {output} ({os.path.getsize(output) / 1e6:.1f} MB), opset {args.opset}")
    print(
        "[input] pixel_values: "
        + str([d.dim_value for d in onnx_model.graph.input[0].type.tensor_type.shape.dim])
        + ("  RGB [0,1]" if args.fold_normalization else "  ImageNet-normalized RGB")
    )
    for graph_output in onnx_model.graph.output:
        print(
            f"[output] {graph_output.name}: "
            + str([d.dim_value for d in graph_output.type.tensor_type.shape.dim])
        )

    if args.check and reference is not None:
        verify(reference, output, resolution, mean, std, args.fold_normalization)

    print("\nDeepStream nvinfer preprocessing:")
    if args.fold_normalization:
        print("  net-scale-factor=0.00392156862745098   # 1/255")
        print("  offsets=0;0;0")
    else:
        print("  net-scale-factor=0.0173520736")
        print("  offsets=123.675;116.28;103.53")
        print("  # NOTE: this uses a common std of 0.226, not the exact per-channel ImageNet std.")
    print("  model-color-format=0                   # RGB")
    print("  maintain-aspect-ratio=0")
    print("  symmetric-padding=0")


if __name__ == "__main__":
    main()
