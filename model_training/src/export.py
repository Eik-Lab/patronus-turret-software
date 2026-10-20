import os

import numpy as np
import onnx
from onnx import TensorProto, numpy_helper


def export_fp16(onnx_fp32_path, output_path=None):
    """Convert FP32 ONNX weights to FP16 for faster edge inference.

    Converts model initializers (weights) to FP16 while keeping graph
    I/O as FP32 so TensorRT can fuse the cast operations automatically.
    Gives ~2x smaller file size with negligible accuracy loss.
    """
    if output_path is None:
        base, ext = os.path.splitext(onnx_fp32_path)
        output_path = f"{base}_fp16{ext}"

    model = onnx.load(onnx_fp32_path)

    for initializer in model.graph.initializer:
        if initializer.data_type == TensorProto.FLOAT:
            arr = numpy_helper.to_array(initializer).astype(np.float16)
            new = numpy_helper.from_array(arr, initializer.name)
            initializer.CopyFrom(new)

    onnx.save(model, output_path)

    size_mb = os.path.getsize(output_path) / (1024 * 1024)
    print(f"  FP16 ONNX exported to {output_path} ({size_mb:.1f} MB)")
