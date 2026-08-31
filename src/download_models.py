#!/usr/bin/env python3
# /// script
# requires-python = "<=3.13"
# dependencies = [
#     "inference == 1.3.1",
# ]
# ///

import shutil
import sys
import tempfile

from inference.models.aliases import RFDETR_ALIASES
from inference_models.developer_tools import (
    get_model_from_provider,
    download_files_to_directory,
)
from pathlib import Path



def usage():
    print("Download RF-DETR ONNX models", file=sys.stderr)
    print("Usage: uv run ./download_weights.py <MODEL_ID>\nMODEL_ID:", file=sys.stderr)
    [print(f"- {key}", file=sys.stderr) for key in RFDETR_ALIASES.keys()]

if len(sys.argv) != 2:
    usage()
    sys.exit(1)

model_id = sys.argv[1]

if model_id not in RFDETR_ALIASES.keys():
    print(f'"{model_id}" is not a valid model', file=sys.stderr)
    usage()
    sys.exit(1)

dst = Path.cwd() / f"{model_id}.onnx"

if dst.exists():
    print(f"{dst} already exists. Can't overwrite", file=sys.stderr)
    sys.exit(1)

print(f"Downloading {model_id}...")

metadata = get_model_from_provider(
    model_id=model_id,
    provider="roboflow",
)

package = next(
    p
    for p in metadata.model_packages
    if p.onnx_package_details is not None
)

files = [
    (f.file_handle, f.download_url, f.md5_hash)
    for f in package.package_artefacts if f.file_handle.endswith(".onnx")
]

with tempfile.TemporaryDirectory() as tmpdir:
    paths = download_files_to_directory(
        target_dir=tmpdir,
        files_specs=files,
    )

    src = next(
        Path(path)
        for path in paths.values()
        if Path(path).suffix == ".onnx"
    )

    shutil.move(src, dst)

print(f"Successfully downloaded {dst}")
