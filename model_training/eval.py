#!/usr/bin/env python3
import argparse
import json
import os
import sys
from pathlib import Path

DATASETS = [
]

def _detect_model_type(model_path):
    p = Path(model_path)
    if p.suffix in ('.pt', '.onnx'):
        return 'yolo'
    if p.is_dir():
        if (p / 'config.json').exists():
            try:
                cfg = json.loads((p / 'config.json').read_text())
                if 'rfdetr' in str(cfg).lower():
                    return 'rfdetr'
            except Exception:
                pass
            return 'hf'
    return 'auto'

def _detect_resolution(model_path):
    return 640

def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--model_path', required=True)
    args = parser.parse_args(argv)
    model_path = args.model_path
    model_type = _detect_model_type(model_path)
    resolution = _detect_resolution(model_path)
    if not DATASETS:
        print('no datasets hardcoded in eval.py')
        return 1
    try:
        from src.eval import run_eval
    except Exception:
        try:
            from eval import run_eval
        except Exception:
            print('eval backend missing')
            return 1
    return run_eval(model_path=model_path, model_type=model_type, resolution=resolution, datasets=DATASETS)

if __name__ == '__main__':
    raise SystemExit(main())
