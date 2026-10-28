from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

from src.data.discovery import inspect_dataset, prepare_dataset


def main(argv=None):
    parser = argparse.ArgumentParser(description="Prepare dataset")
    parser.add_argument("data", nargs="+")
    parser.add_argument("--val-fraction", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", help="Output directory")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--copy", action="store_true")
    args = parser.parse_args(argv)

    info = inspect_dataset(args.data, args.val_fraction, args.seed)
    if isinstance(info, dict):
        items = info.get("datasets") or info
    else:
        items = info
    if isinstance(items, list):
        for item in items:
            if isinstance(item, dict):
                for k, v in item.items():
                    print(f"{k}: {v}")
            elif isinstance(item, list):
                for sub in item:
                    print(sub)
            else:
                print(item)

    output = args.output
    if not output and len(args.data) == 1:
        output = str(Path(args.data[0]) / "prepared")

    if output and len(args.data) == 1:
        summary = prepare_dataset(args.data[0], output, val_fraction=args.val_fraction, seed=args.seed, copy=args.copy)
        if summary:
            print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
