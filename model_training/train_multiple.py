#!/usr/bin/env python3
"""Train each configured model sequentially."""

from __future__ import annotations

import argparse
import sys

from src.config import ConfigError, load_config
from src.trainer import GPUUnavailableError, train


def main(argv: list[str] | None = None) -> int:
    command = argparse.ArgumentParser(description="Train multiple models in sequence (GPU required).")
    command.add_argument("configs", nargs="+", help="YAML or JSON training configs")
    command.add_argument("--force", action="store_true", help="Reuse existing output directories")
    args = command.parse_args(argv)
    for path in args.configs:
        try:
            config = load_config(path)
        except (ConfigError, OSError, ValueError) as exc:
            command.error(f"{path}: {exc}")
        print(f"[train] config: {path}")
        try:
            train(config, force=args.force)
        except GPUUnavailableError as exc:
            print(f"error: {exc}", file=sys.stderr)
            if argv is None:
                sys.exit(1)
            return 1
    return 0


if __name__ == "__main__":
    main()
