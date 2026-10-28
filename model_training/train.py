#!/usr/bin/env python3
"""Configure and launch a single training run."""

from __future__ import annotations

import argparse
import sys

from src.config import ConfigError, load_config
from src.trainer import GPUUnavailableError, train


def parser() -> argparse.ArgumentParser:
    command = argparse.ArgumentParser(description="Train one detection model (GPU required).")
    command.add_argument("config", help="YAML or JSON training config")
    command.add_argument("--force", action="store_true", help="Reuse the existing output directory")
    return command


def main(argv: list[str] | None = None) -> int:
    command = parser()
    args = command.parse_args(argv)
    try:
        config = load_config(args.config)
    except (ConfigError, OSError, ValueError) as exc:
        command.error(str(exc))
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
