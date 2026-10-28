import sys

from src import train


def main(argv: list[str] | None = None) -> int:
    result = train.main(argv)
    if isinstance(result, dict):
        return 0
    return result if result is not None else 0


if __name__ == "__main__":
    raise SystemExit(main())
