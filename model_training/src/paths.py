import os
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent

RUNS_DIR = PROJECT_ROOT / "runs"

IMAGE_EXTENSIONS = frozenset({".jpg", ".jpeg", ".png", ".bmp", ".webp"})


def timestamp() -> str:
    return datetime.now().strftime("%Y%m%d-%H%M%S")


def resolve_path(value, base: Path = PROJECT_ROOT) -> Path:
    path = Path(os.path.expandvars(str(value))).expanduser()
    if not path.is_absolute():
        path = base / path
    return path.resolve()


def resolve_run_dir(configured, force: bool = False, label: str | None = None) -> Path:
    path = resolve_path(configured)
    if force or not path.exists():
        return path
    try:
        if not any(path.iterdir()):
            return path
    except OSError:
        return path

    name = label or path.name
    candidate = path.with_name(f"{path.name}-{timestamp()}")
    counter = 2
    while candidate.exists():
        candidate = path.with_name(f"{path.name}-{timestamp()}-{counter}")
        counter += 1
    print(
        f"[run] {name}: {path} already exists; writing to {candidate} "
        "(pass --force to reuse the existing directory)"
    )
    return candidate


def split_image_dir(split: str) -> str:
    return f"{split}/images"


def split_annotation_dir(split: str) -> str:
    return f"{split}/annotations.json"


DATA_DIR = PROJECT_ROOT / "data"
