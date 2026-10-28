'Shared object-size buckets.'


from __future__ import annotations

TINY_MAX_PCT = 0.5
SMALL_MAX_PCT = 2.0
MEDIUM_MAX_PCT = 6.0

SIZE_BUCKETS = ("tiny", "small", "medium", "large")

_SIZE_LABELS = {
    "tiny": f"tiny (<{TINY_MAX_PCT}%)",
    "small": f"small ({TINY_MAX_PCT}-{SMALL_MAX_PCT}%)",
    "medium": f"medium ({SMALL_MAX_PCT}-{MEDIUM_MAX_PCT}%)",
    "large": f"large (>={MEDIUM_MAX_PCT}%)",
}


def size_bucket(area: float, image_area: float) -> str | None:
    if image_area <= 0 or area <= 0:
        return None
    pct = 100.0 * area / image_area
    if pct < TINY_MAX_PCT:
        return "tiny"
    if pct < SMALL_MAX_PCT:
        return "small"
    if pct < MEDIUM_MAX_PCT:
        return "medium"
    return "large"


def size_label(bucket: str) -> str:
    return _SIZE_LABELS[bucket]


def empty_counts() -> dict[str, int]:
    return {bucket: 0 for bucket in SIZE_BUCKETS}
