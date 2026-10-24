import json
import os
from pathlib import Path

SRC = Path("./repository_location/option_A_half_total")
DST = Path("./data/drone_yolo")


def convert_split(split):
    with open(SRC / split / "annotations.json") as f:
        coco = json.load(f)

    cat_ids = sorted(c["id"] for c in coco["categories"])
    cat_index = {cid: i for i, cid in enumerate(cat_ids)}

    img_dir = DST / "images" / split
    lbl_dir = DST / "labels" / split
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)

    boxes = {}
    for ann in coco["annotations"]:
        boxes.setdefault(ann["image_id"], []).append(ann)

    for img in coco["images"]:
        src = (SRC / split / img["file_name"]).resolve()
        name = Path(img["file_name"]).name
        link = img_dir / name
        if not link.exists():
            os.symlink(src, link)

        w, h = img["width"], img["height"]
        lines = []
        for ann in boxes.get(img["id"], []):
            x, y, bw, bh = ann["bbox"]
            lines.append(
                f"{cat_index[ann['category_id']]} "
                f"{(x + bw / 2) / w:.6f} {(y + bh / 2) / h:.6f} {bw / w:.6f} {bh / h:.6f}"
            )
        (lbl_dir / f"{Path(name).stem}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))

    return {cat_index[c["id"]]: c["name"] for c in coco["categories"]}


def main():
    names = {}
    for split in ("train", "val"):
        names = convert_split(split)
        print(f"{split}: done")

    with open(DST / "data.yaml", "w") as f:
        f.write(f"path: {DST.resolve()}\ntrain: images/train\nval: images/val\nnames:\n")
        for i, n in sorted(names.items()):
            f.write(f"  {i}: {n}\n")


if __name__ == "__main__":
    main()
