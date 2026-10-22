import argparse
import json
import math
import re
import shutil
import zipfile

from pathlib import Path, PurePosixPath

from huggingface_hub import HfApi, hf_hub_download
from PIL import Image
from tqdm import tqdm


# ============================================================
# CONFIGURATION
# ============================================================

REPO_ID = "lgrzybowski/seraphim-drone-detection-dataset"

LOCAL_DIR = Path("./repository_location")
DOWNLOAD_DIR = LOCAL_DIR / "downloads"
EXTRACT_DIR = LOCAL_DIR / "extracted"

COCO_OUTPUT_DIR = LOCAL_DIR / "option_A_half_total"
YOLO_OUTPUT_DIR = Path("./data/drone_yolo")

parser = argparse.ArgumentParser(
    description="Download the Seraphim dataset and convert it."
)
parser.add_argument(
    "--format",
    choices=["coco", "yolo"],
    default="yolo",
    help="output annotation format (default: yolo)",
)
ARGS = parser.parse_args()

OUTPUT_DIR = (
    COCO_OUTPUT_DIR if ARGS.format == "coco" else YOLO_OUTPUT_DIR
)

TRAIN_TOTAL = 75134
TEST_TOTAL = 8349

TRAIN_TARGET = TRAIN_TOTAL // 2
TEST_TARGET = TEST_TOTAL // 2

IMAGE_EXTENSIONS = {
    ".jpg", ".jpeg", ".png", ".bmp", ".webp"
}

CATEGORY = {
    "id": 1,
    "name": "drone",
    "supercategory": "object",
}

for directory in [
    DOWNLOAD_DIR,
    EXTRACT_DIR,
    OUTPUT_DIR,
]:
    directory.mkdir(parents=True, exist_ok=True)


# ============================================================
# STEP 1: INSPECT THIS DATASET'S REPOSITORY
# ============================================================

print("Inspecting Hugging Face repository...")

api = HfApi()

repo_files = api.list_repo_files(
    repo_id=REPO_ID,
    repo_type="dataset",
)

# Find the image and label ZIP files in the four dataset folders.
zip_files = [
    path for path in repo_files
    if re.fullmatch(
        r"(train|test)/(images|labels)/batch_\d+\.zip",
        path,
    )
]

batches = {
    "train": {"images": [], "labels": []},
    "test": {"images": [], "labels": []},
}

for path in zip_files:
    parts = PurePosixPath(path).parts
    split, kind = parts[0], parts[1]
    batches[split][kind].append(path)

for split in ("train", "test"):
    for kind in ("images", "labels"):
        batches[split][kind].sort()

        print(
            f"{split}/{kind}: "
            f"{len(batches[split][kind])} ZIP archive(s)"
        )

if not batches["train"]["images"]:
    raise RuntimeError("No training image ZIP archives found.")

if not batches["train"]["labels"]:
    raise RuntimeError("No training label ZIP archives found.")

if not batches["test"]["images"]:
    raise RuntimeError("No test image ZIP archives found.")

if not batches["test"]["labels"]:
    raise RuntimeError("No test label ZIP archives found.")


# ============================================================
# STEP 2: SELECT IMAGE BATCHES AND DOWNLOAD LABELS
# ============================================================

def download_and_extract(repo_path):
    """Download one archive and extract it into its dataset folder."""

    local_zip = Path(
        hf_hub_download(
            repo_id=REPO_ID,
            repo_type="dataset",
            filename=repo_path,
            local_dir=DOWNLOAD_DIR,
        )
    )

    parts = PurePosixPath(repo_path).parts
    split, kind = parts[0], parts[1]

    destination = EXTRACT_DIR / split / kind
    destination.mkdir(parents=True, exist_ok=True)

    with zipfile.ZipFile(local_zip, "r") as archive:
        archive.extractall(destination)

    print(f"Extracted {repo_path}")

    return local_zip


# Choose approximately half of the TRAIN IMAGE ARCHIVES.
# The dataset stores images in ZIP batches.
train_image_archives = batches["train"]["images"]

number_to_download = max(
    1,
    math.ceil(len(train_image_archives) / 2),
)

selected_train_archives = train_image_archives[
    :number_to_download
]

print("\nDownloading selected training image batches...")

for path in selected_train_archives:
    download_and_extract(path)

# Labels are small compared with the images, so download all
# label archives to ensure annotations can be matched.
print("\nDownloading training labels...")

for path in batches["train"]["labels"]:
    download_and_extract(path)

# The test set may consist of one or more image archives.
# Download all of them so we can select half of the test images.
print("\nDownloading test images...")

for path in batches["test"]["images"]:
    download_and_extract(path)

print("\nDownloading test labels...")

for path in batches["test"]["labels"]:
    download_and_extract(path)


# ============================================================
# STEP 3: MATCH IMAGES WITH YOLO LABELS
# ============================================================

def collect_pairs(split):
    image_dir = EXTRACT_DIR / split / "images"
    label_dir = EXTRACT_DIR / split / "labels"

    images = sorted(
        path
        for path in image_dir.rglob("*")
        if path.is_file()
        and path.suffix.lower() in IMAGE_EXTENSIONS
    )

    # The dataset's labels are YOLO .txt files.
    labels = {}

    for path in label_dir.rglob("*.txt"):
        if path.stem in labels:
            raise RuntimeError(
                f"Duplicate label filename: {path.stem}"
            )

        labels[path.stem] = path

    pairs = [
        (image, labels[image.stem])
        for image in images
        if image.stem in labels
    ]

    print(
        f"{split}: {len(images)} extracted images; "
        f"{len(pairs)} images with matching labels"
    )

    return pairs


train_pairs = collect_pairs("train")
test_pairs = collect_pairs("test")

if len(train_pairs) < TRAIN_TARGET:
    print(
        "\nWARNING: The selected training batches contain "
        f"only {len(train_pairs)} matched images. "
        f"The target is {TRAIN_TARGET}. "
        "The output will contain fewer than half the "
        "training images."
    )

if len(test_pairs) < TEST_TARGET:
    raise RuntimeError(
        f"Only {len(test_pairs)} matched test images found; "
        f"expected at least {TEST_TARGET}. "
        "Check the archive layout and label matching."
    )

# Select approximately half of each original split.
# The sorted order makes the selection reproducible.
train_pairs = train_pairs[:TRAIN_TARGET]
test_pairs = test_pairs[:TEST_TARGET]


# ============================================================
# STEP 4: CONVERT YOLO ANNOTATIONS TO COCO
# ============================================================

def convert_to_coco(pairs, split):
    output_split = OUTPUT_DIR / split
    image_output = output_split / "images"

    image_output.mkdir(parents=True, exist_ok=True)

    coco = {
        "info": {
            "description": (
                f"Seraphim Drone Detection Dataset - {split}"
            ),
            "version": "1.0",
        },
        "licenses": [
            {
                "id": 1,
                "name": "CC BY 4.0",
                "url": (
                    "https://creativecommons.org/licenses/by/4.0/"
                ),
            }
        ],
        "categories": [CATEGORY],
        "images": [],
        "annotations": [],
    }

    annotation_id = 1
    valid_image_id = 1

    for image_path, label_path in tqdm(
        pairs,
        desc=f"Converting {split}",
    ):
        try:
            with Image.open(image_path) as image:
                width, height = image.size
                image.verify()

        except Exception as error:
            print(f"Skipping invalid image {image_path}: {error}")
            continue

        # Use unique filenames in the COCO output.
        filename = (
            f"{valid_image_id:07d}"
            f"{image_path.suffix.lower()}"
        )

        destination = image_output / filename

        shutil.copy2(image_path, destination)

        coco["images"].append({
            "id": valid_image_id,
            "file_name": f"images/{filename}",
            "width": width,
            "height": height,
            "license": 1,
        })

        with open(label_path, "r", encoding="utf-8") as file:
            for line in file:
                parts = line.strip().split()

                if not parts:
                    continue

                if len(parts) != 5:
                    print(
                        f"Skipping malformed annotation: "
                        f"{label_path}"
                    )
                    continue

                try:
                    class_id = int(parts[0])

                    x_center, y_center, box_width, box_height = (
                        float(value)
                        for value in parts[1:]
                    )

                except ValueError:
                    continue

                # This repository defines only one class:
                # YOLO class 0 = drone.
                if class_id != 0:
                    continue

                # Convert normalized YOLO coordinates to pixels.
                x_min = (x_center - box_width / 2) * width
                y_min = (y_center - box_height / 2) * height

                x_max = x_min + box_width * width
                y_max = y_min + box_height * height

                # Clip the bounding box to the image.
                x_min = max(0.0, min(x_min, float(width)))
                y_min = max(0.0, min(y_min, float(height)))
                x_max = max(0.0, min(x_max, float(width)))
                y_max = max(0.0, min(y_max, float(height)))

                coco_width = x_max - x_min
                coco_height = y_max - y_min

                if coco_width <= 0 or coco_height <= 0:
                    continue

                coco["annotations"].append({
                    "id": annotation_id,
                    "image_id": valid_image_id,
                    "category_id": 1,
                    "bbox": [
                        round(x_min, 3),
                        round(y_min, 3),
                        round(coco_width, 3),
                        round(coco_height, 3),
                    ],
                    "area": round(
                        coco_width * coco_height, 3
                    ),
                    "iscrowd": 0,
                    "segmentation": [],
                })

                annotation_id += 1

        valid_image_id += 1

    annotation_path = output_split / "annotations.json"

    with open(annotation_path, "w", encoding="utf-8") as file:
        json.dump(coco, file, indent=2)

    print(f"\nCompleted {split}")
    print(f"Images: {len(coco['images'])}")
    print(f"Annotations: {len(coco['annotations'])}")
    print(f"Annotations saved to: {annotation_path}")

    return coco


# ============================================================
# STEP 4b: COPY YOLO ANNOTATIONS (YOLO OUTPUT)
# ============================================================

def convert_to_yolo(pairs, split):
    """Write images/<split> and labels/<split>; returns counts."""
    image_output = OUTPUT_DIR / "images" / split
    label_output = OUTPUT_DIR / "labels" / split

    image_output.mkdir(parents=True, exist_ok=True)
    label_output.mkdir(parents=True, exist_ok=True)

    num_images = 0
    num_annotations = 0

    for image_path, label_path in tqdm(
        pairs,
        desc=f"Copying {split}",
    ):
        try:
            with Image.open(image_path) as image:
                image.verify()

        except Exception as error:
            print(f"Skipping invalid image {image_path}: {error}")
            continue

        filename = f"{num_images + 1:07d}"

        shutil.copy2(
            image_path,
            image_output / (filename + image_path.suffix.lower()),
        )

        # Keep only well-formed class 0 (drone) boxes.
        lines = []

        with open(label_path, "r", encoding="utf-8") as file:
            for line in file:
                parts = line.strip().split()

                if len(parts) != 5 or parts[0] != "0":
                    continue

                try:
                    values = [float(value) for value in parts[1:]]
                except ValueError:
                    continue

                if values[2] <= 0 or values[3] <= 0:
                    continue

                lines.append(line.strip())

        (label_output / (filename + ".txt")).write_text(
            "\n".join(lines) + ("\n" if lines else ""),
            encoding="utf-8",
        )

        num_images += 1
        num_annotations += len(lines)

    print(f"\nCompleted {split}")
    print(f"Images: {num_images}")
    print(f"Annotations: {num_annotations}")

    return {"images": num_images, "annotations": num_annotations}


# ============================================================
# STEP 5: GENERATE OPTION A
# ============================================================

if ARGS.format == "coco":
    print("\nConverting the selected training images...")

    train_coco = convert_to_coco(train_pairs, "train")

    print("\nConverting the selected test images...")

    test_coco = convert_to_coco(test_pairs, "test")

    with open(
        OUTPUT_DIR / "categories.json",
        "w",
        encoding="utf-8",
    ) as file:
        json.dump([CATEGORY], file, indent=2)

    train_stats = {
        "images": len(train_coco["images"]),
        "annotations": len(train_coco["annotations"]),
    }
    test_stats = {
        "images": len(test_coco["images"]),
        "annotations": len(test_coco["annotations"]),
    }

else:
    print("\nCopying the selected training images...")

    train_stats = convert_to_yolo(train_pairs, "train")

    print("\nCopying the selected test images (as val)...")

    test_stats = convert_to_yolo(test_pairs, "val")

    with open(OUTPUT_DIR / "data.yaml", "w", encoding="utf-8") as file:
        file.write(
            f"path: {OUTPUT_DIR.resolve()}\n"
            "train: images/train\n"
            "val: images/val\n"
            "names:\n"
            f"  0: {CATEGORY['name']}\n"
        )


# ============================================================
# STEP 6: SUMMARY
# ============================================================

total_images = train_stats["images"] + test_stats["images"]

print("\n" + "=" * 60)
print(f"OPTION A: HALF OF THE ENTIRE DATASET ({ARGS.format})")
print("=" * 60)

print(f"Output directory: {OUTPUT_DIR.resolve()}")
print(f"Training images: {train_stats['images']}")
print(f"Test images: {test_stats['images']}")
print(f"Total images: {total_images}")

print(f"Training annotations: {train_stats['annotations']}")
print(f"Test annotations: {test_stats['annotations']}")

print("\nConversion completed.")
