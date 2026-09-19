"""Build a class-balanced, 640px letterboxed INT8 calibration set from V7 train only."""

from __future__ import annotations

import argparse
import hashlib
import json
import random
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATASET = ROOT.parent / "adas_training/datasets/unified21_p2_v7_clean_ccf"
LIGHT_CLASSES = set(range(7, 15))


def read_classes(label: Path) -> set[int]:
    found = set()
    for line in label.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if fields:
            class_id = int(fields[0])
            if not 0 <= class_id < 21:
                raise ValueError(f"Unexpected class {class_id}: {label}")
            found.add(class_id)
    return found


def letterbox(image: np.ndarray, size: int) -> np.ndarray:
    height, width = image.shape[:2]
    scale = min(size / width, size / height)
    target_w = min(size, int(width * scale + 0.5))
    target_h = min(size, int(height * scale + 0.5))
    pad_x = (size - target_w) // 2
    pad_y = (size - target_h) // 2
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    resized = cv2.resize(image, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
    canvas[pad_y : pad_y + target_h, pad_x : pad_x + target_w] = resized
    return canvas


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "calibration/v7")
    parser.add_argument("--count", type=int, default=400)
    parser.add_argument("--size", type=int, default=640)
    parser.add_argument("--seed", type=int, default=3568)
    args = parser.parse_args()
    if args.count < 21 or args.size < 32:
        raise ValueError("Count must cover 21 classes and size must be at least 32")

    image_root = args.dataset / "images/train"
    label_root = args.dataset / "labels/train"
    images = sorted(p for p in image_root.iterdir() if p.suffix.lower() in {".jpg", ".jpeg", ".png"})
    by_class: dict[int, list[Path]] = defaultdict(list)
    class_map: dict[Path, set[int]] = {}
    for image in images:
        label = label_root / f"{image.stem}.txt"
        if not label.is_file():
            raise FileNotFoundError(label)
        classes = read_classes(label)
        class_map[image] = classes
        for class_id in classes:
            by_class[class_id].append(image)
    if any(not by_class[class_id] for class_id in range(21)):
        raise RuntimeError("At least one of 21 classes is absent from training data")

    rng = random.Random(args.seed)
    chosen: list[Path] = []
    seen: set[Path] = set()
    counts: Counter[int] = Counter()
    targets = {class_id: (20 if class_id in LIGHT_CLASSES else 8) for class_id in range(21)}
    for class_id in sorted(targets, key=lambda c: len(by_class[c])):
        candidates = by_class[class_id][:]
        rng.shuffle(candidates)
        for image in candidates:
            if counts[class_id] >= targets[class_id] or len(chosen) >= args.count:
                break
            if image in seen:
                continue
            chosen.append(image)
            seen.add(image)
            counts.update(class_map[image])
    if any(counts[class_id] < targets[class_id] for class_id in range(21)):
        raise RuntimeError(f"Could not meet calibration quotas: {dict(counts)}")
    remainder = [image for image in images if image not in seen]
    rng.shuffle(remainder)
    chosen.extend(remainder[: args.count - len(chosen)])
    if len(chosen) != args.count:
        raise RuntimeError(f"Expected {args.count} calibration images, got {len(chosen)}")

    image_out = args.output_dir / "images"
    image_out.mkdir(parents=True, exist_ok=True)
    manifest = []
    for index, image in enumerate(chosen):
        pixels = cv2.imread(str(image), cv2.IMREAD_COLOR)
        if pixels is None:
            raise RuntimeError(f"Cannot decode {image}")
        digest = hashlib.sha256(image.name.encode("utf-8")).hexdigest()[:8]
        target = image_out / f"{index:04d}_{digest}.jpg"
        if not cv2.imwrite(str(target), letterbox(pixels, args.size), [cv2.IMWRITE_JPEG_QUALITY, 94]):
            raise RuntimeError(f"Cannot write {target}")
        manifest.append({"source": str(image), "image": str(target), "classes": sorted(class_map[image])})
    (args.output_dir / "dataset.txt").write_text(
        "".join(f"images/{Path(row['image']).name}\n" for row in manifest), encoding="utf-8"
    )
    coverage = Counter(class_id for row in manifest for class_id in row["classes"])
    (args.output_dir / "manifest.json").write_text(
        json.dumps({"source": str(args.dataset), "count": len(manifest), "size": args.size,
                    "class_image_counts": dict(sorted(coverage.items())), "images": manifest}, indent=2),
        encoding="utf-8",
    )
    print(json.dumps({"selected": len(manifest), "coverage": dict(sorted(coverage.items())),
                      "dataset": str(args.output_dir / "dataset.txt")}))


if __name__ == "__main__":
    main()
