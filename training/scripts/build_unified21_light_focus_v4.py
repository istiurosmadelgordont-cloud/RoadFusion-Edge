#!/usr/bin/env python3
"""Build the single-model Unified21 dataset from the Unified19 base.

The two new classes are red/green straight-arrow traffic lights. Existing
Unified19 labels are remapped explicitly so the non-light class IDs remain
semantically correct after inserting the new classes.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NAMES = [
    "pedestrian", "rider", "car", "bus", "truck", "motorcycle", "bicycle",
    "traffic_red_circle", "traffic_red_left", "traffic_red_right", "traffic_red_straight",
    "traffic_green_circle", "traffic_green_left", "traffic_green_right", "traffic_green_straight",
    "traffic_sign", "crosswalk", "guide_arrows", "traffic_cone", "roadworks_sign", "delineator",
]

# Unified19 -> Unified21. IDs 10-12 (green lights) shift by one; the original
# non-light tail shifts by two. IDs 10 and 14 are reserved for straight arrows.
OLD19_TO_NEW21 = {
    **{i: i for i in range(10)},
    10: 11, 11: 12, 12: 13,
    13: 15, 14: 16, 15: 17, 16: 18, 17: 19, 18: 20,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", type=Path, default=ROOT / "datasets/unified19_light_focus_v3")
    parser.add_argument("--output", type=Path, default=ROOT / "datasets/unified21_light_focus_v4")
    parser.add_argument("--old19-addition", type=Path, action="append", default=[])
    parser.add_argument("--new21-addition", type=Path, action="append", default=[])
    return parser.parse_args()


def link(source: Path, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(source, target)
    except (OSError, PermissionError):
        shutil.copy2(source, target)


def copy_split(source: Path, output: Path, split: str, prefix: str,
               mapping: dict[int, int] | None) -> tuple[int, Counter[int]]:
    count = 0
    classes: Counter[int] = Counter()
    image_dir = source / "images" / split
    label_dir = source / "labels" / split
    if not image_dir.exists():
        return count, classes
    for image in sorted(image_dir.glob("*.jpg")):
        label = label_dir / f"{image.stem}.txt"
        if not label.exists():
            raise FileNotFoundError(label)
        target_stem = f"{prefix}{image.stem}"
        target_image = output / "images" / split / f"{target_stem}.jpg"
        target_label = output / "labels" / split / f"{target_stem}.txt"
        if target_image.exists() or target_label.exists():
            raise RuntimeError(f"Output collision: {target_stem}")
        converted: list[str] = []
        for line in label.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            row = line.split()
            if len(row) != 5:
                raise ValueError(f"Invalid YOLO row in {label}: {line}")
            old_id = int(row[0])
            class_id = mapping[old_id] if mapping is not None else old_id
            if not 0 <= class_id < len(NAMES):
                raise ValueError(f"Invalid class ID {class_id} in {label}")
            row[0] = str(class_id)
            converted.append(" ".join(row))
            classes[class_id] += 1
        link(image, target_image)
        target_label.parent.mkdir(parents=True, exist_ok=True)
        target_label.write_text("\n".join(converted) + ("\n" if converted else ""), encoding="utf-8")
        count += 1
    return count, classes


def main() -> None:
    args = parse_args()
    if args.output.exists() and any(args.output.rglob("*")):
        raise RuntimeError(f"Output exists and is non-empty: {args.output}")
    sources: list[tuple[Path, str, dict[int, int] | None]] = [
        (args.base, "base19_", OLD19_TO_NEW21),
        *((path, f"old19_{index}_", OLD19_TO_NEW21) for index, path in enumerate(args.old19_addition)),
        *((path, f"new21_{index}_", None) for index, path in enumerate(args.new21_addition)),
    ]
    totals: Counter[int] = Counter()
    report_sources = []
    for source, prefix, mapping in sources:
        if not source.exists():
            raise FileNotFoundError(source)
        item = {"path": str(source), "source_schema": "Unified19" if mapping else "Unified21"}
        for split in ("train", "val"):
            images, classes = copy_split(source, args.output, split, prefix, mapping)
            totals.update(classes)
            item[f"{split}_images"] = images
            item[f"{split}_objects"] = {NAMES[k]: v for k, v in sorted(classes.items())}
        report_sources.append(item)

    missing = [NAMES[i] for i in range(len(NAMES)) if totals[i] == 0]
    config = ROOT / "configs/unified21_light_focus_v4.yaml"
    config.write_text(
        f"path: {args.output.resolve().as_posix()}\n"
        "train: images/train\nval: images/val\nnames:\n"
        + "".join(f"  {i}: {name}\n" for i, name in enumerate(NAMES)),
        encoding="utf-8",
    )
    report = {
        "output": str(args.output),
        "policy": "single 21-class model; circle and straight-arrow signals are distinct",
        "old19_to_new21": OLD19_TO_NEW21,
        "sources": report_sources,
        "total_objects": {NAMES[k]: v for k, v in sorted(totals.items())},
        "missing_classes": missing,
        "config": str(config),
    }
    report_path = ROOT / "reports/unified21_light_focus_v4_build.json"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if missing:
        raise RuntimeError(f"Classes without annotations: {missing}")


if __name__ == "__main__":
    main()
