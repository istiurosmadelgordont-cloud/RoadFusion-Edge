#!/usr/bin/env python3
"""Convert ATLAS still images into isolated eight-light YOLO additions.

Only the eight exact state+pictogram classes used by the 21-class detector are
exported.  Unsupported traffic-light boxes are neutral-masked inside context
crops so they are not learned as background.  No video input is supported.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
TARGET_NAMES = {
    "circle_red": (7, "traffic_red_circle"),
    "arrow_left_red": (8, "traffic_red_left"),
    "arrow_right_red": (9, "traffic_red_right"),
    "arrow_straight_red": (10, "traffic_red_straight"),
    "circle_green": (11, "traffic_green_circle"),
    "arrow_left_green": (12, "traffic_green_left"),
    "arrow_right_green": (13, "traffic_green_right"),
    "arrow_straight_green": (14, "traffic_green_straight"),
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--source", type=Path, default=ROOT / "incoming/atlas/sample/ATLAS")
    p.add_argument("--output", type=Path, default=ROOT / "datasets/atlas_sample_eight_lights")
    p.add_argument("--context", type=float, default=18.0)
    p.add_argument("--min-side", type=int, default=480)
    p.add_argument("--max-train-per-class", type=int, default=1500)
    p.add_argument("--max-val-per-class", type=int, default=300)
    return p.parse_args()


def read_image(path: Path) -> np.ndarray | None:
    try:
        return cv2.imdecode(np.fromfile(str(path), np.uint8), cv2.IMREAD_COLOR)
    except (OSError, ValueError):
        return None


def write_image(path: Path, image: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, data = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 96])
    if not ok:
        raise RuntimeError(f"Could not encode {path}")
    data.tofile(str(path))


def context_crop(box: tuple[float, float, float, float], width: int, height: int,
                 scale: float, min_side: int) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = box
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    side = int(round(max(max(x2 - x1, y2 - y1) * scale, min_side)))
    side = min(side, width, height)
    left = int(round(cx - side / 2))
    top = int(round(cy - side / 2))
    left = min(max(0, left), width - side)
    top = min(max(0, top), height - side)
    return left, top, left + side, top + side


def intersects(box: tuple[float, float, float, float], crop: tuple[int, int, int, int]) -> bool:
    x1, y1, x2, y2 = box
    cx1, cy1, cx2, cy2 = crop
    return min(x2, cx2) > max(x1, cx1) and min(y2, cy2) > max(y1, cy1)


def yolo_box(line: str, width: int, height: int) -> tuple[int, tuple[float, float, float, float]]:
    values = line.split()
    class_id = int(values[0])
    xc, yc, bw, bh = map(float, values[1:5])
    return class_id, (
        (xc - bw / 2) * width,
        (yc - bh / 2) * height,
        (xc + bw / 2) * width,
        (yc + bh / 2) * height,
    )


def encode(class_id: int, box: tuple[float, float, float, float],
           crop: tuple[int, int, int, int]) -> str:
    x1, y1, x2, y2 = box
    cx1, cy1, cx2, cy2 = crop
    width, height = cx2 - cx1, cy2 - cy1
    x1, x2 = max(cx1, x1), min(cx2, x2)
    y1, y2 = max(cy1, y1), min(cy2, y2)
    xc = ((x1 + x2) / 2 - cx1) / width
    yc = ((y1 + y2) / 2 - cy1) / height
    bw = (x2 - x1) / width
    bh = (y2 - y1) / height
    return f"{class_id} {xc:.8f} {yc:.8f} {bw:.8f} {bh:.8f}"


def find_image(label_path: Path) -> Path | None:
    candidate_dir = label_path.parent.parent / "images"
    for suffix in (".jpg", ".jpeg", ".png"):
        candidate = candidate_dir / f"{label_path.stem}{suffix}"
        if candidate.exists():
            return candidate
    return None


def main() -> None:
    args = parse_args()
    classes_path = args.source / "ATLAS_classes.yaml"
    names_doc = yaml.safe_load(classes_path.read_text(encoding="utf-8"))["names"]
    source_names = {int(k): v for k, v in names_doc.items()}
    id_mapping = {source_id: TARGET_NAMES[name] for source_id, name in source_names.items() if name in TARGET_NAMES}

    counts: Counter[str] = Counter()
    skipped: Counter[str] = Counter()
    records: list[dict] = []
    label_files = sorted(p for p in args.source.rglob("*.txt") if p.parent.name == "labels")
    candidates: dict[tuple[str, int], list[tuple[str, int]]] = defaultdict(list)
    for label_path in label_files:
        split = "val" if "test" in label_path.parts else "train"
        object_index = -1
        for line in label_path.read_text().splitlines():
            if not line.strip():
                continue
            object_index += 1
            source_id = int(line.split()[0])
            if source_id not in id_mapping:
                continue
            key = (str(label_path), object_index)
            score = hashlib.sha1(f"3568:{label_path}:{object_index}".encode()).hexdigest()
            candidates[split, source_id].append((score, key))
    selected: set[tuple[str, int]] = set()
    selection_counts: Counter[str] = Counter()
    for (split, source_id), items in candidates.items():
        limit = args.max_val_per_class if split == "val" else args.max_train_per_class
        chosen = sorted(items)[:limit] if limit > 0 else items
        selected.update(key for _, key in chosen)
        selection_counts[f"{split}:{id_mapping[source_id][1]}"] = len(chosen)

    for label_path in label_files:
        image_path = find_image(label_path)
        if image_path is None:
            skipped["missing_image"] += 1
            continue
        image = read_image(image_path)
        if image is None:
            skipped["unreadable_image"] += 1
            continue
        height, width = image.shape[:2]
        objects = [yolo_box(line, width, height) for line in label_path.read_text().splitlines() if line.strip()]
        mapped_objects = [(object_index, src_id, box) for object_index, (src_id, box) in enumerate(objects) if src_id in id_mapping]
        anchors = [item for item in mapped_objects if (str(label_path), item[0]) in selected]
        if not anchors:
            skipped["no_eight_light_target"] += 1
            continue

        split = "val" if "test" in label_path.parts else "train"
        for object_index, _, anchor_box in anchors:
            crop = context_crop(anchor_box, width, height, args.context, args.min_side)
            safe = image.copy()
            unsupported = 0
            for src_id, box in objects:
                if src_id in id_mapping or not intersects(box, crop):
                    continue
                x1, y1, x2, y2 = map(lambda x: int(round(x)), box)
                pad = max(2, int(round(max(x2 - x1, y2 - y1) * 0.08)))
                cv2.rectangle(safe, (max(0, x1-pad), max(0, y1-pad)),
                              (min(width-1, x2+pad), min(height-1, y2+pad)), (114, 114, 114), -1)
                unsupported += 1

            crop_targets = [(src_id, box) for _, src_id, box in mapped_objects if intersects(box, crop)]
            cx1, cy1, cx2, cy2 = crop
            cropped = safe[cy1:cy2, cx1:cx2]
            digest = hashlib.sha1(f"{image_path}:{object_index}".encode()).hexdigest()[:12]
            stem = f"atlas_{image_path.stem}_{object_index}_{digest}"
            output_image = args.output / f"images/{split}/{stem}.jpg"
            output_label = args.output / f"labels/{split}/{stem}.txt"
            write_image(output_image, cropped)
            output_label.parent.mkdir(parents=True, exist_ok=True)
            output_label.write_text(
                "\n".join(encode(id_mapping[src_id][0], box, crop) for src_id, box in crop_targets) + "\n",
                encoding="utf-8",
            )
            for src_id, _ in crop_targets:
                counts[f"{split}:{id_mapping[src_id][1]}"] += 1
            skipped["masked_unsupported_lights"] += unsupported
            records.append({
                "split": split,
                "source": str(image_path),
                "output": str(output_image),
                "targets": len(crop_targets),
                "masked": unsupported,
                "camera": label_path.parent.parent.name,
            })

    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "manifest.jsonl").write_text(
        "".join(json.dumps(item, ensure_ascii=False) + "\n" for item in records), encoding="utf-8"
    )
    report = {
        "source": str(args.source),
        "license": "CC BY-NC-SA 4.0",
        "source_policy": "public ATLAS still images only; no local videos",
        "mapping": {source_names[k]: {"source_id": k, "target_id": v[0], "target_name": v[1]} for k, v in id_mapping.items()},
        "label_files": len(label_files),
        "selected_anchors": dict(selection_counts),
        "exported_images": len(records),
        "object_counts": dict(counts),
        "skipped": dict(skipped),
        "crop": {"context": args.context, "min_side": args.min_side},
        "selection": {
            "method": "deterministic SHA1 ranking per split and class",
            "max_train_per_class": args.max_train_per_class,
            "max_val_per_class": args.max_val_per_class,
        },
    }
    (args.output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
