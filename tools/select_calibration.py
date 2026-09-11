#!/usr/bin/env python3
"""Select a deterministic, class-balanced RKNN INT8 calibration set."""

import argparse
import random
import shutil
from collections import defaultdict
from pathlib import Path


IMPORTANT = (0, 2, 7, 8, 9, 12, 13)


def parse_args():
    root = Path(__file__).resolve().parents[1]
    p = argparse.ArgumentParser()
    p.add_argument("--dataset", type=Path, default=root.parent / "adas_training/datasets/unified17_v10_anchor")
    p.add_argument("--count", type=int, default=300)
    p.add_argument("--seed", type=int, default=3568)
    p.add_argument("--output-dir", type=Path, default=root / "calibration/images")
    p.add_argument("--list", type=Path, default=root / "calibration/dataset.txt")
    return p.parse_args()


def labels_for(image, images_root, labels_root):
    rel = image.relative_to(images_root)
    label = labels_root / rel.with_suffix(".txt")
    classes = set()
    if label.exists():
        for line in label.read_text(encoding="utf-8", errors="ignore").splitlines():
            parts = line.split()
            if parts:
                try:
                    classes.add(int(parts[0]))
                except ValueError:
                    pass
    return classes


def main():
    args = parse_args()
    images_root = args.dataset / "images/val_proxy"
    labels_root = args.dataset / "labels/val_proxy"
    images = sorted(p for p in images_root.rglob("*") if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"})
    if not images:
        raise SystemExit("No calibration images found: %s" % images_root)

    rng = random.Random(args.seed)
    tagged = []
    by_class = defaultdict(list)
    for image in images:
        classes = labels_for(image, images_root, labels_root)
        tagged.append((image, classes))
        for cls in classes:
            by_class[cls].append(image)

    selected = []
    seen = set()
    quota = max(12, args.count // (len(IMPORTANT) * 2))
    for cls in IMPORTANT:
        pool = list(by_class.get(cls, []))
        rng.shuffle(pool)
        for image in pool:
            if image not in seen:
                selected.append(image)
                seen.add(image)
            if sum(1 for p in selected if p in by_class.get(cls, [])) >= quota:
                break

    remainder = [p for p, _ in tagged if p not in seen]
    rng.shuffle(remainder)
    selected.extend(remainder[: max(0, args.count - len(selected))])
    selected = selected[: args.count]

    if args.output_dir.exists():
        shutil.rmtree(args.output_dir)
    args.output_dir.mkdir(parents=True)
    copied = []
    for index, image in enumerate(selected):
        target = args.output_dir / ("%04d_%s" % (index, image.name))
        shutil.copy2(image, target)
        copied.append(target)

    args.list.parent.mkdir(parents=True, exist_ok=True)
    args.list.write_text("\n".join("images/%s" % p.name for p in copied) + "\n", encoding="utf-8")
    print("selected=%d list=%s" % (len(copied), args.list.resolve()))


if __name__ == "__main__":
    main()
