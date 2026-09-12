#!/usr/bin/env python3
"""Export an Ultralytics checkpoint with Rockchip's YOLOv8 output layout."""

import argparse
import os
import shutil
import sys
from pathlib import Path


def parse_args():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", type=Path, default=root / "models/unified21_light_focus_v4_selected_640.pt")
    parser.add_argument("--vendor", type=Path, default=root.parent / "adas_training/vendor/ultralytics_yolov8")
    parser.add_argument("--output", type=Path, default=root / "models/unified21_light_focus_v4_rkopt.onnx")
    parser.add_argument("--imgsz", type=int, default=640)
    return parser.parse_args()


def main():
    args = parse_args()
    os.environ.setdefault("YOLO_CONFIG_DIR", str(args.output.parent / ".yolo"))
    os.environ.setdefault("MPLCONFIGDIR", str(args.output.parent / ".matplotlib"))
    sys.path.insert(0, str(args.vendor.resolve()))

    from ultralytics import YOLO

    model = YOLO(str(args.weights.resolve()))
    exported = Path(model.export(format="rknn", imgsz=args.imgsz, device="cpu"))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if exported.resolve() != args.output.resolve():
        shutil.move(str(exported), str(args.output))
    print(args.output.resolve())


if __name__ == "__main__":
    main()
