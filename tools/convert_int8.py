#!/usr/bin/env python3
"""Convert the Rockchip-optimized YOLOv8 ONNX graph to RK3568 INT8 RKNN."""

import argparse
from pathlib import Path

from rknn.api import RKNN


def parse_args():
    root = Path(__file__).resolve().parents[1]
    p = argparse.ArgumentParser()
    p.add_argument("--onnx", type=Path, default=root / "models/unified17_v10_candidate_640_rkopt.onnx")
    p.add_argument("--dataset", type=Path, default=root / "calibration/dataset.txt")
    p.add_argument("--output", type=Path, default=root / "models/unified17_v10_candidate_640_int8.rknn")
    p.add_argument("--verbose", action="store_true")
    return p.parse_args()


def checked(stage, code):
    if code != 0:
        raise RuntimeError("%s failed with code %s" % (stage, code))


def main():
    args = parse_args()
    rknn = RKNN(verbose=args.verbose)
    try:
        checked("config", rknn.config(
            mean_values=[[0, 0, 0]],
            std_values=[[255, 255, 255]],
            target_platform="rk3568",
            quantized_dtype="asymmetric_quantized-8",
            quantized_algorithm="normal",
            optimization_level=3,
        ))
        checked("load_onnx", rknn.load_onnx(model=str(args.onnx.resolve())))
        checked("build", rknn.build(do_quantization=True, dataset=str(args.dataset.resolve())))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        checked("export_rknn", rknn.export_rknn(str(args.output.resolve())))
        print(args.output.resolve())
    finally:
        rknn.release()


if __name__ == "__main__":
    main()
