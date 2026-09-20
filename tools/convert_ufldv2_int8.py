"""Convert UFLDv2 CULane ResNet18 ONNX to an RK3568 INT8 model."""

from __future__ import annotations

import argparse
from pathlib import Path

from rknn.api import RKNN


def checked(stage: str, code: int) -> None:
    if code != 0:
        raise RuntimeError(f"{stage} failed with code {code}")


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx", type=Path,
                        default=root / "models/ufldv2_culane_res18_1600x320.onnx")
    parser.add_argument("--dataset", type=Path,
                        default=root / "calibration/ufldv2_dataset.txt")
    parser.add_argument("--output", type=Path,
                        default=root / "models/ufldv2_culane_res18_1600x320_int8.rknn")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    rknn = RKNN(verbose=args.verbose)
    try:
        checked("config", rknn.config(
            mean_values=[[123.675, 116.28, 103.53]],
            std_values=[[58.395, 57.12, 57.375]],
            target_platform="rk3568",
            quantized_dtype="asymmetric_quantized-8",
            quantized_algorithm="normal",
            optimization_level=3,
        ))
        checked("load_onnx", rknn.load_onnx(model=str(args.onnx.resolve())))
        checked("build", rknn.build(
            do_quantization=True, dataset=str(args.dataset.resolve())))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        checked("export_rknn", rknn.export_rknn(str(args.output.resolve())))
        print(args.output.resolve())
    finally:
        rknn.release()


if __name__ == "__main__":
    main()
