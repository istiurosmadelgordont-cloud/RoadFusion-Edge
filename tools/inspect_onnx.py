#!/usr/bin/env python3
import argparse
from pathlib import Path

import onnx


def shape(value):
    return [d.dim_value or d.dim_param or "?" for d in value.type.tensor_type.shape.dim]


def main():
    root = Path(__file__).resolve().parents[1]
    p = argparse.ArgumentParser()
    p.add_argument("model", nargs="?", type=Path, default=root / "models/unified17_v10_candidate_640_rkopt.onnx")
    args = p.parse_args()
    model = onnx.load(str(args.model))
    onnx.checker.check_model(model)
    print("inputs:")
    for value in model.graph.input:
        print("  %s %s" % (value.name, shape(value)))
    print("outputs:")
    for value in model.graph.output:
        print("  %s %s" % (value.name, shape(value)))


if __name__ == "__main__":
    main()
