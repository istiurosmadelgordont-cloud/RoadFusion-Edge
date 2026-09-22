"""Export an official UFLDv2 checkpoint without the TensorRT-only helpers.

The upstream exporter imports optional onnxmltools even for FP32 export and
always requires CUDA.  This version exports the four native prediction heads
on CPU so the graph can be passed directly to RKNN Toolkit2.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import torch


class OutputTuple(torch.nn.Module):
    def __init__(self, model: torch.nn.Module):
        super().__init__()
        self.model = model

    def forward(self, image: torch.Tensor):
        result = self.model(image)
        return (result["loc_row"], result["loc_col"],
                result["exist_row"], result["exist_col"])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--upstream", type=Path, required=True)
    parser.add_argument("--config", choices=("culane", "culane_student", "tusimple"), required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--backbone", choices=("18", "34"), default="18")
    args = parser.parse_args()

    sys.path.insert(0, str(args.upstream.resolve()))
    from model.backbone import resnet

    if args.config in ("culane", "culane_student"):
        width, height, grid_row, rows, grid_col, cols, fc_norm = (
            1600, 320, 200, 72, 100, 81, True)
        if args.config == "culane_student":
            width = 800
    else:
        width, height, grid_row, rows, grid_col, cols, fc_norm = (
            800, 320, 100, 56, 100, 41, False)
    lanes = 4

    class ExportNet(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.dim1 = grid_row * rows * lanes
            self.dim2 = grid_col * cols * lanes
            self.dim3 = 2 * rows * lanes
            self.dim4 = 2 * cols * lanes
            self.model = resnet(args.backbone, pretrained=False)
            self.pool = torch.nn.Conv2d(512, 8, 1)
            input_dim = height // 32 * width // 32 * 8
            self.cls = torch.nn.Sequential(
                torch.nn.LayerNorm(input_dim) if fc_norm else torch.nn.Identity(),
                torch.nn.Linear(input_dim, 2048),
                torch.nn.ReLU(),
                torch.nn.Linear(2048, self.dim1 + self.dim2 + self.dim3 + self.dim4),
            )

        def forward(self, image):
            _x2, _x3, feature = self.model(image)
            output = self.cls(self.pool(feature).flatten(1))
            return {
                "loc_row": output[:, :self.dim1].view(-1, grid_row, rows, lanes),
                "loc_col": output[:, self.dim1:self.dim1 + self.dim2].view(
                    -1, grid_col, cols, lanes),
                "exist_row": output[:, self.dim1 + self.dim2:
                                    self.dim1 + self.dim2 + self.dim3].view(
                    -1, 2, rows, lanes),
                "exist_col": output[:, -self.dim4:].view(-1, 2, cols, lanes),
            }

    model = ExportNet()
    checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    source = checkpoint["model"] if "model" in checkpoint else checkpoint
    state = {key.removeprefix("module."): value for key, value in source.items()}
    model.load_state_dict(state, strict=True)
    model.eval()
    wrapper = OutputTuple(model).eval()
    dummy = torch.zeros((1, 3, height, width), dtype=torch.float32)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with torch.inference_mode():
        torch.onnx.export(
            wrapper, dummy, args.output, opset_version=12,
            input_names=["input"],
            output_names=["loc_row", "loc_col", "exist_row", "exist_col"],
            do_constant_folding=True,
        )
    print(f"exported={args.output} bytes={args.output.stat().st_size}")


if __name__ == "__main__":
    main()
