"""Export official UFLD V1 CULane ResNet18 to an RKNN-friendly ONNX."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import torch


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--upstream", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    sys.path.insert(0, str(args.upstream.resolve()))
    from model.model import parsingNet

    model = parsingNet(
        pretrained=False,
        backbone="18",
        cls_dim=(201, 18, 4),
        use_aux=False,
    )
    checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    source = checkpoint["model"] if isinstance(checkpoint, dict) and "model" in checkpoint else checkpoint
    state = {name.removeprefix("module."): value for name, value in source.items()}
    model.load_state_dict(state, strict=False)
    model.eval()
    dummy = torch.zeros((1, 3, 288, 800), dtype=torch.float32)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with torch.inference_mode():
        torch.onnx.export(
            model,
            dummy,
            args.output,
            opset_version=12,
            input_names=["input"],
            output_names=["lane_logits"],
            do_constant_folding=True,
        )
    print(f"exported={args.output.resolve()} bytes={args.output.stat().st_size}")


if __name__ == "__main__":
    main()
