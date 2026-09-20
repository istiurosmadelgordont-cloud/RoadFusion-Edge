"""Create an 800x320 UFLDv2 CULane student initialized from the 1600x320 teacher.

The ResNet, pooling layer and output classifier are copied exactly.  The
LayerNorm and first fully-connected layer are folded pairwise along feature
map width.  This gives a useful initialization before video-logit distillation.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import torch


def halve_feature_width(vector: torch.Tensor, reduce: str) -> torch.Tensor:
    """Fold flattened [8, 10, 50] teacher features to [8, 10, 25]."""
    prefix = vector.shape[:-1]
    shaped = vector.reshape(*prefix, 8, 10, 50)
    paired = shaped.reshape(*prefix, 8, 10, 25, 2)
    folded = paired.sum(-1) if reduce == "sum" else paired.mean(-1)
    return folded.reshape(*prefix, 2000).contiguous()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--teacher", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    checkpoint = torch.load(args.teacher, map_location="cpu", weights_only=False)
    source = checkpoint["model"] if isinstance(checkpoint, dict) and "model" in checkpoint else checkpoint
    state = {name.removeprefix("module."): value for name, value in source.items()}
    state["cls.0.weight"] = halve_feature_width(state["cls.0.weight"], "mean")
    state["cls.0.bias"] = halve_feature_width(state["cls.0.bias"], "mean")
    state["cls.1.weight"] = halve_feature_width(state["cls.1.weight"], "sum")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.save({
        "model": state,
        "student": {"input_width": 800, "input_height": 320,
                    "teacher_width": 1600, "initialization": "pairwise_width_fold"},
    }, args.output)
    print(f"student={args.output.resolve()} bytes={args.output.stat().st_size}")


if __name__ == "__main__":
    main()
