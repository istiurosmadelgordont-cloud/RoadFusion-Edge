#!/usr/bin/env python3
"""Expand the selected Unified19 detector to Unified21 without losing old rows."""

from __future__ import annotations

import copy
import os
from pathlib import Path

import torch


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "weights/unified19_light_focus_v3_light_selected_640.pt"
OUTPUT = ROOT / "weights/unified21_light_focus_v4_initialized.pt"
NAMES = {
    0: "pedestrian", 1: "rider", 2: "car", 3: "bus", 4: "truck", 5: "motorcycle", 6: "bicycle",
    7: "traffic_red_circle", 8: "traffic_red_left", 9: "traffic_red_right", 10: "traffic_red_straight",
    11: "traffic_green_circle", 12: "traffic_green_left", 13: "traffic_green_right", 14: "traffic_green_straight",
    15: "traffic_sign", 16: "crosswalk", 17: "guide_arrows", 18: "traffic_cone", 19: "roadworks_sign", 20: "delineator",
}
OLD_TO_NEW = {
    **{i: i for i in range(10)},
    10: 11, 11: 12, 12: 13,
    13: 15, 14: 16, 15: 17, 16: 18, 17: 19, 18: 20,
}

runtime = ROOT / ".runtime_cache"
for directory in (runtime / "tmp", runtime / "torch", runtime / "huggingface", runtime / "pip"):
    directory.mkdir(parents=True, exist_ok=True)
os.environ["TEMP"] = os.environ["TMP"] = str(runtime / "tmp")
os.environ["TORCH_HOME"] = str(runtime / "torch")
os.environ["HF_HOME"] = str(runtime / "huggingface")
os.environ["PIP_CACHE_DIR"] = str(runtime / "pip")
os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT / "runtime_v21/yolo"))
os.environ.setdefault("YOLO_OFFLINE", "true")
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / "runtime_v21/matplotlib"))


def main() -> None:
    from ultralytics import YOLO
    from ultralytics.nn.tasks import DetectionModel

    source = YOLO(SOURCE)
    old_model = source.model
    if old_model.model[-1].nc != 19:
        raise RuntimeError(f"Expected 19 source classes, got {old_model.model[-1].nc}")

    cfg = copy.deepcopy(old_model.yaml)
    cfg["nc"] = 21
    new_model = DetectionModel(cfg, ch=3, nc=21, verbose=False)
    old_state = old_model.state_dict()
    new_state = new_model.state_dict()
    compatible = {key: value for key, value in old_state.items()
                  if key in new_state and value.shape == new_state[key].shape}
    new_model.load_state_dict(compatible, strict=False)

    old_detect = old_model.model[-1]
    new_detect = new_model.model[-1]
    with torch.no_grad():
        for new_head, old_head in zip(new_detect.cv3, old_detect.cv3):
            new_conv = new_head[-1]
            old_conv = old_head[-1]
            for old_id, new_id in OLD_TO_NEW.items():
                new_conv.weight[new_id].copy_(old_conv.weight[old_id])
                new_conv.bias[new_id].copy_(old_conv.bias[old_id])
            # Straight-arrow rows start from the matching colour circle row.
            # Real straight-arrow instances then separate them during V4 training.
            new_conv.weight[10].copy_(old_conv.weight[7])
            new_conv.bias[10].copy_(old_conv.bias[7])
            new_conv.weight[14].copy_(old_conv.weight[10])
            new_conv.bias[14].copy_(old_conv.bias[10])

    new_model.names = NAMES
    wrapper = YOLO(SOURCE)
    wrapper.model = new_model
    wrapper.ckpt = {"model": new_model, "train_args": getattr(source, "overrides", {})}
    wrapper.save(OUTPUT)

    check = YOLO(OUTPUT)
    if check.model.model[-1].nc != 21 or list(check.names.values()) != list(NAMES.values()):
        raise RuntimeError("Serialized Unified21 model metadata is invalid")
    check_detect = check.model.model[-1]
    for scale, (new_head, old_head) in enumerate(zip(check_detect.cv3, old_detect.cv3)):
        for old_id, new_id in OLD_TO_NEW.items():
            if not torch.equal(new_head[-1].weight[new_id].cpu(), old_head[-1].weight[old_id].cpu()):
                raise RuntimeError(f"Weight mapping failed at scale={scale}, old={old_id}, new={new_id}")
    print(f"saved and verified: {OUTPUT}")


if __name__ == "__main__":
    main()
