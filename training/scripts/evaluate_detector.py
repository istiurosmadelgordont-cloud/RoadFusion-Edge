"""Evaluate all Unified19 classes and save per-class metrics."""

import argparse
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / ".runtime_cache"
for directory in (RUNTIME / "tmp", RUNTIME / "torch", RUNTIME / "huggingface", RUNTIME / "pip"):
    directory.mkdir(parents=True, exist_ok=True)
os.environ["TEMP"] = os.environ["TMP"] = str(RUNTIME / "tmp")
os.environ["TORCH_HOME"] = str(RUNTIME / "torch")
os.environ["HF_HOME"] = str(RUNTIME / "huggingface")
os.environ["PIP_CACHE_DIR"] = str(RUNTIME / "pip")
os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT / ".yolo"))
os.environ.setdefault("YOLO_OFFLINE", "true")
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / ".matplotlib"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("model", type=Path)
    parser.add_argument("--tag", default="unified19_directional_final")
    parser.add_argument("--data", type=Path, default=ROOT / "configs" / "unified19_directional.yaml")
    args = parser.parse_args()
    from ultralytics import YOLO, settings
    settings.update({"sync": False, "weights_dir": str(ROOT / "weights")})
    result = YOLO(args.model).val(
        data=str(args.data),
        imgsz=640, batch=16, device=0, workers=2, plots=True,
        project=str(ROOT / "runs"), name=f"{args.tag}_eval", exist_ok=True,
    )
    per_class = {
        result.names[int(cid)]: {
            "precision": float(result.box.p[i]), "recall": float(result.box.r[i]),
            "map50": float(result.box.ap50[i]), "map50_95": float(result.box.ap[i]),
        }
        for i, cid in enumerate(result.box.ap_class_index.tolist())
    }
    light_names = [name for name in per_class if name.startswith("traffic_red_") or name.startswith("traffic_green_")]
    payload = {
        "model": str(args.model.resolve()),
        "precision": float(result.box.mp), "recall": float(result.box.mr),
        "map50": float(result.box.map50), "map50_95": float(result.box.map),
        "directional_light_macro_map50": sum(per_class[n]["map50"] for n in light_names) / len(light_names),
        "per_class": per_class,
    }
    output = ROOT / "reports" / f"{args.tag}_evaluation.json"
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(payload, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
