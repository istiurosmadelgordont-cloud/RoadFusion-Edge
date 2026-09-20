"""Build a representative UFLDv2 RKNN calibration set from bundled scenes."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenes", type=Path, default=root / "four_view_sample")
    parser.add_argument("--output", type=Path,
                        default=root / "calibration/ufldv2_multiscene")
    parser.add_argument("--list", type=Path,
                        default=root / "calibration/ufldv2_multiscene_dataset.txt")
    parser.add_argument("--frames-per-camera", type=int, default=24)
    parser.add_argument("--include-rear", action="store_true")
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    for stale in args.output.glob("*.jpg"):
        stale.unlink()

    cameras = ("front", "rear") if args.include_rear else ("front",)
    outputs = []
    for scene in sorted(path for path in args.scenes.iterdir() if path.is_dir()):
        for camera in cameras:
            video = scene / f"{camera}.mp4"
            if not video.is_file():
                continue
            capture = cv2.VideoCapture(str(video))
            total = max(1, int(capture.get(cv2.CAP_PROP_FRAME_COUNT)))
            indices = [round(i * (total - 1) / max(args.frames_per_camera - 1, 1))
                       for i in range(args.frames_per_camera)]
            for sample, frame_index in enumerate(indices):
                capture.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
                ok, frame = capture.read()
                if not ok:
                    continue
                image = cv2.resize(frame, (800, 533), interpolation=cv2.INTER_LINEAR)[-320:]
                output = args.output / f"{scene.name}_{camera}_{sample:02d}.jpg"
                cv2.imwrite(str(output), image, [cv2.IMWRITE_JPEG_QUALITY, 96])
                outputs.append(output)
            capture.release()

    # RKNN Toolkit2 runs in WSL in this workspace.
    lines = ["/mnt/" + str(path.resolve())[0].lower() +
             str(path.resolve())[2:].replace("\\", "/") for path in outputs]
    args.list.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"images={len(outputs)} list={args.list.resolve()}")


if __name__ == "__main__":
    main()
