"""Create synchronized, board-friendly four-view clips from one NVIDIA scene."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2


CAMERAS = {
    "front": "camera_front_wide_120fov",
    "rear": "camera_rear_tele_30fov",
    "left": "camera_cross_left_120fov",
    "right": "camera_cross_right_120fov",
}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("scene", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--step", type=int, default=3)
    args = parser.parse_args()
    if args.width < 64 or args.height < 64 or args.step < 1:
        raise ValueError("Invalid output size or frame step")

    captures = {}
    writers = {}
    source_info = {}
    try:
        for name, suffix in CAMERAS.items():
            path = args.scene / f"{args.scene.name}.{suffix}.mp4"
            if not path.is_file():
                raise FileNotFoundError(path)
            capture = cv2.VideoCapture(str(path))
            if not capture.isOpened():
                raise RuntimeError(f"Cannot open {path}")
            captures[name] = capture
            source_info[name] = {
                "path": str(path),
                "fps": capture.get(cv2.CAP_PROP_FPS),
                "frames": int(capture.get(cv2.CAP_PROP_FRAME_COUNT)),
                "width": int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                "height": int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            }
        fps_values = [round(info["fps"], 4) for info in source_info.values()]
        frame_counts = [info["frames"] for info in source_info.values()]
        if len(set(fps_values)) != 1 or len(set(frame_counts)) != 1:
            raise RuntimeError(f"Camera videos are not frame-synchronized: {source_info}")
        output_fps = fps_values[0] / args.step
        args.output.mkdir(parents=True, exist_ok=True)
        for name in CAMERAS:
            path = args.output / f"{name}.mp4"
            writer = cv2.VideoWriter(
                str(path), cv2.VideoWriter_fourcc(*"mp4v"), output_fps,
                (args.width, args.height),
            )
            if not writer.isOpened():
                raise RuntimeError(f"Cannot create {path}")
            writers[name] = writer

        written = 0
        for index in range(frame_counts[0]):
            frames = {}
            for name, capture in captures.items():
                ok, frame = capture.read()
                if not ok:
                    raise RuntimeError(f"Truncated {name} stream at frame {index}")
                frames[name] = frame
            if index % args.step:
                continue
            for name, frame in frames.items():
                writers[name].write(cv2.resize(frame, (args.width, args.height)))
            if written == 0:
                from numpy import hstack, vstack
                preview = vstack((hstack((frames["front"], frames["rear"])),
                                  hstack((frames["left"], frames["right"]))))
                cv2.imwrite(str(args.output / "source_preview.jpg"),
                            cv2.resize(preview, (args.width * 2, args.height * 2)))
            written += 1
        result = {"scene": args.scene.name, "source": source_info,
                  "output_fps": output_fps, "output_frames": written,
                  "width": args.width, "height": args.height,
                  "layout": [["front", "rear"], ["left", "right"]]}
        (args.output / "manifest.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps({"output": str(args.output), "frames": written,
                          "fps": output_fps, "duration": written / output_fps}))
    finally:
        for capture in captures.values():
            capture.release()
        for writer in writers.values():
            writer.release()


if __name__ == "__main__":
    main()
