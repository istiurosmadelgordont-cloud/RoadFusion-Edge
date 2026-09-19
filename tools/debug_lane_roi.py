import argparse
from pathlib import Path

import cv2
import numpy as np


def load_roi(path: Path):
    values = [tuple(map(float, line.split())) for line in path.read_text().splitlines() if line.strip()]
    if len(values) != 4:
        raise ValueError(f"{path} must contain four normalized points")
    return np.asarray(values, dtype=np.float32)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("video", type=Path)
    parser.add_argument("roi", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--frames", default="4,30,60,90,150,300")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    roi = load_roi(args.roi)
    cap = cv2.VideoCapture(str(args.video))
    for index in map(int, args.frames.split(",")):
        cap.set(cv2.CAP_PROP_POS_FRAMES, index)
        ok, frame = cap.read()
        if not ok:
            continue
        h, w = frame.shape[:2]
        points = np.rint(roi * (w, h)).astype(np.int32)
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        white = cv2.inRange(hls, (0, 110, 0), (180, 255, 255))
        yellow = cv2.inRange(hls, (12, 55, 55), (42, 235, 255))
        color = cv2.bitwise_or(white, yellow)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        local = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                      cv2.THRESH_BINARY, 21, -5)
        binary = cv2.bitwise_or(color, local)
        mask = np.zeros_like(binary)
        cv2.fillConvexPoly(mask, points, 255)
        binary = cv2.bitwise_and(binary, mask)
        edges = cv2.Canny(cv2.GaussianBlur(binary, (5, 5), 0), 35, 110)
        overlay = frame.copy()
        cv2.polylines(overlay, [points], True, (0, 200, 255), 2, cv2.LINE_AA)
        tiles = [overlay, cv2.cvtColor(color, cv2.COLOR_GRAY2BGR),
                 cv2.cvtColor(local, cv2.COLOR_GRAY2BGR), cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)]
        sheet = np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])])
        cv2.imwrite(str(args.output / f"frame_{index:04d}.jpg"), sheet)
        print(index, "color", cv2.countNonZero(cv2.bitwise_and(color, mask)),
              "adaptive", cv2.countNonZero(cv2.bitwise_and(local, mask)),
              "edges", cv2.countNonZero(edges))


if __name__ == "__main__":
    main()
