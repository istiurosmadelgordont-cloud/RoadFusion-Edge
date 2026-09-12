#!/usr/bin/env python3
"""Render a deterministic YOLO dataset contact sheet for label QA."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("dataset", type=Path)
    p.add_argument("--split", default="train")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--limit", type=int, default=48)
    p.add_argument("--class-id", type=int, help="Only include images containing this class")
    args = p.parse_args()
    images = sorted((args.dataset / "images" / args.split).glob("*.jpg"))
    if args.class_id is not None:
        images = [
            image for image in images
            if any(
                line.split() and int(line.split()[0]) == args.class_id
                for line in (args.dataset / "labels" / args.split / f"{image.stem}.txt").read_text().splitlines()
            )
        ]
    # Spread samples across the complete sorted set instead of showing only one
    # short sequence from its beginning.
    if len(images) > args.limit:
        indices = np.linspace(0, len(images) - 1, args.limit, dtype=int)
        images = [images[i] for i in indices]
    if not images:
        raise SystemExit("No images found")
    tiles = []
    colours = {7:(30,30,240),8:(40,150,255),9:(200,40,240),10:(40,220,40),11:(240,180,20),12:(240,80,20)}
    for image_path in images:
        image = cv2.imdecode(np.fromfile(str(image_path),np.uint8),cv2.IMREAD_COLOR)
        h,w = image.shape[:2]
        for line in (args.dataset/"labels"/args.split/f"{image_path.stem}.txt").read_text().splitlines():
            c,x,y,bw,bh=line.split(); c=int(c); x,y,bw,bh=map(float,(x,y,bw,bh))
            x1,y1,x2,y2=int((x-bw/2)*w),int((y-bh/2)*h),int((x+bw/2)*w),int((y+bh/2)*h)
            cv2.rectangle(image,(x1,y1),(x2,y2),colours.get(c,(0,255,255)),2)
            cv2.putText(image,str(c),(x1,max(14,y1-3)),cv2.FONT_HERSHEY_SIMPLEX,.45,colours.get(c,(0,255,255)),1,cv2.LINE_AA)
        scale=min(280/w,180/h)
        shown=cv2.resize(image,(max(1,int(w*scale)),max(1,int(h*scale))))
        tile=np.full((210,290,3),20,np.uint8); ox=(290-shown.shape[1])//2
        tile[:shown.shape[0],ox:ox+shown.shape[1]]=shown
        cv2.putText(tile,image_path.stem[:38],(4,202),cv2.FONT_HERSHEY_SIMPLEX,.34,(220,220,220),1,cv2.LINE_AA)
        tiles.append(tile)
    cols=6; rows=(-len(tiles))//cols
    tiles += [np.full((210,290,3),20,np.uint8)]*((-len(tiles))%cols)
    sheet=np.vstack([np.hstack(tiles[i:i+cols]) for i in range(0,len(tiles),cols)])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    cv2.imwrite(str(args.output),sheet)
    print(args.output)


if __name__ == "__main__":
    main()
