"""PC visual probe for UFLDv2 slots. It approximates the C++ overlay, not RKNN."""

import argparse
from pathlib import Path

import cv2
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--model', type=Path, required=True)
parser.add_argument('--image', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--raw-output', type=Path)
parser.add_argument('--input-width', type=int, choices=(800, 1600), default=1600)
args = parser.parse_args()

source = cv2.imread(str(args.image))
if source is None:
    parser.error(f'cannot read image: {args.image}')
image = cv2.resize(source, (640, 360))
width = args.input_width
resized = cv2.resize(image, (width, 533))[-320:]
rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32)
blob = ((rgb - [123.675, 116.28, 103.53]) /
        [58.395, 57.12, 57.375]).astype(np.float32)
net = cv2.dnn.readNetFromONNX(str(args.model))
net.setInput(np.ascontiguousarray(blob.transpose(2, 0, 1)[None]))
loc_row, loc_col, exist_row, exist_col = net.forward(
    ['loc_row', 'loc_col', 'exist_row', 'exist_col'])
shapes = [a.shape for a in (loc_row, loc_col, exist_row, exist_col)]
if shapes != [(1, 200, 72, 4), (1, 100, 81, 4), (1, 2, 72, 4), (1, 2, 81, 4)]:
    raise ValueError(f'unexpected UFLDv2 outputs: {shapes}')
print('shapes', shapes, flush=True)
frame = image.copy()
curves = [None] * 4
anchor_coverage = [0.] * 4
raw = image.copy()
for slot in range(4):
    column = slot in (0, 3)
    loc, exist = (loc_col, exist_col) if column else (loc_row, exist_row)
    grids, anchors = loc.shape[1:3]
    pts = []
    for anchor in range(anchors):
        if exist[0, 1, anchor, slot] <= exist[0, 0, anchor, slot]:
            continue
        best = int(np.argmax(loc[0, :, anchor, slot]))
        low, high = max(0, best-1), min(grids-1, best+1)
        v = loc[0, low:high+1, anchor, slot]
        probs = np.exp(v - v.max())
        pos = np.dot(probs, np.arange(low, high+1)+0.5)/probs.sum()
        x = anchor/(anchors-1)*640 if column else pos/(grids-1)*640
        y = pos/(grids-1)*360 if column else (.42+.58*anchor/(anchors-1))*360
        pts.append((x, y))
    pts.sort(key=lambda p: p[1])
    anchor_coverage[slot] = len(pts)/anchors
    print('slot', slot, 'head', 'col' if column else 'row', 'anchors', len(pts), '/', anchors,
          'bottom', pts[-1] if pts else None, flush=True)
    if len(pts) < 5:
        continue
    p = np.asarray(pts)
    fit = np.polyfit(p[:, 1], p[:, 0], 2)
    yy = np.linspace(min(p[:, 1]), max(p[:, 1]), 32)
    curve = np.stack((np.polyval(fit, yy), yy), axis=-1).astype(np.int32)
    curves[slot] = curve
    color = [(200, 180, 100), (80, 255, 80),
             (80, 255, 80), (255, 180, 80)][slot]
    cv2.polylines(raw, [curve], False, color, 2, cv2.LINE_AA)
    for x, y in pts:
        cv2.circle(raw, (round(x), round(y)), 2, color, -1, cv2.LINE_AA)
if args.raw_output:
    args.raw_output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.raw_output), raw):
        raise OSError(f'cannot write {args.raw_output}')

def fill(poly, color, alpha):
    mask = np.zeros(frame.shape[:2], np.uint8)
    cv2.fillPoly(mask, [poly.astype(np.int32)], 255, cv2.LINE_AA)
    amount = (mask.astype(np.float32) / 255 * alpha)[:, :, None]
    frame[:] = np.uint8(frame * (1 - amount) + np.array(color, np.float32) * amount)

if curves[1] is not None and curves[2] is not None:
    markings = ['UNKNOWN'] * 4
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    paint = cv2.bitwise_or(cv2.inRange(hls, (0, 155, 0), (180, 255, 125)),
                           cv2.inRange(hsv, (12, 65, 120), (42, 255, 255)))
    for slot in range(4):
        if curves[slot] is None:
            continue
        hits = []
        for x, y in curves[slot]:
            x0, x1 = max(0, x-6), min(640, x+7)
            y0, y1 = max(0, y-6), min(360, y+7)
            hits.append(cv2.countNonZero(paint[y0:y1, x0:x1]) /
                        max(1, (x1-x0)*(y1-y0)) >= .055)
        gap = runs = longest = 0
        in_run = False
        for v in hits:
            if v:
                runs += not in_run
                in_run = True
                gap = 0
            else:
                in_run = False
                gap += 1
                longest = max(longest, gap)
        coverage = np.mean(hits)
        marking = ('SOLID' if coverage >= .62 and longest <= 5 else
                   'DASHED' if .18 <= coverage <= .78 and runs >= 2 and longest >= 2
                   else 'UNKNOWN')
        markings[slot] = marking
        print('paint', slot, round(float(coverage), 2), runs, longest,
              marking, flush=True)
    # Use only rows covered by each observed pair.
    for outer, inner, tint in [(0, 1, (70, 145, 230)),
                               (3, 2, (230, 130, 160))]:
        if curves[outer] is None or anchor_coverage[outer] <= .25:
            continue
        far = max(curves[outer][0, 1], curves[inner][0, 1])
        near = min(curves[outer][-1, 1], curves[inner][-1, 1])
        if near - far < 360 * .15:
            continue
        yy = np.linspace(far, near, 24)
        xx0 = np.interp(yy, curves[outer][:, 1], curves[outer][:, 0])
        xx1 = np.interp(yy, curves[inner][:, 1], curves[inner][:, 0])
        ego_left = np.interp(yy, curves[1][:, 1], curves[1][:, 0])
        ego_right = np.interp(yy, curves[2][:, 1], curves[2][:, 0])
        ego_width = ego_right - ego_left
        gap = xx1 - xx0 if outer == 0 else xx0 - xx1
        if (np.any(xx0 < 0) or np.any(xx0 >= 640) or
                np.any(gap < np.maximum(8, ego_width * .35)) or
                np.any(gap > ego_width * 2.2)):
            continue
        a = np.stack([xx0, yy], -1)
        b = np.stack([xx1, yy], -1)
        fill(np.vstack([a, b[::-1]]), tint, .24)
        if markings[outer] == 'UNKNOWN':
            for x, y in curves[outer][::3]:
                cv2.circle(frame, (x, y), 2, tint, -1, cv2.LINE_AA)
        else:
            cv2.polylines(frame, [curves[outer]], False, tint, 2, cv2.LINE_AA)
    left, right = curves[1], curves[2]
    far = max(left[0, 1], right[0, 1])
    near = min(left[-1, 1], right[-1, 1])
    yy = np.linspace(far, near, 32)
    lx = np.interp(yy, left[:, 1], left[:, 0])
    rx = np.interp(yy, right[:, 1], right[:, 0])
    lane = np.vstack([np.stack([lx, yy], -1),
                      np.stack([rx[::-1], yy[::-1]], -1)])
    fill(lane, (55, 205, 65), .24)
    for i in range(4, 27, 4):
        x = int((lx[i]+rx[i])/2)
        y0, y1 = round(yy[i+2]), round(yy[i])
        half = max(4, int((rx[i]-lx[i])*.14))
        arrow = np.array([[x-half, y0], [x, y1], [x+half, y0],
                          [x+half*3//4, y0], [x, y1+(y0-y1)//3],
                          [x-half*3//4, y0]], np.int32)
        cv2.fillPoly(frame, [arrow], (110, 250, 115), cv2.LINE_AA)
    cv2.polylines(frame, [left], False, (55, 255, 70), 3, cv2.LINE_AA)
    cv2.polylines(frame, [right], False, (55, 255, 70), 3, cv2.LINE_AA)
args.output.parent.mkdir(parents=True, exist_ok=True)
if not cv2.imwrite(str(args.output), frame):
    raise OSError(f'cannot write {args.output}')
