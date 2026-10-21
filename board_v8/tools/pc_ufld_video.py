"""PC UFLDv2 probe: original decision geometry, optional visual stabilization.

Decode/geometry/tracking mirror ufld_v2_decode.hpp, ufld_lane_detector.cpp
and lane_geometry_tracker.hpp. This does not reproduce RKNN quantization,
the shared NPU scheduler, marking semantics or the LDW decision monitor.
The adjacent-lane visual tracker is presentation-only and never changes Lane.
"""
from __future__ import annotations

import argparse
import copy
import csv
import json
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
from pc_lane_visual import AdjacentLaneVisualTracker
from pc_lane_markings import draw_boundary

ROOT = Path(__file__).resolve().parents[2]
HEADS = ('loc_row', 'loc_col', 'exist_row', 'exist_col')
SHAPES = ((1, 200, 72, 4), (1, 100, 81, 4), (1, 2, 72, 4), (1, 2, 81, 4))
COLORS = ((70, 145, 230), (60, 240, 70), (60, 240, 70), (230, 130, 160))


@dataclass
class Lane:
    valid: bool = False
    partial: bool = False
    uncertain: bool = False
    held: bool = False
    reassignment: str = 'NONE'
    offset: float = 0.0
    boundaries: list = field(default_factory=lambda: [None] * 4)
    polygons: list = field(default_factory=lambda: [None] * 3)
    coverage: list = field(default_factory=lambda: [0.0] * 4)


def decode(outputs):
    if tuple(x.shape for x in outputs) != SHAPES:
        raise ValueError(f'Unexpected model output shapes: {[x.shape for x in outputs]}')
    row, col, erow, ecol = outputs
    slots = []
    for slot in range(4):
        column = slot in (0, 3)
        loc, existence = (col, ecol) if column else (row, erow)
        grids, anchors = loc.shape[1:3]
        points = []
        for anchor in range(anchors):
            absent, present = existence[0, :, anchor, slot]
            if not np.isfinite([absent, present]).all() or present <= absent:
                continue
            logits = loc[0, :, anchor, slot]
            finite = np.isfinite(logits)
            if not finite.any():
                continue
            best = int(np.argmax(np.where(finite, logits, -np.inf)))
            indices = np.arange(max(0, best - 1), min(grids - 1, best + 1) + 1)
            indices = indices[finite[indices]]
            probabilities = np.exp(logits[indices] - logits[best])
            position = float(np.dot(probabilities, indices + .5) / probabilities.sum() / (grids - 1))
            x = anchor / 80 * 640 if column else position * 640
            y = position * 360 if column else (.42 + .58 * anchor / 71) * 360
            points.append((x, y))
        slots.append(np.asarray(points, np.float64).reshape(-1, 2))
    return slots


def fit(points):
    if len(points) < 5:
        return None
    yy = points[:, 1]
    design = np.column_stack((yy * yy, yy, np.ones(len(yy))))
    success, solution = cv2.solve(design, points[:, 0, None], flags=cv2.DECOMP_SVD)
    return solution[:, 0] if success and np.isfinite(solution).all() else None


def refine(points):
    initial = fit(points)
    if initial is None:
        return None
    residuals = np.abs(points[:, 0] - np.polyval(initial, points[:, 1]))
    limit = max(640 * .008, 2.5 * np.sort(residuals)[len(residuals) // 2])
    inliers = points[residuals <= limit]
    return fit(inliers) if len(inliers) >= 5 and len(inliers) * 2 >= len(points) else None


def curve(coefficients, yy, clip=False):
    xx = np.polyval(coefficients, yy)
    if clip:
        xx = np.clip(xx, 0, 639)
    return np.rint(np.column_stack((xx, yy))).astype(np.int32)


def make_lane(slots):
    result = Lane()
    for slot, points in enumerate(slots):
        result.coverage[slot] = len(points) / (81 if slot in (0, 3) else 72)
        result.boundaries[slot] = np.rint(points[np.argsort(-points[:, 1])]).astype(np.int32)
    left, right = slots[1:3]
    if len(left) < 5 or len(right) < 5:
        return result
    far = max(left[:, 1].min(), right[:, 1].min())
    near = min(left[:, 1].max(), right[:, 1].max(), 360 * .98)
    if near - far < 360 * .16:
        return result
    lf, rf = refine(left), refine(right)
    if lf is None or rf is None:
        return result
    yy = np.linspace(near, far, 32)
    lc, rc = curve(lf, yy, True), curve(rf, yy, True)
    if np.any(rc[:, 0] - lc[:, 0] <= 5):
        return result
    bottom_width = float(rc[0, 0] - lc[0, 0])
    if bottom_width < 640 * .15 or bottom_width >= 640 * .75 or lc[0, 0] >= 640 * .62 or rc[0, 0] <= 640 * .38:
        return result
    result.valid = True
    result.partial = result.coverage[1] <= .5 or result.coverage[2] <= .5
    result.offset = (320 - .5 * (lc[0, 0] + rc[0, 0])) / max(1, bottom_width)
    result.boundaries[1:3] = [lc, rc]
    result.polygons[1] = np.vstack((lc, rc[::-1]))
    for slot in (0, 3):
        points = slots[slot]
        if len(points) < 5 or result.coverage[slot] <= .25:
            continue
        outer = refine(points)
        if outer is None:
            continue
        top, bottom = max(far, points[:, 1].min()), min(near, points[:, 1].max())
        if bottom - top < 360 * .15:
            continue
        inner = lf if slot == 0 else rf
        check_y = np.linspace(top, bottom, 7)
        outside, inside = np.polyval(outer, check_y), np.polyval(inner, check_y)
        ego_width = np.polyval(rf, check_y) - np.polyval(lf, check_y)
        gap = inside - outside if slot == 0 else outside - inside
        if np.any((outside < 0) | (outside >= 640) | (ego_width < 10) | (gap < np.maximum(8, ego_width * .35)) | (gap > ego_width * 2.2)):
            continue
        yy = np.linspace(bottom, top, 24)
        oc, ic = curve(outer, yy), curve(inner, yy)
        result.boundaries[slot] = oc
        result.polygons[0 if slot == 0 else 2] = np.vstack((oc, ic[::-1])) if slot == 0 else np.vstack((ic, oc[::-1]))
    return result


class GeometryTracker:
    def __init__(self):
        self.previous, self.misses = Lane(), 0

    def update(self, measured):
        old = self.previous
        side = 'NONE'
        if not measured.valid:
            self.misses += 1
            if old.valid and self.misses <= 3:
                held = copy.deepcopy(old)
                held.partial = held.uncertain = held.held = True
                held.reassignment = 'NONE'
                return held
            self.previous = measured
            return measured
        if old.valid:
            ow = float(old.boundaries[2][0, 0] - old.boundaries[1][0, 0])
            nw = float(measured.boundaries[2][0, 0] - measured.boundaries[1][0, 0])
            shift = float(measured.boundaries[2][0, 0] + measured.boundaries[1][0, 0] - old.boundaries[2][0, 0] - old.boundaries[1][0, 0]) * .5
            width_stable = ow > 1 and ow * .68 < nw < ow * 1.42
            if not (width_stable and nw > 1 and abs(shift) < ow * .32):
                self.misses += 1
                if width_stable and old.offset < -.15 and shift < -ow * .32:
                    side = 'LEFT'
                if width_stable and old.offset > .15 and shift > ow * .32:
                    side = 'RIGHT'
                if self.misses <= 4:
                    held = copy.deepcopy(old)
                    held.partial = held.uncertain = held.held = True
                    held.reassignment = side
                    return held
                measured = copy.deepcopy(measured)
                measured.uncertain, measured.reassignment = True, side
        self.misses = 0
        self.previous = measured
        return measured


class Detector:
    def __init__(self, model, threads=4):
        self.backend = 'OpenCV DNN / CPU / ONNX FP32'
        cv2.setNumThreads(threads)
        self.net = cv2.dnn.readNetFromONNX(str(model))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def detect(self, image):
        started = time.perf_counter()
        resized = cv2.resize(image, (1600, 533))[-320:]
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32)
        blob = (rgb - np.array([123.675, 116.28, 103.53], np.float32)) / np.array([58.395, 57.12, 57.375], np.float32)
        self.net.setInput(np.ascontiguousarray(blob.transpose(2, 0, 1)[None]))
        infer_started = time.perf_counter()
        outputs = self.net.forward(list(HEADS))
        inference_ms = (time.perf_counter() - infer_started) * 1000
        slots = decode(outputs)
        lane = make_lane(slots)
        return lane, slots, inference_ms, (time.perf_counter() - started) * 1000


class CudaDetector:
    def __init__(self, checkpoint, threads=4):
        import torch
        sys.path.insert(0,str(ROOT/'training'))
        from train_ufld_qat import LaneNet
        if not torch.cuda.is_available():
            raise RuntimeError('CUDA is unavailable; use --backend onnx-cpu')
        self.torch=torch
        torch.set_num_threads(threads)
        cv2.setNumThreads(threads)
        torch.backends.cuda.matmul.allow_tf32=False
        torch.backends.cudnn.allow_tf32=False
        torch.backends.cudnn.benchmark=True
        self.net=LaneNet(ROOT.parent/'external/Ultra-Fast-Lane-Detection-v2')
        saved=torch.load(checkpoint,map_location='cpu',weights_only=False)
        state=saved.get('model',saved)
        self.net.load_state_dict({k.removeprefix('module.'):v for k,v in state.items()},strict=True)
        self.net=self.net.eval().cuda()
        self.backend='PyTorch FP32 / CUDA / '+torch.cuda.get_device_name(0)

    def detect(self,image):
        torch=self.torch
        started=time.perf_counter()
        resized=cv2.resize(image,(1600,533))[-320:]
        rgb=cv2.cvtColor(resized,cv2.COLOR_BGR2RGB).astype(np.float32)
        blob=(rgb-np.array([123.675,116.28,103.53],np.float32))/np.array([58.395,57.12,57.375],np.float32)
        tensor=torch.from_numpy(np.ascontiguousarray(blob.transpose(2,0,1)[None])).cuda()
        torch.cuda.synchronize()
        infer_started=time.perf_counter()
        with torch.inference_mode(): outputs=self.net(tensor)
        torch.cuda.synchronize()
        inference_ms=(time.perf_counter()-infer_started)*1000
        outputs=[value.cpu().numpy() for value in outputs]
        slots=decode(outputs)
        return make_lane(slots),slots,inference_ms,(time.perf_counter()-started)*1000


def draw(image, lane, adjacent_visuals=None, markings=None):
    frame = image.copy()
    if not lane.valid:
        return frame
    layer = frame.copy()
    for index, color in enumerate((COLORS[0], COLORS[1], COLORS[3])):
        if adjacent_visuals is not None and index!=1:
            continue
        polygon = lane.polygons[index]
        if polygon is not None:
            cv2.fillPoly(layer, [polygon], color, cv2.LINE_AA)
    frame = cv2.addWeighted(layer, .24, frame, .76, 0)
    if adjacent_visuals is not None:
        for side,visual in enumerate(adjacent_visuals):
            if visual is None:
                continue
            layer=frame.copy()
            color=COLORS[0 if side==0 else 3]
            cv2.fillPoly(layer,[visual.polygon],color,cv2.LINE_AA)
            frame=cv2.addWeighted(layer,visual.alpha,frame,1-visual.alpha,0)
    for slot, points in enumerate(lane.boundaries):
        if adjacent_visuals is not None and slot in (0,3):
            visual=adjacent_visuals[0 if slot==0 else 1]
            if visual is not None:
                layer=frame.copy()
                if markings is not None:
                    draw_boundary(layer,visual.boundary,markings[slot],COLORS[slot],2)
                elif visual.held:
                    for first in range(0,len(visual.boundary)-1,4):
                        cv2.polylines(layer,[visual.boundary[first:first+3]],False,COLORS[slot],2,cv2.LINE_AA)
                else:
                    cv2.polylines(layer,[visual.boundary],False,COLORS[slot],2,cv2.LINE_AA)
                opacity=max(.22,visual.alpha/.24) if visual.held else 1.
                frame=cv2.addWeighted(layer,opacity,frame,1-opacity,0)
                if visual.held:
                    x,y=np.mean(visual.polygon,axis=0).astype(int)
                    label=f'{"L" if slot==0 else "R"} HOLD {visual.age_ms:.0f}ms'
                    cv2.putText(frame,label,(max(4,min(x-45,510)),max(22,min(y,350))),cv2.FONT_HERSHEY_SIMPLEX,.4,(205,205,205),1,cv2.LINE_AA)
            continue
        if slot in (0, 3) and lane.polygons[0 if slot == 0 else 2] is None:
            continue
        if points is not None and len(points):
            if markings is None:
                cv2.polylines(frame, [points], False, COLORS[slot], 3 if slot in (1, 2) else 2, cv2.LINE_AA)
            else:
                draw_boundary(frame,points,markings[slot],COLORS[slot],3 if slot in (1,2) else 2)
    left, right = lane.boundaries[1:3]
    for index in range(4, 28, 4):
        x = int((left[index, 0] + right[index, 0]) / 2)
        y = int(left[index, 1])
        half = max(4, int((right[index, 0] - left[index, 0]) * .14))
        cv2.polylines(frame, [np.array([[x-half, y+9], [x, y], [x+half, y+9]], np.int32)], False, (110, 250, 115), 2, cv2.LINE_AA)
    return frame


def panel(image, slots, lane, inference_ms, detect_ms, seconds, scene, result_fps,
          backend='OpenCV DNN / CPU / ONNX FP32',adjacent_visuals=None):
    raw = image.copy()
    for slot, points in enumerate(slots):
        for x, y in np.rint(points).astype(np.int32):
            if 0 <= x < 640 and 0 <= y < 360:
                cv2.circle(raw, (x, y), 2, COLORS[slot], -1, cv2.LINE_AA)
    canvas = np.full((480, 1280, 3), (22, 16, 11), np.uint8)
    canvas[70:430, :640] = raw
    canvas[70:430, 640:] = draw(image, lane,adjacent_visuals)
    text = lambda s, xy, scale=.65: cv2.putText(canvas, s, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, (235, 230, 215), 1, cv2.LINE_AA)
    text('UFLDv2 ResNet18 1600x320 | '+backend, (16, 25), .57)
    text('Raw model anchors (four slots)', (16, 56))
    status=lambda v:'--' if v is None else 'HOLD' if v.held else 'LIVE'
    caption='Board geometry + temporal hold' if adjacent_visuals is None else f'Visual tracking | L:{status(adjacent_visuals[0])} R:{status(adjacent_visuals[1])}'
    text(caption, (656, 56),.56)
    count = sum(p is not None for p in lane.polygons)
    text(f'{scene[:28]} t={seconds:.2f}s valid={lane.valid} decision_regions={count} partial={lane.partial} held={lane.held}', (16, 452), .54)
    rate = f'{result_fps:.2f}' if result_fps is not None else 'warming'
    text(f'PC forward={inference_ms:.1f}ms  detect={detect_ms:.1f}ms  completed results={rate}FPS; offline diagnostic', (16, 475), .56)
    return canvas


def export(detector, video, output, stride, max_seconds,compare=False,cache_slots=False,stable_display=True):
    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        raise OSError(f'Cannot open {video}')
    fps = capture.get(cv2.CAP_PROP_FPS)
    if not np.isfinite(fps) or not 1 <= fps <= 240:
        capture.release()
        raise ValueError('Invalid source FPS')
    output.mkdir(parents=True, exist_ok=True)
    destination = output / 'ufld_pc.mp4'
    temporary = output / 'ufld_pc.partial.mp4'
    writer = cv2.VideoWriter(str(temporary), cv2.VideoWriter_fourcc(*'mp4v'), fps / stride, (1280, 480))
    if not writer.isOpened():
        capture.release()
        raise OSError('Cannot create MP4 writer')
    comparison=None
    if compare:
        comparison=cv2.VideoWriter(str(output/'stability_comparison.partial.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),fps/stride,(1280,480))
        if not comparison.isOpened():
            writer.release();capture.release()
            raise OSError('Cannot create comparison MP4')
    rows, tracker, snapshots = [], GeometryTracker(), set()
    visual_tracker=AdjacentLaneVisualTracker(refine)
    cached_slots=[]
    completions = deque(maxlen=15)
    first_completed = last_completed = None
    index = 0
    started = time.perf_counter()
    try:
        while index / fps < max_seconds:
            ok, image = capture.read()
            if not ok:
                break
            if index % stride == 0:
                # Preserve original camera detail for the 1600-wide model.
                # Only the display uses 640x360; downscaling before inference
                # removes thin lane markings and biases the screening result.
                raw, slots, inference_ms, detect_ms = detector.detect(image)
                image = cv2.resize(image, (640, 360))
                lane = tracker.update(raw)
                seconds = index / fps
                visual_started=time.perf_counter()
                visuals=visual_tracker.update(raw,slots,lane,seconds) if stable_display else None
                visual_ms=(time.perf_counter()-visual_started)*1000
                last_completed = time.perf_counter()
                if first_completed is None:
                    first_completed = last_completed
                completions.append(last_completed)
                result_fps = (len(completions) - 1) / (completions[-1] - completions[0]) if len(completions) > 1 else None
                frame = panel(image, slots, lane, inference_ms, detect_ms, seconds, output.name, result_fps,detector.backend,visuals)
                writer.write(frame)
                if comparison is not None:
                    paired=frame.copy()
                    paired[70:430,:640]=draw(image,lane)
                    paired[70:430,640:]=draw(image,lane,visuals)
                    cv2.rectangle(paired,(0,31),(1280,69),(22,16,11),-1)
                    for caption,xy in [('BEFORE: full-span geometry gate',(16,56)),('AFTER: supported span + independent tracking',(656,56))]:
                        cv2.putText(paired,caption,xy,cv2.FONT_HERSHEY_SIMPLEX,.54,(235,230,215),1,cv2.LINE_AA)
                    comparison.write(paired)
                if cache_slots:
                    packed=np.full((4,81,2),np.nan,np.float64)
                    for slot,points in enumerate(slots):packed[slot,:len(points)]=points
                    cached_slots.append(packed)
                shown=[visuals[s] is not None if visuals is not None else lane.polygons[0 if s==0 else 2] is not None for s in (0,1)]
                held=[bool(visuals is not None and visuals[s] is not None and visuals[s].held) for s in (0,1)]
                rows.append(dict(frame=index, seconds=seconds, raw_valid=raw.valid, raw_partial=raw.partial,
                                 raw_regions=sum(p is not None for p in raw.polygons),valid=lane.valid, partial=lane.partial,
                                 held=lane.held, identity_uncertain=lane.uncertain, reassignment=lane.reassignment,
                                 offset=lane.offset, lanes=sum(p is not None for p in lane.polygons),
                                 left_adjacent=lane.polygons[0] is not None, right_adjacent=lane.polygons[2] is not None,
                                 visual_left=shown[0],visual_right=shown[1],
                                 visual_left_held=held[0],visual_right_held=held[1],
                                 visual_left_age_ms=visuals[0].age_ms if visuals is not None and visuals[0] is not None else 0.,
                                 visual_right_age_ms=visuals[1].age_ms if visuals is not None and visuals[1] is not None else 0.,
                                 visual_processing_ms=visual_ms,
                                 forward_ms=inference_ms, detect_ms=detect_ms, completed_result_fps=result_fps,
                                 **{f'coverage_{s}': v for s, v in enumerate(raw.coverage)}))
                for target in (3, 8, 14):
                    if target not in snapshots and seconds >= target:
                        if not cv2.imwrite(str(output / f'frame_{target:02d}s.jpg'), frame):
                            raise OSError('Snapshot write failed')
                        snapshots.add(target)
                        if comparison is not None:cv2.imwrite(str(output/f'comparison_{target:02d}s.jpg'),paired)
                if lane.valid and sum(p is not None for p in lane.polygons) == 3 and not (output / 'three_lanes.jpg').exists():
                    cv2.imwrite(str(output / 'three_lanes.jpg'), frame)
                if raw.valid and not raw.partial and not lane.held and not lane.uncertain and sum(p is not None for p in raw.polygons)==3 and not (output/'three_lanes_confirmed.jpg').exists():
                    # This proof image must retain CURRENT decision geometry,
                    # rather than being labelled confirmed based on held areas.
                    confirmed=panel(image,slots,lane,inference_ms,detect_ms,seconds,output.name,result_fps,detector.backend)
                    cv2.imwrite(str(output/'three_lanes_confirmed.jpg'),confirmed)
                if len(rows) % 20 == 0:
                    print(f'{output.name}: samples={len(rows)} source={seconds:.1f}s detect={detect_ms:.0f}ms lanes={rows[-1]["lanes"]}', flush=True)
            index += 1
    finally:
        capture.release()
        writer.release()
        if comparison is not None:comparison.release()
    if not rows:
        raise ValueError('Empty video test')
    temporary.replace(destination)
    if comparison is not None:(output/'stability_comparison.partial.mp4').replace(output/'stability_comparison.mp4')
    if cache_slots:np.savez_compressed(output/'decoded_slots.npz',slots=np.stack(cached_slots),frames=np.array([r['frame'] for r in rows]),seconds=np.array([r['seconds'] for r in rows]))
    with (output / 'frames.csv').open('w', newline='', encoding='utf-8') as stream:
        csv_writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        csv_writer.writeheader()
        csv_writer.writerows(rows)
    times = np.array([r['detect_ms'] for r in rows])
    result = dict(source=str(video.resolve()), source_fps=fps, stride=stride, samples=len(rows), export_fps=fps / stride,
                  backend=detector.backend, input_source='native camera resolution; 1600x533 resize and bottom 320 crop',
                  temporal_filter='unchanged decision geometry; presentation-only adjacent tracking' if stable_display else 'board geometry gate and bounded hold; no Kalman',
                  raw_complete_fraction=float(np.mean([r['raw_valid'] and not r['raw_partial'] for r in rows])),
                  raw_complete_three_region_fraction=float(np.mean([r['raw_valid'] and not r['raw_partial'] and r['raw_regions']==3 for r in rows])),
                  raw_valid_fraction=float(np.mean([r['raw_valid'] for r in rows])),
                  valid_fraction=float(np.mean([r['valid'] for r in rows])),
                  held_fraction=float(np.mean([r['held'] for r in rows])),
                  three_lane_fraction=float(np.mean([r['lanes'] == 3 for r in rows])),
                  left_adjacent_fraction=float(np.mean([r['left_adjacent'] for r in rows])),
                  right_adjacent_fraction=float(np.mean([r['right_adjacent'] for r in rows])),
                  display_left_fraction=float(np.mean([r['visual_left'] for r in rows])),
                  display_right_fraction=float(np.mean([r['visual_right'] for r in rows])),
                  display_left_held_fraction=float(np.mean([r['visual_left_held'] for r in rows])),
                  display_right_held_fraction=float(np.mean([r['visual_right_held'] for r in rows])),
                  display_hold_max_age_ms=max(max(r['visual_left_age_ms'],r['visual_right_age_ms']) for r in rows),
                  display_transition_counts={side:dict(before=sum(a[side+'_adjacent']!=b[side+'_adjacent'] for a,b in zip(rows,rows[1:])),
                    after=sum(a['visual_'+side]!=b['visual_'+side] for a,b in zip(rows,rows[1:]))) for side in ('left','right')},
                  visual_processing_mean_ms=float(np.mean([r['visual_processing_ms'] for r in rows])),
                  detect_mean_ms=float(times.mean()), detect_p95_ms=float(np.percentile(times, 95)),
                  completed_result_fps=(len(rows) - 1) / (last_completed - first_completed) if len(rows) > 1 else None,
                  forward_mean_ms=float(np.mean([r['forward_ms'] for r in rows])),
                  export_wall_seconds=time.perf_counter() - started, video=str(destination.resolve()),
                  note='Sampled offline diagnostic. Validity is not accuracy. No ground-truth labels, RKNN INT8, shared NPU scheduling, marking semantics or LDW evaluation.')
    (output / 'report.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result), flush=True)
    return result


def play(paths,titles=None):
    title = 'RoadFusion PC | UFLDv2 lane detection test'
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, 1280, 480)
    scene, paused, capture, frame = 0, False, None, None
    try:
        while True:
            if capture is None:
                capture = cv2.VideoCapture(str(paths[scene]))
                if not capture.isOpened():
                    raise OSError(f'Cannot play {paths[scene]}')
                fps = capture.get(cv2.CAP_PROP_FPS)
                width=int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
                height=int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
                if width>0 and height>0:
                    scale=min(1280/width,800/height,1.)
                    cv2.resizeWindow(title,round(width*scale),round(height*scale))
                deadline = time.perf_counter()
                print(f'Playing {scene+1}/{len(paths)}: {paths[scene]}', flush=True)
            if not paused:
                ok, frame = capture.read()
                if not ok:
                    capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
            if frame is not None:
                shown = frame.copy()
                cv2.rectangle(shown, (0, 0), (shown.shape[1], 30), (22, 16, 11), -1)
                name=titles[scene] if titles else paths[scene].parent.name
                cv2.putText(shown, f'{scene+1}/{len(paths)} {name} | TEST REPLAY {fps:g}FPS | 1-{len(paths)} switch | SPACE pause | Q exit', (12, 22), cv2.FONT_HERSHEY_SIMPLEX, .51, (235, 230, 215), 1, cv2.LINE_AA)
                cv2.imshow(title, shown)
            if not paused:
                deadline += 1 / fps
            key = cv2.waitKey(max(1, min(200, int((deadline - time.perf_counter()) * 1000))) if not paused else 30) & 255
            if key in (27, ord('q')) or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == 32:
                paused = not paused
                deadline = time.perf_counter()
            selected = key - ord('1')
            if 0 <= selected < len(paths):
                scene = selected
                capture.release()
                capture, frame = None, None
    finally:
        if capture is not None:
            capture.release()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=ROOT / 'models/ufldv2_culane_res18_1600x320.onnx')
    parser.add_argument('--videos', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, default=ROOT / 'board_v8/output/pc_ufld_test')
    parser.add_argument('--stride', type=int, default=6)
    parser.add_argument('--max-seconds', type=float, default=20)
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--backend', choices=('onnx-cpu','torch-cuda'),default='onnx-cpu')
    parser.add_argument('--checkpoint',type=Path,default=ROOT.parent/'adas_training/weights/culane_res18.pth')
    parser.add_argument('--play', nargs='+', type=Path, help='Replay completed exports without rerunning inference')
    parser.add_argument('--playlist',type=Path,help='Open the curated NVIDIA test manifest')
    parser.add_argument('--compare',action='store_true',help='Export a before/after view from identical per-frame detections')
    parser.add_argument('--cache-slots',action='store_true',help='Save decoded model points for reproducible postprocess replay')
    parser.add_argument('--legacy-overlay',action='store_true',help='Use the previous unsmoothed adjacent-lane overlay')
    args = parser.parse_args()
    if args.playlist:
        clips=json.loads(args.playlist.read_text(encoding='utf-8'))['clips']
        paths=[Path(clip['video']) for clip in clips]
        if not paths or not all(path.is_file() for path in paths):
            parser.error('Playlist contains a missing test export')
        play(paths,[clip['title_en'] for clip in clips])
        return
    if args.play:
        play(args.play)
        return
    if not args.videos or not 1 <= args.stride <= 120 or not 0 < args.max_seconds <= 3600 or not 1 <= args.threads <= 64:
        parser.error('Specify --videos and valid stride/duration/thread limits')
    detector = CudaDetector(args.checkpoint,args.threads) if args.backend=='torch-cuda' else Detector(args.model, args.threads)
    warm = cv2.VideoCapture(str(args.videos[0]))
    ok, image = warm.read()
    warm.release()
    if not ok:
        raise OSError('Cannot read warmup frame')
    for _ in range(2):
        detector.detect(image)
    reports = []
    for video in args.videos:
        name = video.parent.parent.name if video.parent.name == 'original_four_view' else video.parent.name if video.stem=='front' else video.stem
        reports.append(export(detector, video, args.output / name, args.stride, args.max_seconds,args.compare,args.cache_slots,not args.legacy_overlay))
    (args.output / 'summary.json').write_text(json.dumps(dict(model=str(args.model.resolve()), reports=reports), indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
