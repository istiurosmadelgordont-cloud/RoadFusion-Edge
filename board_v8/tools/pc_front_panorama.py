"""Calibrated three-camera front panorama, with synchronized NVIDIA video.

Uses FTheta lens calibration and rotation into a common cylinder. Translation
and scene depth are not corrected: close objects can disagree at the seams.
No generated texture, BEV flattening, YOLO or UFLD is used in this prototype.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

BOARD = Path(__file__).resolve().parents[1]
DEPENDENCIES = BOARD / 'output/pc_surround_review/deps'
sys.path.insert(0, str(DEPENDENCIES))
from scipy.spatial.transform import Rotation
from build_nvidia_avm_calibration import CAMERAS, calibration_table, project_ftheta
from pc_surround_review import NativeReplay
from panorama_seams import SeamConfig, TemporalSeam, disagreement, seam_difference

VIEWS = ('left', 'front', 'right')
TITLE = 'RoadFusion PC | NVIDIA front panorama'


class Panorama:
    def __init__(self, group, width=1920, height=440, horizontal_fov=240, adaptive=True):
        manifest = json.loads((group / 'additional_views/manifest.json').read_text(encoding='utf-8'))
        self.clip = manifest['clip_id']
        root, chunk = group / 'additional_views/calibration', int(manifest['chunk'])
        intrinsics = calibration_table(root, 'camera_intrinsics.offline', chunk, self.clip)
        extrinsics = calibration_table(root, 'sensor_extrinsics.offline', chunk, self.clip)
        yy, xx = np.indices((height, width))
        scale = width / np.deg2rad(horizontal_fov)
        yaw = (.5 - (xx + .5) / width) * np.deg2rad(horizontal_fov)
        # Rig axes are X forward, Y left, Z up. A cylinder keeps azimuth linear.
        rays = np.stack((np.cos(yaw), np.sin(yaw), (height * .43 - yy) / scale), axis=-1)
        self.maps, self.info, scores = {}, {}, []
        for name in VIEWS:
            camera = CAMERAS[name]
            parameters = json.loads(intrinsics.loc[camera, 'model_parameters'])
            pose = extrinsics.loc[camera]
            rotation = Rotation.from_quat(pose[['qx', 'qy', 'qz', 'qw']]).as_matrix()
            local = rays @ rotation
            pixel, valid = project_ftheta(local, parameters)
            w, h = parameters['resolution']
            valid &= (local[..., 2] > 0) & (pixel[..., 0] >= 1) & (pixel[..., 0] < w-2) & (pixel[..., 1] >= 1) & (pixel[..., 1] < h-2)
            ego = cv2.imdecode(np.frombuffer(intrinsics.loc[camera, 'ego_mask_image_png'], np.uint8), cv2.IMREAD_GRAYSCALE)
            if ego is None:
                raise ValueError(f'Missing camera body mask: {camera}')
            mx, my = pixel[..., 0].astype(np.float32), pixel[..., 1].astype(np.float32)
            valid &= cv2.remap(ego, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=255) == 0
            self.maps[name] = (mx, my, valid, (int(w), int(h)))
            # Prefer the camera looking most directly at each direction, with
            # fixed owners across the video. Only the narrow seam is blended.
            score = np.maximum(local[..., 2] / np.linalg.norm(local, axis=-1), 0) ** 4
            scores.append(np.where(valid, score, -1))
            axis = rotation[:, 2]
            self.info[name] = dict(camera=camera, yaw_degrees=float(np.degrees(np.arctan2(axis[1], axis[0]))),
                                   position_metres=pose[['x', 'y', 'z']].to_list())
        stack = np.stack(scores)
        self.owner = stack.argmax(axis=0)
        self.covered = stack.max(axis=0) >= 0
        weights = []
        for index, name in enumerate(VIEWS):
            hard = ((self.owner == index) & self.covered).astype(np.float32)
            soft = cv2.GaussianBlur(hard, (15, 15), 2.5) * self.maps[name][2]
            weights.append(soft)
        total = np.sum(weights, axis=0)
        if np.any(self.covered & (total <= 0)):
            raise ValueError('Covered panorama pixels have no blending weight')
        self.weights = [w / np.maximum(total, 1e-12) for w in weights]
        self.adaptive = adaptive
        self.seams = {}
        self.seam_metrics = []
        self.last_seconds = None
        self.last_baseline = None
        for side, index in (('left', 0), ('right', 2)):
            overlap = self.maps[side][2] & self.maps['front'][2]
            difference = np.where(overlap, abs(stack[index]-stack[1]), np.inf)
            preferred = difference.argmin(axis=1)
            # Rows without overlap keep a representative boundary; ownership
            # will only be changed where both source cameras have real pixels.
            preferred[~overlap.any(axis=1)] = int(np.median(preferred[overlap.any(axis=1)])) if overlap.any() else width//2
            self.seams[side] = TemporalSeam(overlap, preferred, SeamConfig())
        self.gains = {name: np.zeros(3, np.float32) for name in VIEWS}

    def render(self, frames, seconds=None):
        images = {}
        started = time.perf_counter()
        for name in VIEWS:
            mx, my, valid, resolution = self.maps[name]
            source = frames[name]
            if (source.shape[1], source.shape[0]) != resolution:
                raise ValueError(f'Camera resolution does not match calibration: {name}')
            images[name] = cv2.remap(source, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT).astype(np.float32)
        # Restrained, smoothed exposure offsets; no geometric changes per frame.
        for name in ('left', 'right'):
            overlap = self.maps[name][2] & self.maps['front'][2]
            gray = cv2.cvtColor(images['front'].astype(np.uint8), cv2.COLOR_BGR2GRAY)
            grad = np.abs(cv2.Sobel(gray, cv2.CV_32F, 1, 0)) + np.abs(cv2.Sobel(gray, cv2.CV_32F, 0, 1))
            select = overlap & (grad < 35) & (gray > 30) & (gray < 220)
            if select.sum() > 500:
                delta = np.clip(np.median(images['front'][select] - images[name][select], axis=0), -12, 12)
                self.gains[name] = self.gains[name] * .9 + delta * .1
        corrected = {name: image+self.gains[name] for name, image in images.items()}
        self.last_baseline = np.zeros_like(images['front'])
        for name, weight in zip(VIEWS, self.weights):
            self.last_baseline += corrected[name] * weight[..., None]
        self.last_baseline[~self.covered] = (18, 18, 18)
        self.last_baseline = np.clip(self.last_baseline, 0, 255).astype(np.uint8)
        weights = self.weights
        if self.adaptive:
            seconds = float(seconds) if seconds is not None else (0 if self.last_seconds is None else self.last_seconds+.1)
            owner = self.owner.copy()
            xx = np.arange(owner.shape[1])[None, :]
            frame_metrics = {'seconds': seconds}
            for side, index in (('left', 0), ('right', 2)):
                seam = self.seams[side]
                path = seam.update(disagreement(corrected[side], corrected['front']), seconds)
                pair_owner = np.where(xx < path[:, None], 0, 1) if side == 'left' else np.where(xx < path[:, None], 1, 2)
                owner[seam.overlap] = pair_owner[seam.overlap]
                frame_metrics[side] = dict(before=seam_difference(corrected[side], corrected['front'], seam.preferred, seam.overlap),
                                           after=seam_difference(corrected[side], corrected['front'], path, seam.overlap),
                                           motion_pixels=seam.last_motion, reset=seam.last_reset)
            raw_weights = [cv2.GaussianBlur(((owner == index) & self.covered).astype(np.float32), (7, 7), 1.)*self.maps[name][2]
                           for index, name in enumerate(VIEWS)]
            total = np.sum(raw_weights, axis=0)
            if np.any(self.covered & (total <= 0)):
                raise ValueError('Adaptive seam removed observed panorama pixels')
            weights = [weight/np.maximum(total, 1e-12) for weight in raw_weights]
            self.last_weights = weights
            self.seam_metrics.append(frame_metrics)
            self.last_seconds = seconds
        result = np.zeros_like(images['front'])
        for name, weight in zip(VIEWS, weights):
            result += corrected[name] * weight[..., None]
        result[~self.covered] = (18, 18, 18)
        return np.clip(result, 0, 255).astype(np.uint8), (time.perf_counter() - started) * 1000


def panel(frames, panorama, seconds, group, render_ms):
    canvas = np.full((704, 1280, 3), (22, 16, 11), np.uint8)
    text = lambda s, x, y, scale=.6: cv2.putText(canvas, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (240, 235, 220), 1, cv2.LINE_AA)
    text('NVIDIA THREE-CAMERA FRONT PANORAMA | CALIBRATED CYLINDRICAL PROJECTION', 14, 28)
    for index, name in enumerate(VIEWS):
        x0, x1 = round(index * 1280 / 3), round((index + 1) * 1280 / 3)
        canvas[80:320, x0:x1] = cv2.resize(frames[name], (x1-x0, 240))
        text(CAMERAS[name], x0+10, 67, .48)
    canvas[370:663] = cv2.resize(panorama, (1280, 293))
    text(f'{group}  t={seconds:.2f}s | fused 240-degree angular canvas | projection+blend {render_ms:.1f} ms', 14, 353, .53)
    text('Content-aware, rate-limited seams. B before/after | 1/2 switch | SPACE pause | Q exit. Video replay.', 14, 693, .52)
    return canvas


def comparison_panel(before, after, seconds, group):
    canvas = np.full((704, 1280, 3), (22, 16, 11), np.uint8)
    cv2.putText(canvas, f'BEFORE: fixed angular seam | {group} t={seconds:.2f}s', (14, 28), cv2.FONT_HERSHEY_SIMPLEX, .6, (235, 230, 215), 1, cv2.LINE_AA)
    canvas[42:335] = cv2.resize(before, (1280, 293))
    cv2.putText(canvas, 'AFTER: content-aware seam, narrow feather, bounded motion', (14, 364), cv2.FONT_HERSHEY_SIMPLEX, .6, (235, 230, 215), 1, cv2.LINE_AA)
    canvas[377:670] = cv2.resize(after, (1280, 293))
    cv2.putText(canvas, 'Same images, timestamps and projection. Near-object parallax and unobserved areas remain.', (14, 695), cv2.FONT_HERSHEY_SIMPLEX, .52, (235, 230, 215), 1, cv2.LINE_AA)
    return canvas


def export(group, output, start, duration, fps):
    renderer = Panorama(group)
    replay = NativeReplay(group / 'original_four_view', group, DEPENDENCIES)
    if renderer.clip != replay.clip:
        replay.close()
        raise ValueError('Calibration/video clip mismatch')
    output.mkdir(parents=True, exist_ok=True)
    temporary = output / 'front_panorama.partial.mp4'
    destination = output / 'front_panorama.mp4'
    writer = cv2.VideoWriter(str(temporary), cv2.VideoWriter_fourcc(*'mp4v'), fps, (1280, 704))
    comparison_temporary = output / 'comparison.partial.mp4'
    comparison_writer = cv2.VideoWriter(str(comparison_temporary), cv2.VideoWriter_fourcc(*'mp4v'), fps, (1280, 704))
    if not writer.isOpened() or not comparison_writer.isOpened():
        replay.close()
        writer.release()
        comparison_writer.release()
        raise OSError('Cannot write panorama video')
    timings, selected_times, written = [], [], 0
    last_index = -1
    try:
        for seconds in np.arange(start, min(start + duration, replay.duration), 1 / fps):
            index = int(np.argmin(abs(replay.seconds - seconds)))
            if index == last_index:
                continue
            last_index = index
            frames = replay.read(index)
            fused, elapsed = renderer.render(frames, replay.seconds[index])
            shown = panel(frames, fused, replay.seconds[index], group.name, elapsed)
            compared = comparison_panel(renderer.last_baseline, fused, replay.seconds[index], group.name)
            writer.write(shown)
            comparison_writer.write(compared)
            if written in (0, round(fps*2), round(fps*4)):
                cv2.imwrite(str(output / f'preview_{written:03d}.jpg'), shown)
                cv2.imwrite(str(output / f'panorama_{written:03d}.jpg'), fused)
                cv2.imwrite(str(output / f'comparison_{written:03d}.jpg'), compared)
            timings.append(elapsed)
            selected_times.append(float(replay.seconds[index]))
            written += 1
            if written % 20 == 0:
                print(f'{group.name}: fused {written} frames', flush=True)
    finally:
        writer.release()
        comparison_writer.release()
        replay.close()
    if not written:
        raise ValueError('No frames exported')
    temporary.replace(destination)
    comparison_temporary.replace(output / 'comparison.mp4')
    seam_summary = {}
    for side in ('left', 'right'):
        metrics = [r[side] for r in renderer.seam_metrics]
        seam_summary[side] = dict(mean_color_difference_before=float(np.mean([r['before'] for r in metrics if r['before'] is not None])),
                                  mean_color_difference_after=float(np.mean([r['after'] for r in metrics if r['after'] is not None])),
                                  max_step_pixels=float(max(r['motion_pixels'] for r in metrics)),
                                  resets=sum(r['reset'] for r in metrics))
    report = dict(clip_id=renderer.clip, cameras=renderer.info, frames=written, export_fps=fps,
                  source_times=selected_times, timestamp_max_difference_ms={k: replay.max_delta_ms[k] for k in VIEWS},
                  covered_angular_canvas_fraction=float(renderer.covered.mean()),
                  overlap_front_left_fraction=float((renderer.maps['front'][2] & renderer.maps['left'][2]).mean()),
                  overlap_front_right_fraction=float((renderer.maps['front'][2] & renderer.maps['right'][2]).mean()),
                  render_mean_ms=float(np.mean(timings)), render_p95_ms=float(np.percentile(timings,95)),
                  seam_summary=seam_summary, seam_config=vars(SeamConfig()),
                  projection='240-degree cylindrical canvas, calibrated rotation and FTheta, translation/depth uncorrected',
                  note='No detection inference. Angular coverage is not geometric accuracy or complete 360 AVM.', video=str(destination.resolve()))
    (output / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    (output / 'seams.json').write_text(json.dumps(renderer.seam_metrics, indent=2), encoding='utf-8')
    print(json.dumps(report), flush=True)


def play(paths):
    cv2.namedWindow(TITLE, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(TITLE, 1280, 704)
    index, paused, cap, frame = 0, False, None, None
    compare = False
    resume_seconds = 0.
    try:
        while True:
            if cap is None:
                path = paths[index].parent / 'comparison.mp4' if compare else paths[index]
                cap = cv2.VideoCapture(str(path))
                if not cap.isOpened():
                    raise OSError(f'Cannot open preview: {paths[index]}')
                fps = cap.get(cv2.CAP_PROP_FPS)
                if resume_seconds:
                    cap.set(cv2.CAP_PROP_POS_MSEC, resume_seconds*1000)
                    resume_seconds = 0.
                deadline = time.perf_counter()
            if not paused or frame is None:
                ok, frame = cap.read()
                if not ok:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    deadline = time.perf_counter()
                    continue
            cv2.imshow(TITLE, frame)
            if not paused:
                deadline += 1/fps
            key = cv2.waitKey(max(1, min(100, int((deadline-time.perf_counter())*1000))) if not paused else 30) & 255
            if key in (27, ord('q')) or cv2.getWindowProperty(TITLE, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == 32:
                paused = not paused
                deadline = time.perf_counter()
            if key == ord('b') and (paths[index].parent / 'comparison.mp4').is_file():
                compare = not compare
                resume_seconds = max(0., cap.get(cv2.CAP_PROP_POS_MSEC)/1000-1/fps)
                cap.release()
                cap, frame = None, None
            selected = key - ord('1')
            if 0 <= selected < len(paths):
                index = selected
                cap.release()
                cap, frame = None, None
                resume_seconds = 0.
    finally:
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--groups', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, default=BOARD / 'output/pc_front_panorama_v2')
    parser.add_argument('--start', type=float, default=3)
    parser.add_argument('--duration', type=float, default=5)
    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--play', nargs='+', type=Path)
    args = parser.parse_args()
    if args.play:
        play(args.play)
    else:
        if not args.groups or not 1 <= args.fps <= 30 or not 0 <= args.start <= 19 or not 0 < args.duration <= 20:
            parser.error('Specify groups and valid time/FPS limits')
        cv2.setNumThreads(4)
        for group in args.groups:
            export(group, args.output / group.name, args.start, args.duration, args.fps)
