"""Review one NVIDIA rig with its own calibration and source timestamps.

Writes strict four/seven-camera comparisons and a seven-camera AVM video.
Coverage always describes observed ground, before display texture extension.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from pc_avm_preview import DEFAULT_SCENE, NAMES, ROOT, Renderer, panel

PLAYER_TITLE = 'RoadFusion PC | NVIDIA 7-camera AVM review'


def focus_existing_player():
    if sys.platform != 'win32':
        return False
    import ctypes
    user32 = ctypes.windll.user32
    user32.FindWindowW.restype = ctypes.c_void_p
    handle = user32.FindWindowW(None, PLAYER_TITLE)
    if not handle:
        return False
    user32.ShowWindow(ctypes.c_void_p(handle), 9)
    user32.SetForegroundWindow(ctypes.c_void_p(handle))
    print('The AVM player is already open; brought its window to the front.', flush=True)
    return True


def nearest(reference, samples):
    after = np.searchsorted(samples, reference).clip(0, len(samples) - 1)
    before = np.maximum(0, after - 1)
    return np.where(abs(samples[before] - reference) <= abs(samples[after] - reference),
                    before, after)


class NativeReplay:
    def __init__(self, scene, group, dependencies):
        sys.path.insert(0, str(dependencies))
        import pandas as pd

        extra = json.loads((group / 'additional_views/manifest.json').read_text(encoding='utf-8'))
        self.clip = extra['clip_id']
        if (scene / 'manifest.json').is_file():
            primary = json.loads((scene / 'manifest.json').read_text(encoding='utf-8'))
            if primary['scene'] != extra['clip_id']:
                raise ValueError('Primary and extra cameras belong to different clips')
            paths = {k: Path(v['path']) for k, v in primary['source'].items()}
        else:
            paths = {k: scene / (k+'.mp4') for k in NAMES[:4]}
        paths.update({v['view']: group / 'additional_views' / v['video'] for v in extra['views']})
        timestamps = {}
        self.caps, self.next, self.metadata = {}, {}, {}
        try:
            for name in NAMES:
                path = paths[name]
                timestamp_path = path.with_suffix('.timestamps.parquet')
                samples = pd.read_parquet(timestamp_path)['timestamp'].to_numpy(dtype=np.int64)
                cap = cv2.VideoCapture(str(path))
                self.caps[name] = cap
                if not cap.isOpened() or len(samples) != int(cap.get(cv2.CAP_PROP_FRAME_COUNT)):
                    raise ValueError(f'Video/timestamp mismatch: {name}')
                if len(samples) < 2 or np.any(np.diff(samples) <= 0):
                    raise ValueError(f'Invalid camera timestamps: {name}')
                timestamps[name] = samples
                self.next[name] = 0
                self.metadata[name] = {'path': str(path), 'frames': len(samples),
                    'fps': cap.get(cv2.CAP_PROP_FPS),
                    'width': int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                    'height': int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}
            # Clips have equal frame counts but different exposure start times.
            # Restrict export to the shared time window; never repeat an ended
            # camera to manufacture rear coverage in the final few frames.
            common_start = max(v[0] for v in timestamps.values())
            common_end = min(v[-1] for v in timestamps.values())
            front_times = timestamps['front']
            self.reference = front_times[(front_times >= common_start) & (front_times <= common_end)]
            if len(self.reference) < 2:
                raise ValueError('Cameras have no shared replay window')
            self.seconds = (self.reference - self.reference[0]) / 1e6
            self.duration = self.seconds[-1] + np.median(np.diff(self.seconds))
            self.matches = {k: nearest(self.reference, v) for k, v in timestamps.items()}
            self.max_delta_ms = {k: float(np.max(abs(timestamps[k][v] - self.reference)) / 1000)
                                 for k, v in self.matches.items()}
            if max(self.max_delta_ms.values()) > 34:
                raise ValueError('Timestamp alignment exceeds one 30 FPS frame')
        except Exception:
            self.close()
            raise

    def read(self, index):
        frames = {}
        for name, cap in self.caps.items():
            target = int(self.matches[name][index])
            if target < self.next[name] or target - self.next[name] > 45:
                cap.set(cv2.CAP_PROP_POS_FRAMES, target)
                self.next[name] = target
            while self.next[name] < target:
                if not cap.grab():
                    raise RuntimeError(f'Unexpected video end: {name}')
                self.next[name] += 1
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError(f'Cannot decode {name} frame {target}')
            self.next[name] += 1
            frames[name] = frame
        return frames

    def close(self):
        for cap in self.caps.values():
            cap.release()


def coverage(renderer, calibration):
    yy, xx = np.indices((256, 256))
    fs = cv2.FileStorage(str(calibration), cv2.FILE_STORAGE_READ)
    width = fs.getNode('vehicle_width').real()
    fs.release()
    ppm = renderer.ppm * .4
    body = (abs(xx - 128) <= width * ppm / 2) & (abs(yy - 128) <= renderer.length * ppm / 2)
    regions = {'outside_body': ~body,
        'left_near': (xx < 128 - width * ppm / 2) & (abs(yy - 128) < (renderer.length / 2 + 3) * ppm),
        'right_near': (xx > 128 + width * ppm / 2) & (abs(yy - 128) < (renderer.length / 2 + 3) * ppm),
        'rear_left': (xx < 128 - width * ppm / 2) & (yy > 128 + renderer.length * ppm / 2),
        'rear_right': (xx > 128 + width * ppm / 2) & (yy > 128 + renderer.length * ppm / 2)}
    result, maps = {}, []
    for count in [4, 7]:
        observed = np.logical_or.reduce([renderer.maps[k][2] > 0 for k in NAMES[:count]])
        result[str(count)] = {name: float(observed[region].mean()) for name, region in regions.items()}
        image = np.full((256, 256, 3), (32, 40, 48), np.uint8)
        image[observed] = (128, 156, 70)
        image[body] = (190, 190, 190)
        maps.append(cv2.resize(image, (640, 640), interpolation=cv2.INTER_NEAREST))
    return result, maps


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, default=DEFAULT_SCENE)
    parser.add_argument('--group', type=Path)
    parser.add_argument('--calibration', type=Path, default=ROOT / 'output/calibrated_bev/seven.yml')
    parser.add_argument('--dependencies', type=Path, default=ROOT / 'output/pc_surround_review/deps')
    parser.add_argument('--output', type=Path, default=ROOT / 'output/pc_surround_review')
    parser.add_argument('--video-fps', type=int, default=30,
                        help='Source sampling frame rate for export (1-30, default: 30)')
    parser.add_argument('--snapshots-only', action='store_true')
    parser.add_argument('--near-view', action='store_true')
    parser.add_argument('--seam-refinement', action='store_true')
    parser.add_argument('--antialias-ground', action='store_true')
    parser.add_argument('--play', action='store_true', help='Open the already exported review video')
    parser.add_argument('--play-all', action='store_true', help='Open generated groups with scene switching')
    parser.add_argument('--diagnostics', action='store_true', help='Log GUI submission timing; not sensor or inference FPS')
    args = parser.parse_args()
    if args.play or args.play_all:
        if focus_existing_player():
            return
        videos = sorted((ROOT / 'output/pc_surround_groups').glob('*/nvidia_seven_camera_avm.mp4')) if args.play_all else [args.output / 'nvidia_seven_camera_avm.mp4']
        if not videos:
            raise ValueError('No exported AVM groups found')
        play(videos, args.diagnostics)
        return
    if not 1 <= args.video_fps <= 30:
        raise ValueError('Preview export FPS must be between 1 and 30')
    args.output.mkdir(parents=True, exist_ok=True)
    renderer = Renderer(args.calibration)
    renderer.surround_guides = True
    renderer.seam_refinement = args.seam_refinement
    renderer.antialias_ground = args.antialias_ground
    renderer.display_crop = 512 if args.near_view else 640
    group = args.group or args.scene.parents[1]
    replay = NativeReplay(args.scene, group, args.dependencies)
    if renderer.clip_id and renderer.clip_id != replay.clip:
        replay.close()
        raise ValueError('Calibration and source cameras belong to different clips')
    fractions, maps = coverage(renderer, args.calibration)
    cv2.imwrite(str(args.output / 'coverage_four_vs_seven.png'), np.concatenate(maps, axis=1))
    report = {'clip_id': replay.clip, 'cameras': replay.metadata,
        'calibration_clip_id': renderer.clip_id or None,
        'max_timestamp_difference_ms': replay.max_delta_ms,
        'coverage_fractions_before_visual_fill': fractions,
        'duration_seconds': replay.duration, 'export_fps': args.video_fps,
        'display_extent_fraction': renderer.display_crop/640,
        'seam_refinement': args.seam_refinement,
        'antialias_ground': args.antialias_ground,
        'notes': ['Coverage is geometric ground validity; not scene recognition accuracy.',
                  'Texture extension is presentation only; elevated objects still stretch.',
                  'No YOLO/UFLD inference is performed in this AVM review.']}
    writer = None
    try:
        for seconds in [3., 8., 14.]:
            index = int(np.argmin(abs(replay.seconds - seconds)))
            frames = replay.read(index)
            four, mask4 = renderer.ground({k: frames[k] for k in NAMES[:4]})
            seven, mask7 = renderer.ground(frames)
            strict4, _ = renderer.view(four, mask4, False)
            strict7, _ = renderer.view(seven, mask7, False)
            display, _ = renderer.view(seven, mask7, True)
            pictures = [strict4, strict7, display]
            comparison = np.full((700, 1920, 3), (17, 25, 34), np.uint8)
            for i, (title, picture) in enumerate(zip(
                ['4 CAMERAS: OBSERVED ONLY', '7 CAMERAS: OBSERVED ONLY',
                 '7 CAMERAS: DISPLAY TEXTURE EXTENSION'], pictures)):
                cv2.putText(comparison, title, (i * 640 + 12, 30), 0, .52, (230, 224, 207), 1, cv2.LINE_AA)
                comparison[45:685, i * 640:(i + 1) * 640] = picture
            cv2.imwrite(str(args.output / f'comparison_{seconds:04.1f}s.jpg'), comparison)
            cv2.imwrite(str(args.output / f'ui_{seconds:04.1f}s.jpg'),
                        panel(frames, display, True, seconds, replay.duration))
            print(f'Snapshot {seconds:.1f}s saved', flush=True)
        if not args.snapshots_only:
            renderer.color_offsets = None
            staged_video = args.output / 'nvidia_seven_camera_avm.partial.mp4'
            writer = cv2.VideoWriter(str(staged_video),
                                     cv2.VideoWriter_fourcc(*'mp4v'), args.video_fps, (1440, 840))
            if not writer.isOpened():
                raise RuntimeError('Cannot create preview video')
            samples = np.arange(0, replay.duration, 1 / args.video_fps)
            begun = time.perf_counter()
            for i, seconds in enumerate(samples):
                index = int(np.argmin(abs(replay.seconds - seconds)))
                frames = replay.read(index)
                ground, observed = renderer.ground(frames)
                display, _ = renderer.view(ground, observed, True)
                writer.write(panel(frames, display, True, seconds, replay.duration))
                if i % 30 == 0:
                    print(f'Export {i}/{len(samples)}', flush=True)
            report['export_frames'] = len(samples)
            report['export_processing_seconds'] = time.perf_counter() - begun
            writer.release()
            writer = None
            staged_video.replace(args.output / 'nvidia_seven_camera_avm.mp4')
        (args.output / 'review.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps({'coverage': fractions, 'max_timestamp_difference_ms': replay.max_delta_ms}), flush=True)
    finally:
        if writer is not None:
            writer.release()
        replay.close()


def player_buttons(paths, height=880):
    # Coordinates are in the displayed image; OpenCV maps scaled window clicks.
    buttons = []
    for i, path in enumerate(paths):
        column, row = i % 8, i // 8
        name = path.parent.name.split('_', 1)[-1].replace('_', ' ')
        buttons.append(((12 + column*178, 90 + row*36, 182 + column*178, 122 + row*36),
                        f'{i+1}: {name[:18]}', ('scene', i)))
    for i, (label, action) in enumerate([('PREVIOUS', 'previous'), ('NEXT', 'next'),
            ('PAUSE / PLAY', 'pause'), ('ENLARGE', 'enlarge'), ('COMPARE', 'compare')]):
        buttons.append(((420+i*195, height-46, 608+i*195, height-8), label, (action, None)))
    return buttons


def button_at(buttons, x, y):
    for (left, top, right, bottom), _, action in buttons:
        if left <= x <= right and top <= y <= bottom:
            return action
    return None


def load_seam_overlay(path):
    """Show the exported rig's actual static camera boundaries, not auto-alignment."""
    report_path, calibration = path.parent / 'review.json', path.parent / 'seven.yml'
    if not report_path.is_file() or not calibration.is_file():
        return None
    report = json.loads(report_path.read_text(encoding='utf-8'))
    if not report.get('seam_refinement'):
        return None
    renderer = Renderer(calibration)
    if renderer.clip_id and renderer.clip_id != report['clip_id']:
        raise ValueError('Seam inspection calibration belongs to another clip')
    active = tuple((name,report['cameras'][name]['width'],report['cameras'][name]['height'])
                   for name in NAMES if name in report['cameras'] and name in renderer.maps)
    weights = renderer.seam_weights(active)
    valid = np.sum(weights,axis=0)>1e-5
    owner = np.argmax(weights,axis=0)
    edges = np.zeros_like(valid)
    horizontal = (owner[:,1:]!=owner[:,:-1]) & valid[:,1:] & valid[:,:-1]
    vertical = (owner[1:]!=owner[:-1]) & valid[1:] & valid[:-1]
    edges[:,1:] |= horizontal
    edges[1:] |= vertical
    colors = np.array([(255,180,70),(100,100,255),(100,255,120),
                       (70,230,255),(255,100,230),(255,230,100),(170,130,255)],np.uint8)
    lines = colors[owner]
    lines[~edges] = 0
    lines = cv2.resize(lines,(640,640),interpolation=cv2.INTER_NEAREST)
    # Match the same sprite bounds and central crop used by Renderer.view().
    height = round(renderer.length*renderer.ppm*1.55)
    width = round(height*renderer.asset.shape[1]/renderer.asset.shape[0])
    lines[320-height//2-3:320+(height+1)//2+3,320-width//2-3:320+(width+1)//2+3] = 0
    crop = int(round(report['display_extent_fraction']*640))
    margin = (640-crop)//2
    return cv2.resize(lines[margin:margin+crop,margin:margin+crop],(600,600),interpolation=cv2.INTER_NEAREST)


def player_frame(frame, paths, selected, seconds, duration, fps, paused,
                 enlarged=False, comparison=None, seam_overlay=None):
    if seam_overlay is not None and comparison is None:
        frame = frame.copy()
        tile = frame[110:710,820:1420]
        present = np.any(seam_overlay,axis=2)
        tile[present] = seam_overlay[present]
    height = 960 if enlarged else 880
    if comparison is not None:
        shown = np.full((height,1440,3), (17,25,34), np.uint8)
        shown[135:660] = cv2.resize(comparison, (1440,525))
        cv2.putText(shown, '4 cameras observed / 7 cameras observed / display fill - fixed 8s comparison',
                    (25,735), 0, .62, (230,224,207), 1, cv2.LINE_AA)
    elif enlarged:
        shown = np.full((height,1440,3), (17,25,34), np.uint8)
        shown[130:890,340:1100] = cv2.resize(frame[110:710,820:1420], (760,760))
    else:
        shown = np.full((height,1440,3), (17,25,34), np.uint8)
        # Keep the full exported camera/BEV content below the scene toolbar.
        shown[134:814] = frame[110:790]
    cv2.rectangle(shown, (0,0), (1439,128), (17,25,34), -1)
    cv2.putText(shown, f'NVIDIA AVM REVIEW  {selected+1}/{len(paths)}  {paths[selected].parent.name}',
                (22,35), 0, .72, (232,218,187), 1, cv2.LINE_AA)
    cv2.putText(shown, '7 timestamp-aligned cameras | Display fill is approximate | No live inference',
                (22,70), 0, .53, (112,201,236), 1, cv2.LINE_AA)
    if seam_overlay is not None:
        cv2.rectangle(shown,(0,46),(1439,82),(17,25,34),-1)
        cv2.putText(shown,'SEAM CHECK (S): colored lines = camera boundaries; inspect road continuity, not vehicle roofs',
                    (22,70),0,.52,(112,201,236),1,cv2.LINE_AA)
    cv2.rectangle(shown, (0,height-48), (1439,height-1), (17,25,34), -1)
    status = 'PAUSED' if paused else '1.0x'
    cv2.putText(shown, f'{seconds:04.1f}/{duration:04.1f}s | {fps:g} FPS export | {status}',
                (12,height-18), 0, .46, (205,198,169), 1, cv2.LINE_AA)
    buttons = player_buttons(paths, height)
    for bounds, label, action in buttons:
        left, top, right, bottom = bounds
        active = action == ('scene', selected) or (action[0]=='pause' and paused) or (action[0]=='enlarge' and enlarged) or (action[0]=='compare' and comparison is not None)
        if action[0] == 'pause':
            label = 'RESUME' if paused else 'PAUSE'
        cv2.rectangle(shown, (left,top), (right,bottom), (112,78,22) if active else (45,39,30), -1)
        cv2.rectangle(shown, (left,top), (right,bottom), (220,170,65) if active else (95,83,61), 1)
        cv2.putText(shown, label, (left+6,top+23), 0, .40, (240,230,211), 1, cv2.LINE_AA)
    return shown, buttons


def play(paths, diagnostics=False):
    timer = None
    if sys.platform == 'win32':
        import ctypes
        # Prevent Windows scaling the requested 1400 px window beyond the screen.
        user32 = ctypes.windll.user32
        user32.SetProcessDpiAwarenessContext(ctypes.c_void_p(-4))
        screen_width, screen_height = user32.GetSystemMetrics(0), user32.GetSystemMetrics(1)
    else:
        screen_width, screen_height = 1600, 1080

    def resize_window(height):
        scale = min(1400/1440, (screen_width-60)/1440, (screen_height-110)/height)
        cv2.resizeWindow(title, int(1440*scale), int(height*scale))
    selected = 0
    cap = cv2.VideoCapture(str(paths[selected]))
    if not cap.isOpened():
        raise ValueError(f'Preview video missing: {paths[selected]}')
    title = PLAYER_TITLE
    fps = cap.get(cv2.CAP_PROP_FPS)
    count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if fps <= 0 or count <= 0:
        cap.release()
        raise ValueError('Preview has no decodable timeline')
    started, paused, held, last = time.perf_counter(), False, 0., -1
    enlarged, compare = False, False
    inspect_seams = False
    seam_overlay = load_seam_overlay(paths[selected])
    comparison = cv2.imread(str(paths[selected].parent / 'comparison_08.0s.jpg'))
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    resize_window(880)
    cv2.moveWindow(title, 30, 50)
    buttons = player_buttons(paths)
    clicks = []
    redraw, canvas_height = True, 880
    submitted_at, last_submit = [], None
    skipped, stats_start = 0, time.perf_counter()

    def clicked(event, x, y, flags, data):
        if event == cv2.EVENT_LBUTTONUP:
            action = button_at(buttons, x, y)
            if action is not None:
                clicks.append(action)

    cv2.setMouseCallback(title, clicked)
    if sys.platform == 'win32':
        # Scope finer timer resolution to this active preview only.
        winmm = ctypes.windll.winmm
        if winmm.timeBeginPeriod(1) == 0:
            timer = winmm
    print(f'NVIDIA AVM review opened: {len(paths)} scenes. Click numbered scene buttons; arrows or A/D also switch. Exported video, no live inference.', flush=True)
    try:
        while True:
            new_frame = False
            seconds = held if paused else (time.perf_counter() - started) % (count / fps)
            index = min(count-1, int(seconds*fps))
            if index != last:
                if diagnostics and last >= 0 and index > last+1:
                    skipped += index-last-1
                if index != last + 1:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, index)
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError('Preview video failed to decode')
                last = index
                new_frame = True
                redraw = True
            if redraw:
                shown, buttons = player_frame(frame, paths, selected, seconds, count/fps, fps,
                    paused, enlarged, comparison if compare else None,
                    seam_overlay if inspect_seams else None)
                if shown.shape[0] != canvas_height:
                    canvas_height = shown.shape[0]
                    resize_window(canvas_height)
                cv2.imshow(title, shown)
                if diagnostics and new_frame and not paused:
                    submitted = time.perf_counter()
                    if last_submit is not None:
                        submitted_at.append(submitted-last_submit)
                    last_submit = submitted
                    if submitted-stats_start >= 10 and submitted_at:
                        print(f'GUI timing scene={selected+1}: source={fps:g} FPS, '
                              f'submit_gap_p95={np.percentile(submitted_at,95)*1000:.1f}ms, '
                              f'skipped_video_frames={skipped}; not inference FPS', flush=True)
                        submitted_at, stats_start, skipped = [], submitted, 0
                redraw = False
            if paused:
                delay = 15
            else:
                clock = (time.perf_counter()-started) % (count/fps)
                remaining = (index+1)/fps-clock
                delay = max(1,min(15,int(np.ceil(remaining*1000))))
            key = cv2.waitKeyEx(delay)
            if key in (27, ord('q')) or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                break
            command = clicks.pop(0) if clicks else (None, None)
            action, target = command
            if key == ord(' ') or action == 'pause':
                if paused:
                    started = time.perf_counter() - held
                else:
                    held = seconds
                paused = not paused
                submitted_at, last_submit, skipped, stats_start = [], None, 0, time.perf_counter()
                redraw = True
            if key == ord('r'):
                started, held, last = time.perf_counter(), 0., -1
            if key == ord('v') or action == 'enlarge':
                enlarged, compare = not enlarged, False
                redraw = True
            if key == ord('b') or action == 'compare':
                compare = not compare
                enlarged = False
                redraw = True
            if key == ord('s'):
                inspect_seams = not inspect_seams
                compare = False
                redraw = True
            if key in (ord('['), ord('a'), 2424832) or action == 'previous':
                target = (selected-1) % len(paths)
            elif key in (ord(']'), ord('d'), 2555904) or action == 'next':
                target = (selected+1) % len(paths)
            elif ord('1') <= key <= ord('9') and key-ord('1') < len(paths):
                target = key-ord('1')
            if target is not None and target != selected:
                selected = target
                cap.release()
                cap = cv2.VideoCapture(str(paths[selected]))
                if not cap.isOpened():
                    raise ValueError(f'Cannot open {paths[selected]}')
                fps = cap.get(cv2.CAP_PROP_FPS)
                count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
                if fps <= 0 or count <= 0:
                    raise ValueError(f'Preview has no decodable timeline: {paths[selected]}')
                comparison = cv2.imread(str(paths[selected].parent / 'comparison_08.0s.jpg'))
                seam_overlay = load_seam_overlay(paths[selected])
                started, held, last, paused = time.perf_counter(), 0., -1, False
                submitted_at, last_submit, skipped, stats_start = [], None, 0, time.perf_counter()
                print(f'SCENE {selected+1}/{len(paths)} {paths[selected].parent.name}', flush=True)
    finally:
        if timer is not None:
            timer.timeEndPeriod(1)
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
