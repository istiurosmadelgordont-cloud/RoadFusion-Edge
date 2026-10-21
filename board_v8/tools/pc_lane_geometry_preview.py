"""Replay cached PC detections as filled lanes with directional chevrons."""
import argparse
import csv
import hashlib
import json
from collections import Counter
from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np

from pc_ufld_video import GeometryTracker, draw, make_lane, refine
from pc_lane_visual import AdjacentLaneVisualTracker
from pc_lane_markings import MarkingTracker, PaintFrame


@lru_cache(maxsize=18)
def marking_caption(left, right, original_left=False):
    from PIL import Image, ImageDraw, ImageFont
    font = ImageFont.truetype('C:/Windows/Fonts/msyh.ttc', 18)
    image = Image.new('RGB', (1280, 32), (11, 16, 22))
    names = {'SOLID':'实线', 'DASHED':'虚线', 'UNKNOWN':'未知'}
    left_label = '原始视频' if original_left else '原有显示'
    text = (f'左图：{left_label}    右图：虚实线区分    |    '
        f'本车道左边界：{names[left]}    右边界：{names[right]}    |    未知：灰色点线')
    ImageDraw.Draw(image).text((12, 3), text, font=font, fill=(235, 230, 215))
    return np.asarray(image)[:, :, ::-1].copy()


def export(clip, output, distinguish_markings=False, original_left=False):
    cache = Path(clip.get('cached_points') or Path(clip['video']).parent/'decoded_slots.npz')
    report_path = Path(clip.get('inference_report') or cache.parent/'report.json')
    source = Path(clip['source_video'])
    cache_hash = hashlib.sha256(cache.read_bytes()).hexdigest()
    if clip.get('cache_sha256') and clip['cache_sha256'] != cache_hash:
        raise ValueError('Detection cache changed since inference')
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    if clip.get('source_sha256') and clip['source_sha256'] != source_hash:
        raise ValueError('Source video changed since inference')
    with np.load(cache) as saved:
        packed, indices, seconds = saved['slots'], saved['frames'], saved['seconds']
    with (report_path.parent/'frames.csv').open(newline='', encoding='utf-8') as stream:
        expected = list(csv.DictReader(stream))
    if (not len(indices) or packed.shape != (len(indices), 4, 81, 2) or
            not np.all(np.diff(indices) == 1) or len(expected) != len(indices)):
        raise ValueError('Expected a complete sequential inference cache')
    capture = cv2.VideoCapture(str(source))
    fps = capture.get(cv2.CAP_PROP_FPS)
    if not capture.isOpened() or fps <= 0 or not np.allclose(seconds, indices/fps, atol=1e-6):
        capture.release()
        raise ValueError('Source video and detection timestamps differ')
    output.mkdir(parents=True, exist_ok=True)
    destination = output/'filled_lanes.mp4'
    temporary = output/'filled_lanes.partial.mp4'
    writer = cv2.VideoWriter(str(temporary), cv2.VideoWriter_fourcc(*'mp4v'), fps, (1280, 392))
    if not writer.isOpened():
        capture.release()
        raise OSError('Cannot create filled-lane video')
    tracker = GeometryTracker()
    visual_tracker = AdjacentLaneVisualTracker(refine)
    marking_tracker = MarkingTracker()
    marking_rows = []
    capture.set(cv2.CAP_PROP_POS_FRAMES, int(indices[0]))
    try:
        for n, (frame_index, points_by_slot) in enumerate(zip(indices, packed)):
            ok, native = capture.read()
            if not ok:
                raise ValueError(f'Source truncated at frame {frame_index}')
            slots = [points[np.isfinite(points).all(axis=1)] for points in points_by_slot]
            raw = make_lane(slots)
            lane = tracker.update(raw)
            visuals = visual_tracker.update(raw, slots, lane, float(seconds[n]))
            values = dict(raw_valid=raw.valid, raw_partial=raw.partial,
                raw_regions=sum(p is not None for p in raw.polygons), valid=lane.valid,
                partial=lane.partial, held=lane.held, identity_uncertain=lane.uncertain,
                reassignment=lane.reassignment, lanes=sum(p is not None for p in lane.polygons),
                left_adjacent=lane.polygons[0] is not None, right_adjacent=lane.polygons[2] is not None,
                visual_left=visuals[0] is not None, visual_right=visuals[1] is not None,
                visual_left_held=bool(visuals[0] is not None and visuals[0].held),
                visual_right_held=bool(visuals[1] is not None and visuals[1].held))
            row = expected[n]
            if int(row['frame']) != frame_index or abs(float(row['seconds'])-seconds[n]) > 1e-6:
                raise ValueError('Frame order differs from the inference report')
            for key, value in values.items():
                if key in row and str(value) != row[key]:
                    raise ValueError(f'Geometry replay differs at {frame_index}: {key}')
            if abs(float(row['offset'])-lane.offset) > 1e-7:
                raise ValueError(f'Geometry offset differs at {frame_index}')
            for side, visual in zip(('left', 'right'), visuals):
                age = visual.age_ms if visual is not None else 0.
                if abs(float(row[f'visual_{side}_age_ms'])-age) > 1e-6:
                    raise ValueError(f'Visual age differs at {frame_index}')
            markings = None
            if distinguish_markings:
                evidence_curves = list(raw.boundaries)
                for side, slot in enumerate((0,3)):
                    visual = visuals[side]
                    evidence_curves[slot] = (visual.boundary if visual is not None and not visual.held else None)
                paint = PaintFrame(native)
                observations = [paint.observe(points) for points in evidence_curves]
                markings = marking_tracker.update(observations,lane,float(seconds[n]),
                    tuple(v is not None and v.held for v in visuals))
                marking_rows.append(dict(frame=int(frame_index), seconds=float(seconds[n]),
                    **{f'slot_{slot}':value for slot,value in enumerate(markings)},
                    **{f'observed_{slot}':value.label for slot,value in enumerate(observations)},
                    **{f'paint_coverage_{slot}':value.coverage for slot,value in enumerate(observations)}))
            image = cv2.resize(native, (640, 360))
            canvas = np.full((392, 1280, 3), (22, 16, 11), np.uint8)
            canvas[:360, :640] = image if original_left else draw(image, lane)
            canvas[:360, 640:] = draw(image, lane, visuals, markings)
            caption = (f'{clip["clip_id"][:8]} t={seconds[n]:.2f}s valid={lane.valid} '
                f'decision_regions={values["lanes"]} partial={lane.partial} held={lane.held}'
                f'  |  L: {"original" if original_left else "geometry"}  R: stabilized')
            if markings is None:
                cv2.putText(canvas, caption, (12, 382), cv2.FONT_HERSHEY_SIMPLEX,
                    .50, (235, 230, 215), 1, cv2.LINE_AA)
            else:
                canvas[360:] = marking_caption(markings[1],markings[2],original_left)
            writer.write(canvas)
            if n in (round(3*fps), round(8*fps), round(14*fps), len(indices)-1):
                cv2.imwrite(str(output/f'frame_{seconds[n]:05.2f}s.jpg'), canvas)
    finally:
        capture.release()
        writer.release()
    temporary.replace(destination)
    result = dict(clip)
    result.pop('point_source', None)
    result.update(title_en=clip['title_en'].removeprefix('Raw points: '),
        input_preview_video=clip['video'], video=str(destination.resolve()),
        preview_video=str((output/'filled_lanes_preview.mp4').resolve()),
        cached_points=str(cache.resolve()), cache_sha256=cache_hash, source_sha256=source_hash,
        samples=len(indices), fps=fps, resolution=[1280, 392],
        snapshot=str((output/'frame_08.00s.jpg').resolve()),
        style='green ego-lane fill and chevrons; orange left and purple right; two panels',
        left_panel='original decision geometry', right_panel='presentation-only adjacent-lane tracking',
        geometry_replay_matches_original_csv=True)
    if distinguish_markings:
        marking_csv = output/'markings.csv'
        with marking_csv.open('w',newline='',encoding='utf-8') as stream:
            saved = csv.DictWriter(stream,fieldnames=list(marking_rows[0]))
            saved.writeheader()
            saved.writerows(marking_rows)
        result.update(marking_csv=str(marking_csv.resolve()),
            marking_source='automatic local road-paint contrast and continuity; two fresh confirmations; display only',
            line_style={'SOLID':'continuous coloured line','DASHED':'segmented coloured line','UNKNOWN':'neutral grey dots'},
            marking_counts={side:dict(Counter(r[f'slot_{slot}'] for r in marking_rows)) for side,slot in (('left',1),('right',2))},
            left_panel='previous geometry display', right_panel='stabilized display with estimated paint types')
    if original_left:
        result['left_panel'] = 'original unannotated source video'
    (output/'render_report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--limit', type=int)
    parser.add_argument('--markings', action='store_true', help='Estimate paint type from original image pixels and show solid/dashed/unknown styles')
    parser.add_argument('--original-left', action='store_true', help='Show the synchronized original video on the left')
    args = parser.parse_args()
    cv2.setNumThreads(2)
    original = json.loads(args.manifest.read_text(encoding='utf-8'))
    clips = original['clips']
    if args.limit is not None:
        if not 1 <= args.limit <= len(clips):
            parser.error('Invalid clip limit')
        clips = clips[:args.limit]
    rendered = []
    for clip in clips:
        rendered.append(export(clip, args.output/clip['clip_id'],args.markings,args.original_left))
        print('Filled-lane replay:', clip['clip_id'][:8], rendered[-1]['samples'], flush=True)
    manifest = dict(original)
    manifest.update(mode='filled-lane geometry comparison from unchanged per-frame model cache',
        input_manifest=str(args.manifest.resolve()), clips=rendered,
        full_sequence_test_videos=len(rendered), full_sequence_test_frames=sum(c['samples'] for c in rendered),
        scope='PC presentation replay; line colours do not indicate dashed/solid classification')
    if args.markings:
        manifest.update(mode='filled lane preview with automatic image-based solid/dashed/unknown estimates',
            scope='PC display only; colours denote lane position; stroke style denotes estimated paint type; no change to board lane-change decisions')
    if args.original_left:
        manifest.update(mode='synchronized original video on the left and lane detection on the right',
            left_panel='original unannotated source video')
    (args.output/'test_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
