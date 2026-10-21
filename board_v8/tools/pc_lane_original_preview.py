"""Pair original source frames with the right pane of an existing lane preview."""
import argparse
import csv
from datetime import date
import hashlib
import json
from pathlib import Path
import shutil

import cv2
import numpy as np

from pc_lane_geometry_preview import marking_caption


def compose(clip, output):
    source = Path(clip['source_video'])
    cached = Path(clip['cached_points'])
    for path, key in ((source, 'source_sha256'), (cached, 'cache_sha256')):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != clip[key]:
            raise ValueError(f'{key} differs from the inference input')
    with np.load(cached) as saved:
        indices, seconds = saved['frames'], saved['seconds']
    with Path(clip['marking_csv']).open(encoding='utf-8', newline='') as stream:
        markings = list(csv.DictReader(stream))
    if (len(indices) != clip['samples'] or len(markings) != len(indices)
            or not len(indices) or not np.all(np.diff(indices) == 1)):
        raise ValueError('Incomplete or non-sequential frame mapping')

    original = cv2.VideoCapture(str(source))
    processed = cv2.VideoCapture(clip['video'])
    writer = None
    output.mkdir(parents=True, exist_ok=True)
    temporary = output/'filled_lanes.partial.mp4'
    destination = output/'filled_lanes.mp4'
    right_input = hashlib.sha256()
    right_composed = hashlib.sha256()
    try:
        fps = original.get(cv2.CAP_PROP_FPS)
        if (not original.isOpened() or not processed.isOpened() or fps <= 0
                or abs(fps - processed.get(cv2.CAP_PROP_FPS)) > 1e-6
                or not np.allclose(seconds, indices/fps, atol=1e-6)
                or int(processed.get(cv2.CAP_PROP_FRAME_COUNT)) != len(indices)):
            raise ValueError('Original and processed video timing differs')
        original.set(cv2.CAP_PROP_POS_FRAMES, int(indices[0]))
        if abs(original.get(cv2.CAP_PROP_POS_FRAMES) - indices[0]) > .1:
            raise ValueError('Cannot seek to the first source frame')
        writer = cv2.VideoWriter(str(temporary), cv2.VideoWriter_fourcc(*'mp4v'), fps, (1280, 392))
        if not writer.isOpened():
            raise OSError('Cannot create original/detection video')
        for n, (frame_index, timestamp, row) in enumerate(zip(indices, seconds, markings)):
            if int(row['frame']) != frame_index or abs(float(row['seconds']) - timestamp) > 1e-6:
                raise ValueError('Marking labels and source frame indices differ')
            ok_original, native = original.read()
            ok_processed, comparison = processed.read()
            if not ok_original or not ok_processed or comparison.shape != (392, 1280, 3):
                raise ValueError(f'Missing or invalid video frame {frame_index}')
            right_input.update(comparison[:360, 640:].tobytes())
            canvas = comparison.copy()
            canvas[:360, :640] = cv2.resize(native, (640, 360))
            canvas[360:] = marking_caption(row['slot_1'], row['slot_2'], original_left=True)
            right_composed.update(canvas[:360, 640:].tobytes())
            writer.write(canvas)
            if n in (round(3*fps), round(8*fps), round(14*fps), len(indices)-1):
                cv2.imwrite(str(output/f'frame_{timestamp:05.2f}s.jpg'), canvas)
        if processed.read()[0]:
            raise ValueError('Processed preview has unexpected extra frames')
        if right_input.digest() != right_composed.digest():
            raise ValueError('The original right pane changed during composition')
    finally:
        original.release()
        processed.release()
        if writer is not None:
            writer.release()
    temporary.replace(destination)
    check = cv2.VideoCapture(str(destination))
    try:
        if (not check.isOpened() or int(check.get(cv2.CAP_PROP_FRAME_COUNT)) != len(indices)
                or abs(check.get(cv2.CAP_PROP_FPS) - fps) > 1e-6):
            raise ValueError('Written video frame count or rate differs')
    finally:
        check.release()
    shutil.copyfile(clip['marking_csv'], output/'markings.csv')
    result = dict(clip)
    result.update(video=str(destination.resolve()),
        preview_video=str((output/'filled_lanes_preview.mp4').resolve()),
        input_preview_video=clip['video'],
        snapshot=str((output/'frame_08.00s.jpg').resolve()),
        marking_csv=str((output/'markings.csv').resolve()),
        left_panel='original unannotated source video',
        same_source_frame_verified=True,
        right_pane_unchanged_before_encoding=True,
        right_pane_pixel_sha256=right_input.hexdigest())
    (output/'render_report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    cv2.setNumThreads(2)
    original = json.loads(args.manifest.read_text(encoding='utf-8'))
    rendered = []
    for clip in original['clips']:
        rendered.append(compose(clip, args.output/clip['clip_id']))
        print(f'Original/detection: {clip["clip_id"][:8]} {rendered[-1]["samples"]} frames', flush=True)
    manifest = dict(original)
    manifest.update(date=date.today().isoformat(),
        mode='synchronized original video on the left and lane detection on the right',
        input_manifest=str(args.manifest.resolve()), clips=rendered,
        left_panel='original unannotated source video',
        full_sequence_test_videos=len(rendered),
        full_sequence_test_frames=sum(c['samples'] for c in rendered))
    (args.output/'test_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
