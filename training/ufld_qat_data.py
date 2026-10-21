"""Build a video-disjoint, 1600x320 replay set for teacher-guided lane QAT.

No human lane labels are fabricated. Validation measures teacher agreement;
the five existing board-demo clips are excluded from both data splits.
"""
import argparse
import hashlib
import json
import random
from pathlib import Path

import cv2
import numpy as np

EXCLUDED = ('66b5fa4b', 'e716f3ed', '050541b1', '668b1ac0', 'ddfc4b8d')


def prepare(video_root, output, max_videos=12, frames_per_video=32, seed=20261016):
    output.mkdir(parents=True, exist_ok=True)
    manifest_path = output / 'manifest.json'
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        if manifest['frames_per_video'] != frames_per_video or manifest['seed'] != seed:
            raise RuntimeError('Existing data manifest differs; choose a new output directory')
        if any(not (output / item['image']).is_file() for item in manifest['samples']):
            raise RuntimeError('Existing data manifest has missing images')
        return manifest_path
    videos = {}
    for path in video_root.rglob('*.camera_front_wide_120fov.mp4'):
        if any(token in path.name for token in EXCLUDED):
            continue
        videos.setdefault(path.name, path)
    candidates = sorted(videos.values(), key=lambda p: p.name)
    random.Random(seed).shuffle(candidates)
    selected, samples, seen_hashes = [], [], set()
    for path in candidates:
        if len(selected) >= max_videos:
            break
        capture = cv2.VideoCapture(str(path))
        total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = float(capture.get(cv2.CAP_PROP_FPS))
        if total < frames_per_video * 2 or not 1 < fps < 120:
            capture.release()
            continue
        clip_samples = []
        clip_id = path.name.split('.camera_')[0]
        folder = output / 'images' / clip_id
        folder.mkdir(parents=True, exist_ok=True)
        indices = np.unique(np.linspace(int(total*.05),int(total*.95)-1,
                                        frames_per_video,dtype=int))
        for index in indices:
            capture.set(cv2.CAP_PROP_POS_FRAMES, int(index))
            ok, frame = capture.read()
            if not ok:
                continue
            crop = cv2.resize(frame,(1600,533),interpolation=cv2.INTER_LINEAR)[-320:]
            digest = hashlib.sha256(crop.tobytes()).hexdigest()
            if digest in seen_hashes:
                continue
            seen_hashes.add(digest)
            target = folder / f'{index:06d}.jpg'
            if not cv2.imwrite(str(target),crop,[cv2.IMWRITE_JPEG_QUALITY,95]):
                raise RuntimeError(f'Cannot save {target}')
            clip_samples.append(dict(image=target.relative_to(output).as_posix(),
                                     video=str(path.resolve()),clip=clip_id,
                                     frame=int(index),seconds=int(index)/fps,
                                     decoded_crop_sha256=digest))
        capture.release()
        if len(clip_samples) < 8:
            continue
        selected.append(clip_id)
        samples.extend(clip_samples)
        print(f'prepared clip={clip_id} images={len(clip_samples)}',flush=True)
    if len(selected) < 4:
        raise RuntimeError('Need at least four distinct non-demo clips for video-disjoint QAT')
    val_count = max(1,len(selected)//4)
    val_clips = set(selected[-val_count:])
    for item in samples:
        item['split'] = 'val' if item['clip'] in val_clips else 'train'
    manifest = dict(schema=1,input_shape=[1,3,320,1600],seed=seed,
                    frames_per_video=frames_per_video,video_count=len(selected),
                    excluded_board_demo_ids=list(EXCLUDED),
                    split_policy='disjoint video IDs; no adjacent-frame split',
                    labels='none; frozen float teacher supplies distillation targets',
                    validation_scope='teacher agreement, not ground-truth lane accuracy',
                    samples=samples)
    manifest_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    print(f'data_ready train={sum(s["split"]=="train" for s in samples)} '
          f'val={sum(s["split"]=="val" for s in samples)} manifest={manifest_path}',flush=True)
    return manifest_path


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--video-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--max-videos',type=int,default=12)
    parser.add_argument('--frames-per-video',type=int,default=32)
    args=parser.parse_args()
    prepare(args.video_root,args.output,args.max_videos,args.frames_per_video)
