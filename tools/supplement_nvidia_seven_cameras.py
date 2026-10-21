"""Complete existing NVIDIA front-video folders with seven original cameras.

Extract only requested ZIP members. Preserve existing front.mp4 files and save
the original per-camera timestamps plus a nearest-timestamp replay frame map.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import logging
import os
from pathlib import Path
import time
import zipfile
import zlib

os.environ['HF_HUB_DISABLE_PROGRESS_BARS'] = '1'
for logger in ('huggingface_hub', 'fsspec', 'urllib3', 'httpx', 'requests'):
    logging.getLogger(logger).setLevel(logging.CRITICAL)

import cv2
from huggingface_hub import HfApi, HfFileSystem
import numpy as np
import pandas as pd

ROOT = Path('D:/brige/multi_camera_material/nvidia_physical_ai_av')
REPO = 'nvidia/PhysicalAI-Autonomous-Vehicles'
SOURCE = 'https://huggingface.co/datasets/' + REPO
CAMERAS = {
    'front': ('camera_front_wide_120fov', '前广角'),
    'front_tele': ('camera_front_tele_30fov', '前长焦'),
    'left': ('camera_cross_left_120fov', '左侧'),
    'right': ('camera_cross_right_120fov', '右侧'),
    'rear_left': ('camera_rear_left_70fov', '左后'),
    'rear_right': ('camera_rear_right_70fov', '右后'),
    'rear': ('camera_rear_tele_30fov', '后视'),
}


def save_json(path, data):
    temporary = path.with_suffix(path.suffix + '.partial')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    temporary.replace(path)


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def extract_member(archive, member, target):
    entry = archive.getinfo(member)
    if target.exists():
        crc = 0
        with target.open('rb') as stream:
            for block in iter(lambda: stream.read(1024*1024), b''):
                crc = zlib.crc32(block, crc)
        if target.stat().st_size != entry.file_size or crc != entry.CRC:
            raise ValueError('Existing original differs from the official ZIP member')
        return False, entry.CRC
    temporary = target.with_suffix(target.suffix + '.part')
    crc = 0
    with archive.open(entry) as source, temporary.open('wb') as output:
        while block := source.read(1024*1024):
            output.write(block)
            crc = zlib.crc32(block, crc)
    if temporary.stat().st_size != entry.file_size or crc != entry.CRC:
        raise ValueError('Incomplete official ZIP member')
    temporary.replace(target)
    return True, entry.CRC


def inspect_video(video, timestamps):
    table = pd.read_parquet(timestamps)
    times = table['timestamp'].to_numpy(dtype=np.int64)
    indices = table.index.to_numpy(dtype=np.int64)
    if (len(times) < 30 or not np.all(np.diff(times) > 0)
            or not np.array_equal(indices, np.arange(len(times)))):
        raise ValueError('Invalid original camera frame timestamps')
    capture = cv2.VideoCapture(str(video))
    try:
        fps = capture.get(cv2.CAP_PROP_FPS)
        count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if not capture.isOpened() or count != len(times) or abs(fps-30) > .01:
            raise ValueError('Video and original timestamp counts differ')
        checked = (0, count//2, count-1)
        for frame_index in checked:
            capture.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
            ok, frame = capture.read()
            if not ok or frame.shape[:2] != (height, width):
                raise ValueError('Cannot decode original camera sample')
    finally:
        capture.release()
    return dict(fps=fps, frames=count, width=width, height=height,
        timestamp_unit='microseconds', timestamp_count=len(times),
        first_timestamp_us=int(times[0]), last_timestamp_us=int(times[-1]),
        sample_frames_decoded=list(checked))


def download_camera(folder, view, feature, chunk, revision):
    receipt = folder/'seven_camera_sources'/f'{view}.json'
    video = folder/f'{view}.mp4'
    timestamps = folder/f'{view}.timestamps.parquet'
    if receipt.is_file():
        saved = json.loads(receipt.read_text(encoding='utf-8'))
        if (saved.get('revision') == revision and saved.get('feature') == feature
                and video.is_file() and timestamps.is_file()
                and sha256(video) == saved.get('sha256')
                and sha256(timestamps) == saved.get('timestamps_sha256')):
            return saved
    if view == 'front':
        original = json.loads((folder/'source.json').read_text(encoding='utf-8'))
        if sha256(video) != original['sha256']:
            raise ValueError('Existing front video changed since lane testing')
    remote = f'datasets/{REPO}@{revision}/camera/{feature}/{feature}.chunk_{chunk:04d}.zip'
    last_error = None
    for attempt in range(3):
        try:
            fs = HfFileSystem()
            with fs.open(remote, 'rb', block_size=2*1024*1024) as stream, zipfile.ZipFile(stream) as archive:
                added, crc = extract_member(archive, f'{folder.name}.{feature}.mp4', video)
                _, timestamp_crc = extract_member(archive, f'{folder.name}.{feature}.timestamps.parquet', timestamps)
            metadata = inspect_video(video, timestamps)
            result = dict(clip_id=folder.name, view=view, title_zh=CAMERAS[view][1],
                feature=feature, chunk=chunk, source=SOURCE, revision=revision,
                video=str(video.resolve()), timestamps=str(timestamps.resolve()),
                bytes=video.stat().st_size, sha256=sha256(video), zip_crc32=crc,
                timestamps_sha256=sha256(timestamps), timestamps_zip_crc32=timestamp_crc,
                newly_downloaded=added, original_unmodified=True, **metadata)
            receipt.parent.mkdir(exist_ok=True)
            save_json(receipt, result)
            return result
        except Exception as error:
            last_error = type(error).__name__
            if attempt < 2:
                time.sleep(1 + attempt)
    # Exception text from HTTP libraries can contain signed credentials.
    raise RuntimeError(last_error) from None


def align_and_preview(folder, records, tolerance_us=20000):
    times = {v:pd.read_parquet(records[v]['timestamps'])['timestamp'].to_numpy(dtype=np.int64)
        for v in CAMERAS}
    reference = times['front']
    valid = np.ones(len(reference), dtype=bool)
    chosen = {}
    for view, timestamps in times.items():
        right = np.searchsorted(timestamps, reference).clip(0, len(timestamps)-1)
        left = (right-1).clip(0, len(timestamps)-1)
        indices = np.where(np.abs(timestamps[left]-reference) <= np.abs(timestamps[right]-reference), left, right)
        chosen[view] = indices
        valid &= np.abs(timestamps[indices]-reference) <= tolerance_us
    if valid.sum() < len(reference)*.95:
        raise ValueError('Insufficient timestamp overlap across seven cameras')
    mapping = pd.DataFrame({'timestamp_us':reference[valid]})
    maximum_skew = {}
    for view, indices in chosen.items():
        mapping[view+'_frame'] = indices[valid]
        mapping[view+'_offset_us'] = times[view][indices[valid]]-reference[valid]
        maximum_skew[view] = int(np.abs(mapping[view+'_offset_us']).max())
    mapping.to_csv(folder/'seven_camera_frame_map.csv', index=False, encoding='utf-8')
    # A still preview is sampled from a single shared timestamp, not frame count.
    row = mapping.iloc[len(mapping)//2]
    tiles = {}
    for view, record in records.items():
        capture = cv2.VideoCapture(record['video'])
        try:
            capture.set(cv2.CAP_PROP_POS_FRAMES, int(row[view+'_frame']))
            ok, image = capture.read()
            if not ok:
                raise ValueError('Cannot decode synchronized preview frame')
        finally:
            capture.release()
        tile = cv2.resize(image, (480, 270), interpolation=cv2.INTER_AREA)
        cv2.rectangle(tile, (0,0), (480,30), (20,18,15), -1)
        cv2.putText(tile, view+' | '+record['feature'].removeprefix('camera_'),
            (8,21), cv2.FONT_HERSHEY_SIMPLEX, .44, (240,240,235), 1, cv2.LINE_AA)
        tiles[view] = tile
    center = np.full((270,480,3), (20,18,15), dtype=np.uint8)
    for text, y in ((folder.name[:8],70), ('7 ORIGINAL NVIDIA CAMERAS',115),
                    (f't = {row["timestamp_us"]/1e6:.3f}s',155), ('timestamp-aligned preview',190)):
        cv2.putText(center, text, (35,y), cv2.FONT_HERSHEY_SIMPLEX, .65, (235,230,215), 1, cv2.LINE_AA)
    sheet = np.vstack((np.hstack((tiles['left'],tiles['front'],tiles['right'])),
        np.hstack((tiles['rear_left'],center,tiles['rear_right'])),
        np.hstack((tiles['front_tele'],tiles['rear'],np.zeros_like(center)))))
    if not cv2.imwrite(str(folder/'seven_camera_preview.jpg'), sheet):
        raise OSError('Cannot write seven-camera preview')
    manifest = dict(clip_id=folder.name, source=SOURCE,
        revision=next(iter(records.values()))['revision'],
        complete=True, camera_count=7, cameras=records,
        alignment='nearest original camera timestamp to front wide camera; no video rewritten',
        tolerance_us=tolerance_us, aligned_frames=len(mapping),
        alignment_start_us=int(mapping['timestamp_us'].iloc[0]),
        alignment_end_us=int(mapping['timestamp_us'].iloc[-1]),
        maximum_absolute_camera_offset_us=maximum_skew,
        frame_map=str((folder/'seven_camera_frame_map.csv').resolve()),
        preview=str((folder/'seven_camera_preview.jpg').resolve()))
    save_json(folder/'seven_camera_manifest.json', manifest)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folders', nargs='+', type=Path, required=True)
    parser.add_argument('--workers', type=int, default=6)
    parser.add_argument('--revision', required=True)
    args = parser.parse_args()
    if not 1 <= args.workers <= 8:
        parser.error('Expected 1 to 8 download workers')
    cv2.setNumThreads(1)
    presence = pd.read_parquet(ROOT/'metadata/metadata/feature_presence.parquet')
    index = pd.read_parquet(ROOT/'metadata/clip_index.parquet')
    folders = [p.parent for batch in args.folders for p in sorted(batch.glob('*/front.mp4'))]
    if not folders or len({p.name for p in folders}) != len(folders):
        parser.error('Missing or duplicate existing source clips')
    for folder in folders:
        if not presence.loc[folder.name,[feature for feature,_ in CAMERAS.values()]].fillna(False).all():
            raise ValueError('Requested source clip lacks one of the seven cameras')
    results = {folder:{} for folder in folders}
    completed = {}
    errors = []
    status_path = args.folders[0].parent/'lane_middle_seven_camera_status_20261018.json'
    def checkpoint():
        status = dict(source=SOURCE, revision=args.revision,
            updated_utc=datetime.now(timezone.utc).isoformat(),
            total_clips=len(folders), complete_clips=len(completed),
            complete_cameras=sum(len(r) for r in results.values()),
            total_cameras=7*len(folders), errors=errors,
            clips=[dict(clip_id=p.name, directory=str(p.resolve()),
                camera_count=len(results[p]), complete=p in completed) for p in folders])
        save_json(status_path,status)
        return status
    print(f'Completing {len(folders)} original clips, {len(folders)*7} camera streams', flush=True)
    checkpoint()
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        jobs = {pool.submit(download_camera,folder,view,feature,
            int(index.loc[folder.name,'chunk']),args.revision):(folder,view)
            for folder in folders for view,(feature,_) in CAMERAS.items()}
        done = 0
        for future in as_completed(jobs):
            folder, view = jobs[future]
            try:
                results[folder][view] = future.result()
                if len(results[folder]) == 7:
                    completed[folder] = align_and_preview(folder, results[folder])
                    print(f'COMPLETE {len(completed)}/{len(folders)}: {folder.name[:8]} seven original cameras', flush=True)
            except Exception as error:
                errors.append(dict(clip_id=folder.name,view=view,error_type=type(error).__name__))
                print(f'Camera failed {folder.name[:8]} {view}: {type(error).__name__}',flush=True)
            done += 1
            if done%7 == 0 or done == len(jobs):
                print(f'Camera jobs checked {done}/{len(jobs)}',flush=True)
            checkpoint()
    for batch in args.folders:
        clips = [completed[p] for p in folders if p.parent==batch and p in completed]
        save_json(batch/'seven_camera_downloads.json',dict(source=SOURCE,revision=args.revision,
            expected_clips=sum(p.parent==batch for p in folders),complete_clips=len(clips),clips=clips))
        (batch/'七路原视频说明.txt').write_text(
            '来源：NVIDIA PhysicalAI-Autonomous-Vehicles 原始相机视频。\n'
            '每个 UUID 子目录是一段相同场景的七路原始视频，文件未重新编码。\n\n'+
            '\n'.join(f'{view}.mp4：{title}（{feature}）' for view,(feature,title) in CAMERAS.items())+
            '\n\n各路对应的 *.timestamps.parquet 是原始帧时间戳（微秒）。\n'
            '七路帧数可能不同；同步播放请使用 seven_camera_frame_map.csv 中的帧索引。\n'
            'seven_camera_manifest.json：来源、哈希、解码抽查和同步检查结果。\n'
            'seven_camera_preview.jpg：按同一时间戳抽取的七路缩略图。\n',encoding='utf-8-sig')
    status=checkpoint()
    print(f'Finished: {status["complete_clips"]}/{len(folders)} clips; {status["complete_cameras"]}/{7*len(folders)} cameras',flush=True)
    if errors or len(completed) != len(folders):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
