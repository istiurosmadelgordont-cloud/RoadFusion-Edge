"""Fetch only missing camera members for selected locally downloaded NVIDIA clips.

Uses the existing Hugging Face login. Never downloads whole chunk archives.
"""
import argparse
import json
import shutil
import sys
import time
import zipfile
from pathlib import Path

from pc_avm_preview import DEFAULT_SCENE, ROOT

sys.path.insert(0, str(ROOT / 'output/pc_surround_review/deps'))
import pandas as pd
from huggingface_hub import HfFileSystem, hf_hub_download

EXTRA_GROUPS = ['01_daylight_intersection', '02_daylight_side_traffic',
                '05_overcast_following', '26_night_urban_intersection']
REPO = 'nvidia/PhysicalAI-Autonomous-Vehicles'
VIEWS = [('rear_left', 'camera_rear_left_70fov'),
         ('rear_right', 'camera_rear_right_70fov'), ('front_tele', 'camera_front_tele_30fov')]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--groups', nargs='+', default=EXTRA_GROUPS)
    args = parser.parse_args()
    material = DEFAULT_SCENE.parents[2]
    index = pd.read_parquet(material.parent / 'nvidia_physical_ai_av/metadata/clip_index.parquet')
    fs = HfFileSystem()
    used, limit = 0, 2_000_000_000
    for group_name in args.groups:
        if Path(group_name).name != group_name:
            raise ValueError('Group must be a directory name')
        group = material / group_name
        info = json.loads((group / 'group.json').read_text(encoding='utf-8-sig'))
        if info.get('source') != f'https://huggingface.co/datasets/{REPO}':
            raise ValueError('Group is not sourced from this NVIDIA dataset')
        clip = info['clip_id']
        chunk = int(index.loc[clip, 'chunk'])
        target = group / 'additional_views'
        target.mkdir(exist_ok=True)
        records = []
        for view, camera in VIEWS:
            remote = f'datasets/{REPO}/camera/{camera}/{camera}.chunk_{chunk:04d}.zip'
            for attempt in range(3):
                try:
                    with fs.open(remote, 'rb', block_size=1024*1024) as stream, zipfile.ZipFile(stream) as archive:
                        for suffix in ['mp4', 'timestamps.parquet']:
                            member = f'{clip}.{camera}.{suffix}'
                            entry = archive.getinfo(member)
                            destination = target / f'{view}.{suffix}'
                            if destination.is_file() and destination.stat().st_size == entry.file_size:
                                continue
                            if used + entry.file_size > limit:
                                raise ValueError('Download exceeds the 2 GB budget')
                            temporary = destination.with_suffix(destination.suffix + '.part')
                            with archive.open(member) as source, temporary.open('wb') as output:
                                shutil.copyfileobj(source, output, length=1024*1024)
                            if temporary.stat().st_size != entry.file_size:
                                raise ValueError('Incomplete ZIP member')
                            temporary.replace(destination)
                            used += entry.file_size
                    break
                except Exception as error:
                    # Remote exceptions can contain signed URLs. Log type only.
                    print(f'{group_name} {view}: retry {attempt+1}, {type(error).__name__}', flush=True)
                    if attempt == 2:
                        raise RuntimeError(f'Cannot retrieve {group_name}/{view}: {type(error).__name__}') from None
                    time.sleep(2)
            records.append({'view': view, 'camera': camera, 'video': f'{view}.mp4',
                            'timestamps': f'{view}.timestamps.parquet'})
            print(f'{group_name} {view} complete; {used/1e6:.1f} MB new', flush=True)
        for feature in ['camera_intrinsics.offline', 'sensor_extrinsics.offline', 'vehicle_dimensions']:
            hf_hub_download(REPO, f'calibration/{feature}/{feature}.chunk_{chunk:04d}.parquet',
                            repo_type='dataset', local_dir=target)
        (target / 'manifest.json').write_text(json.dumps({'clip_id': clip, 'chunk': chunk,
            'source': f'https://huggingface.co/datasets/{REPO}', 'views': records,
            'note': 'Original same-clip camera streams; align exposure timestamps before fusion.'},
            indent=2), encoding='utf-8')
        print(f'COMPLETE GROUP {group_name}', flush=True)
    print(f'ALL GROUPS DOWNLOADED: {used} bytes new', flush=True)


if __name__ == '__main__':
    main()
