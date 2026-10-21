"""Find new daylit lane-footage candidates in official NVIDIA metadata.

Downloads only selected front-camera ZIP members using the existing HF login.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor,as_completed
import hashlib
import io
import json
import random
import re
import shutil
import zipfile
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
from huggingface_hub import HfFileSystem

DATA=Path('D:/brige/multi_camera_material/nvidia_physical_ai_av')
REPO='nvidia/PhysicalAI-Autonomous-Vehicles'
CAMERA='camera_front_wide_120fov'


def motion_summary(row):
    """Inspect a small original motion member before downloading camera video."""
    remote=f"datasets/{REPO}/labels/egomotion/egomotion.chunk_{row['chunk']:04d}.zip"
    try:
        fs=HfFileSystem()
        with fs.open(remote,'rb',block_size=1024*1024) as stream,zipfile.ZipFile(stream) as archive:
            data=archive.read(row['clip_id']+'.egomotion.parquet')
        frame=pd.read_parquet(io.BytesIO(data))
        velocities=frame[['vx','vy','vz']].to_numpy(dtype=float)
        valid=np.isfinite(velocities).all(axis=1)
        speed=np.linalg.norm(velocities[valid],axis=1)
        if len(speed)<30:raise ValueError('Insufficient motion records')
        curvature=frame['curvature'].to_numpy(dtype=float)
        curvature=curvature[np.isfinite(curvature)]
        return dict(clip_id=row['clip_id'],median_speed_mps=float(np.median(speed)),
            p10_speed_mps=float(np.percentile(speed,10)),
            median_abs_curvature=float(np.median(np.abs(curvature))) if len(curvature) else None,
            motion_samples=len(speed),motion_sha256=hashlib.sha256(data).hexdigest())
    except Exception as error:
        # Preserve diagnostic type without exposing a signed download URL.
        return dict(clip_id=row['clip_id'],error_type=type(error).__name__)


def run(args):
    args.output.mkdir(parents=True,exist_ok=True)
    metadata=DATA/'metadata'
    collection=pd.read_parquet(metadata/'metadata/data_collection.parquet')
    presence=pd.read_parquet(metadata/'metadata/feature_presence.parquet')
    index=pd.read_parquet(metadata/'clip_index.parquet')
    labels=pd.read_parquet(metadata/'reasoning/ood_reasoning.parquet')
    mask=collection['platform_class'].eq('hyperion_8') & collection['hour_of_day'].between(*args.hours)
    mask &= collection['country'].isin(args.countries)
    if args.months:mask &= collection['month'].isin(args.months)
    videos=list(DATA.rglob('*.mp4'))
    existing={p.name.split('.')[0] for p in videos}
    # New candidates use <clip-id>/front.mp4 rather than an ID in the filename.
    existing.update(p.parent.name for p in videos if p.name=='front.mp4')
    candidates=[]
    if args.daytime_sample:
        pool=list(collection.index[mask])
        random.Random(args.seed).shuffle(pool)
    else:
        pool=collection.index[mask]
    for clip in pool:
        if args.clip_ids and clip not in args.clip_ids:continue
        if clip in existing or clip not in presence.index or clip not in index.index:continue
        if not bool(presence.loc[clip,CAMERA]) or not bool(index.loc[clip,'clip_is_valid']):continue
        if args.require_four_view:
            cameras=[CAMERA,'camera_rear_tele_30fov','camera_cross_left_120fov','camera_cross_right_120fov']
            if not presence.loc[clip,cameras].fillna(False).all():continue
        event_data=labels.loc[clip,'events'] if clip in labels.index else None
        events=json.loads(event_data) if isinstance(event_data,str) else []
        caption=' '.join(str(e.get('coc','')) for e in events)
        if not args.daytime_sample and not args.clip_ids and not re.search(r'\b(highway|freeway|motorway|multi.lane|three lanes|four lanes|lane|merg\w*|overtak\w*)\b',caption,re.I):continue
        if re.search(r'\b(snow|rain|fog|night|construction|work.?zone|intersection|traffic light|toll)\b',caption,re.I):continue
        score=4*bool(re.search(r'highway|freeway|motorway|multi.lane|three lanes|four lanes',caption,re.I))+bool(re.search(r'lane chang|chang.*lane|merg|overtak',caption,re.I))
        candidates.append(dict(clip_id=str(clip),chunk=int(index.loc[clip,'chunk']),
            country=str(collection.loc[clip,'country']),hour=int(collection.loc[clip,'hour_of_day']),
            month=int(collection.loc[clip,'month']),platform_class='hyperion_8',source='https://huggingface.co/datasets/'+REPO,
            metadata_events=events,screening_score=int(score),
            selection_mode='daylight metadata sample; road type requires visual review' if args.daytime_sample else 'reasoning caption match'))
        if args.daytime_sample and len(candidates)>=args.daytime_sample:break
    if not args.daytime_sample:
        candidates.sort(key=lambda r:(-r['screening_score'],abs(r['hour']-12),r['clip_id']))
    (args.output/'candidates.json').write_text(json.dumps(candidates,indent=2),encoding='utf-8')
    print('New official-metadata lane candidates:',len(candidates),flush=True)
    if args.min_speed_mps>0:
        motion=[]
        with ThreadPoolExecutor(max_workers=args.motion_workers) as pool:
            jobs=[pool.submit(motion_summary,row) for row in candidates]
            for future in as_completed(jobs):
                motion.append(future.result())
                if len(motion)%8==0 or len(motion)==len(candidates):
                    eligible=sum(r.get('median_speed_mps',0)>=args.min_speed_mps and r.get('p10_speed_mps',0)>=args.min_speed_mps*.6 for r in motion)
                    print(f'Motion screened {len(motion)}/{len(candidates)}; sustained-speed candidates={eligible}',flush=True)
        (args.output/'motion_screening.json').write_text(json.dumps(motion,indent=2),encoding='utf-8')
        by_id={r['clip_id']:r for r in motion}
        candidates=[dict(row,motion=by_id[row['clip_id']]) for row in candidates
            if args.min_speed_mps<=by_id[row['clip_id']].get('median_speed_mps',0)<55 and
            by_id[row['clip_id']].get('p10_speed_mps',0)>=args.min_speed_mps*.6]
        candidates.sort(key=lambda r:-r['motion']['median_speed_mps'])
        # Keep several countries represented, rather than taking all the
        # highest-speed clips from one recording region.
        buckets={country:[r for r in candidates if r['country']==country] for country in args.countries}
        diverse=[]
        while any(buckets.values()):
            for country in args.countries:
                if buckets[country]:diverse.append(buckets[country].pop(0))
        candidates=diverse
        (args.output/'motion_selected.json').write_text(json.dumps(candidates,indent=2),encoding='utf-8')
        print('Sustained-speed video candidates:',len(candidates),flush=True)
    for row in candidates[:args.download]:
        print(row['clip_id'],row['hour'],str(row['metadata_events'][0].get('coc',''))[:170] if row['metadata_events'] else 'Daylight camera candidate',flush=True)
    if not args.download:return
    fs=HfFileSystem();used=0;manifest=[]
    for row in candidates[:args.download]:
        folder=args.output/row['clip_id'];folder.mkdir(exist_ok=True)
        remote=f"datasets/{REPO}/camera/{CAMERA}/{CAMERA}.chunk_{row['chunk']:04d}.zip"
        member=f"{row['clip_id']}.{CAMERA}.mp4"
        path=folder/'front.mp4'
        try:
            with fs.open(remote,'rb',block_size=args.range_block_mb*1024*1024) as stream,zipfile.ZipFile(stream) as archive:
                entry=archive.getinfo(member)
                if used+entry.file_size>args.max_bytes:break
                with archive.open(entry) as source,path.with_suffix('.mp4.part').open('wb') as target:
                    shutil.copyfileobj(source,target,length=1024*1024)
                if path.with_suffix('.mp4.part').stat().st_size!=entry.file_size:raise RuntimeError('Incomplete video')
                path.with_suffix('.mp4.part').replace(path);used+=entry.file_size
            capture=cv2.VideoCapture(str(path));fps=capture.get(cv2.CAP_PROP_FPS);count=int(capture.get(cv2.CAP_PROP_FRAME_COUNT));width=int(capture.get(cv2.CAP_PROP_FRAME_WIDTH));height=int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT));capture.release()
            if not fps or count<30:raise RuntimeError('Invalid camera stream')
            record=dict(row,video=str(path.resolve()),camera=CAMERA,bytes=path.stat().st_size,
                sha256=hashlib.sha256(path.read_bytes()).hexdigest(),fps=fps,frames=count,width=width,height=height,
                selection_status='metadata candidate; visual/model review pending')
            (folder/'source.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
            manifest.append(record)
            (args.output/'downloads.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
            print(f"Downloaded {row['clip_id'][:8]}: {count/fps:.1f}s, {used/1e6:.1f}MB total",flush=True)
        except Exception as error:
            # Network exceptions may contain signed URLs. Never log their text.
            print(f"Download failed {row['clip_id'][:8]}: {type(error).__name__}",flush=True)
    print('Downloaded candidates:',len(manifest),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--output',type=Path,default=DATA/'candidate/lane_pc_20261016')
    p.add_argument('--download',type=int,default=0)
    p.add_argument('--max-bytes',type=int,default=350_000_000)
    p.add_argument('--daytime-sample',type=int,default=0,help='Sample this many new daylit clips, including clips without OOD captions')
    p.add_argument('--seed',type=int,default=20261016)
    p.add_argument('--hours',type=int,nargs=2,default=(10,15),metavar=('START','END'))
    p.add_argument('--clip-ids',nargs='+',help='Restrict downloads to explicitly selected official clip IDs')
    p.add_argument('--require-four-view',action='store_true',help='Require all four original cameras to be available for later board replay')
    p.add_argument('--countries',nargs='+',default=['United States'])
    p.add_argument('--months',type=int,nargs='+')
    p.add_argument('--min-speed-mps',type=float,default=0,help='Preselect using original egomotion speed, not a visual speed estimate')
    p.add_argument('--motion-workers',type=int,default=4)
    p.add_argument('--range-block-mb',type=int,default=4)
    run(p.parse_args())
