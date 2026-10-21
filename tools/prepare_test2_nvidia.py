"""Build board replay mosaics from real NVIDIA cameras and audited frame maps."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

GROUPS=('12_highway_following','20_highway_left_overtake','48_highway_right_vehicle_merge')
VIEWS=('front','rear','left','right')


def export(group,output):
    info=json.loads((group/'group.json').read_text(encoding='utf-8-sig'))
    assert info['source']=='https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles'
    alignment=group/'aligned_frame_map.csv'
    records=list(csv.DictReader(alignment.open(encoding='utf-8-sig')))
    timestamps=[int(r['timestamp']) for r in records]
    assert len(records)>30 and all(b>a for a,b in zip(timestamps,timestamps[1:]))
    interval=float(np.median(np.diff(timestamps)))
    assert abs(interval-1e6/30)<100
    for view in VIEWS:
        indices=[int(r[view+'_frame']) for r in records]
        assert min(indices)>=0 and all(b>=a for a,b in zip(indices,indices[1:]))
    output.mkdir(parents=True,exist_ok=True)
    paths=[output/(view+'.mp4') for view in VIEWS]
    captures=[cv2.VideoCapture(str(group/'original_four_view'/f'{v}.mp4')) for v in VIEWS]
    if not all(c.isOpened() for c in captures):
        raise RuntimeError('Four original cameras are required')
    writers=[cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'mp4v'),30.,(640,360)) for path in paths]
    if not all(w.isOpened() for w in writers):
        raise RuntimeError('Cannot create four board camera clips')
    positions=[-1]*4
    frames=[None]*4
    try:
        for index,record in enumerate(records):
            tiles=[]
            for camera,view in enumerate(VIEWS):
                target=int(record[view+'_frame'])
                while positions[camera]<target:
                    ok,frame=captures[camera].read()
                    if not ok: raise RuntimeError(f'Truncated {view} at source frame {target}')
                    positions[camera]+=1
                    frames[camera]=frame
                tiles.append(cv2.resize(frames[camera],(640,360),interpolation=cv2.INTER_AREA))
                writers[camera].write(tiles[-1])
            mosaic=np.vstack((np.hstack(tiles[:2]),np.hstack(tiles[2:])))
            if index==30: cv2.imwrite(str(output/'source_preview.jpg'),mosaic)
            if index%150==0: print(f'{group.name}: {index}/{len(records)}',flush=True)
    finally:
        for writer in writers: writer.release()
        for capture in captures: capture.release()
    for path in paths:
        check=cv2.VideoCapture(str(path))
        count=int(check.get(cv2.CAP_PROP_FRAME_COUNT));check.release()
        if count!=len(records): raise RuntimeError('Encoded frame count mismatch')
    (output/'aligned_frame_map.csv').write_bytes(alignment.read_bytes())
    manifest=dict(group=group.name,clip_id=info['clip_id'],title=info['title'],
        source=info['source'],source_directory=str(group/'original_four_view'),
        source_type='real synchronized NVIDIA camera footage; no synthesized camera views',
        alignment='per-row source frame indices from aligned_frame_map.csv',
        output_fps=30.,output_frames=count,duration_seconds=count/30.,
        layout=[['front','rear'],['left','right']],per_camera_resolution=[640,360],
        camera_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
        purpose=info.get('purpose',''),collision_ground_truth=False)
    (output/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    print(f'COMPLETE {group.name}: {count} frames, {count/30.:.2f}s',flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--material',type=Path,default=Path('D:/brige/multi_camera_material/adas_demo_groups'))
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'board_v8/output/test2_nvidia_scenes')
    parser.add_argument('--groups',nargs='+',choices=GROUPS,default=GROUPS)
    args=parser.parse_args()
    cv2.setNumThreads(2)
    for name in args.groups: export(args.material/name,args.output/name)
