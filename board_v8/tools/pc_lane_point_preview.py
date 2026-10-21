"""Render original UFLDv2 points from an existing sequential inference cache."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from pc_ufld_video import COLORS,ROOT


def export(clip,output):
    cache=Path(clip['video']).parent/'decoded_slots.npz'
    with np.load(cache) as saved:
        slots=saved['slots']
        indices=saved['frames']
        seconds=saved['seconds']
    if slots.shape!=(len(indices),4,81,2) or not np.all(np.diff(indices)>0):
        raise ValueError('Invalid sequential point cache')
    capture=cv2.VideoCapture(clip['source_video'])
    fps=capture.get(cv2.CAP_PROP_FPS)
    if not capture.isOpened() or fps<=0 or not np.allclose(seconds,indices/fps,atol=1e-6):
        capture.release()
        raise ValueError('Source and point-cache timestamps do not match')
    if not np.all(np.diff(indices)==1):
        capture.release()
        raise ValueError('Use a full-frame cache for an original-speed preview')
    output.mkdir(parents=True,exist_ok=True)
    video=output/'raw_points.mp4'
    temporary=output/'raw_points.partial.mp4'
    writer=cv2.VideoWriter(str(temporary),cv2.VideoWriter_fourcc(*'mp4v'),fps,(1280,720))
    if not writer.isOpened():
        capture.release()
        raise OSError('Cannot create raw-point video')
    capture.set(cv2.CAP_PROP_POS_FRAMES,int(indices[0]))
    try:
        for n,(frame_index,points_by_slot) in enumerate(zip(indices,slots)):
            ok,native=capture.read()
            if not ok:
                raise ValueError(f'Source truncated at frame {frame_index}')
            image=cv2.resize(native,(1280,720))
            for slot,points in enumerate(points_by_slot):
                points=points[np.isfinite(points).all(axis=1)]
                for x,y in np.rint(points).astype(np.int32):
                    # Identical visibility check and colours to the reference's
                    # left panel, simply enlarged to a single front-camera view.
                    if 0<=x<640 and 0<=y<360:
                        cv2.circle(image,(int(x)*2,int(y)*2),4,COLORS[slot],-1,cv2.LINE_AA)
            cv2.rectangle(image,(0,0),(1280,40),(22,16,11),-1)
            cv2.putText(image,'UFLDv2 | RAW LANE POINTS | PC FP32',
                (16,28),cv2.FONT_HERSHEY_SIMPLEX,.7,(235,230,215),1,cv2.LINE_AA)
            cv2.rectangle(image,(0,686),(1280,720),(22,16,11),-1)
            cv2.putText(image,f'{clip["title_en"]} | {seconds[n]:.2f}s | {fps:g}FPS replay',
                (16,709),cv2.FONT_HERSHEY_SIMPLEX,.57,(235,230,215),1,cv2.LINE_AA)
            writer.write(image)
            if n in (round(3*fps),round(8*fps),round(14*fps)):
                cv2.imwrite(str(output/f'frame_{round(seconds[n]):02d}s.jpg'),image)
    finally:
        capture.release()
        writer.release()
    temporary.replace(video)
    return dict(title_zh=clip['title_zh'],title_en='Raw points: '+clip['title_en'],
        clip_id=clip['clip_id'],video=str(video.resolve()),
        preview_video=str((output/'raw_points_preview.mp4').resolve()),
        source_video=clip['source_video'],cached_points=str(cache.resolve()),
        cache_sha256=hashlib.sha256(cache.read_bytes()).hexdigest(),
        samples=len(indices),fps=fps,resolution=[1280,720],
        point_source='original per-frame UFLDv2 FP32 output; no curve fitting, temporal filtering or polygon filling',
        snapshot=str((output/'frame_08s.jpg').resolve()))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,default=ROOT/'board_v8/output/pc_lane_stability_20261017/test_manifest.json')
    parser.add_argument('--output',type=Path,default=ROOT/'board_v8/output/pc_lane_raw_points_20261017')
    parser.add_argument('--limit',type=int,default=3)
    args=parser.parse_args()
    cv2.setNumThreads(2)
    clips=json.loads(args.manifest.read_text(encoding='utf-8'))['clips']
    if not 1<=args.limit<=len(clips):
        parser.error('Invalid clip limit')
    exports=[]
    for clip in clips[:args.limit]:
        exports.append(export(clip,args.output/clip['clip_id']))
        print('Raw-point preview:',clip['clip_id'][:8],flush=True)
    (args.output/'test_manifest.json').write_text(json.dumps(dict(date='2026-10-17',
        mode='raw-point presentation of cached real PC inference',clips=exports),ensure_ascii=False,indent=2),encoding='utf-8')
