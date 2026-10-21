"""Screen downloaded NVIDIA camera clips with native-resolution PC inference.

This is sparse-frame material selection. The fractions are geometry coverage,
not labelled detection accuracy or a substitute for sequential video testing.
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'board_v8/tools'))
from pc_ufld_video import COLORS,CudaDetector,Detector,draw


def run(args):
    args.output.mkdir(parents=True,exist_ok=True)
    clips=json.loads(args.downloads.read_text(encoding='utf-8'))
    if not clips:
        raise ValueError('No downloaded clips to screen')
    detector=CudaDetector(args.checkpoint,4) if args.backend=='torch-cuda' else Detector(ROOT/'models/ufldv2_culane_res18_1600x320.onnx',4)
    results=[]
    ranked=[]
    for clip in clips:
        capture=cv2.VideoCapture(clip['video'])
        records=[]
        previews=[]
        raw_previews=[]
        point_previews=[]
        for seconds in (1.,3.,6.,10.,14.,18.):
            capture.set(cv2.CAP_PROP_POS_MSEC,seconds*1000)
            ok,native=capture.read()
            if not ok:continue
            lane,slots,forward_ms,detect_ms=detector.detect(native)
            regions=sum(p is not None for p in lane.polygons)
            complete=lane.valid and not lane.partial and not lane.uncertain
            records.append(dict(seconds=seconds,valid=lane.valid,complete=complete,
                regions=regions,coverage=lane.coverage,offset=lane.offset,
                forward_ms=forward_ms,detect_ms=detect_ms))
            if seconds in (3.,10.,18.):
                tile=draw(cv2.resize(native,(640,360)),lane)
                cv2.rectangle(tile,(0,0),(640,28),(12,15,18),-1)
                cv2.putText(tile,f'{clip["clip_id"][:8]} {seconds:g}s complete={complete} regions={regions}',
                    (8,19),cv2.FONT_HERSHEY_SIMPLEX,.48,(235,235,235),1,cv2.LINE_AA)
                previews.append(tile)
                raw=cv2.resize(native,(640,360))
                point_view=raw.copy()
                for slot,points in enumerate(slots):
                    for x,y in np.rint(points).astype(np.int32):
                        if 0<=x<640 and 0<=y<360:
                            cv2.circle(point_view,(x,y),2,COLORS[slot],-1,cv2.LINE_AA)
                for view in (raw,point_view):
                    cv2.rectangle(view,(0,0),(640,28),(12,15,18),-1)
                    cv2.putText(view,f'{clip["clip_id"][:8]} {clip.get("country","")} {seconds:g}s',
                        (8,19),cv2.FONT_HERSHEY_SIMPLEX,.48,(235,235,235),1,cv2.LINE_AA)
                raw_previews.append(raw)
                point_previews.append(point_view)
        capture.release()
        if not records:continue
        complete=sum(r['complete'] for r in records)/len(records)
        three=sum(r['complete'] and r['regions']==3 for r in records)/len(records)
        result=dict(source=clip,samples=records,complete_fraction=complete,
            complete_three_region_fraction=three,
            scope='six independent native-resolution frames; geometry coverage, not GT accuracy')
        results.append(result)
        if previews:
            cv2.imwrite(str(args.output/(clip['clip_id']+'.jpg')),np.hstack(previews))
            cv2.imwrite(str(args.output/(clip['clip_id']+'_raw.jpg')),np.hstack(raw_previews))
            cv2.imwrite(str(args.output/(clip['clip_id']+'_points.jpg')),np.hstack(point_previews))
        # Keep partial results reviewable while processing the remaining clips.
        ranked=sorted(results,key=lambda r:(r['complete_three_region_fraction'],r['complete_fraction']),reverse=True)
        (args.output/'ranking.json').write_text(json.dumps(ranked,indent=2),encoding='utf-8')
        print(f'{clip["clip_id"][:8]} complete={complete:.3f} complete_three={three:.3f}',flush=True)
    for start in range(0,len(ranked),4):
        for suffix in ('','_raw','_points'):
            rows=[cv2.imread(str(args.output/(r['source']['clip_id']+suffix+'.jpg'))) for r in ranked[start:start+4]]
            cv2.imwrite(str(args.output/f'contact_{start//4+1:02d}{suffix}.jpg'),np.vstack(rows))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--downloads',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--backend',choices=('torch-cuda','onnx-cpu'),default='torch-cuda')
    parser.add_argument('--checkpoint',type=Path,default=ROOT.parent/'adas_training/weights/culane_res18.pth')
    args=parser.parse_args()
    cv2.setNumThreads(2)
    run(args)
