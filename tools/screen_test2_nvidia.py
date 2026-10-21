"""Rank existing NVIDIA footage by actual UFLDv2 observations, not scene names."""
import argparse
import json
import sys
from pathlib import Path
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'board_v8/tools'))
from pc_ufld_video import Detector,draw

GROUPS=('02_daylight_side_traffic','03_daylight_following','04_clear_open_road',
        '05_overcast_following','11_side_vehicle_bsd','12_highway_following',
        '18_right_ldw_replay_candidate','19_workzone_left_maneuver',
        '20_highway_left_overtake','24_workzone_right_merge',
        '43_daylight_workzone_right_merge','46_tunnel_exit_light_transition',
        '48_highway_right_vehicle_merge','49_highway_truck_side_tracking')


def run(args):
    args.output.mkdir(parents=True,exist_ok=True)
    detector=Detector(ROOT/'models/ufldv2_culane_res18_1600x320.onnx',4)
    saved=args.output/'ranking.json'
    results=json.loads(saved.read_text(encoding='utf-8')) if saved.exists() else []
    groups=sorted(p.name for p in args.material.iterdir() if p.is_dir()) if args.all else GROUPS
    for name in groups:
        folder=args.material/name
        if not (folder/'original_four_view/front.mp4').is_file():continue
        info=json.loads((folder/'group.json').read_text(encoding='utf-8-sig'))
        if info.get('source')!='https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles':continue
        if any(r['group']==name for r in results):continue
        capture=cv2.VideoCapture(str(folder/'original_four_view/front.mp4'))
        records=[];best=(-1,None)
        for seconds in (1.,3.,6.,10.,14.,18.):
            capture.set(cv2.CAP_PROP_POS_MSEC,seconds*1000)
            ok,image=capture.read()
            if not ok: continue
            image=cv2.resize(image,(640,360))
            lane,slots,_,_=detector.detect(image)
            count=sum(p is not None for p in lane.polygons)
            reliable=lane.valid and not lane.partial
            records.append(dict(seconds=seconds,valid=lane.valid,reliable=reliable,
                regions=count,coverage=lane.coverage,offset=lane.offset))
            score=int(reliable)*4+count
            if score>best[0]:
                preview=draw(image,lane)
                cv2.putText(preview,f'{name} {seconds:.0f}s regions={count} reliable={reliable}',
                    (8,22),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1,cv2.LINE_AA)
                best=(score,preview)
        capture.release()
        if not records: continue
        reliable=sum(r['reliable'] for r in records)/len(records)
        three=sum(r['regions']==3 for r in records)/len(records)
        both=sum(r['reliable'] and r['regions']==3 for r in records)/len(records)
        result=dict(group=name,samples=records,reliable_fraction=reliable,three_region_fraction=three,
                    reliable_three_region_fraction=both,scope='FP32 observation screening, not GT accuracy')
        results.append(result)
        cv2.imwrite(str(args.output/(name+'.jpg')),best[1])
        results.sort(key=lambda r:(r['reliable_three_region_fraction'],r['reliable_fraction'],r['three_region_fraction']),reverse=True)
        (args.output/'ranking.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
        print(f'{name}: reliable={reliable:.2f} three={three:.2f} both={both:.2f}',flush=True)
    tiles=[]
    for r in results:
        tiles.append(cv2.imread(str(args.output/(r['group']+'.jpg'))))
    if len(tiles)%2:tiles.append(np.zeros_like(tiles[0]))
    cv2.imwrite(str(args.output/'screening.jpg'),np.vstack([np.hstack(tiles[i:i+2]) for i in range(0,len(tiles),2)]))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--material',type=Path,default=Path('D:/brige/multi_camera_material/adas_demo_groups'))
    p.add_argument('--output',type=Path,default=ROOT/'board_v8/output/test2_nvidia_screening')
    p.add_argument('--all',action='store_true')
    run(p.parse_args())
