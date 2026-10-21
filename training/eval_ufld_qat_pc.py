"""PC native INT8 comparison against saved, identical-input FP32 references.

The visualization uses sampled validation frames, not a temporal jitter test.
QNNPACK CPU latency must not be described as CUDA or RK3568 NPU latency.
"""
from __future__ import annotations
import argparse
import gc
import json
import sys
import time
from pathlib import Path
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
SHAPES=((1,200,72,4),(1,100,81,4),(1,2,72,4),(1,2,81,4))

def load_heads(folder,label,index):
    heads=tuple(np.fromfile(folder/f'{label}_{index}.output{h}.f32',np.float32).reshape(s) for h,s in enumerate(SHAPES))
    if not all(np.isfinite(h).all() for h in heads):raise ValueError('Nonfinite model output')
    return heads

def log_softmax(x):
    x=x-x.max(axis=1,keepdims=True)
    return x-np.log(np.exp(x).sum(axis=1,keepdims=True))

def agreement(candidate,teacher):
    result=0.
    for h,(a,b) in enumerate(zip(candidate,teacher)):
        la,lb=log_softmax(a/2),log_softmax(b/2)
        result+=(1. if h<2 else .5)*float((np.exp(lb)*(lb-la)).sum(axis=1).mean())*4
    return result

def ego_comparison(candidate,teacher):
    errors=[];equal=0;total=0;teacher_present=0;missed=0
    for slot in (1,2):
        ae=teacher[2][0,:,:,slot].argmax(0).astype(bool)
        be=candidate[2][0,:,:,slot].argmax(0).astype(bool)
        equal+=int((ae==be).sum());total+=len(ae)
        teacher_present+=int(ae.sum());missed+=int((ae&~be).sum())
        for anchor in np.flatnonzero(ae&be):
            def position(logits):
                best=int(np.argmax(logits));idx=np.arange(max(0,best-1),min(199,best+1)+1)
                prob=np.exp(logits[idx]-logits[best])
                return float(np.dot(prob,idx+.5)/prob.sum()/199*640)
            errors.append(abs(position(teacher[0][0,:,anchor,slot])-position(candidate[0][0,:,anchor,slot])))
    return errors,equal,total,teacher_present,missed

def native(args):
    import torch
    from train_ufld_qat import ReplayDataset
    sys.path.insert(0,str(ROOT/'board_v8/tools'))
    from pc_ufld_video import decode,make_lane
    torch.set_num_threads(args.threads);cv2.setNumThreads(1)
    torch.backends.quantized.engine='qnnpack'
    dataset=ReplayDataset(args.manifest,'val')
    models=[('qat_first',ROOT/'training/ufldv2_qat_20261016/export/ufldv2_culane_res18_1600x320_qat_int8.pt'),
            ('qat_round2',ROOT/'training/ufldv2_qat_20261020/export/ufldv2_culane_res18_1600x320_qat_round2_int8.pt')]
    args.output.mkdir(parents=True,exist_ok=True)
    results={}
    for label,path in models:
        print('Loading '+label,flush=True)
        model=torch.jit.load(str(path),map_location='cpu').eval()
        records=[];errors=[];equal=total=present=missed=0
        with torch.inference_mode():
            image=dataset[0][None]
            for _ in range(2):model(image)
            for i,sample in enumerate(dataset.records):
                image=dataset[i][None]
                start=time.perf_counter();pred=model(image)
                ms=(time.perf_counter()-start)*1000
                prediction=tuple(t.numpy().astype(np.float32) for t in pred)
                if tuple(t.shape for t in prediction)!=SHAPES:raise ValueError('Wrong head schema')
                if not all(np.isfinite(t).all() for t in prediction):raise ValueError('Nonfinite INT8 output')
                for h,array in enumerate(prediction):array.tofile(args.output/f'{label}_{i}.output{h}.f32')
                teacher=load_heads(args.output,'fp32',i)
                er,eq,tot,pr,mi=ego_comparison(prediction,teacher)
                errors.extend(er);equal+=eq;total+=tot;present+=pr;missed+=mi
                lane=make_lane(decode(prediction));reference=make_lane(decode(teacher))
                records.append(dict(clip=sample['clip'],frame=sample['frame'],seconds=sample['seconds'],
                    teacher_kl=agreement(prediction,teacher),cpu_native_ms=ms,
                    ego_mean_x_error_px640=float(np.mean(er)) if er else None,
                    matched_ego_anchors=len(er),ego_presence_agreement=eq/tot,
                    geometry_valid=lane.valid,fp32_geometry_valid=reference.valid))
                if i%24==23:print(f'{label}: {i+1}/{len(dataset)}',flush=True)
        timings=[r['cpu_native_ms'] for r in records]
        results[label]=dict(samples=len(records),mean_teacher_kl=float(np.mean([r['teacher_kl'] for r in records])),
            ego_presence_agreement=equal/total,teacher_present_anchors=present,teacher_present_but_candidate_absent=missed,
            matched_ego_anchors=len(errors),ego_mean_x_error_px640=float(np.mean(errors)),
            ego_p95_x_error_px640=float(np.percentile(errors,95)),
            geometry_valid_frames=sum(r['geometry_valid'] for r in records),
            fp32_geometry_valid_frames=sum(r['fp32_geometry_valid'] for r in records),
            cpu_native_mean_ms=float(np.mean(timings)),cpu_native_p95_ms=float(np.percentile(timings,95)),
            per_video={clip:dict(samples=sum(r['clip']==clip for r in records),
                mean_teacher_kl=float(np.mean([r['teacher_kl'] for r in records if r['clip']==clip])))
                for clip in sorted({r['clip'] for r in records})},records=records)
        del model,pred,image;gc.collect()
    report=dict(scope='96 identical JPEG crops from 3 held-out videos; FP32 teacher agreement, without human labels',
        backend=f'PC WSL CPU / Torch {torch.__version__} / QNNPACK / {args.threads} threads',
        latency_scope='INT8 CPU model call only; no decode/render; not GPU or board NPU timing',
        temporal_scope='Sparse validation samples; cannot measure frame-to-frame flicker',models=results)
    (args.output/'pc_comparison.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:{a:b for a,b in v.items() if a!='records'} for k,v in results.items()},indent=2),flush=True)

def render(args):
    sys.path.insert(0,str(ROOT/'board_v8/tools'))
    from pc_ufld_video import decode,make_lane
    records=[r for r in json.loads(args.manifest.read_text(encoding='utf-8'))['samples'] if r['split']=='val']
    labels=['Original','FP32 CUDA reference','QAT first / CPU INT8','QAT continued / CPU INT8']
    video_path=args.output/'pc_qat_comparison_sampled.mp4'
    writer=cv2.VideoWriter(str(video_path),cv2.VideoWriter_fourcc(*'mp4v'),30,(2560,416))
    if not writer.isOpened():raise RuntimeError('Cannot create preview video')
    captures={};valid_frames={label:[] for label in ('fp32','qat_first','qat_round2')}
    def panel(image,index,label):
        canvas=cv2.resize(image,(640,360))
        if label:
            lane=make_lane(decode(load_heads(args.output,label,index)))
            valid_frames[label].append(lane.valid)
            if lane.valid:
                tint=canvas.copy();cv2.fillPoly(tint,[lane.polygons[1]],(40,180,40))
                canvas=cv2.addWeighted(canvas,.80,tint,.20,0)
                for slot in (1,2):cv2.polylines(canvas,[lane.boundaries[slot]],False,(50,240,60),3,cv2.LINE_AA)
                left,right=lane.boundaries[1:3]
                for k in range(2,len(left)-1,4):
                    center=(left[k]+right[k])//2
                    cv2.polylines(canvas,[np.array([center+(-10,7),center,center+(10,7)])],False,(80,240,100),2,cv2.LINE_AA)
            else:cv2.putText(canvas,'EGO GEOMETRY REJECTED',(12,338),cv2.FONT_HERSHEY_SIMPLEX,.55,(60,100,255),1,cv2.LINE_AA)
        return canvas
    try:
        for i,record in enumerate(records):
            video=record['video']
            if video not in captures:
                captures[video]=cv2.VideoCapture(video)
                if not captures[video].isOpened():raise RuntimeError('Cannot open source '+video)
            cap=captures[video];cap.set(cv2.CAP_PROP_POS_FRAMES,record['frame'])
            ok,image=cap.read()
            if not ok:raise RuntimeError('Cannot read source frame')
            views=[panel(image,i,label) for label in (None,'fp32','qat_first','qat_round2')]
            result=np.zeros((416,2560,3),np.uint8)
            result[32:392]=np.hstack(views)
            for c,name in enumerate(labels):cv2.putText(result,name,(c*640+12,23),cv2.FONT_HERSHEY_SIMPLEX,.65,(230,230,230),1,cv2.LINE_AA)
            text=f'{record["clip"][:8]}  t={record["seconds"]:.2f}s | sampled ~0.6s apart; identical geometry rules; no temporal hold'
            cv2.putText(result,text,(12,410),cv2.FONT_HERSHEY_SIMPLEX,.55,(215,215,215),1,cv2.LINE_AA)
            # 18 display frames per sample approximates the source sampling
            # interval; this is explicitly a sampled, rather than real-time, demo.
            for _ in range(18):writer.write(result)
            if i in (8,40,72):
                cv2.imwrite(str(args.output/f'pc_qat_comparison_{i}.jpg'),result)
    finally:
        writer.release()
        for cap in captures.values():cap.release()
    check=cv2.VideoCapture(str(video_path))
    ok,first=check.read();count=int(check.get(cv2.CAP_PROP_FRAME_COUNT));fps=check.get(cv2.CAP_PROP_FPS);check.release()
    if not ok or first.shape!=(416,2560,3) or count!=96*18:raise RuntimeError('Preview validation failed')
    info=dict(path=str(video_path),frames=count,fps=fps,duration_seconds=count/fps,
        scope='Sparse held-out validation frames; original | FP32 | first native QAT | continued native QAT',
        source_count=len(captures),valid_frames={k:sum(v) for k,v in valid_frames.items()})
    (args.output/'preview.json').write_text(json.dumps(info,indent=2),encoding='utf-8')
    print(json.dumps(info,indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=['native','render'])
    p.add_argument('--manifest',type=Path,default=ROOT/'training/ufldv2_qat_20261016/data/manifest.json')
    p.add_argument('--output',type=Path,default=ROOT/'training/ufldv2_qat_20261020/evaluation')
    p.add_argument('--threads',type=int,default=4)
    args=p.parse_args()
    native(args) if args.action=='native' else render(args)
