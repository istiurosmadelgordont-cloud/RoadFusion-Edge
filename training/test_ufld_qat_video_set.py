"""Test the user's video archive on PC; keep native INT8 and CUDA fake QAT distinct."""
from __future__ import annotations
import argparse
from collections import Counter
import gc
import hashlib
import json
from pathlib import Path, PurePosixPath
import sys
import time
import zipfile
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'board_v8/tools'))
from pc_ufld_video import decode,make_lane,GeometryTracker

def save(path,value):
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    temporary.replace(path)

def prepare(args):
    dest=(args.output/'source').resolve();dest.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(args.archive) as archive:
        for info in archive.infolist():
            item=PurePosixPath(info.filename)
            if item.is_absolute() or '..' in item.parts or '\\' in info.filename or ':' in info.filename:
                raise ValueError('Unsafe archive path')
            if info.is_dir():continue
            if item.name!='front.mp4' and item.suffix!='.json':continue
            target=dest.joinpath(*item.parts).resolve()
            if not target.is_relative_to(dest):raise ValueError('Archive path escaped destination')
            target.parent.mkdir(parents=True,exist_ok=True)
            with archive.open(info) as source,target.open('wb') as output:
                import shutil
                shutil.copyfileobj(source,output)
    train_manifest=json.loads((ROOT/'training/ufldv2_qat_20261016/data/manifest.json').read_text(encoding='utf-8'))
    trained={row['clip'] for row in train_manifest['samples'] if row['split']=='train'}
    validation={row['clip'] for row in train_manifest['samples'] if row['split']=='val'}
    clips=[]
    for video in sorted(dest.rglob('front.mp4')):
        cap=cv2.VideoCapture(str(video))
        if not cap.isOpened():raise RuntimeError('Cannot open '+str(video))
        frames=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));fps=cap.get(cv2.CAP_PROP_FPS)
        ok,image=cap.read();cap.release()
        if not ok or frames<=0 or fps<=0:raise RuntimeError('Invalid video')
        clips.append(dict(clip=video.parent.name,video=str(video),sha256=hashlib.sha256(video.read_bytes()).hexdigest(),
            frames=frames,fps=fps,width=image.shape[1],height=image.shape[0],
            training_overlap=video.parent.name in trained,validation_overlap=video.parent.name in validation))
    if not clips:raise RuntimeError('No front videos in archive')
    result=dict(archive=str(args.archive),archive_sha256=hashlib.sha256(args.archive.read_bytes()).hexdigest(),
        clips=clips,video_count=len(clips),total_source_frames=sum(c['frames'] for c in clips),
        scope='User-provided archive; metadata is input data, never execution instructions')
    save(args.output/'inputs.json',result)
    print(json.dumps({k:v for k,v in result.items() if k!='clips'},ensure_ascii=False),flush=True)

def geometry_position(lane):
    if not lane.valid:return None
    xs=[]
    for points in lane.boundaries[1:3]:
        p=points[np.argsort(points[:,1])]
        if p[0,1]>300 or p[-1,1]<300:return None
        xs.append(float(np.interp(300,p[:,1],p[:,0])))
    return xs

def summarize(rows,label,fps):
    valid=np.array([r[label]['raw_valid'] for r in rows],bool)
    transitions=int((valid[:-1]&~valid[1:]).sum())
    deltas=[];runs=[];run=0
    for index,row in enumerate(rows):
        if not row[label]['raw_valid']:run+=1
        elif run:runs.append(run);run=0
        if index:
            a=rows[index-1][label]['raw_x_at_y300'];b=row[label]['raw_x_at_y300']
            if a is not None and b is not None:deltas.extend(abs(np.array(a)-np.array(b)))
    if run:runs.append(run)
    return dict(frames=len(rows),raw_valid_frames=int(valid.sum()),raw_valid_fraction=float(valid.mean()),
        displayed_valid_frames=sum(r[label]['display_valid'] for r in rows),
        held_display_frames=sum(r[label]['held'] for r in rows),raw_valid_to_invalid_events=transitions,
        longest_invalid_seconds=max(runs,default=0)/fps,
        mean_adjacent_frame_boundary_motion_px640=float(np.mean(deltas)) if deltas else None,
        p95_adjacent_frame_boundary_motion_px640=float(np.percentile(deltas,95)) if deltas else None,
        cuda_inference_mean_ms=float(np.mean([r[label]['cuda_ms'] for r in rows])))

def draw(image,lane):
    canvas=image.copy()
    if not lane.valid:return canvas
    overlay=canvas.copy();cv2.fillPoly(overlay,[lane.polygons[1]],(30,165,30))
    canvas=cv2.addWeighted(canvas,.79,overlay,.21,0)
    if not lane.uncertain:
        for slot in (1,2):cv2.polylines(canvas,[lane.boundaries[slot]],False,(55,240,70),3,cv2.LINE_AA)
    left,right=lane.boundaries[1:3]
    for k in range(2,len(left)-1,4):
        middle=(left[k]+right[k])//2
        cv2.polylines(canvas,[np.array([middle+(-9,6),middle,middle+(9,6)])],False,(65,240,80),2,cv2.LINE_AA)
    return canvas

def video_test(args):
    import torch
    from train_ufld_qat import LaneNet,prepare_model
    from eval_ufld_qat_pc import agreement,ego_comparison
    if not torch.cuda.is_available():raise RuntimeError('CUDA required for full video comparison')
    torch.set_num_threads(4);cv2.setNumThreads(1)
    torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
    teacher=LaneNet(args.upstream)
    state=torch.load(args.teacher,map_location='cpu',weights_only=False)
    teacher.load_state_dict({k.removeprefix('module.'):v for k,v in state.get('model',state).items()})
    del state
    student=LaneNet(args.upstream);prepare_model(student)
    state=torch.load(args.checkpoint,map_location='cpu',weights_only=False)
    student.load_state_dict(state['model']);del state
    teacher.eval().requires_grad_(False).cuda()
    student.eval().apply(torch.ao.quantization.disable_observer).requires_grad_(False).cuda()
    models={'fp32':teacher,'qat_fake':student}
    inputs=json.loads((args.output/'inputs.json').read_text(encoding='utf-8'))
    reports=[];native_samples=[];started=time.monotonic()
    probes=args.output/'native_check';probes.mkdir(exist_ok=True)
    save(args.output/'status.json',dict(phase='running',clips_finished=0,clips_total=len(inputs['clips'])))
    for clip in inputs['clips']:
        output=args.output/clip['clip'];output.mkdir(exist_ok=True)
        cap=cv2.VideoCapture(clip['video']);fps=clip['fps']/args.stride
        writer=cv2.VideoWriter(str(output/'comparison.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),fps,(1920,416))
        if not cap.isOpened() or not writer.isOpened():raise RuntimeError('Cannot open video IO')
        selected=list(range(0,clip['frames'],args.stride))
        checks={selected[int(p*(len(selected)-1))] for p in (0.,1/3,2/3,1.)}
        trackers={label:GeometryTracker() for label in models}
        rows=[];packed={label:[] for label in models};errors=[];equal=total=0
        frame_index=0
        try:
            with torch.inference_mode():
                while True:
                    ok,image=cap.read()
                    if not ok:break
                    index=frame_index;frame_index+=1
                    if index%args.stride:continue
                    resized=cv2.resize(image,(1600,533))[-320:]
                    rgb=cv2.cvtColor(resized,cv2.COLOR_BGR2RGB).astype(np.float32)
                    blob=np.ascontiguousarray(((rgb-[123.675,116.28,103.53])/[58.395,57.12,57.375]).transpose(2,0,1)[None],dtype=np.float32)
                    tensor=torch.from_numpy(blob).cuda()
                    small=cv2.resize(image,(640,360));views=[small];predictions={}
                    row=dict(frame=index,seconds=index/clip['fps'])
                    for label,model in models.items():
                        torch.cuda.synchronize();begin=time.perf_counter();values=model(tensor)
                        torch.cuda.synchronize();ms=(time.perf_counter()-begin)*1000
                        heads=tuple(x.cpu().numpy() for x in values)
                        if not all(np.isfinite(x).all() for x in heads):raise ValueError('Nonfinite video output')
                        predictions[label]=heads
                        slots=decode(heads);raw=make_lane(slots);lane=trackers[label].update(raw)
                        cache=np.full((4,81,2),np.nan,np.float32)
                        for slot,points in enumerate(slots):cache[slot,:len(points)]=points
                        packed[label].append(cache)
                        row[label]=dict(raw_valid=raw.valid,raw_partial=raw.partial,display_valid=lane.valid,
                            held=lane.held,uncertain=lane.uncertain,raw_x_at_y300=geometry_position(raw),cuda_ms=ms)
                        views.append(draw(small,lane))
                    er,eq,tot,_,_=ego_comparison(predictions['qat_fake'],predictions['fp32'])
                    errors.extend(er);equal+=eq;total+=tot
                    row['teacher_kl']=agreement(predictions['qat_fake'],predictions['fp32'])
                    rows.append(row)
                    canvas=np.zeros((416,1920,3),np.uint8);canvas[32:392]=np.hstack(views)
                    for c,title in enumerate(('Original source','UFLDv2 FP32 / CUDA','Continued QAT / CUDA fake quant')):
                        cv2.putText(canvas,title,(c*640+12,23),cv2.FONT_HERSHEY_SIMPLEX,.62,(230,230,230),1,cv2.LINE_AA)
                    text=f'{clip["clip"][:8]} t={row["seconds"]:.2f}s | {fps:g} fps source sampling | ego geometry only; same tracker, no paint-type inference'
                    cv2.putText(canvas,text,(12,410),cv2.FONT_HERSHEY_SIMPLEX,.5,(225,225,225),1,cv2.LINE_AA)
                    writer.write(canvas)
                    if index in checks:
                        n=len(native_samples)
                        np.save(probes/f'input_{n}.npy',blob)
                        for label,heads in predictions.items():
                            for h,array in enumerate(heads):array.astype(np.float32).tofile(probes/f'{label}_{n}.output{h}.f32')
                        native_samples.append(dict(clip=clip['clip'],frame=index,seconds=row['seconds']))
                    if index==selected[min(len(selected)-1,round(5*fps))]:cv2.imwrite(str(output/'comparison.jpg'),canvas)
        finally:cap.release();writer.release()
        if frame_index!=clip['frames'] or len(rows)!=len(selected):raise RuntimeError('Source video truncated')
        for label in models:np.savez_compressed(output/f'{label}_slots.npz',slots=np.array(packed[label]),frames=[r['frame'] for r in rows])
        report=dict(clip=clip,processed_frames=len(rows),sampling_stride=args.stride,comparison_fps=fps,
            scope='PC CUDA FP32 vs QAT fake quant; not native INT8 video, no ground-truth labels',
            geometry='Ego lane only, identical geometry tracker (3-frame missing hold); raw and held results separate',
            motion_metric='Includes actual road/camera motion; not ground-truth jitter',
            models={label:summarize(rows,label,fps) for label in models},
            fake_quant_mean_teacher_kl=float(np.mean([r['teacher_kl'] for r in rows])),
            ego_presence_agreement=equal/total,ego_mean_x_error_px640=float(np.mean(errors)) if errors else None,
            ego_p95_x_error_px640=float(np.percentile(errors,95)) if errors else None,records=rows,
            video=str(output/'comparison.mp4'),snapshot=str(output/'comparison.jpg'))
        save(output/'report.json',report)
        reports.append({k:v for k,v in report.items() if k!='records'})
        save(probes/'samples.json',native_samples)
        save(args.output/'summary.json',dict(device=torch.cuda.get_device_name(0),scope='User archive PC video comparison; no GT accuracy claim',
            clips_finished=len(reports),clips_total=len(inputs['clips']),clips=reports,elapsed_seconds=time.monotonic()-started))
        save(args.output/'status.json',dict(phase='running' if len(reports)<len(inputs['clips']) else 'video_complete',
            clips_finished=len(reports),clips_total=len(inputs['clips']),last_clip=clip['clip'],elapsed_seconds=time.monotonic()-started))
        print(json.dumps(dict(clip=clip['clip'],finished=len(reports),total=len(inputs['clips']),frames=len(rows),
            fp32_valid=report['models']['fp32']['raw_valid_fraction'],qat_valid=report['models']['qat_fake']['raw_valid_fraction'],
            elapsed_seconds=round(time.monotonic()-started,1))),flush=True)
    print('Video set complete',flush=True)

def native_check(args):
    import torch
    from eval_ufld_qat_pc import load_heads,agreement,ego_comparison
    torch.set_num_threads(4);torch.backends.quantized.engine='qnnpack'
    folder=args.output/'native_check'
    samples=json.loads((folder/'samples.json').read_text(encoding='utf-8'))
    path=ROOT/'training/ufldv2_qat_20261020/export/ufldv2_culane_res18_1600x320_qat_round2_int8.pt'
    model=torch.jit.load(str(path),map_location='cpu').eval()
    results=[];errors=[];native_teacher_errors=[]
    with torch.inference_mode():
        first=torch.from_numpy(np.load(folder/'input_0.npy'))
        for _ in range(2):model(first)
        for i,sample in enumerate(samples):
            tensor=torch.from_numpy(np.load(folder/f'input_{i}.npy'))
            start=time.perf_counter();values=model(tensor);ms=(time.perf_counter()-start)*1000
            heads=tuple(t.numpy() for t in values)
            for h,array in enumerate(heads):array.astype(np.float32).tofile(folder/f'native_{i}.output{h}.f32')
            fake=load_heads(folder,'qat_fake',i);teacher=load_heads(folder,'fp32',i)
            er,_,_,_,_=ego_comparison(heads,fake);errors.extend(er)
            er,_,_,_,_=ego_comparison(heads,teacher);native_teacher_errors.extend(er)
            results.append(dict(**sample,native_vs_fake_kl=agreement(heads,fake),native_vs_teacher_kl=agreement(heads,teacher),
                fake_vs_teacher_kl=agreement(fake,teacher),native_cpu_ms=ms))
    report=dict(scope='Native QNNPACK CPU INT8 check on exactly matching normalized inputs; 4 frames per user video',
        samples=len(results),clips=len({r['clip'] for r in results}),
        native_vs_fake_mean_kl=float(np.mean([r['native_vs_fake_kl'] for r in results])),
        native_vs_teacher_mean_kl=float(np.mean([r['native_vs_teacher_kl'] for r in results])),
        fake_vs_teacher_mean_kl=float(np.mean([r['fake_vs_teacher_kl'] for r in results])),
        native_vs_fake_ego_mean_x_error_px640=float(np.mean(errors)) if errors else None,
        native_vs_teacher_ego_mean_x_error_px640=float(np.mean(native_teacher_errors)) if native_teacher_errors else None,
        cpu_native_mean_ms=float(np.mean([r['native_cpu_ms'] for r in results])),records=results)
    save(folder/'comparison.json',report)
    print(json.dumps({k:v for k,v in report.items() if k!='records'},indent=2),flush=True)

def native_video(args):
    """Actual exported INT8 on a complete representative video, offline replay."""
    import os
    import torch
    torch.set_num_threads(4);cv2.setNumThreads(1);torch.backends.quantized.engine='qnnpack'
    inputs=json.loads((args.output/'inputs.json').read_text(encoding='utf-8'))
    clip=next(c for c in inputs['clips'] if c['clip']==args.clip)
    source=clip['video']
    if os.name!='nt' and len(source)>2 and source[1]==':':
        source='/mnt/'+source[0].lower()+'/'+source[3:].replace('\\','/')
    output=args.output/clip['clip']
    with np.load(output/'fp32_slots.npz') as cache:
        packed=cache['slots'];indices=cache['frames']
    if not np.array_equal(indices,np.arange(0,clip['frames'],args.stride)):
        raise ValueError('FP32 video cache frame mismatch')
    path=ROOT/'training/ufldv2_qat_20261020/export/ufldv2_culane_res18_1600x320_qat_round2_int8.pt'
    model=torch.jit.load(str(path),map_location='cpu').eval()
    with torch.inference_mode():
        for _ in range(2):model(torch.zeros(1,3,320,1600))
    cap=cv2.VideoCapture(source);fps=clip['fps']/args.stride
    writer=cv2.VideoWriter(str(output/'native_comparison.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),fps,(1920,416))
    if not cap.isOpened() or not writer.isOpened():raise RuntimeError('Cannot open native video IO')
    trackers={'fp32':GeometryTracker(),'native_int8':GeometryTracker()}
    frame=0;rows=[];start_all=time.monotonic()
    try:
        with torch.inference_mode():
            while True:
                ok,image=cap.read()
                if not ok:break
                index=frame;frame+=1
                if index%args.stride:continue
                n=len(rows);resized=cv2.resize(image,(1600,533))[-320:]
                rgb=cv2.cvtColor(resized,cv2.COLOR_BGR2RGB).astype(np.float32)
                blob=np.ascontiguousarray(((rgb-[123.675,116.28,103.53])/[58.395,57.12,57.375]).transpose(2,0,1)[None],dtype=np.float32)
                tensor=torch.from_numpy(blob)
                start=time.perf_counter();heads=tuple(t.numpy() for t in model(tensor));ms=(time.perf_counter()-start)*1000
                if not all(np.isfinite(h).all() for h in heads):raise ValueError('Nonfinite native output')
                slots=decode(heads)
                reference=[p[np.isfinite(p).all(axis=1)].astype(np.float64) for p in packed[n]]
                small=cv2.resize(image,(640,360));views=[small]
                row=dict(frame=index,seconds=index/clip['fps'],native_cpu_ms=ms)
                for label,points in [('fp32',reference),('native_int8',slots)]:
                    raw=make_lane(points);lane=trackers[label].update(raw)
                    views.append(draw(small,lane))
                    row[label]=dict(raw_valid=raw.valid,display_valid=lane.valid,held=lane.held,
                        raw_x_at_y300=geometry_position(raw),cuda_ms=0.)
                rows.append(row)
                canvas=np.zeros((416,1920,3),np.uint8);canvas[32:392]=np.hstack(views)
                for c,title in enumerate(('Original source','FP32 CUDA cached / same frames','Continued QAT / native CPU INT8')):
                    cv2.putText(canvas,title,(c*640+12,23),cv2.FONT_HERSHEY_SIMPLEX,.62,(230,230,230),1,cv2.LINE_AA)
                caption=f'{clip["clip"][:8]} t={row["seconds"]:.2f}s | offline replay at {fps:g} fps; actual INT8 CPU inference, not real-time'
                cv2.putText(canvas,caption,(12,410),cv2.FONT_HERSHEY_SIMPLEX,.52,(230,230,230),1,cv2.LINE_AA)
                writer.write(canvas)
                if index==int(5*clip['fps']):cv2.imwrite(str(output/'native_comparison.jpg'),canvas)
                if n%75==74:print(f'Native INT8 video: {n+1}/{len(indices)}',flush=True)
    finally:cap.release();writer.release()
    if frame!=clip['frames'] or len(rows)!=len(indices):raise RuntimeError('Native sequence truncated')
    results={label:summarize(rows,label,fps) for label in trackers}
    for result in results.values():result.pop('cuda_inference_mean_ms')
    timings=[r['native_cpu_ms'] for r in rows]
    report=dict(clip=clip['clip'],frames=len(rows),source_frames=frame,sampling_stride=args.stride,
        video_fps=fps,scope='Real exported QNNPACK INT8 PC CPU, same source frames and geometry as CUDA FP32 cache',
        cpu_inference_mean_ms=float(np.mean(timings)),cpu_inference_p95_ms=float(np.percentile(timings,95)),
        elapsed_seconds=time.monotonic()-start_all,models=results,records=rows)
    save(output/'native_report.json',report)
    print(json.dumps({k:v for k,v in report.items() if k!='records'},indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','video','native-check','native-video'])
    p.add_argument('--archive',type=Path)
    p.add_argument('--output',type=Path,default=ROOT/'training/ufldv2_qat_20261020/user_zip_test')
    p.add_argument('--upstream',type=Path,default=ROOT.parent/'external/Ultra-Fast-Lane-Detection-v2')
    p.add_argument('--teacher',type=Path,default=ROOT.parent/'adas_training/weights/culane_res18.pth')
    p.add_argument('--checkpoint',type=Path,default=ROOT/'training/ufldv2_qat_20261020/run/best.pt')
    p.add_argument('--stride',type=int,default=2)
    p.add_argument('--clip',default='7679660a-42a3-4271-9d61-4f6153a961a4')
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    {'prepare':prepare,'video':video_test,'native-check':native_check,'native-video':native_video}[args.action](args)
