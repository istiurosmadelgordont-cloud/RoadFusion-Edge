"""Same-input comparisons for a QAT continuation. No ground-truth accuracy claim."""
import argparse,json,sys
from pathlib import Path
import numpy as np
import torch
from train_ufld_qat import LaneNet,ReplayDataset,prepare_model,disable_observer,agreement_loss

SHAPES=((1,200,72,4),(1,100,81,4),(1,2,72,4),(1,2,81,4))

def teacher_reference(args):
    torch.set_num_threads(4)
    torch.backends.cuda.matmul.allow_tf32=False
    torch.backends.cudnn.allow_tf32=False
    model=LaneNet(args.upstream)
    saved=torch.load(args.teacher,map_location='cpu',weights_only=False)
    model.load_state_dict({k.removeprefix('module.'):v for k,v in saved.get('model',saved).items()});del saved
    model.eval().requires_grad_(False).cuda()
    student=LaneNet(args.upstream);prepare_model(student)
    saved=torch.load(args.checkpoint,map_location='cpu',weights_only=False)
    student.load_state_dict(saved['model']);del saved
    student.eval().apply(disable_observer);student.cuda()
    dataset=ReplayDataset(args.manifest,'val')
    args.output.mkdir(parents=True,exist_ok=True)
    losses=[]
    with torch.inference_mode():
        for index in range(len(dataset)):
            image=dataset[index][None].cuda()
            target=model(image);prediction=student(image)
            losses.append(float(agreement_loss(prediction,target)))
            for head,values in enumerate(target):
                values.cpu().numpy().astype(np.float32).tofile(args.output/f'fp32_{index}.output{head}.f32')
    report=dict(scope='FP32 teacher agreement only; unlabelled video-disjoint validation',samples=len(losses),
                fake_quant_mean_kl=float(np.mean(losses)),per_sample_kl=losses)
    (args.output/'teacher_reference.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='per_sample_kl'}),flush=True)

def comparison(args):
    torch.set_num_threads(2)
    samples=json.loads((args.probe/'samples.json').read_text(encoding='utf-8'))
    results={}
    for label in ['baseline','qat_first','qat_round2']:
        records=[];all_ego_errors=[];presence_equal=presence_total=0
        for index,sample in enumerate(samples):
            def load(root,prefix):
                return tuple(torch.from_numpy(np.fromfile(root/f'{prefix}_{index}.output{head}.f32',np.float32).reshape(shape)) for head,shape in enumerate(SHAPES))
            teacher=load(args.output,'fp32');candidate=load(args.probe,label)
            if not all(torch.isfinite(t).all() for t in candidate):raise ValueError('Nonfinite probe output')
            errors=[];same=total=0
            for slot in (1,2):
                a=teacher[0][0,:,:,slot].numpy();b=candidate[0][0,:,:,slot].numpy()
                ae=teacher[2][0,:,:,slot].argmax(0).numpy().astype(bool)
                be=candidate[2][0,:,:,slot].argmax(0).numpy().astype(bool)
                same+=int((ae==be).sum());total+=len(ae)
                for anchor in np.flatnonzero(ae&be):
                    def position(logits):
                        best=int(np.argmax(logits));idx=np.arange(max(0,best-1),min(199,best+1)+1)
                        probabilities=np.exp(logits[idx]-logits[best]);return float(np.dot(probabilities,idx+.5)/probabilities.sum()/199*640)
                    errors.append(abs(position(a[:,anchor])-position(b[:,anchor])))
            row=dict(clip=sample['clip'],frame=sample['frame'],teacher_kl=float(agreement_loss(candidate,teacher)),
                     matched_ego_anchors=len(errors),ego_mean_x_error_px640=float(np.mean(errors)) if errors else None,
                     ego_presence_agreement=same/total)
            records.append(row);all_ego_errors.extend(errors);presence_equal+=same;presence_total+=total
        import csv
        timing=list(csv.DictReader((args.probe/(label+'.timings.csv')).open()))
        ms=np.array([float(r['inference_inputs_run_outputs_ms']) for r in timing])
        results[label]=dict(samples=len(records),mean_teacher_kl=float(np.mean([r['teacher_kl'] for r in records])),
            ego_presence_agreement=presence_equal/presence_total,matched_ego_anchors=len(all_ego_errors),
            ego_mean_x_error_px640=float(np.mean(all_ego_errors)),ego_p95_x_error_px640=float(np.percentile(all_ego_errors,95)),
            inference_mean_ms=float(ms.mean()),inference_p50_ms=float(np.percentile(ms,50)),
            inference_p95_ms=float(np.percentile(ms,95)),records=records)
    report=dict(scope='96 identical 1600x320 JPEG crops from 3 held-out videos; actual RK3568 outputs vs frozen FP32 teacher. No GT accuracy claim.',
        geometry_units='x displacement in 640-pixel view at matched ego anchors; presence agreement reported separately',
        baseline_deployment_unchanged=True,models=results)
    (args.output/'comparison.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({label:{k:v for k,v in result.items() if k!='records'} for label,result in results.items()},indent=2),flush=True)

if __name__=='__main__':
    root=Path(__file__).resolve().parents[1]
    p=argparse.ArgumentParser();p.add_argument('action',choices=['reference','compare'])
    p.add_argument('--upstream',type=Path,default=root.parent/'external/Ultra-Fast-Lane-Detection-v2')
    p.add_argument('--teacher',type=Path,default=root.parent/'adas_training/weights/culane_res18.pth')
    p.add_argument('--manifest',type=Path,default=root/'training/ufldv2_qat_20261016/data/manifest.json')
    p.add_argument('--checkpoint',type=Path,default=root/'training/ufldv2_qat_20261020/run/best.pt')
    p.add_argument('--output',type=Path,default=root/'training/ufldv2_qat_20261020/evaluation')
    p.add_argument('--probe',type=Path,default=root/'training/ufldv2_qat_20261020/board_probe')
    args=p.parse_args()
    teacher_reference(args) if args.action=='reference' else comparison(args)
