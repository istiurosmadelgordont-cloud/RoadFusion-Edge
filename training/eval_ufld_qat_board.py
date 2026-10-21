"""Compare real RK3568 outputs with the original float teacher and native QAT.

Three held-out-video crops are a conversion smoke test, not a labelled score.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import torch
from train_ufld_qat import LaneNet, agreement_loss, HEADS

SHAPES=((1,200,72,4),(1,100,81,4),(1,2,72,4),(1,2,81,4))


def comparison(student,teacher):
    result={'teacher_kl_t2':float(agreement_loss(student,teacher))}
    for index,name in enumerate(HEADS):
        result[name+'_argmax_agreement']=float((student[index].argmax(1)==teacher[index].argmax(1)).float().mean())
    return result


def run(args):
    torch.set_num_threads(2)
    cv2.setNumThreads(1)
    records=json.loads((args.probe/'samples.json').read_text(encoding='utf-8'))
    model=LaneNet(args.upstream)
    saved=torch.load(str(args.checkpoint),map_location='cpu')
    state=saved.get('model',saved)
    model.load_state_dict({k.removeprefix('module.'):v for k,v in state.items()},strict=True)
    del saved,state
    model.eval()
    native=torch.jit.load(str(args.native),map_location='cpu').eval()
    results=[]
    with torch.inference_mode():
        for index,record in enumerate(records):
            crop=cv2.imread(str(args.data/record['image']))
            assert crop.shape==(320,1600,3)
            rgb=cv2.cvtColor(crop,cv2.COLOR_BGR2RGB).astype(np.float32)
            values=(rgb-[123.675,116.28,103.53])/[58.395,57.12,57.375]
            tensor=torch.from_numpy(np.ascontiguousarray(values.transpose(2,0,1),dtype=np.float32))[None]
            teacher=model(tensor)
            quantized=native(tensor)
            outputs={}
            for label in ('baseline','qat'):
                outputs[label]=tuple(torch.from_numpy(np.fromfile(
                    args.probe/f'{label}_{index}.output{head}.f32',dtype=np.float32).reshape(shape))
                    for head,shape in enumerate(SHAPES))
                assert all(torch.isfinite(v).all() for v in outputs[label])
            item={'clip':record['clip'],'frame':record['frame'],
                  'baseline_vs_teacher':comparison(outputs['baseline'],teacher),
                  'qat_board_vs_teacher':comparison(outputs['qat'],teacher),
                  'qat_native_vs_teacher':comparison(quantized,teacher),
                  'qat_board_vs_native':comparison(outputs['qat'],quantized)}
            results.append(item)
            print(json.dumps(item),flush=True)
    summary={key:{metric:sum(r[key][metric] for r in results)/len(results)
                  for metric in results[0][key]} for key in results[0] if key.endswith('teacher') or key.endswith('native')}
    report={'scope':'conversion smoke test on 3 held-out-video crops; no ground-truth lane accuracy',
            'baseline_kept':True,'samples':results,'mean':summary}
    (args.probe/'comparison.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(summary,indent=2),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--probe',type=Path,required=True)
    p.add_argument('--data',type=Path,required=True)
    p.add_argument('--native',type=Path,required=True)
    p.add_argument('--checkpoint',type=Path,required=True)
    p.add_argument('--upstream',type=Path,required=True)
    run(p.parse_args())
