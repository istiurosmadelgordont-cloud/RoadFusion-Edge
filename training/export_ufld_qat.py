"""Export learned eager QAT state without observer recalibration or PTQ.

Run with the SDK-matched Linux Torch for RKNN-Toolkit2 1.5 compatibility.
The existing deployed RKNN is never overwritten.
"""
import argparse
import inspect
import json
import time
from pathlib import Path

import torch
from train_ufld_qat import LaneNet, prepare_model


def export(args):
    torch.set_num_threads(4)
    torch.backends.quantized.engine='qnnpack'
    kwargs={'map_location':'cpu'}
    if 'weights_only' in inspect.signature(torch.load).parameters:
        kwargs['weights_only']=False
    saved=torch.load(args.checkpoint,**kwargs)
    if saved.get('format')!='native_eager_QAT_prepared_state':
        raise RuntimeError('Expected learned QAT weights and observer state')
    model=LaneNet(args.upstream)
    count=prepare_model(model)
    model.load_state_dict(saved['model'],strict=True)
    metadata=dict(training_checkpoint=str(args.checkpoint.resolve()),
                  optimizer_steps=saved['optimizer_steps'],torch_version=torch.__version__,
                  quantized_layers=count,engine='qnnpack',recalibrated=False,
                  input_shape=[1,3,320,1600],head_names=['loc_row','loc_col','exist_row','exist_col'])
    del saved
    model.eval()
    model.apply(torch.ao.quantization.disable_observer)
    torch.ao.quantization.convert(model,inplace=True)
    tensor=torch.zeros(1,3,320,1600)
    started=time.monotonic()
    with torch.inference_mode():
        outputs=model(tensor)
        if [list(o.shape) for o in outputs]!=[[1,200,72,4],[1,100,81,4],[1,2,72,4],[1,2,81,4]]:
            raise RuntimeError('QAT conversion changed the four-head schema')
        if not all(torch.isfinite(output).all() for output in outputs):
            raise RuntimeError('QAT converted model has nonfinite outputs')
        trace=torch.jit.trace(model,tensor,strict=False,check_trace=False)
    graph=str(trace.inlined_graph)
    if 'quantized::conv2d' not in graph or 'quantized::linear' not in graph:
        raise RuntimeError('Expected actual INT8 Conv/Linear operators in exported graph')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    trace.save(str(args.output))
    metadata.update(native_quantized_operators=True,output_shapes=[list(o.shape) for o in outputs],
                    conversion_seconds=round(time.monotonic()-started,2))
    args.output.with_suffix('.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
    print(json.dumps(metadata,indent=2),flush=True)


if __name__=='__main__':
    root=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser()
    parser.add_argument('--checkpoint',type=Path,required=True)
    parser.add_argument('--upstream',type=Path,default=root.parent/'external/Ultra-Fast-Lane-Detection-v2')
    parser.add_argument('--output',type=Path,required=True)
    export(parser.parse_args())
