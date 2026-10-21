"""Import an already quantized native QAT graph; never apply a second PTQ pass."""
import argparse
import json
from pathlib import Path
from rknn.api import RKNN


def checked(stage,code):
    if code:
        raise RuntimeError('%s failed: %s'%(stage,code))


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--torchscript',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    metadata=args.torchscript.with_suffix('.json')
    info=json.loads(metadata.read_text(encoding='utf-8'))
    if not info.get('native_quantized_operators') or info.get('recalibrated'):
        raise RuntimeError('Expected verified native QAT graph without recalibration')
    runtime=RKNN(verbose=True)
    try:
        checked('config',runtime.config(target_platform='rk3568',
            mean_values=[[123.675,116.28,103.53]],
            std_values=[[58.395,57.12,57.375]],optimization_level=3))
        checked('load_pytorch',runtime.load_pytorch(model=str(args.torchscript.resolve()),
                                                   input_size_list=[[1,3,320,1600]]))
        checked('build',runtime.build(do_quantization=False))
        args.output.parent.mkdir(parents=True,exist_ok=True)
        checked('export_rknn',runtime.export_rknn(str(args.output.resolve())))
        info.update(target='rk3568',native_qat_import=True,second_ptq=False,
                    rknn_output=str(args.output.resolve()),board_validation='pending')
        args.output.with_suffix('.json').write_text(json.dumps(info,indent=2),encoding='utf-8')
        print('QAT candidate exported: '+str(args.output),flush=True)
    finally:
        runtime.release()
