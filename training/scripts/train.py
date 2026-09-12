"""Train a source-specific YOLOv8n baseline after dataset preparation.

No training starts on import. This project deliberately does not mix datasets
whose images lack annotations for the other sources' foreground classes.
"""
import argparse
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
RUNTIME_CACHE=ROOT/'.runtime_cache'
for cache_dir in ('tmp','torch','huggingface','pip'):(RUNTIME_CACHE/cache_dir).mkdir(parents=True,exist_ok=True)
os.environ['TEMP']=os.environ['TMP']=str(RUNTIME_CACHE/'tmp')
os.environ['TORCH_HOME']=str(RUNTIME_CACHE/'torch')
os.environ['HF_HOME']=str(RUNTIME_CACHE/'huggingface')
os.environ['PIP_CACHE_DIR']=str(RUNTIME_CACHE/'pip')
os.environ.setdefault('YOLO_CONFIG_DIR',str(ROOT/'.yolo'))
os.environ.setdefault('YOLO_OFFLINE','true')
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'.matplotlib'))
CONFIGS={'road':'bdd12.yaml','crosswalk':'crosswalk2.yaml','workzone':'workzone3_grouped.yaml',
         'unified':'unified14.yaml','unified17':'unified17_v3_clean.yaml',
         'unified17v4':'unified17_v4_core.yaml','unified17v5':'unified17_v5_core.yaml',
         'unified17v6':'unified17_v6_s2tld.yaml','unified17v7':'unified17_v7_balanced.yaml',
         'unified17v8':'unified17_v8_targeted.yaml','unified17v9':'unified17_v9_newdata.yaml',
         'unified17v10':'unified17_v10_anchor.yaml','unified17v11':'unified17_v11_hard.yaml',
         'unified19dir':'unified19_directional.yaml',
         'unified19dirv2':'unified19_directional_v2.yaml',
         'unified19lightv3':'unified19_light_focus_v3.yaml',
         'unified19lightv4':'unified19_light_focus_v4.yaml',
         'unified21lightv4':'unified21_light_focus_v4.yaml'}
STATUS=ROOT/'reports'/'training_status.json'


def write_status(state, **details):
    payload={'state':state,'updated_at':datetime.now().astimezone().isoformat(),'pid':os.getpid(),**details}
    temp=STATUS.with_name(f'{STATUS.stem}.{os.getpid()}.tmp')
    for attempt in range(20):
        try:
            temp.write_text(json.dumps(payload,ensure_ascii=False,indent=2),encoding='utf-8')
            temp.replace(STATUS)
            return True
        except PermissionError:
            time.sleep(0.05*(attempt+1))
    # Progress reporting must never terminate a healthy training run.
    print(f'WARNING: status file remained locked; skipped one {state} update.',file=sys.stderr,flush=True)
    return False


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',choices=CONFIGS,default='road')
    p.add_argument('--epochs',type=int,default=5)
    p.add_argument('--batch',type=int,default=8)
    p.add_argument('--imgsz',type=int,default=640)
    p.add_argument('--workers',type=int,default=2)
    p.add_argument('--patience',type=int,default=15,help='Stop after this many epochs without validation fitness improvement')
    p.add_argument('--weights',default='yolov8n.pt')
    p.add_argument('--lr0',type=float,default=0.01,help='Initial learning rate')
    p.add_argument('--lrf',type=float,default=0.01,help='Final learning-rate fraction')
    p.add_argument('--optimizer',default='auto',help='Ultralytics optimizer, e.g. auto, SGD or AdamW')
    p.add_argument('--warmup-epochs',type=float,default=3.0)
    p.add_argument('--warmup-bias-lr',type=float,default=0.1)
    p.add_argument('--close-mosaic',type=int,default=10)
    p.add_argument('--mosaic',type=float,default=1.0)
    p.add_argument('--hsv-s',type=float,default=0.3)
    p.add_argument('--hsv-v',type=float,default=0.3)
    p.add_argument('--save-period',type=int,default=-1,help='Save a checkpoint every N epochs; -1 disables periodic saves')
    p.add_argument('--freeze',type=int,default=0,help='Freeze the first N model layers')
    p.add_argument('--name')
    p.add_argument('--resume',action='store_true',help='Resume optimizer, scheduler and epoch from an interrupted last.pt checkpoint')
    p.add_argument('--check-only',action='store_true',help='Validate paths and labels without importing torch or starting training')
    args=p.parse_args()
    import yaml
    config_path=ROOT/'configs'/CONFIGS[args.task]
    config=yaml.safe_load(config_path.read_text(encoding='utf-8'))
    summary={}
    identifiers={}
    for split in ('train','val'):
        images=Path(config['path'])/config[split]
        labels=Path(config['path'])/'labels'/Path(config[split]).name
        files=sorted(images.glob('*.jpg'))
        if not files:raise RuntimeError(f'No images in {images}')
        counts=[0]*len(config['names'])
        for image in files:
            label=labels/(image.stem+'.txt')
            if not label.exists():raise RuntimeError(f'Missing annotation: {label}')
            for line in label.read_text().splitlines():
                fields=line.split()
                if len(fields)!=5:raise ValueError(f'Invalid detection row: {label}')
                cid=int(fields[0]);xc,yc,w,h=map(float,fields[1:])
                if not 0<=cid<len(counts) or not all(0<=v<=1 for v in (xc,yc,w,h)) or min(w,h)<=0:
                    raise ValueError(f'Invalid normalized box: {label}')
                if xc-w/2 < -1e-7 or yc-h/2 < -1e-7 or xc+w/2>1+1e-7 or yc+h/2>1+1e-7:
                    raise ValueError(f'Box outside image: {label}')
                counts[cid]+=1
        if not all(counts):raise RuntimeError(f'{split} has classes with no annotations: {counts}')
        identifiers[split]={x.stem for x in files}
        summary[split]={'images':len(files),'instances':counts}
    if identifiers['train'] & identifiers['val']:
        raise RuntimeError('Overlapping image names between training and validation')
    print(json.dumps(summary,indent=2),flush=True)
    if args.check_only:return
    write_status('initializing',task=args.task,epochs=args.epochs,batch=args.batch,imgsz=args.imgsz)
    try:
        import torch
        from ultralytics import YOLO, settings
    except ImportError as exc:
        raise SystemExit('Training dependencies are not installed in this Python. See README.md; .venv-audit is for data checks only.') from exc
    if not torch.cuda.is_available():raise SystemExit('CUDA is unavailable; install the CUDA PyTorch build before training.')
    settings.update({'sync':False,'weights_dir':str(ROOT/'weights')})
    if args.epochs<1 or args.batch<1 or args.patience<1:raise ValueError('epochs, batch and patience must be positive')
    print('GPU:',torch.cuda.get_device_name(0),flush=True)
    print('Data is provisional: read reports/数据检查报告.md before interpreting model accuracy.',flush=True)
    model=YOLO(args.weights)
    progress={'last_update':0.0,'batch_in_epoch':0}
    def record(trainer,state):
        write_status(state,task=args.task,epoch=trainer.epoch+1,epochs=trainer.epochs,
                     batch_in_epoch=progress['batch_in_epoch'],batches_per_epoch=len(trainer.train_loader),
                     save_dir=str(trainer.save_dir),metrics={k:float(v) for k,v in trainer.metrics.items()})
    def epoch_start(trainer):
        progress['batch_in_epoch']=0
        record(trainer,'training')
    def batch_end(trainer):
        progress['batch_in_epoch']+=1
        if time.monotonic()-progress['last_update']>=15:
            record(trainer,'training')
            progress['last_update']=time.monotonic()
    model.add_callback('on_train_epoch_start',epoch_start)
    model.add_callback('on_train_batch_end',batch_end)
    model.add_callback('on_fit_epoch_end',lambda trainer:record(trainer,'epoch_completed'))
    model.add_callback('on_train_end',lambda trainer:record(trainer,'completed'))
    if args.resume:
        model.train(resume=True)
    else:
        model.train(data=str(config_path),epochs=args.epochs,imgsz=args.imgsz,batch=args.batch,
                    device=0,workers=args.workers,patience=args.patience,cache=False,amp=True,
                    hsv_h=0.0,hsv_s=args.hsv_s,hsv_v=args.hsv_v,mosaic=args.mosaic,
                    flipud=0.0,fliplr=0.0,seed=3568,project=str(ROOT/'runs'),
                    optimizer=args.optimizer,lr0=args.lr0,lrf=args.lrf,cos_lr=True,
                    warmup_epochs=args.warmup_epochs,warmup_bias_lr=args.warmup_bias_lr,
                    close_mosaic=args.close_mosaic,save_period=args.save_period,
                    freeze=args.freeze,
                    name=args.name or f'{args.task}_v8n_{args.epochs}epochs',exist_ok=False)


if __name__=='__main__':
    try:
        main()
    except BaseException as exc:
        if '--check-only' not in sys.argv:
            write_status('failed',error=repr(exc))
        raise
