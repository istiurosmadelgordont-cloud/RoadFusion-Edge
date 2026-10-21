"""Teacher-guided INT8 QAT of the existing 1600x320 UFLDv2 ResNet18.

Conv/Linear layers use native eager QAT with Quant/DeQuant boundaries, matching
the project's RKNN QAT import route. LayerNorm/residual arithmetic stays float.
The loss measures float-teacher agreement; it is not a labelled CULane score.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import random
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import torch
from torch import nn
from torch.ao.quantization import (
    QuantStub, DeQuantStub, FakeQuantize, QConfig, MovingAverageMinMaxObserver,
    disable_observer, enable_observer, prepare_qat,
)
from torch.utils.data import Dataset, DataLoader

ROOT = Path(__file__).resolve().parents[1]
HEADS = ('loc_row','loc_col','exist_row','exist_col')


class QuantLayer(nn.Module):
    def __init__(self, layer):
        super().__init__()
        self.quant = QuantStub()
        self.layer = layer
        self.dequant = DeQuantStub()

    def forward(self, image):
        return self.dequant(self.layer(self.quant(image)))


class LaneNet(nn.Module):
    def __init__(self, upstream):
        super().__init__()
        sys.path.insert(0,str(upstream))
        from model.backbone import resnet
        self.model = resnet('18',pretrained=False)
        self.pool = nn.Conv2d(512,8,1)
        self.cls = nn.Sequential(nn.LayerNorm(4000),nn.Linear(4000,2048),
                                 nn.ReLU(),nn.Linear(2048,91224))

    def forward(self, image):
        _,_,features = self.model(image)
        output = self.cls(self.pool(features).flatten(1))
        return (output[:,:57600].view(-1,200,72,4),
                output[:,57600:90000].view(-1,100,81,4),
                output[:,90000:90576].view(-1,2,72,4),
                output[:,90576:].view(-1,2,81,4))


def prepare_model(model):
    config = QConfig(
        activation=FakeQuantize.with_args(observer=MovingAverageMinMaxObserver,
            quant_min=0,quant_max=255,dtype=torch.quint8,
            qscheme=torch.per_tensor_affine,reduce_range=False),
        weight=FakeQuantize.with_args(observer=MovingAverageMinMaxObserver,
            quant_min=-128,quant_max=127,dtype=torch.qint8,
            qscheme=torch.per_tensor_symmetric,reduce_range=False))
    count = 0
    def visit(parent):
        nonlocal count
        for name,child in list(parent.named_children()):
            if isinstance(child,(nn.Conv2d,nn.Linear)):
                wrapper=QuantLayer(child)
                wrapper.qconfig=config
                setattr(parent,name,wrapper)
                count += 1
            else:
                visit(child)
    model.qconfig=None
    visit(model)
    model.train()
    prepare_qat(model,inplace=True)
    return count


def freeze_bn(model):
    for module in model.modules():
        if isinstance(module,nn.modules.batchnorm._BatchNorm):
            module.eval()


class ReplayDataset(Dataset):
    def __init__(self, manifest_path, split):
        self.root=manifest_path.parent
        self.records=[r for r in json.loads(manifest_path.read_text(encoding='utf-8'))['samples']
                      if r['split']==split]

    def __len__(self):
        return len(self.records)

    def __getitem__(self,index):
        image=cv2.imread(str(self.root/self.records[index]['image']))
        if image is None or image.shape != (320,1600,3):
            raise RuntimeError('QAT input must be a decoded 1600x320 crop')
        rgb=cv2.cvtColor(image,cv2.COLOR_BGR2RGB).astype(np.float32)
        normalized=(rgb-[123.675,116.28,103.53])/[58.395,57.12,57.375]
        return torch.from_numpy(np.ascontiguousarray(normalized.transpose(2,0,1),dtype=np.float32))


def agreement_loss(prediction, target, temperature=2.):
    loss=prediction[0].new_zeros(())
    for index,(student,teacher) in enumerate(zip(prediction,target)):
        weight=1. if index<2 else .5
        term=torch.nn.functional.kl_div(
            torch.log_softmax(student/temperature,dim=1),
            torch.softmax(teacher/temperature,dim=1),reduction='none')
        loss=loss+weight*term.sum(dim=1).mean()*(temperature**2)
    return loss


def atomic_json(path,data):
    temporary=path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    os.replace(temporary,path)


def run(args):
    args.output.mkdir(parents=True,exist_ok=True)
    status_path=args.output/'status.json'
    state=dict(phase='loading',updated_at='',optimizer_steps=0,epoch=0,
               input_shape=[1,3,320,1600],qat=True,
               validation_scope='video-disjoint float-teacher agreement; no GT accuracy claim',
               baseline_rknn_unchanged=True,source_checkpoint=str(args.checkpoint.resolve()),
               manifest=str(args.manifest.resolve()),torch_version=torch.__version__)
    def status(**values):
        state.update(values)
        state['updated_at']=datetime.now(timezone.utc).isoformat()
        atomic_json(status_path,state)
    status()
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA is required for this 1600x320 QAT run')
    torch.set_num_threads(4)
    cv2.setNumThreads(1)
    random.seed(args.seed);np.random.seed(args.seed);torch.manual_seed(args.seed)
    torch.cuda.manual_seed_all(args.seed)
    torch.backends.cudnn.benchmark=False
    train=ReplayDataset(args.manifest,'train')
    validation=ReplayDataset(args.manifest,'val')
    if not len(train) or not len(validation):
        raise RuntimeError('Both video-disjoint splits must be nonempty')
    train_clips={r['clip'] for r in train.records}
    val_clips={r['clip'] for r in validation.records}
    if train_clips & val_clips:
        raise RuntimeError('A source video appears in both splits')
    teacher=LaneNet(args.upstream)
    checkpoint=torch.load(args.checkpoint,map_location='cpu',weights_only=False)
    source=checkpoint['model'] if 'model' in checkpoint else checkpoint
    source={key.removeprefix('module.'):value for key,value in source.items()}
    teacher.load_state_dict(source,strict=True)
    del checkpoint,source
    teacher.eval().requires_grad_(False)
    student=copy.deepcopy(teacher)
    student.requires_grad_(True)
    layer_count=prepare_model(student)
    teacher=teacher.cuda();student=student.cuda()
    optimizer=torch.optim.SGD(student.parameters(),lr=args.lr,momentum=.9,weight_decay=1e-5)
    start_epoch=next_batch=global_step=0
    best=float('inf')
    last_path=args.output/'last.pt'
    if args.resume and args.resume_from is not None:
        raise ValueError('Use --resume or --resume-from, not both')
    resume_path=args.resume_from if args.resume_from is not None else last_path
    if args.resume_from is not None and not resume_path.is_file():
        raise FileNotFoundError(resume_path)
    if (args.resume or args.resume_from is not None) and resume_path.exists():
        saved=torch.load(resume_path,map_location='cpu',weights_only=False)
        student.load_state_dict(saved['model'],strict=True)
        optimizer.load_state_dict(saved['optimizer'])
        start_epoch=saved['epoch'];next_batch=saved['next_batch']
        global_step=saved['optimizer_steps'];best=saved['best_validation_kl']
        random.setstate(saved['python_rng']);np.random.set_state(saved['numpy_rng'])
        torch.set_rng_state(saved['torch_rng'])
        torch.cuda.set_rng_state_all(saved['cuda_rng'])
        previous_best=best
        # An independent continuation selects its own best epoch; it must not
        # refer to an absent best.pt from another output directory.
        if args.resume_from is not None:
            best=float('inf')
        # load_state_dict restores the old LR too. Apply the requested value
        # explicitly so a lower-rate continuation really uses the lower rate.
        for group in optimizer.param_groups:
            group['lr']=args.lr
        status(resume_checkpoint=str(resume_path.resolve()),initial_epoch=start_epoch,
               initial_optimizer_steps=global_step,
               previous_best_validation_kl=previous_best if math.isfinite(previous_best) else None,
               independent_continuation=args.resume_from is not None)
        del saved
    status(device=torch.cuda.get_device_name(0),train_images=len(train),val_images=len(validation),
           train_videos=len(train_clips),val_videos=len(val_clips),quantized_layers=layer_count,
           epochs=args.epochs,accumulation=args.accumulation,learning_rate=args.lr,
           optimizer_steps=global_step,resumed=bool(global_step),fake_quant_enabled=True,
           observer_freeze_epoch=args.freeze_observers_epoch)
    config={key:str(value) if isinstance(value,Path) else value for key,value in vars(args).items()}
    atomic_json(args.output/'config.json',config)
    val_loader=DataLoader(validation,batch_size=1,shuffle=False,num_workers=0)

    def save(epoch,batch):
        target=last_path.with_suffix('.pt.tmp')
        # Move state tensors to CPU for portable restore without moving the
        # running model or its observers off the GPU.
        model_state={k:v.detach().cpu() for k,v in student.state_dict().items()}
        torch.save(dict(model=model_state,optimizer=optimizer.state_dict(),epoch=epoch,
                        next_batch=batch,optimizer_steps=global_step,best_validation_kl=best,
                        python_rng=random.getstate(),numpy_rng=np.random.get_state(),
                        torch_rng=torch.get_rng_state(),cuda_rng=torch.cuda.get_rng_state_all(),
                        quantized_layers=layer_count,input_shape=[1,3,320,1600],
                        format='native_eager_QAT_prepared_state',config=config),target)
        os.replace(target,last_path)

    if args.resume_from is not None:
        save(start_epoch,next_batch)

    # Initialise observer ranges with training data only. This is not PTQ:
    # fake quant remains active during every subsequent gradient update.
    if global_step == 0:
        status(phase='observer_warmup')
        student.train();freeze_bn(student)
        with torch.no_grad():
            for index in range(min(args.warmup_samples,len(train))):
                student(train[index].unsqueeze(0).cuda())
                status(warmup_samples=index+1)
    started=time.monotonic()
    metrics_path=args.output/'metrics.jsonl'
    for epoch in range(start_epoch,args.epochs):
        student.train();freeze_bn(student)
        student.apply(disable_observer if epoch>=args.freeze_observers_epoch else enable_observer)
        loader=DataLoader(train,batch_size=1,shuffle=True,num_workers=0,
                          generator=torch.Generator().manual_seed(args.seed+epoch))
        optimizer.zero_grad(set_to_none=True)
        accumulated=0;step_loss=0.
        for batch_index,images in enumerate(loader):
            if epoch==start_epoch and batch_index<next_batch:
                continue
            images=images.cuda(non_blocking=True)
            with torch.no_grad():
                target=teacher(images)
            prediction=student(images)
            loss=agreement_loss(prediction,target)
            if not torch.isfinite(loss):
                raise RuntimeError('Non-finite QAT loss')
            (loss/args.accumulation).backward()
            accumulated+=1;step_loss+=float(loss.detach())
            if accumulated<args.accumulation and batch_index+1<len(loader):
                continue
            # Correct a partial final accumulation group.
            if accumulated<args.accumulation:
                for parameter in student.parameters():
                    if parameter.grad is not None:
                        parameter.grad.mul_(args.accumulation/accumulated)
            torch.nn.utils.clip_grad_norm_(student.parameters(),5.)
            optimizer.step();optimizer.zero_grad(set_to_none=True)
            global_step+=1
            loss_value=step_loss/accumulated
            accumulated=0;step_loss=0.
            status(phase='training',epoch=epoch+1,batch=batch_index+1,
                   batches_per_epoch=len(loader),optimizer_steps=global_step,
                   last_train_kl=loss_value,elapsed_seconds=round(time.monotonic()-started,1),
                   gpu_peak_mb=round(torch.cuda.max_memory_allocated()/1048576,1))
            with metrics_path.open('a',encoding='utf-8') as handle:
                handle.write(json.dumps(dict(step=global_step,epoch=epoch+1,train_kl=loss_value))+'\n')
            if global_step<=2 or global_step%10==0:
                print(f'QAT epoch={epoch+1}/{args.epochs} step={global_step} '
                      f'batch={batch_index+1}/{len(loader)} loss={loss_value:.6f}',flush=True)
            if global_step in (1,2) or global_step%args.save_every==0:
                save(epoch,batch_index+1)
            if args.max_steps and global_step>=args.max_steps:
                save(epoch,batch_index+1)
                status(phase='step_limit_complete',checkpoint=str(last_path.resolve()))
                return
        next_batch=0
        status(phase='validation',epoch=epoch+1)
        student.eval();student.apply(disable_observer)
        losses=[]
        with torch.no_grad():
            for images in val_loader:
                images=images.cuda()
                losses.append(float(agreement_loss(student(images),teacher(images))))
        score=sum(losses)/len(losses)
        by_video={}
        for record,value in zip(validation.records,losses):
            by_video.setdefault(record['clip'],[]).append(value)
        validation_record=dict(kind='validation',step=global_step,epoch=epoch+1,
            validation_kl=score,per_video_kl={clip:sum(values)/len(values) for clip,values in by_video.items()})
        with metrics_path.open('a',encoding='utf-8') as handle:
            handle.write(json.dumps(validation_record)+'\n')
        if score<best:
            best=score
            target=args.output/'best.pt.tmp'
            torch.save(dict(model={k:v.detach().cpu() for k,v in student.state_dict().items()},
                            optimizer_steps=global_step,validation_kl=score,
                            format='native_eager_QAT_prepared_state',config=config),target)
            os.replace(target,args.output/'best.pt')
        save(epoch+1,0)
        status(last_validation_kl=score,best_validation_kl=best)
        print(f'QAT validation epoch={epoch+1} teacher_agreement_kl={score:.6f}',flush=True)
    status(phase='training_complete_pending_export',checkpoint=str(last_path.resolve()),
           best_checkpoint=str((args.output/'best.pt').resolve()))


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--upstream',type=Path,default=ROOT.parent/'external/Ultra-Fast-Lane-Detection-v2')
    parser.add_argument('--checkpoint',type=Path,default=ROOT.parent/'adas_training/weights/culane_res18.pth')
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--epochs',type=int,default=8)
    parser.add_argument('--accumulation',type=int,default=4)
    parser.add_argument('--lr',type=float,default=1e-5)
    parser.add_argument('--freeze-observers-epoch',type=int,default=2)
    parser.add_argument('--warmup-samples',type=int,default=8)
    parser.add_argument('--save-every',type=int,default=50)
    parser.add_argument('--seed',type=int,default=20261016)
    parser.add_argument('--max-steps',type=int,default=0)
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--resume-from',type=Path,
                        help='Continue a full training checkpoint into an independent output directory')
    args=parser.parse_args()
    try:
        run(args)
    except BaseException as error:
        args.output.mkdir(parents=True,exist_ok=True)
        path=args.output/'status.json'
        previous=json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
        previous.update(phase='failed',error=f'{type(error).__name__}: {error}',
                        updated_at=datetime.now(timezone.utc).isoformat())
        atomic_json(path,previous)
        raise
