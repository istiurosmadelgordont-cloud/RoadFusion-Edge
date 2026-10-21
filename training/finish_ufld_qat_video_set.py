"""Verify and package the PC test results without changing deployed models."""
from __future__ import annotations
import argparse
import hashlib
import html
import json
import os
from pathlib import Path
import subprocess
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]

def save(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')

def encode(args):
    summary=json.loads((args.output/'summary.json').read_text(encoding='utf-8'))
    if summary['clips_finished']!=summary['clips_total']:raise RuntimeError('Video tests incomplete')
    verified=[]
    for clip in summary['clips']:
        folder=args.output/clip['clip']['clip']
        sources=[folder/'comparison.mp4']
        if (folder/'native_report.json').exists():sources.append(folder/'native_comparison.mp4')
        for source in sources:
            target=source.with_name(source.stem+'_preview.mp4')
            subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(source),'-an',
                '-c:v','libx264','-preset','veryfast','-crf','20','-threads','2','-pix_fmt','yuv420p',
                '-movflags','+faststart',str(target)],check=True)
            info=json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0',
                '-show_entries','stream=nb_frames,avg_frame_rate,width,height,codec_name','-of','json',str(target)]))['streams'][0]
            numerator,denominator=map(float,info['avg_frame_rate'].split('/'))
            if (int(info['nb_frames'])!=clip['processed_frames'] or info['width']!=1920 or info['height']!=416
                    or abs(numerator/denominator-clip['comparison_fps'])>1e-6 or info['codec_name']!='h264'):
                raise RuntimeError('Encoded output verification failed: '+str(target))
            verified.append(dict(path=str(target),**info))
            print('Verified '+str(target.relative_to(args.output)),flush=True)
    # A five-second excerpt from every tested clip; a full video remains available for each.
    command=['ffmpeg','-hide_banner','-loglevel','error','-y']
    for clip in summary['clips']:command+=['-i',str(args.output/clip['clip']['clip']/'comparison_preview.mp4')]
    graph=[]
    for i in range(len(summary['clips'])):
        graph.append(f'[{i}:v]trim=start=5:duration=5,setpts=PTS-STARTPTS[v{i}]')
    graph.append(''.join(f'[v{i}]' for i in range(len(summary['clips'])))+f'concat=n={len(summary["clips"])}:v=1:a=0[out]')
    montage=args.output/'all_12_cuda_comparison.mp4'
    command+=['-filter_complex',';'.join(graph),'-map','[out]','-an','-c:v','libx264','-preset','veryfast',
              '-crf','20','-threads','2','-pix_fmt','yuv420p','-movflags','+faststart',str(montage)]
    subprocess.run(command,check=True)
    info=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','format=duration','-of','json',str(montage)]))
    if abs(float(info['format']['duration'])-5*len(summary['clips']))>.1:raise RuntimeError('Montage duration mismatch')
    save(args.output/'encoded_verification.json',dict(videos=verified,montage=str(montage),montage_seconds=float(info['format']['duration'])))

def report(args):
    summary=json.loads((args.output/'summary.json').read_text(encoding='utf-8'))
    inputs=json.loads((args.output/'inputs.json').read_text(encoding='utf-8'))
    native_check=json.loads((args.output/'native_check/comparison.json').read_text(encoding='utf-8'))
    encoded=json.loads((args.output/'encoded_verification.json').read_text(encoding='utf-8'))
    if summary['clips_finished']!=12 or native_check['clips']!=12 or native_check['samples']!=48:
        raise RuntimeError('Required tests incomplete')
    clips=summary['clips'];frames=sum(c['processed_frames'] for c in clips)
    aggregates={}
    for label in ('fp32','qat_fake'):
        valid=sum(c['models'][label]['raw_valid_frames'] for c in clips)
        displayed=sum(c['models'][label]['displayed_valid_frames'] for c in clips)
        aggregates[label]=dict(raw_valid_frames=valid,raw_valid_fraction=valid/frames,
            display_valid_frames=displayed,display_valid_fraction=displayed/frames,
            raw_valid_to_invalid_events=sum(c['models'][label]['raw_valid_to_invalid_events'] for c in clips))
    for clip in inputs['clips']:
        source=Path(clip['video'])
        source_metadata=json.loads(source.with_name('source.json').read_text(encoding='utf-8'))
        if source_metadata.get('sha256')!=clip['sha256'] or hashlib.sha256(source.read_bytes()).hexdigest()!=clip['sha256']:
            raise RuntimeError('Archive video hash/source metadata mismatch')
    native_folder=args.output/'7679660a-42a3-4271-9d61-4f6153a961a4'
    continuous=json.loads((native_folder/'native_report.json').read_text(encoding='utf-8'))
    result=dict(phase='complete',videos=12,source_frames=inputs['total_source_frames'],tested_frames_per_model=frames,
        stride=2,backend=summary['device'],source_archive=inputs['archive'],aggregate_geometry=aggregates,
        geometry_scope='Valid geometry under existing gates, not labelled accuracy; CUDA comparison uses fake quant',
        native_int8_sparse_check={k:v for k,v in native_check.items() if k!='records'},
        native_int8_continuous_check={k:v for k,v in continuous.items() if k!='records'},
        board_model_unchanged=True)
    save(args.output/'results.json',result)
    lines=['# 用户车道素材 PC 测试结果','',
        '已测试提供的 lane_middle_new_20261017.zip：12 段、7219 个源帧。每隔一帧推理，保持原视频时间线（15 FPS），每个模型共处理 '+str(frames)+' 帧。素材与训练集及验证集均无视频 ID 重叠。视频 SHA256 已与包内 source.json 核对。','',
        '## 结果怎么理解','',
        '完整素材对照是 RTX 4070 上的 FP32 与续训 QAT fake quant。为核对真实量化行为，另对全部 12 段各抽 4 帧（48 帧）运行导出的 QNNPACK INT8，并对 7679660a 的整段视频连续运行真实 INT8。GPU fake quant 的耗时不代表 NPU 或真实 INT8 的耗时。','',
        '只显示本车道绿色区域和箭头，使用相同几何门限与跟踪器；丢失可保持 3 个处理帧，统计分别记录原始有效与保持后的显示。该对照只测试几何，不测试虚实线分类，也不包含红绿灯和测试二按键逻辑。未知线型未画白线。','',
        '几何有效帧占比不是正确率；没有人工标注，不能据此报告检测准确率或 CULane F1。相邻帧边界运动包含实际道路和相机运动，也不是纯抖动误差。','',
        '| 片段 | 测试帧 | FP32 原始几何有效 | QAT 模拟原始几何有效 | FP32 / QAT 消失事件 |',
        '| --- | ---: | ---: | ---: | ---: |']
    table=[]
    cards=[]
    for c in clips:
        key=c['clip']['clip'];a=c['models']['fp32'];b=c['models']['qat_fake']
        lines.append(f'| {key[:8]} | {c["processed_frames"]} | {a["raw_valid_fraction"]:.1%} | {b["raw_valid_fraction"]:.1%} | {a["raw_valid_to_invalid_events"]} / {b["raw_valid_to_invalid_events"]} |')
        table.append(f'<tr><td>{key[:8]}</td><td>{c["processed_frames"]}</td><td>{a["raw_valid_fraction"]:.1%}</td><td>{b["raw_valid_fraction"]:.1%}</td><td>{a["raw_valid_to_invalid_events"]} / {b["raw_valid_to_invalid_events"]}</td></tr>')
        cards.append(f'<details><summary>{key} · FP32 {a["raw_valid_fraction"]:.1%} / QAT 模拟 {b["raw_valid_fraction"]:.1%}</summary><video controls preload="none" poster="{key}/comparison.jpg" src="{key}/comparison_preview.mp4"></video><p><a href="{key}/report.json">逐帧数据</a> · <a href="source/lane_middle_new_20261017/{key}/front.mp4">原始视频</a></p></details>')
    lines+=['',f'汇总：FP32 原始几何有效 {aggregates["fp32"]["raw_valid_fraction"]:.2%}，QAT 模拟 {aggregates["qat_fake"]["raw_valid_fraction"]:.2%}。两者均有失败场景，不能认为续训已解决消失或闪烁。','',
        '## 真实 INT8 核验','',
        f'- 48 帧：原生 INT8 vs fake quant KL {native_check["native_vs_fake_mean_kl"]:.6f}；原生 INT8 vs FP32 KL {native_check["native_vs_teacher_mean_kl"]:.6f}。',
        f'- 48 帧共有本车道点：原生 INT8 vs fake quant 平均横向差 {native_check["native_vs_fake_ego_mean_x_error_px640"]:.3f} px（640 像素视图）。',
        f'- 连续整段：{continuous["frames"]} 帧，FP32 / 原生 INT8 原始几何有效 {continuous["models"]["fp32"]["raw_valid_fraction"]:.2%} / {continuous["models"]["native_int8"]["raw_valid_fraction"]:.2%}。',
        f'- 原生 INT8 PC CPU 模型调用均值 {continuous["cpu_inference_mean_ms"]:.1f} ms；视频为离线处理后按 15 FPS 回放，不能当作实时推理帧率。','',
        '## 文件','',
        '- index.html：全部视频入口和结果表。',
        '- all_12_cuda_comparison.mp4：12 段各 5 秒的 FP32 / QAT 模拟对照，共 60 秒。',
        '- 7679660a-42a3-4271-9d61-4f6153a961a4/native_comparison_preview.mp4：原视频 / 同帧 FP32 / 真实 INT8 的完整连续对照。',
        '- 每段目录 comparison_preview.mp4 为完整 CUDA 对照，report.json 和两份 slots.npz 保留逐帧结果。',
        '- native_check/comparison.json：48 帧真实 INT8 输出核验。','',
        '测试未替换板端模型，板端原界面继续运行。']
    (args.output/'RESULTS.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    native_rel=native_folder.name+'/native_comparison_preview.mp4'
    document='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>UFLDv2 QAT PC 测试</title><style>
body{max-width:1400px;margin:30px auto;padding:0 20px;background:#101820;color:#e0ece9;font:16px/1.7 "Microsoft YaHei",sans-serif}video{width:100%;background:#000}table{width:100%;border-collapse:collapse}td,th{padding:8px;border-bottom:1px solid #3b4c54;text-align:left}a{color:#80d9b5}details{margin:18px 0;padding:12px;background:#192730}summary{cursor:pointer}h1,h2{color:#9ee8bb}.note{background:#23353b;padding:14px}</style><h1>UFLDv2 1600×320 · PC 车道测试</h1>'''
    document+=f'<p>用户压缩包全部 12 段，{frames} 个处理帧 / 模型；15 FPS 时间线。PC：RTX 4070 Laptop，板端模型未替换。</p>'
    document+='<p class="note">下面第一个视频使用导出的真实 INT8 模型在 PC CPU 上离线处理。后面的 12 段对照使用 CUDA FP32 与 QAT fake quant；两者已用 48 帧同输入原生 INT8 输出交叉核对。没有人工标签，表中几何有效占比不是准确率。只测试本车道几何，未进行虚实线分类。</p>'
    document+=f'<h2>真实 INT8 连续对照</h2><video controls preload="metadata" poster="{native_folder.name}/native_comparison.jpg" src="{native_rel}"></video><p>左：原视频；中：同帧 FP32；右：原生 INT8。离线回放，视频帧率不等于推理帧率。</p>'
    document+='<h2>全部 12 段的 CUDA 对照合集</h2><video controls preload="metadata" src="all_12_cuda_comparison.mp4"></video><p>每段节选 5 秒；右侧为 QAT 量化模拟。</p>'
    document+='<h2>逐段结果</h2><table><thead><tr><th>片段</th><th>帧数</th><th>FP32 几何有效</th><th>QAT 模拟几何有效</th><th>消失事件 FP32 / QAT</th></tr></thead><tbody>'+''.join(table)+'</tbody></table>'
    document+=''.join(cards)+'<p><a href="RESULTS.md">完整报告</a> · <a href="results.json">汇总数据</a> · <a href="native_check/comparison.json">真实 INT8 核验</a></p></html>'
    (args.output/'index.html').write_text(document,encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('native_int8_continuous_check','native_int8_sparse_check')},ensure_ascii=False,indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=['encode','report'])
    p.add_argument('--output',type=Path,default=ROOT/'training/ufldv2_qat_20261020/user_zip_test')
    args=p.parse_args();encode(args) if args.action=='encode' else report(args)
