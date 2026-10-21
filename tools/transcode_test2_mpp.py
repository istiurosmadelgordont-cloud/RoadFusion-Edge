"""Make the generated aligned NVIDIA clips compatible with the board MPP path."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path


def transcode(folder):
    manifest_path=folder/'manifest.json'
    manifest=json.loads(manifest_path.read_text(encoding='utf-8'))
    assert manifest['source_type'].startswith('real synchronized NVIDIA')
    for view in ('front','rear','left','right'):
        path=folder/(view+'.mp4');temporary=folder/(view+'.mpp.mp4')
        subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-threads','2',
            '-i',str(path),'-map','0:v:0','-an','-c:v','libx264','-preset','fast','-crf','18',
            '-pix_fmt','yuv420p','-g','30','-bf','0','-threads','2','-movflags','+faststart',
            str(temporary)],check=True)
        stream=json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0',
            '-show_entries','stream=codec_name,width,height,nb_frames,r_frame_rate','-of','json',str(temporary)]))['streams'][0]
        assert stream['codec_name']=='h264' and int(stream['nb_frames'])==manifest['output_frames']
        reference=folder/(view+'.mp4v_reference.mp4')
        if not reference.exists():path.rename(reference)
        temporary.replace(path)
        (folder/(view+'.mp4.info')).write_text(f"{stream['width']} {stream['height']} 30 {stream['nb_frames']}\n",encoding='ascii')
        manifest['camera_sha256'][path.name]=hashlib.sha256(path.read_bytes()).hexdigest()
        print(f'MPP H264 ready: {folder.name}/{view}',flush=True)
    manifest.update(codec='h264',transcode_crf=18,decoder='RK3568 MPP NV12',timestamp_alignment_unchanged=True)
    manifest_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folders',nargs='+',type=Path)
    for folder in p.parse_args().folders:transcode(folder)
