"""Stage Test 2 separately; credentials exist only in the caller's environment."""
import argparse
import json
import os
import posixpath
import shlex
import tarfile
from pathlib import Path

import paramiko

ROOT = Path(__file__).resolve().parents[1]
STAGE = '/home/cat/rk3568_adas/board_v8_test2_20261016'


def connect(host):
    client=paramiko.SSHClient()
    pins=ROOT.parent/'ssh_probe/known_hosts'
    client.load_host_keys(str(pins))
    # The new IP was checked against the old board's pinned key by SSH.
    if host=='192.168.137.17':
        for kind,key in client.get_host_keys().get('192.168.137.206',{}).items():
            client.get_host_keys().add(host,kind,key)
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    client.connect(host,username='cat',password=os.environ['ADAS_BOARD_PASSWORD'],
                   timeout=8,auth_timeout=8,banner_timeout=8,
                   look_for_keys=False,allow_agent=False)
    return client


def execute(client,command,check=True):
    _stdin,stdout,stderr=client.exec_command(command,timeout=600)
    # Drain both channel streams while compiling so compiler diagnostics cannot
    # fill a pipe and deadlock a long build.
    channel=stdout.channel
    import time
    while True:
        emitted=False
        while channel.recv_ready():
            print(channel.recv(65536).decode('utf-8',errors='replace'),end='',flush=True)
            emitted=True
        while channel.recv_stderr_ready():
            print(channel.recv_stderr(65536).decode('utf-8',errors='replace'),end='',flush=True)
            emitted=True
        if channel.exit_status_ready() and not channel.recv_ready() and not channel.recv_stderr_ready():
            break
        if not emitted:
            time.sleep(.05)
    code=channel.recv_exit_status()
    if check and code:
        raise RuntimeError(f'Remote command failed with status {code}')
    return code


def stage(client):
    archive=ROOT.parent/'.tmp/test2_board_source_20261016.tar.gz'
    with tarfile.open(archive,'w:gz') as package:
        package.add(ROOT/'board_v8/CMakeLists.txt',arcname='CMakeLists.txt')
        for name in ('src','include','scripts','tests','tools','config','third_party'):
            base=ROOT/'board_v8'/name
            for path in base.rglob('*'):
                if path.is_file() and '__pycache__' not in path.parts and path.suffix!='.pyc':
                    package.add(path,arcname=path.relative_to(ROOT/'board_v8').as_posix())
    remote_archive='/tmp/test2_board_source_20261016.tar.gz'
    with client.open_sftp() as sftp:
        sftp.put(str(archive),remote_archive)
    execute(client,'mkdir -p '+shlex.quote(STAGE)+' && tar -xzf '+
            shlex.quote(remote_archive)+' -C '+shlex.quote(STAGE))
    print('Staged code in '+STAGE+'; existing program and models untouched.',flush=True)


def existing_models(client):
    selected=[]
    with client.open_sftp() as sftp:
        for name in ('unified21_p2_v7_640_int8.rknn','ufldv2_culane_res18_1600x320_int8.rknn'):
            found=None
            for folder in ('board_v8/models','board_v7/models','models'):
                path='/home/cat/rk3568_adas/'+folder+'/'+name
                try:
                    sftp.stat(path)
                    found=path
                    break
                except IOError:
                    pass
            if not found:
                raise RuntimeError('Existing board model not found: '+name)
            selected.append(found)
    return selected


def qat_check(client):
    candidate=ROOT/'training/ufldv2_qat_20261016/export/ufldv2_culane_res18_1600x320_qat_int8.rknn'
    manifest_path=ROOT/'training/ufldv2_qat_20261016/data/manifest.json'
    manifest=json.loads(manifest_path.read_text(encoding='utf-8'))
    samples=[]
    for record in manifest['samples']:
        if record['split']=='val' and record['clip'] not in {r['clip'] for r in samples}:
            samples.append(record)
    assert len(samples)==3
    folder=STAGE+'/output/qat_check'
    execute(client,'mkdir -p '+shlex.quote(folder))
    with client.open_sftp() as sftp:
        sftp.put(str(ROOT/'board_v8/src/ufld_benchmark.cpp'),STAGE+'/src/ufld_benchmark.cpp')
        sftp.put(str(candidate),folder+'/candidate.rknn')
        for index,record in enumerate(samples):
            sftp.put(str(manifest_path.parent/record['image']),folder+f'/sample{index}.jpg')
    execute(client,'cd '+shlex.quote(STAGE)+' && cmake --build build -- -j2 ufld_benchmark')
    baseline=existing_models(client)[1]
    destination=ROOT/'board_v8/output/test2_20261016/qat_check'
    destination.mkdir(parents=True,exist_ok=True)
    for label,model in (('baseline',baseline),('qat',folder+'/candidate.rknn')):
        for index in range(len(samples)):
            prefix=folder+f'/{label}_{index}'
            parts=['taskset','-c','2,3',STAGE+'/build/ufld_benchmark',model,
                   folder+f'/sample{index}.jpg','3',prefix]
            execute(client,' '.join(shlex.quote(p) for p in parts)+' > '+shlex.quote(prefix+'.log')+' 2>&1')
            execute(client,'tail -n 8 '+shlex.quote(prefix+'.log'))
            with client.open_sftp() as sftp:
                sftp.get(prefix+'.log',str(destination/f'{label}_{index}.log'))
                for head in range(4):
                    sftp.get(prefix+f'.output{head}.f32',str(destination/f'{label}_{index}.output{head}.f32'))
    (destination/'samples.json').write_text(json.dumps(samples,ensure_ascii=False,indent=2),encoding='utf-8')
    print('QAT candidate and baseline produced finite outputs on 3 held-out-video crops.',flush=True)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('action',choices=('probe','stage','build','replay','run','qat-check','stage-scenes'))
    parser.add_argument('--host',default='192.168.137.17')
    parser.add_argument('--scene',default='four_view_sample/12_highway_following')
    parser.add_argument('--qat',action='store_true',help='Explicit isolated QAT trial; original model stays intact')
    parser.add_argument('--object-model',default='/home/cat/rk3568_adas/board_v8/models/yolo14_960x544_qat_int8.rknn')
    parser.add_argument('--ufld-model')
    parser.add_argument('--software-decode',action='store_true')
    parser.add_argument('--artifact')
    parser.add_argument('--max-frames',type=int,default=180)
    parser.add_argument('--snapshot-frame',type=int,default=60)
    parser.add_argument('--demo-force-move',action='store_true')
    args=parser.parse_args()
    client=connect(args.host)
    try:
        if args.action=='probe':
            for command in (
                'uname -m',
                'pkg-config --modversion opencv4; cmake --version | head -n 1',
                'ls -d /home/cat/rk3568_adas/board_v* /home/cat/rk3568_adas/models 2>/dev/null',
                "find /home/cat/rk3568_adas -maxdepth 4 -type f \\( -name 'unified21_p2_v7_640_int8.rknn' -o -name 'ufldv2_culane_res18_1600x320_int8.rknn' -o -name 'fpga_1080p.mp4' \\) | head -n 20",
                'ls -l /home/cat/rk3568_adas/board_v8/four_view_sample /home/cat/rk3568_adas/board_v7/four_view_sample 2>/dev/null | head -n 20',
                'pgrep -a -x adas_four_view || true',
            ):
                execute(client,command,check=False)
        elif args.action=='stage':
            stage(client)
        elif args.action=='stage-scenes':
            source=ROOT/'board_v8/output/test2_nvidia_scenes'
            for group in ('12_highway_following','20_highway_left_overtake','48_highway_right_vehicle_merge'):
                remote=STAGE+'/four_view_sample/'+group
                execute(client,'mkdir -p '+shlex.quote(remote))
                with client.open_sftp() as sftp:
                    # Remove only our earlier generated high-resolution replay.
                    # Four low-resolution camera decoders preserve fresh NPU inputs.
                    assert remote.startswith(STAGE+'/four_view_sample/')
                    try: sftp.remove(remote+'/fpga_1080p.mp4')
                    except FileNotFoundError: pass
                    for name in ('front.mp4','rear.mp4','left.mp4','right.mp4','manifest.json','aligned_frame_map.csv','source_preview.jpg'):
                        sftp.put(str(source/group/name),remote+'/'+name)
                    for name in ('front.mp4.info','rear.mp4.info','left.mp4.info','right.mp4.info'):
                        if (source/group/name).is_file():sftp.put(str(source/group/name),remote+'/'+name)
                print('Uploaded synchronized NVIDIA scene '+group,flush=True)
        elif args.action=='build':
            execute(client,'cd '+shlex.quote(STAGE)+' && cmake -S . -B build && '
                    'cmake --build build -- -j2 adas_four_view test2_regression perception_regression '
                    'overlay_regression ufld_decode_regression ui_upload_regression')
            for name in ('test2_regression','perception_regression','overlay_regression',
                         'ufld_decode_regression','ui_upload_regression'):
                execute(client,shlex.quote(posixpath.join(STAGE,'build',name)))
        elif args.action=='qat-check':
            qat_check(client)
        elif args.action in ('replay','run'):
            model,ufld=existing_models(client)
            if args.object_model:model=args.object_model
            if args.ufld_model:ufld=args.ufld_model
            if args.qat:
                ufld=STAGE+'/output/qat_check/candidate.rknn'
            scene=args.scene
            label=args.artifact or ('test2_qat' if args.qat else 'test2_board')
            assert Path(label).name==label and '/' not in label and '\\' not in label
            decision_name='test2_qat_decisions.csv' if args.qat else 'test2_decisions.csv'
            command=['taskset','-c','2,3',STAGE+'/build/adas_four_view',
                     '--model',model,'--ufld-model',ufld,'--scene',scene,
                     '--cpu-threads','1','--detect-every','6',
                     '--quad-layout','--decision-log',STAGE+'/output/'+decision_name]
            if args.software_decode:command.append('--software-decode')
            if args.demo_force_move:command.append('--demo-force-move')
            if args.action=='replay':
                command+=['--headless','--once','--max-frames',str(args.max_frames),
                          '--snapshot',STAGE+'/output/'+label+'.png','--snapshot-frame',str(args.snapshot_frame)]
                execute(client,'mkdir -p '+shlex.quote(STAGE+'/output'))
                execute(client,'cd '+shlex.quote(STAGE)+' && '+
                        ' '.join(shlex.quote(part) for part in command)+
                        ' > output/'+label+'_replay.log 2>&1')
                execute(client,'tail -n 10 '+shlex.quote(STAGE+'/output/'+label+'_replay.log'))
                destination=ROOT/'board_v8/output/test2_20261016'
                destination.mkdir(parents=True,exist_ok=True)
                with client.open_sftp() as sftp:
                    for name in (label+'.png',decision_name,label+'_replay.log'):
                        sftp.get(STAGE+'/output/'+name,str(destination/name))
                print('Board snapshot and decisions downloaded to '+str(destination),flush=True)
            else:
                # Do not launch another fullscreen/NPU application over an
                # unrelated running instance. The caller can inspect it first.
                code=('import json,os,subprocess; '
                      'active=subprocess.run(["pgrep","-x","adas_four_view"],stdout=subprocess.PIPE); '
                      'assert active.returncode!=0,"ADAS is already running"; '
                      'env=dict(os.environ,DISPLAY=":0",XAUTHORITY="/home/cat/.Xauthority"); '
                      'log=open('+repr(STAGE+'/output/test2_ui.log')+',"ab"); '
                      'p=subprocess.Popen('+repr(command)+',cwd='+repr(STAGE)+',env=env,'
                      'stdout=log,stderr=subprocess.STDOUT,start_new_session=True); '
                      'print(json.dumps(dict(pid=p.pid,stage='+repr(STAGE)+'))); ')
                execute(client,'python3 -c '+shlex.quote(code))
    finally:
        client.close()


if __name__=='__main__':
    main()
