"""Restore the original video ZIP from GitHub parts and verify every byte."""
from pathlib import Path
import hashlib
import shutil
import zipfile

ROOT=Path(__file__).resolve().parent
EXPECTED='8421cc96ae832c6e41e770ddfbab276d8547b1159acf4cc92eeebc367692a15f'
EXPECTED_SIZE=44869261

def main():
    parts=sorted((ROOT/'parts').glob('video-compatible.zip.[0-9][0-9][0-9]'))
    names=[f'video-compatible.zip.{i:03d}' for i in range(1,31)]
    if [p.name for p in parts]!=names:
        raise SystemExit('Missing parts. Download or clone the complete video-download branch first.')
    output=ROOT/'video-compatible.zip'
    temp=ROOT/'video-compatible.zip.restoring'
    with temp.open('wb') as dest:
        for part in parts:
            with part.open('rb') as src:shutil.copyfileobj(src,dest)
    h=hashlib.sha256()
    with temp.open('rb') as src:
        for chunk in iter(lambda:src.read(1024*1024),b''):h.update(chunk)
    if temp.stat().st_size!=EXPECTED_SIZE or h.hexdigest()!=EXPECTED:
        temp.unlink()
        raise SystemExit('Integrity check failed. Please download the complete branch again.')
    temp.replace(output)
    target=ROOT/'video';target.mkdir(exist_ok=True)
    with zipfile.ZipFile(output) as archive:
        for name in ['silk-road-compatible.mp4','play-test.mp4','readme.txt']:
            info=archive.getinfo(name)
            with archive.open(info) as src,(target/name).open('wb') as dest:shutil.copyfileobj(src,dest)
    print('SHA-256 verified. Full video restored:')
    print(target/'silk-road-compatible.mp4')
    print('Duration: 8 minutes 12 seconds.')

if __name__=='__main__':main()
