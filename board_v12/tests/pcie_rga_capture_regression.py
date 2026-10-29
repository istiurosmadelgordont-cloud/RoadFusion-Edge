#!/usr/bin/env python3
"""编译固定 RGA PCIe 采集回归；RGA/PCIe 边界用 mock，不需要设备或 RKNN。

优先 pkg-config opencv4/opencv；也可设置 OPENCV_CFLAGS / OPENCV_LIBS。
"""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
rga_header = r'''
#pragma once
enum IM_STATUS {IM_STATUS_FAILED=-1,IM_STATUS_SUCCESS=1};
enum {RK_FORMAT_BGR_565=36,RK_FORMAT_BGR_888=7};
struct rga_buffer_t {int fd=0,width=0,height=0,wstride=0,hstride=0,format=0;void* vir_addr=nullptr;};
extern "C" rga_buffer_t wrapbuffer_fd_t(int,int,int,int,int,int);
extern "C" rga_buffer_t wrapbuffer_virtualaddr_t(void*,int,int,int,int,int);
extern "C" IM_STATUS imcvtcolor_t(rga_buffer_t,rga_buffer_t,int,int,int,int);
extern "C" const char* imStrError_t(IM_STATUS);
#define imStrError imStrError_t
'''
def cv_flags(option):
    key = 'OPENCV_CFLAGS' if option == '--cflags' else 'OPENCV_LIBS'
    if key in os.environ:
        return shlex.split(os.environ[key])
    for package in ['opencv4', 'opencv']:
        found = subprocess.run(['pkg-config', option, package], capture_output=True, text=True)
        if found.returncode == 0:
            return shlex.split(found.stdout)
    raise RuntimeError('OpenCV development files needed; set ' + key)


with tempfile.TemporaryDirectory(prefix='board-v12-rga-capture-') as folder:
    out = Path(folder)
    (out / 'rga').mkdir()
    (out / 'rga/im2d.h').write_text(rga_header)
    (out / 'rga/rga.h').write_text('#pragma once\n')
    common = [os.environ.get('CXX', 'g++'), '-std=c++11', '-O1', '-UNDEBUG', '-Wall', '-Wextra',
              '-I', str(out), '-I', str(root / 'include')] + cv_flags('--cflags')
    link = ['-pthread', '-Wl,--wrap=open,--wrap=close,--wrap=ioctl,--wrap=read'] + cv_flags('--libs')
    name = 'pcie_rga_capture_regression'
    binary = out / name
    subprocess.run(common + [str(root / 'tests' / (name + '.cpp'))] +
                   ['-o', str(binary)] + link, check=True)
    print('SOFTWARE DEVICE/RGA MOCK ONLY: ' + name, flush=True)
    subprocess.run([str(binary)], check=True, timeout=40)
