#!/usr/bin/env python3
"""对真实联通程序包装系统调用/RGA，检查顺序、步长和失败清理；不访问硬件。"""
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
rga = r'''
#pragma once
enum IM_STATUS {IM_STATUS_FAILED=-1,IM_STATUS_SUCCESS=1};
enum {RK_FORMAT_BGR_565=36,RK_FORMAT_BGR_888=7};
struct rga_buffer_t {int fd,width,height,wstride,hstride,format;};
rga_buffer_t wrapbuffer_fd_t(int,int,int,int,int,int);
IM_STATUS imcvtcolor_t(rga_buffer_t,rga_buffer_t,int,int,int,int);
const char* imStrError(IM_STATUS);
'''
drm = r'''
#pragma once
#include <stdint.h>
#include <sys/ioctl.h>
#define DRM_CLOEXEC 1
#define DRM_RDWR 2
struct drm_prime_handle {uint32_t handle,flags;int32_t fd;};
struct drm_mode_create_dumb {uint32_t height,width,bpp,flags,handle,pitch;uint64_t size;};
struct drm_mode_destroy_dumb {uint32_t handle;};
#define DRM_IOCTL_PRIME_HANDLE_TO_FD _IOWR('d',0x2d,struct drm_prime_handle)
#define DRM_IOCTL_MODE_CREATE_DUMB _IOWR('d',0xb2,struct drm_mode_create_dumb)
#define DRM_IOCTL_MODE_DESTROY_DUMB _IOWR('d',0xb4,struct drm_mode_destroy_dumb)
'''
unit = r'''
#define PCIE_RGA_PROBE_NO_MAIN
#include "src/pcie_rga_probe.cpp"
#include <cassert>
#include <cstdarg>
#include <algorithm>
namespace mock {
enum Fault {NONE,EXPORT,CREATE,PRIME,START,GET,GET_INTERRUPT,RGA,PUT,VERIFY,STOP};
Fault fault=NONE;
unsigned exports=0,starts=0,stops=0,get_count=0,put_count=0,conversions=0,creates=0,closes=0,maps=0,unmaps=0;
unsigned cpu_reads=0,cpu_writes=0,sync_ends=0;
bool running=false,in_rga=false,finished=false,eintr=false,quit_after_first=false;
bool opened[256]={};
std::array<std::vector<uint8_t>,4> raw;
std::vector<uint8_t> target;
unsigned pitch=0;
void reset(Fault f=NONE){
 fault=f;exports=starts=stops=get_count=put_count=conversions=creates=closes=maps=unmaps=0;
 cpu_reads=cpu_writes=sync_ends=0;running=in_rga=finished=eintr=quit_after_first=false;
 std::fill(opened,opened+256,false);pcie_rga::interrupted=0;
 for(unsigned slot=0;slot<4;++slot){
  raw[slot].resize(4149248);
  for(unsigned y=0;y<1080;++y)for(unsigned x=0;x<1920;++x){
   unsigned v=((x%32)<<11)|((y%64)<<5)|((x+y+slot)%32);
   raw[slot][(y*1920+x)*2]=v&255;raw[slot][(y*1920+x)*2+1]=v>>8;
  }
 }
 target.clear();
}
int fail(int error){errno=error;return -1;}
void assert_closed(){for(bool value:opened)assert(!value);assert(maps==unmaps && cpu_reads+cpu_writes==sync_ends);}
}
extern "C" int __wrap_open(const char *path,int flags,...){
 const int fd=std::string(path)=="mock-pcie"?100:101;
 assert(!mock::opened[fd] && (flags&O_CLOEXEC));mock::opened[fd]=true;return fd;
}
extern "C" int __wrap_close(int fd){
 assert(mock::opened[fd]);
 if(fd==100){for(unsigned i=110;i<114;++i)assert(!mock::opened[i]);}
 if(fd>=110&&fd<114)assert(!mock::running || mock::fault==mock::STOP);
 mock::opened[fd]=false;++mock::closes;return 0;
}
extern "C" int __wrap_ioctl(int fd,unsigned long command,...){
 va_list args;va_start(args,command);void *value=va_arg(args,void*);va_end(args);
 assert(mock::opened[fd]);
 using namespace mock;
 if(command==FPGA_IOC_GET_INFO){
  auto &i=*static_cast<fpga_info*>(value);i={};i.abi_version=2;i.width=1920;i.height=1080;i.stride=3840;
  i.image_bytes=4147200;i.slot_count=4;i.pixel_format=FPGA_FORMAT_RGB565;i.map_bytes=4149248;return 0;
 }
 if(command==FPGA_IOC_GET_CAPS){auto &c=*static_cast<fpga_caps*>(value);c={};c.version=1;c.flags=FPGA_CAP_DMABUF;return 0;}
 if(command==FPGA_IOC_EXPORT_DMABUF){
  auto &r=*static_cast<fpga_dmabuf_export*>(value);assert(!running && r.slot==exports && !r.flags && !r.reserved);
  if(fault==EXPORT&&r.slot==1)return fail(ENOMEM);
  r.fd=110+r.slot;opened[r.fd]=true;++exports;return 0;
 }
 if(command==DRM_IOCTL_MODE_CREATE_DUMB){
  assert(!starts);if(fault==CREATE)return fail(EINVAL);
  auto &d=*static_cast<drm_mode_create_dumb*>(value);assert(d.bpp==24 && d.width==1920 && d.height==1080);
  d.handle=7;d.pitch=pitch=d.width*3+48;d.size=((uint64_t(d.pitch)*d.height+4095)/4096)*4096;
  target.resize(d.size);++creates;return 0;
 }
 if(command==DRM_IOCTL_PRIME_HANDLE_TO_FD){
  if(fault==PRIME)return fail(ENOMEM);
  auto &p=*static_cast<drm_prime_handle*>(value);assert(p.handle==7);p.fd=120;opened[120]=true;return 0;
 }
 if(command==DRM_IOCTL_MODE_DESTROY_DUMB)return 0;
 if(command==FPGA_IOC_START){++starts;assert(exports==4&&creates==1);if(fault==START)return fail(EIO);running=true;return 0;}
 if(command==FPGA_IOC_STOP){++stops;assert(!in_rga);if(fault==STOP)return fail(ETIMEDOUT);running=false;return 0;}
 if(command==FPGA_IOC_GET_FRAME){
  assert(running && !in_rga && get_count==put_count);
  if(eintr){eintr=false;return fail(EINTR);}if(fault==GET)return fail(ETIMEDOUT);
  if(fault==GET_INTERRUPT){pcie_rga::interrupted=1;return fail(EINTR);}
  auto &f=*static_cast<fpga_frame*>(value);f={};f.session=3;f.slot=get_count%4;f.sequence=get_count+1;
  f.token=(f.sequence<<2)|f.slot;f.bytes_used=4147200;++get_count;finished=false;return 0;
 }
 if(command==FPGA_IOC_PUT_FRAME){
  assert(finished&&!in_rga && conversions==get_count);
  if(fault==PUT)return fail(ESTALE);
  ++put_count;if(quit_after_first)pcie_rga::interrupted=1;return 0;
 }
 if(command==DMA_BUF_IOCTL_SYNC){
  auto &s=*static_cast<dma_buf_sync*>(value);
  if(s.flags&DMA_BUF_SYNC_END){++sync_ends;return 0;}
  if(s.flags&DMA_BUF_SYNC_WRITE){assert(fd==120&&!starts);++cpu_writes;}
  else{assert(finished);++cpu_reads;}
  return 0;
 }
 assert(false);return -1;
}
extern "C" void *__wrap_mmap(void*,size_t bytes,int protection,int flags,int fd,off_t offset){
 using namespace mock;assert(opened[fd] && flags==MAP_SHARED && offset==0);++maps;
 if(fd==120){assert(bytes==target.size());return target.data();}
 assert(fd>=110&&fd<114 && bytes==4149248 && protection==PROT_READ && finished);return raw[fd-110].data();
}
extern "C" int __wrap_munmap(void*,size_t){++mock::unmaps;return 0;}
rga_buffer_t wrapbuffer_fd_t(int fd,int w,int h,int ws,int hs,int format){return {fd,w,h,ws,hs,format};}
const char* imStrError(IM_STATUS){return "mock RGA failure";}
IM_STATUS imcvtcolor_t(rga_buffer_t src,rga_buffer_t dst,int sfmt,int dfmt,int mode,int sync){
 using namespace mock;
 assert(running && sync==1 && mode==0 && get_count==put_count+1 && src.fd==110+int((get_count-1)%4));
 assert(src.width==1920&&src.height==1080&&src.wstride==1920&&src.hstride==1080);
 assert(dst.width==1920&&dst.height==1080&&dst.wstride*3==int(pitch)&&dst.hstride==1080);
 assert(sfmt==RK_FORMAT_RGB_565&&dfmt==RK_FORMAT_BGR_888);
 in_rga=true;
 if(fault==RGA){in_rga=false;return IM_STATUS_FAILED;}
 const auto &pixels=raw[src.fd-110];
 for(unsigned y=0;y<1080;++y)for(unsigned x=0;x<1920;++x){
  const unsigned v=pixels[(y*1920+x)*2]|(unsigned(pixels[(y*1920+x)*2+1])<<8);
  uint8_t *p=target.data()+y*pitch+x*3;p[0]=(v&31)<<3;p[1]=((v>>5)&63)<<2;p[2]=((v>>11)&31)<<3;
 }
 if(fault==VERIFY)target[0]^=255;
 ++conversions;finished=true;in_rga=false;return IM_STATUS_SUCCESS;
}
int main(int argc,char **argv){
 assert(argc==2);pcie_rga::Options o;o.device="mock-pcie";o.drm="mock-drm";o.frames=3;
 mock::reset();mock::eintr=true;
 assert(pcie_rga::run(o)==0);assert(mock::conversions==3&&mock::put_count==3&&mock::stops==1);
 assert(!mock::maps && mock::creates==1);mock::assert_closed();
 mock::reset();o.verify=true;o.output=argv[1];
 assert(pcie_rga::run(o)==0 && mock::maps==3 && mock::cpu_reads==2 && mock::cpu_writes==1);
 mock::assert_closed();o.output.clear();
 for(auto f:{mock::EXPORT,mock::CREATE,mock::PRIME,mock::START,mock::GET,mock::RGA,mock::PUT,mock::VERIFY,mock::STOP}){
  mock::reset(f);bool threw=false;
  try{pcie_rga::run(o);}catch(const std::exception&){threw=true;}
  assert(threw);mock::assert_closed();
  if(f==mock::EXPORT||f==mock::CREATE||f==mock::PRIME)assert(!mock::starts&&!mock::stops);
  else assert(mock::stops>=1);
  if(f==mock::RGA||f==mock::PUT||f==mock::VERIFY)assert(!mock::put_count);
 }
 mock::reset();o.verify=false;mock::quit_after_first=true;
 assert(pcie_rga::run(o)==130 && mock::put_count==1&&mock::stops==1);mock::assert_closed();
 mock::reset(mock::GET_INTERRUPT);
 assert(pcie_rga::run(o)==130 && !mock::get_count && mock::stops==1);mock::assert_closed();
 std::cout<<"PCIE_RGA_PROBE_PASS: sync-before-PUT, pixel/stride verification, reused output, no CPU mapping on normal path, error/STOP/fd cleanup\n";
}
'''

with tempfile.TemporaryDirectory(prefix='board-v10-pcie-rga-') as folder:
    out = Path(folder)
    (out / 'rga').mkdir()
    (out / 'rga/im2d.h').write_text(rga)
    (out / 'rga/rga.h').write_text('#pragma once\n')
    (out / 'drm.h').write_text(drm)
    (out / 'drm_mode.h').write_text('#include "drm.h"\n')
    (out / 'probe_test.cpp').write_text(unit)
    binary = out / 'probe-test'
    subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++11', '-O1', '-UNDEBUG', '-Wall', '-Wextra',
                    '-I', str(out), '-I', str(root), '-I', str(root / 'include'),
                    str(out / 'probe_test.cpp'), '-o', str(binary),
                    '-Wl,--wrap=open,--wrap=close,--wrap=ioctl,--wrap=mmap,--wrap=munmap'], check=True)
    picture = out / 'first.ppm'
    print('SOFTWARE MOCK ONLY: the following FPS values do not measure hardware.', flush=True)
    subprocess.run([str(binary), str(picture)], check=True, timeout=30)
    content = picture.read_bytes()
    assert content.startswith(b'P6\n1920 1080\n255\n')
    assert len(content) == len(b'P6\n1920 1080\n255\n') + 1920 * 1080 * 3
