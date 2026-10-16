#!/usr/bin/env python3
"""从当前驱动抽取实际函数，在 Linux 用户态替换内核依赖；不访问设备。"""
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'my_pciedriver.c').read_text(encoding='utf-8-sig')

def function(name):
    import re
    m = re.search(r'^static [^;{]*\b' + name + r'\([^;{]*\)\n\{', source, re.M)
    if not m:
        raise RuntimeError('missing function: ' + name)
    return source[m.start():source.index('\n}', m.end()) + 2]

prefix = r'''
#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "image_protocol.h"
typedef uint32_t u32;
typedef uint64_t u64;
typedef unsigned long ktime_t;
#define __user
#define ERESTARTSYS 512
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#define THIS_MODULE 0
#define FPGA_ABI_VERSION 2U
#define FPGA_FRAME_WIDTH 1920U
#define FPGA_FRAME_HEIGHT 1080U
#define FPGA_IMAGE_BYTES 4147200U
#define FPGA_MAP_BYTES 4149248UL
#define FPGA_REG_IMAGE_WORD0 0x200U
#define FPGA_REG_IMAGE_COMMIT 0x214U
#define FPGA_STOP_TIMEOUT_US 1000U
#define dev_err(...) ((void)0)
struct mutex {int held;};
struct pci_dev {int master;};
enum {BASIC_IO_UNSET,BASIC_IO_READ,BASIC_IO_MMAP};
enum {BASIC_IDLE,BASIC_RUNNING,BASIC_STOPPING,BASIC_QUARANTINED};
struct basic_pcie_dev {
 struct pci_dev *pdev; void *bar;
 struct mutex lock; bool present, opened, dma_guard; int state,io_mode;
 u32 cookie_counter,stop_cookie;
};
struct file {struct basic_pcie_dev *private_data;};
static unsigned writes, frees, quarantines, puts, unmaps, releases, disables;
static unsigned offsets[100];static u32 values[100];static int refs, ack;
static ktime_t now;
static bool force_copy_fault;
static void basic_write_reg32(struct basic_pcie_dev *d,unsigned off,u32 value){
 assert(d->lock.held);assert(writes<100);offsets[writes]=off;values[writes++]=value;
}
static int mutex_lock_interruptible(struct mutex *m){assert(!m->held);m->held=1;return 0;}
static void mutex_unlock(struct mutex *m){assert(m->held);m->held=0;}
static int copy_from_user(void *to,const void *from,size_t bytes){
 if(!from || force_copy_fault)return 1;memcpy(to,from,bytes);return 0;
}
static int copy_to_user(void *to,const void *from,size_t bytes){
 if(!to || force_copy_fault)return 1;memcpy(to,from,bytes);return 0;
}
static void pci_clear_master(struct pci_dev *p){p->master=0;}
static void module_put(int unused){(void)unused;--refs;}
static void basic_notify_locked(struct basic_pcie_dev *d){(void)d;}
static bool basic_stop_ack(struct basic_pcie_dev *d){(void)d;return ack;}
static void basic_submit_stop_locked(struct basic_pcie_dev *d){d->state=BASIC_STOPPING;}
static ktime_t ktime_get(void){return now;}
static ktime_t ktime_add_us(ktime_t t,unsigned us){return t+us;}
static int ktime_compare(ktime_t a,ktime_t b){return (a>b)-(a<b);}
static void usleep_range(unsigned a,unsigned b){(void)b;now+=a;}
static void basic_quarantine(struct basic_pcie_dev *d){++quarantines;d->state=BASIC_QUARANTINED;}
static void basic_free_dma(struct basic_pcie_dev *d){(void)d;++frees;}
static void pci_iounmap(struct pci_dev *p,void *bar){(void)p;(void)bar;++unmaps;}
static void pci_disable_device(struct pci_dev *p){(void)p;++disables;}
static void pci_release_region(struct pci_dev *p,int bar){(void)p;(void)bar;++releases;}
#define FPGA_BAR_INDEX 1
static int basic_get_frame(struct file *f,void *p){(void)f;(void)p;return -EPIPE;}
static int basic_start_locked(struct basic_pcie_dev *d){(void)d;return 0;}
static int basic_put_frame_locked(struct basic_pcie_dev *d,const struct fpga_frame *f){(void)d;(void)f;++puts;return 0;}
'''
test = r'''
int main(void){
 _Static_assert(sizeof(struct fpga_info)==72,"frame ABI info changed");
 _Static_assert(sizeof(struct fpga_frame)==32,"frame ABI lease changed");
 _Static_assert(sizeof(struct fpga_image_submit)==32,"submit ABI");
 _Static_assert(sizeof(struct fpga_caps)==16,"caps ABI");
 assert(FPGA_IOC_START==0x5001 && FPGA_IOC_STOP==0x5002);
 assert(FPGA_IOC_GET_INFO==0x80485003UL && FPGA_IOC_GET_FRAME==0x80205004UL);
 assert(FPGA_IOC_PUT_FRAME==0x40205005UL && FPGA_IOC_SUBMIT_IMAGE==0x40205012UL);
 struct pci_dev p={0};struct basic_pcie_dev d={.pdev=&p,.present=true,.state=BASIC_IDLE};
 struct file f={.private_data=&d};struct fpga_image_submit c={{0}, {0}};
 c.words[0]=~0U;c.words[1]=0;c.words[2]=0xffff;c.words[3]=~0U;c.words[4]=~0U;
 u32 clean[5];fpga_cfg_sanitize(c.words,clean);
 assert(basic_ioctl(&f,FPGA_IOC_SUBMIT_IMAGE,(unsigned long)&c)==0);
 assert(writes==6 && p.master==0 && !d.dma_guard);
 for(unsigned i=0;i<5;++i){assert(offsets[i]==0x200+4*i);assert(values[i]==clean[i]);}
 assert(offsets[5]==0x214 && values[5]==1);
 assert(clean[0]==0x19ff && (clean[1]&255)==1 && ((clean[2]>>9)&63)==32);
 assert((clean[2]&511)==256 && ((clean[3]>>16)&7)==4 && ((clean[4]>>16)&511)==256);
 d.state=BASIC_RUNNING;assert(basic_ioctl(&f,FPGA_IOC_SUBMIT_IMAGE,(unsigned long)&c)==0);
 assert(writes==12); // 没有等待硬件 ACK 的单次在途限制。
 c.reserved[2]=1;assert(basic_ioctl(&f,FPGA_IOC_SUBMIT_IMAGE,(unsigned long)&c)==-EINVAL);c.reserved[2]=0;
 d.state=BASIC_STOPPING;assert(basic_ioctl(&f,FPGA_IOC_SUBMIT_IMAGE,(unsigned long)&c)==-EBUSY);
 d.state=BASIC_IDLE;force_copy_fault=true;
 assert(basic_ioctl(&f,FPGA_IOC_SUBMIT_IMAGE,(unsigned long)&c)==-EFAULT);
 force_copy_fault=false;d.present=false;
 assert(basic_ioctl(&f,FPGA_IOC_SUBMIT_IMAGE,(unsigned long)&c)==-ENODEV);d.present=true;
 assert(writes==12);
 assert(basic_ioctl(&f,FPGA_IOC_SET_IMAGE,0)==-EOPNOTSUPP);
 assert(basic_ioctl(&f,FPGA_IOC_GET_IMAGE,0)==-EOPNOTSUPP);assert(writes==12);
 struct fpga_caps caps={0};assert(basic_ioctl(&f,FPGA_IOC_GET_CAPS,(unsigned long)&caps)==0);
 assert(caps.version==1 && caps.flags==FPGA_CAP_IMAGE_SUBMIT && !caps.reserved[0] && !caps.reserved[1]);
 struct fpga_info info={0};assert(basic_ioctl(&f,FPGA_IOC_GET_INFO,(unsigned long)&info)==0);
 assert(info.abi_version==2 && info.map_bytes==4149248 && info.offset[3]==12447744);
 // STOP 成功释放额外模块引用，但超时保留 DMA、Bus Master 与引用。
 d.lock.held=1;d.state=BASIC_RUNNING;d.dma_guard=true;refs=1;p.master=1;ack=1;
 assert(basic_stop_locked(&d)==0);assert(!d.dma_guard && refs==0 && !p.master && d.state==BASIC_IDLE);
 d.state=BASIC_RUNNING;d.dma_guard=true;refs=1;p.master=1;ack=0;now=0;
 assert(basic_stop_locked(&d)==-ETIMEDOUT);assert(d.dma_guard && refs==1 && p.master && d.state==BASIC_STOPPING);
 ack=1;assert(basic_stop_locked(&d)==0);assert(refs==0);d.lock.held=0;
 // 调参不妨碍正常释放；STOP 未完成或 mmap 消费者仍存在时隔离。
 d.state=BASIC_IDLE;d.opened=false;basic_release_resources(&d);assert(frees==1 && !quarantines);
 d.state=BASIC_STOPPING;basic_release_resources(&d);assert(frees==1 && quarantines==1);
 d.state=BASIC_IDLE;d.opened=true;d.io_mode=BASIC_IO_MMAP;basic_release_resources(&d);
 assert(frees==1 && quarantines==2 && unmaps==3 && releases==3 && disables==3);
 puts("PASS: new submit order/clamp; legacy rejection; stable video ABI; STOP and DMA lifetime");
}
'''
# 避免测试计数器与 stdio puts() 重名。
prefix=prefix.replace('releases, disables;', 'releases, disables;').replace('quarantines, puts, unmaps', 'quarantines, frame_puts, unmaps').replace('++puts;', '++frame_puts;')
with tempfile.TemporaryDirectory(prefix='pcie-v7-driver-test-') as out:
    unit = Path(out)/'control.c'
    unit.write_text(prefix + '\n\n'.join(function(n) for n in
        ['basic_release_resources','basic_stop_locked','basic_cfg_submit','basic_ioctl']) + test)
    exe=Path(out)/'control-test'
    subprocess.run([os.environ.get('CC','cc'),'-std=gnu11','-Wall','-Wextra',
        '-Wno-sign-compare','-Wno-misleading-indentation','-I',str(root),str(unit),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
