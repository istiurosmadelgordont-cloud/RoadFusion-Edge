#!/usr/bin/env python3
"""从当前驱动抽取实际函数，在 Linux 用户态替换内核依赖；不访问设备。"""
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1] / 'driver'
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
#include <stdlib.h>
#include "image_protocol.h"
typedef uint32_t u32;
typedef uint8_t u8;
typedef uint32_t __le32;
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
#define FPGA_REG_IMAGE_ACK_ADDR 0x218U
#define FPGA_IMAGE_ACK_BYTES 128U
#define U32_MAX UINT32_MAX
#define U64_MAX UINT64_MAX
#define FPGA_REG_START 0x190U
#define FPGA_MARKER_BYTES 64U
#define READ_ONCE(x) (x)
#define le32_to_cpu(x) (x)
#define lower_32_bits(x) ((u32)(x))
static unsigned barriers;
#define dma_rmb() (++barriers)
#define dma_wmb() (++barriers)
#define FPGA_STOP_TIMEOUT_US 1000U
#define dev_err(...) ((void)0)
struct mutex {int held;};
struct pci_dev {int master;};
enum {BASIC_IO_UNSET,BASIC_IO_READ,BASIC_IO_MMAP};
enum {BASIC_IDLE,BASIC_RUNNING,BASIC_STOPPING,BASIC_QUARANTINED};
struct basic_dma_buffer {void *cpu_addr;u64 dma_addr;size_t size;};
struct basic_pcie_dev {
 struct pci_dev *pdev; void *bar;
 struct mutex lock; bool present, opened, dma_guard; int state,io_mode;
 u32 cookie_counter,stop_cookie;
 bool image_bound,image_pending;
 u32 image_cookie_counter;
 struct fpga_image_ack_status image_result;
 struct basic_dma_buffer image_ack,status,frame[4];
 u32 held_token[4];u64 session;bool snapshot_ready;size_t snapshot_pos;
};
struct file {struct basic_pcie_dev *private_data;};
static unsigned writes, frees, quarantines, puts, unmaps, releases, disables;
static unsigned offsets[100];static u32 values[100];static int refs, ack;
static ktime_t now;
static bool force_copy_fault;
static int try_module_get(int m){(void)m;++refs;return 1;}
static void pci_set_master(struct pci_dev *p){p->master=1;}
static int basic_alloc_buffer(struct basic_pcie_dev *d,struct basic_dma_buffer *b,size_t n){
 (void)d;b->size=n;b->dma_addr=0x9000000;b->cpu_addr=calloc(1,n);return b->cpu_addr?0:-ENOMEM;
}
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
 assert(caps.version==1 && caps.flags==(FPGA_CAP_IMAGE_SUBMIT|FPGA_CAP_IMAGE_ACK) && !caps.reserved[0] && !caps.reserved[1]);
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
 // 新 ACK 接口使用实际驱动函数：没有 APPLIED 时不能覆盖，不混用单向提交。
 d.state=BASIC_IDLE;d.opened=false;d.dma_guard=false;d.lock.held=0;refs=0;
 d.status.dma_addr=0x10000;d.status.size=64;
 for(unsigned n=0;n<4;++n){d.frame[n].dma_addr=0x1000000+n*0x800000;d.frame[n].size=FPGA_MAP_BYTES;}
 writes=0;
 assert(basic_ioctl(&f,FPGA_IOC_PREPARE_IMAGE_ACK,0)==0);
 assert(d.image_bound && d.dma_guard && p.master && refs==1);
 assert(writes==1 && offsets[0]==0x218 && values[0]==d.image_ack.dma_addr);
 assert(basic_ioctl(&f,FPGA_IOC_PREPARE_IMAGE_ACK,0)==-EBUSY);
 assert(basic_ioctl(&f,FPGA_IOC_SUBMIT_IMAGE,(unsigned long)&c)==-EBUSY);
 force_copy_fault=true;
 assert(basic_ioctl(&f,FPGA_IOC_SUBMIT_IMAGE_ACK,(unsigned long)&c)==-EFAULT);
 force_copy_fault=false;assert(writes==1 && !d.image_pending);
 assert(basic_ioctl(&f,FPGA_IOC_SUBMIT_IMAGE_ACK,(unsigned long)&c)==0);
 assert(writes==7 && values[6]==1 && offsets[6]==0x214 && d.image_pending);
 for(unsigned n=0;n<5;++n)assert(values[n+1]==clean[n]);
 struct fpga_image_ack_status s={0};u32 *mailbox=d.image_ack.cpu_addr;
 assert(basic_ioctl(&f,FPGA_IOC_GET_IMAGE_ACK,(unsigned long)&s)==0);
 assert(s.cookie==1 && s.flags==(FPGA_IMAGE_BOUND|FPGA_IMAGE_PENDING));
 for(unsigned n=16;n<32;++n)mailbox[n]=1; // APPLIED 不能独立证明接收。
 basic_ioctl(&f,FPGA_IOC_GET_IMAGE_ACK,(unsigned long)&s);assert(!(s.flags&FPGA_IMAGE_APPLIED));
 for(unsigned n=0;n<15;++n)mailbox[n]=1; // 少一个 DWORD 也不能确认。
 basic_ioctl(&f,FPGA_IOC_GET_IMAGE_ACK,(unsigned long)&s);assert(!(s.flags&FPGA_IMAGE_RECEIVED));
 assert(basic_ioctl(&f,FPGA_IOC_SUBMIT_IMAGE_ACK,(unsigned long)&c)==-EBUSY && writes==7);
 for(unsigned n=0;n<32;++n)mailbox[n]=99; // 全长但陈旧 cookie。
 basic_ioctl(&f,FPGA_IOC_GET_IMAGE_ACK,(unsigned long)&s);assert(!(s.flags&FPGA_IMAGE_RECEIVED));
 for(unsigned n=0;n<16;++n)mailbox[n]=1;
 basic_ioctl(&f,FPGA_IOC_GET_IMAGE_ACK,(unsigned long)&s);assert(s.flags&FPGA_IMAGE_RECEIVED);
 assert(!(s.flags&FPGA_IMAGE_APPLIED) && d.image_pending);
 for(unsigned n=16;n<32;++n)mailbox[n]=1;
 force_copy_fault=true;assert(basic_ioctl(&f,FPGA_IOC_GET_IMAGE_ACK,(unsigned long)&s)==-EFAULT);
 unsigned finished_barriers=barriers;
 force_copy_fault=false;assert(basic_ioctl(&f,FPGA_IOC_GET_IMAGE_ACK,(unsigned long)&s)==0);
 assert(s.cookie==1 && (s.flags&FPGA_IMAGE_APPLIED) && !d.image_pending && barriers);
 assert(barriers==finished_barriers); // 已完成结果直接返回缓存，不再扫描 coherent 邮箱。
 assert(basic_ioctl(&f,FPGA_IOC_SUBMIT_IMAGE_ACK,(unsigned long)&c)==0 && values[12]==2);
 for(unsigned n=0;n<32;++n)assert(mailbox[n]==0);
 // 视频未启动，也必须发 STOP；超时保留邮箱、cookie、模块与 BME。
 ack=0;now=0;u32 stop_cookie=d.stop_cookie;
 assert(basic_ioctl(&f,FPGA_IOC_STOP,0)==-ETIMEDOUT);
 assert(d.state==BASIC_STOPPING && d.image_bound && d.image_pending && d.dma_guard && p.master);
 basic_ioctl(&f,FPGA_IOC_GET_IMAGE_ACK,(unsigned long)&s);assert(s.flags&FPGA_IMAGE_DRAINING);
 assert(basic_ioctl(&f,FPGA_IOC_PREPARE_IMAGE_ACK,0)==-EBUSY);
 ack=1;assert(basic_ioctl(&f,FPGA_IOC_STOP,0)==0 && d.stop_cookie==stop_cookie);
 assert(!d.image_bound && !d.image_pending && !d.dma_guard && !p.master && refs==0);
 basic_ioctl(&f,FPGA_IOC_GET_IMAGE_ACK,(unsigned long)&s);
 assert(s.cookie==2 && (s.flags&FPGA_IMAGE_STOPPED) && !(s.flags&FPGA_IMAGE_BOUND));
 assert(basic_ioctl(&f,FPGA_IOC_PREPARE_IMAGE_ACK,0)==0);
 assert(basic_ioctl(&f,FPGA_IOC_SUBMIT_IMAGE_ACK,(unsigned long)&c)==0 && d.image_result.cookie==3);
 // START 必须复用 PREPARE 已预留的 STOP cookie，不能耗尽后留下无保护的邮箱。
 for(unsigned n=0;n<4;++n){d.frame[n].cpu_addr=calloc(1,FPGA_MAP_BYTES);assert(d.frame[n].cpu_addr);}
 stop_cookie=d.stop_cookie;u32 saved_counter=d.cookie_counter;d.cookie_counter=U32_MAX;
 assert(basic_ioctl(&f,FPGA_IOC_START,0)==0 && d.stop_cookie==stop_cookie && refs==1);
 assert(d.state==BASIC_RUNNING && d.image_bound);d.cookie_counter=saved_counter;
 assert(basic_ioctl(&f,FPGA_IOC_STOP,0)==0);
 assert(basic_ioctl(&f,FPGA_IOC_PREPARE_IMAGE_ACK,0)==0);d.image_cookie_counter=U32_MAX;
 unsigned before=writes;assert(basic_ioctl(&f,FPGA_IOC_SUBMIT_IMAGE_ACK,(unsigned long)&c)==-EOVERFLOW);
 assert(writes==before);assert(basic_ioctl(&f,FPGA_IOC_STOP,0)==0);
 // 使用完整分配长度检查重叠，而非仅图像字节数；非法登记不写 BAR。
 d.image_ack.dma_addr=d.frame[0].dma_addr+FPGA_MAP_BYTES-64;before=writes;
 assert(basic_ioctl(&f,FPGA_IOC_PREPARE_IMAGE_ACK,0)==-ERANGE && writes==before);
 d.image_ack.dma_addr=d.status.dma_addr;
 assert(basic_ioctl(&f,FPGA_IOC_PREPARE_IMAGE_ACK,0)==-ERANGE && writes==before);
 free(d.image_ack.cpu_addr);
 for(unsigned n=0;n<4;++n)free(d.frame[n].cpu_addr);
 puts("PASS: real driver one-way + two-stage ACK; partial/stale replies; one outstanding; STOP timeout/drain; cookie lifetime; ABI and overlap");
}
'''
# 避免测试计数器与 stdio puts() 重名。
prefix=prefix.replace('releases, disables;', 'releases, disables;').replace('quarantines, puts, unmaps', 'quarantines, frame_puts, unmaps').replace('++puts;', '++frame_puts;')
with tempfile.TemporaryDirectory(prefix='pcie-v7-driver-test-') as out:
    unit = Path(out)/'control.c'
    unit.write_text(prefix + '\n\n'.join(function(n) for n in
        ['basic_prepare_dma','basic_image_record_matches','basic_image_poll_locked',
         'basic_image_stopped_locked','basic_dma_overlaps','basic_image_prepare_locked',
         'basic_image_submit_locked','basic_image_status_locked','basic_release_resources',
         'basic_stop_locked','basic_start_locked','basic_cfg_submit','basic_ioctl']) + test)
    exe=Path(out)/'control-test'
    subprocess.run([os.environ.get('CC','cc'),'-std=gnu11','-Wall','-Wextra',
        '-Wno-sign-compare','-Wno-misleading-indentation','-I',str(root),str(unit),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
