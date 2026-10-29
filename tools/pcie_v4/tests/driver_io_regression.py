#!/usr/bin/env python3
"""抽取驱动的真实 read/等待函数，模拟用户缺页期间 STOP、重启和解绑。

不访问 FPGA。内核模块编译仍需目标板头文件；此检查只验证并发边界和返回值。
"""
import os
from pathlib import Path
import re
import subprocess
import tempfile

source = (Path(__file__).resolve().parents[1] / 'my_pciedriver.c').read_text(encoding='utf-8-sig')


def function(name):
    match = re.search(r'^static [^;{]*\b' + name + r'\([^;{]*\)\n\{', source, re.M)
    if not match:
        raise RuntimeError('missing driver function: ' + name)
    return source[match.start():source.index('\n}', match.end()) + 2]


prefix = r'''
#define _GNU_SOURCE
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
typedef uint8_t u8;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int64_t ktime_t;
#define __user
#define READ_ONCE(x) (x)
#define ERESTARTSYS 512
#define FPGA_IMAGE_BYTES 64U
#define FPGA_FRAME_SLOTS 4U
#define FPGA_FRAME_TIMEOUT_US 5000U
#define min_t(type,a,b) ((type)(a)<(type)(b)?(type)(a):(type)(b))
#define clamp_t(type,n,lo,hi) ((type)(n)<(type)(lo)?(type)(lo):(type)(n)>(type)(hi)?(type)(hi):(type)(n))
enum {BASIC_IO_UNSET, BASIC_IO_READ, BASIC_IO_MMAP, BASIC_IO_DMABUF};
enum {BASIC_IDLE, BASIC_RUNNING, BASIC_STOPPING};
struct mutex {pthread_mutex_t native; bool held;};
struct basic_dma_buffer {void *cpu_addr;};
struct basic_pcie_dev {
 struct mutex lock, read_lock;
 bool present, snapshot_ready;
 int state, io_mode, waitq;
 u64 session;
 unsigned int wake_seq;
 u32 held_token[4], last_sequence;
 struct basic_dma_buffer frame[4];
 void *snapshot;
 size_t snapshot_pos;
 bool frame_available;
};
struct file {struct basic_pcie_dev *private_data;int f_flags;};
static ktime_t now;
static unsigned int frame_poll_us=1000,hr_waits,tick_waits;
static ktime_t last_delay;
static int wait_result, wait_action;
static struct basic_pcie_dev *waiting_device;
static void mutex_init(struct mutex *m){assert(!pthread_mutex_init(&m->native,NULL));m->held=false;}
static void mutex_lock(struct mutex *m){assert(!pthread_mutex_lock(&m->native));assert(!m->held);m->held=true;}
static int mutex_lock_interruptible(struct mutex *m){mutex_lock(m);return 0;}
static void mutex_unlock(struct mutex *m){assert(m->held);m->held=false;assert(!pthread_mutex_unlock(&m->native));}
static ktime_t ktime_get(void){return now;}
static ktime_t ktime_add_us(ktime_t value,unsigned int us){return value+(ktime_t)us*1000;}
static ktime_t ktime_sub(ktime_t a,ktime_t b){return a-b;}
static ktime_t ktime_to_ns(ktime_t value){return value;}
static ktime_t ns_to_ktime(u64 value){return value;}
static int ktime_compare(ktime_t a,ktime_t b){return (a>b)-(a<b);}
static unsigned int msecs_to_jiffies(unsigned int value){return value;}
static bool basic_find_frame_locked(struct basic_pcie_dev *d,unsigned int *slot,u32 *token){
 assert(d->lock.held);
 if(!d->frame_available)return false;
 d->frame_available=false;*slot=0;*token=4;return true;
}
static void basic_release_frame_locked(struct basic_pcie_dev *d,unsigned int slot,u32 token){
 assert(d->lock.held && d->read_lock.held && token==4);
 // 归还后立刻模拟 FPGA 覆盖 DMA 槽；用户仍必须读到之前的普通副本。
 memset(d->frame[slot].cpu_addr,0xdd,FPGA_IMAGE_BYTES);
}
static long simulate_wait(ktime_t delay,bool high_resolution){
 struct basic_pcie_dev *d=waiting_device;
 assert(!d->lock.held && d->read_lock.held);
 if(high_resolution)++hr_waits;else ++tick_waits;
 last_delay=delay;now+=delay;
 if(wait_action==1)d->frame_available=true;
 if(wait_action==2){d->state=BASIC_IDLE;++d->wake_seq;}
 if(wait_action==3){++d->session;++d->wake_seq;}
 if(wait_action==4){d->present=false;++d->wake_seq;}
 return wait_result;
}
#define wait_event_interruptible_hrtimeout(q,c,t) ((c)?0:simulate_wait((t),true))
#define wait_event_interruptible_timeout(q,c,t) ((c)?1:simulate_wait((ktime_t)(t)*3333333,false))
enum {COPY_NORMAL, COPY_STOP, COPY_RESTART, COPY_DETACH};
static struct basic_pcie_dev *copy_device;
static int copy_action;
static size_t copy_missing;
static void *control_during_copy(void *unused){
 struct basic_pcie_dev *d=copy_device;
 struct timespec until;assert(!clock_gettime(CLOCK_REALTIME,&until));
 until.tv_sec+=1;
 // 一秒内拿不到锁就失败：证明 copy_to_user 不再把控制线程堵在设备锁上。
 assert(!pthread_mutex_timedlock(&d->lock.native,&until));
 assert(!d->lock.held);d->lock.held=true;
 if(copy_action==COPY_STOP){d->state=BASIC_IDLE;d->snapshot_ready=false;d->snapshot_pos=0;}
 if(copy_action==COPY_RESTART){++d->session;d->state=BASIC_RUNNING;d->snapshot_ready=false;d->snapshot_pos=0;}
 if(copy_action==COPY_DETACH){
  d->present=false;d->snapshot_ready=false;d->snapshot_pos=0;
  // remove 可以释放 DMA；进行中的用户复制只能继续读取独立 snapshot。
  free(d->frame[0].cpu_addr);d->frame[0].cpu_addr=NULL;
 }
 mutex_unlock(&d->lock);(void)unused;return NULL;
}
static unsigned long copy_to_user(void *user,const void *src,size_t size){
 struct basic_pcie_dev *d=copy_device;
 assert(d->read_lock.held && !d->lock.held);
 assert(src>=(const void*)d->snapshot && (const u8*)src+size<=(const u8*)d->snapshot+FPGA_IMAGE_BYTES);
 if(copy_action!=COPY_NORMAL){
  pthread_t worker;assert(!pthread_create(&worker,NULL,control_during_copy,NULL));
  assert(!pthread_join(worker,NULL));
 }
 const size_t missing=copy_missing<size?copy_missing:size;
 memcpy(user,src,size-missing);return missing;
}
'''

tests = r'''
static void setup(struct basic_pcie_dev *d){
 memset(d,0,sizeof(*d));mutex_init(&d->lock);mutex_init(&d->read_lock);
 d->snapshot=calloc(1,FPGA_IMAGE_BYTES);d->frame[0].cpu_addr=malloc(FPGA_IMAGE_BYTES);
 assert(d->snapshot && d->frame[0].cpu_addr);
 for(unsigned int i=0;i<FPGA_IMAGE_BYTES;++i)((u8*)d->frame[0].cpu_addr)[i]=(u8)i;
 d->present=true;d->state=BASIC_RUNNING;d->session=7;d->frame_available=true;
 waiting_device=copy_device=d;now=0;copy_action=COPY_NORMAL;copy_missing=0;
 frame_poll_us=1000;hr_waits=tick_waits=0;wait_action=1;wait_result=-ETIME;
}
static void teardown(struct basic_pcie_dev *d){
 assert(!d->lock.held && !d->read_lock.held);
 assert(!pthread_mutex_destroy(&d->lock.native));assert(!pthread_mutex_destroy(&d->read_lock.native));
 free(d->snapshot);free(d->frame[0].cpu_addr);
}
static void pixels(const u8 *data,size_t first,size_t count){
 for(size_t i=0;i<count;++i)assert(data[i]==(u8)(first+i));
}
static void read_tests(void){
 struct basic_pcie_dev d;struct file f={.private_data=&d};u8 output[128]={0};
 setup(&d);
 assert(basic_read(&f,(char*)output,20,NULL)==20);pixels(output,0,20);
 assert(d.snapshot_ready && d.snapshot_pos==20);
 assert(basic_read(&f,(char*)output,100,NULL)==44);pixels(output,20,44);
 assert(!d.snapshot_ready && d.snapshot_pos==0);teardown(&d);
 setup(&d);copy_missing=10;
 assert(basic_read(&f,(char*)output,30,NULL)==20);pixels(output,0,20);
 assert(d.snapshot_pos==20);copy_missing=100;
 assert(basic_read(&f,(char*)output,10,NULL)==-EFAULT && d.snapshot_pos==20);
 copy_missing=0;assert(basic_read(&f,(char*)output,100,NULL)==44);pixels(output,20,44);teardown(&d);
 for(int action=COPY_STOP;action<=COPY_DETACH;++action){
  setup(&d);copy_action=action;
  assert(basic_read(&f,(char*)output,20,NULL)==20);pixels(output,0,20);
  assert(!d.snapshot_ready && d.snapshot_pos==0); // 旧 read 不能恢复已清理的进度。
  if(action==COPY_RESTART)assert(d.session==8 && d.state==BASIC_RUNNING);
  if(action==COPY_STOP)assert(d.state==BASIC_IDLE);
  if(action==COPY_DETACH)assert(!d.present);
  teardown(&d);
 }
 setup(&d);d.io_mode=BASIC_IO_MMAP;
 assert(basic_read(&f,(char*)output,20,NULL)==-EBUSY);
 d.io_mode=BASIC_IO_DMABUF;
 assert(basic_read(&f,(char*)output,20,NULL)==-EBUSY);
 d.io_mode=BASIC_IO_UNSET;f.f_flags=O_NONBLOCK;
 assert(basic_read(&f,(char*)output,20,NULL)==-EOPNOTSUPP);f.f_flags=0;
 d.present=false;assert(basic_read(&f,(char*)output,20,NULL)==-ENODEV);
 assert(basic_read(&f,(char*)output,0,NULL)==0);teardown(&d);
 puts("READ_CONCURRENCY_PASS: copy outside state lock, stable snapshot, short/fault reads, concurrent STOP/restart/remove, no stale progress");
}
static int wait_one(struct basic_pcie_dev *d){
 unsigned int slot=0;u32 token=0;
 mutex_lock(&d->read_lock);mutex_lock(&d->lock);
 const int ret=basic_wait_frame_locked(d,&slot,&token);
 assert(d->lock.held && d->read_lock.held);
 mutex_unlock(&d->lock);mutex_unlock(&d->read_lock);return ret;
}
static void wait_tests(void){
 struct basic_pcie_dev d;
 setup(&d);d.frame_available=false;
 assert(wait_one(&d)==1 && hr_waits==1 && !tick_waits && last_delay==1000000);teardown(&d);
 setup(&d);d.frame_available=false;frame_poll_us=0;wait_result=0;
 assert(wait_one(&d)==1 && !hr_waits && tick_waits==1);teardown(&d);
 setup(&d);d.frame_available=false;frame_poll_us=1;
 assert(wait_one(&d)==1 && last_delay==100000);teardown(&d);
 setup(&d);d.frame_available=false;frame_poll_us=100000;
 assert(wait_one(&d)==1 && last_delay==(ktime_t)FPGA_FRAME_TIMEOUT_US*1000);teardown(&d);
 setup(&d);d.frame_available=false;wait_action=0;
 assert(wait_one(&d)==-ETIMEDOUT && now==(ktime_t)FPGA_FRAME_TIMEOUT_US*1000);teardown(&d);
 setup(&d);d.frame_available=false;wait_result=-ERESTARTSYS;
 assert(wait_one(&d)==-ERESTARTSYS);teardown(&d);
 setup(&d);d.frame_available=false;wait_action=2;wait_result=0;
 assert(wait_one(&d)==0);teardown(&d);
 setup(&d);d.frame_available=false;wait_action=3;wait_result=0;
 assert(wait_one(&d)==-ECANCELED);teardown(&d);
 setup(&d);d.frame_available=false;wait_action=4;wait_result=0;
 assert(wait_one(&d)==-ENODEV);teardown(&d);
 setup(&d);d.frame_available=false;for(unsigned int i=0;i<4;++i)d.held_token[i]=4+i;
 assert(wait_one(&d)==-ENOBUFS && !hr_waits);teardown(&d);
 puts("FRAME_WAIT_PASS: high-res/legacy, clamping/deadline, -ETIME retry, signal, STOP, session, removal and held slots");
}
int main(void){read_tests();wait_tests();return 0;}
'''

with tempfile.TemporaryDirectory(prefix='board-v10-driver-io-') as out:
    unit = Path(out) / 'io.c'
    unit.write_text(prefix + function('basic_wait_frame_locked') + '\n' + function('basic_read') + tests)
    binary = Path(out) / 'io-test'
    subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu11', '-O1', '-Wall', '-Wextra',
                    '-Wno-unused-parameter', str(unit), '-o', str(binary), '-pthread'], check=True)
    subprocess.run([str(binary)], check=True, timeout=15)
