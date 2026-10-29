#!/usr/bin/env python3
"""执行真实 DMA-BUF 导出/映射回调，验证 fd 失败回滚、独立 DMA 映射和引用寿命。

此处替换的是内核依赖，不模拟真实缓存/IOMMU/RGA；这些仍须板级验证。
"""
import os
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'my_pciedriver.c').read_text(encoding='utf-8-sig')


def function(name):
    match = re.search(r'^static [^;{]*\b' + name + r'\([^;{]*\)\n\{', source, re.M)
    if not match:
        raise RuntimeError('missing function ' + name)
    return source[match.start():source.index('\n}', match.end()) + 2]


prefix = r'''
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fpga_uapi.h"
typedef uint32_t u32;
typedef uint8_t u8;
typedef uint64_t u64;
#define __user
#define IS_ENABLED(x) 1
#define GFP_KERNEL 0
#define ERESTARTSYS 512
#define DRIVER_NAME "basic_pcie"
#define FPGA_MAP_BYTES 4096
#define FPGA_IMAGE_BYTES 128
#define PAGE_SHIFT 12
#define DMA_ATTR_SKIP_CPU_SYNC 4
#define ERR_PTR(n) ((void*)(intptr_t)(n))
#define PTR_ERR(p) ((intptr_t)(p))
#define IS_ERR(p) ((uintptr_t)(p)>=(uintptr_t)-4095)
#define VM_MAYSHARE 1U
#define VM_READ 2U
#define VM_WRITE 4U
#define VM_EXEC 8U
#define VM_MAYWRITE 16U
#define VM_MAYEXEC 32U
#define VM_DONTCOPY 64U
#define VM_DONTEXPAND 128U
#define VM_DONTDUMP 256U
enum {BASIC_IO_UNSET,BASIC_IO_READ,BASIC_IO_MMAP,BASIC_IO_DMABUF};
enum {BASIC_IDLE,BASIC_RUNNING,BASIC_STOPPING};
enum dma_data_direction {DMA_BIDIRECTIONAL,DMA_TO_DEVICE,DMA_FROM_DEVICE,DMA_NONE};
struct mutex {bool held;};
struct device {uintptr_t domain;};
struct pci_dev {struct device dev;};
struct basic_dma_buffer {void *cpu_addr;uintptr_t dma_addr;size_t size;};
struct basic_pcie_dev {
 struct pci_dev *pdev;struct mutex lock,read_lock;bool present,opened;
 int io_mode,state;u32 exported_slots;struct basic_dma_buffer frame[4];
 u32 held_token[4],last_frame_token[4],last_sequence;u64 session;
};
struct dma_buf;
struct file {struct basic_pcie_dev *private_data;int refs;struct dma_buf *dmabuf;int f_flags;};
struct scatterlist {uintptr_t page,dma;unsigned length,offset;struct scatterlist *next;};
struct sg_table {struct scatterlist *sgl;unsigned nents,orig_nents;};
struct dma_buf_attachment {struct device *dev;struct dma_buf *dmabuf;};
struct vm_area_struct {unsigned long vm_pgoff,vm_end,vm_start,vm_flags;};
struct dma_buf_ops {
 int (*attach)(struct dma_buf*,struct dma_buf_attachment*);
 struct sg_table *(*map_dma_buf)(struct dma_buf_attachment*,enum dma_data_direction);
 void (*unmap_dma_buf)(struct dma_buf_attachment*,struct sg_table*,enum dma_data_direction);
 void (*release)(struct dma_buf*);
 int (*mmap)(struct dma_buf*,struct vm_area_struct*);
 int (*begin_cpu_access)(struct dma_buf*,enum dma_data_direction);
 int (*end_cpu_access)(struct dma_buf*,enum dma_data_direction);
 void *(*map)(struct dma_buf*,unsigned long);
 void (*unmap)(struct dma_buf*,unsigned long,void*);
};
struct dma_buf {void *priv;struct file *file;const struct dma_buf_ops *ops;};
struct dma_buf_export_info {const char *exp_name;const struct dma_buf_ops *ops;size_t size;int flags;void *priv;};
#define DEFINE_DMA_BUF_EXPORT_INFO(n) struct dma_buf_export_info n={0}
static bool fail_alloc,fail_sg,fail_export,fail_reserve,fault_in,fault_out,fail_map;
static unsigned sg_allocations,sg_frees,unmaps,closed,barriers,frame_releases,ready_slot=2;
static struct dma_buf *installed[64];
static bool reserved[64];
static struct mutex *device_lock;
static void *kzalloc(size_t n,int flags){(void)flags;return fail_alloc?NULL:calloc(1,n);}
static void kfree(void *p){free(p);}
static void mutex_lock(struct mutex *m){assert(!m->held);m->held=true;}
static int mutex_lock_interruptible(struct mutex *m){mutex_lock(m);return 0;}
static int mutex_trylock(struct mutex *m){if(m->held)return 0;mutex_lock(m);return 1;}
static void mutex_unlock(struct mutex *m){assert(m->held);m->held=false;}
static int copy_from_user(void *to,const void *from,size_t n){if(fault_in)return 1;memcpy(to,from,n);return 0;}
static int copy_to_user(void *to,const void *from,size_t n){
 if(n==sizeof(struct fpga_dmabuf_export)){
  const struct fpga_dmabuf_export *r=from;
  assert(reserved[r->fd] && !installed[r->fd]); // 此时还不能安装可见 fd。
 }
 if(fault_out)return 1;memcpy(to,from,n);return 0;
}
static struct file *get_file(struct file *f){assert(f->refs>0);++f->refs;return f;}
static void fput(struct file *f){
 assert(f->refs>0);if(--f->refs)return;
 if(f->dmabuf){struct dma_buf *b=f->dmabuf;b->ops->release(b);free(b);free(f);}
 else{assert(!device_lock->held);++closed;f->private_data->opened=false;}
}
static int sg_alloc_table(struct sg_table *t,unsigned n,int flags){
 (void)flags;if(fail_sg)return -ENOMEM;t->sgl=calloc(n,sizeof(*t->sgl));assert(t->sgl);
 t->orig_nents=t->nents=n;
 for(unsigned i=0;i+1<n;++i)t->sgl[i].next=&t->sgl[i+1];++sg_allocations;return 0;
}
static void sg_free_table(struct sg_table *t){assert(t->sgl);free(t->sgl);t->sgl=NULL;++sg_frees;}
#define sg_page(s) ((s)->page)
#define sg_next(s) ((s)->next)
#define sg_set_page(s,p,n,o) do{(s)->page=(p);(s)->length=(n);(s)->offset=(o);}while(0)
#define for_each_sg(first,s,n,i) for((i)=0,(s)=(first);(i)<(n);++(i),(s)=sg_next(s))
static int dma_get_sgtable(struct device *d,struct sg_table *t,void *cpu,uintptr_t dma,size_t n){
 (void)d;(void)cpu;(void)dma;int r=sg_alloc_table(t,2,0);if(r)return r;
 t->sgl[0].page=0x100000;t->sgl[1].page=0x400000;
 t->sgl[0].length=t->sgl[1].length=n/2;return 0;
}
static int dma_map_sg_attrs(struct device *d,struct scatterlist *s,unsigned n,
 enum dma_data_direction direction,unsigned long attrs){
 assert(n==2 && direction!=DMA_NONE && attrs==DMA_ATTR_SKIP_CPU_SYNC);
 if(fail_map)return 0;
 for(unsigned i=0;i<n;++i,s=sg_next(s))s->dma=s->page+d->domain;
 return 1; // 模拟导入设备合并 SG；unmap 仍须使用 orig_nents=2。
}
static void dma_unmap_sg_attrs(struct device *d,struct scatterlist *s,unsigned n,
 enum dma_data_direction direction,unsigned long attrs){
 assert(n==2 && s->dma==s->page+d->domain && direction!=DMA_NONE && attrs==DMA_ATTR_SKIP_CPU_SYNC);++unmaps;
}
static int dma_mmap_coherent(struct device *d,struct vm_area_struct *v,void *cpu,uintptr_t dma,size_t n){
 (void)d;(void)cpu;(void)dma;assert(n==FPGA_MAP_BYTES && !v->vm_pgoff);return 0;
}
#define dma_rmb() (++barriers)
#define dma_wmb() (++barriers)
static struct dma_buf *dma_buf_export(struct dma_buf_export_info *info){
 if(fail_export)return ERR_PTR(-ENOMEM);
 assert(info->flags==O_RDONLY && info->size==FPGA_MAP_BYTES && info->ops && info->ops->map && info->ops->mmap);
 struct dma_buf *b=calloc(1,sizeof(*b));b->file=calloc(1,sizeof(*b->file));
 b->ops=info->ops;b->priv=info->priv;b->file->refs=1;b->file->dmabuf=b;return b;
}
static void dma_buf_put(struct dma_buf *b){fput(b->file);}
static int get_unused_fd_flags(unsigned flags){
 assert(flags==O_CLOEXEC);if(fail_reserve)return -EMFILE;
 for(int i=10;i<64;++i)if(!reserved[i]&&!installed[i]){reserved[i]=true;return i;}return -EMFILE;
}
static void put_unused_fd(int fd){assert(reserved[fd]);reserved[fd]=false;}
static void fd_install(int fd,struct file *f){assert(reserved[fd]&&!installed[fd]);installed[fd]=f->dmabuf;reserved[fd]=false;}
static void close_export(int fd){struct dma_buf *b=installed[fd];assert(b);installed[fd]=NULL;dma_buf_put(b);}
static int basic_wait_frame_locked(struct basic_pcie_dev *d,unsigned *slot,u32 *token){
 assert(d->lock.held && d->read_lock.held);if(d->state!=BASIC_RUNNING)return 0;
 *slot=ready_slot;*token=(9U<<2)|ready_slot;return 1;
}
static void basic_release_frame_locked(struct basic_pcie_dev *d,unsigned slot,u32 token){
 assert(d->lock.held && d->held_token[slot]==token);++frame_releases;
}
static void basic_notify_locked(struct basic_pcie_dev *d){assert(d->lock.held);}
'''

tests = r'''
static void reject_export(struct file *f,int expected){
 struct fpga_dmabuf_export r={.slot=0,.fd=-1};unsigned before=sg_allocations-sg_frees;
 assert(basic_export_dmabuf(f,&r)==expected);
 assert(!device_lock->held && f->refs==1 && f->private_data->exported_slots==0);
 assert(sg_allocations-sg_frees==before);
 for(int i=0;i<64;++i)assert(!reserved[i]&&!installed[i]);
}
int main(void){
 _Static_assert(sizeof(struct fpga_dmabuf_export)==16,"export ABI");
 assert(FPGA_IOC_EXPORT_DMABUF==0xc0105006UL);
 struct pci_dev p={0};struct basic_pcie_dev d={.pdev=&p,.present=true,.opened=true};
 struct file f={.private_data=&d,.refs=1};device_lock=&d.lock;
 for(unsigned i=0;i<4;++i){d.frame[i].size=FPGA_MAP_BYTES;d.frame[i].dma_addr=0xdead0000+i*0x10000;d.frame[i].cpu_addr=(void*)(uintptr_t)(0x1000+i*0x1000);}
 fault_in=true;reject_export(&f,-EFAULT);fault_in=false;
 d.present=false;reject_export(&f,-ENODEV);d.present=true;
 d.state=BASIC_RUNNING;reject_export(&f,-EBUSY);d.state=BASIC_IDLE;
 for(int mode=BASIC_IO_READ;mode<=BASIC_IO_MMAP;++mode){d.io_mode=mode;reject_export(&f,-EBUSY);}d.io_mode=BASIC_IO_UNSET;
 fail_alloc=true;reject_export(&f,-ENOMEM);fail_alloc=false;
 fail_sg=true;reject_export(&f,-ENOMEM);fail_sg=false;
 fail_export=true;reject_export(&f,-ENOMEM);fail_export=false;
 fail_reserve=true;reject_export(&f,-EMFILE);fail_reserve=false;
 fault_out=true;reject_export(&f,-EFAULT);fault_out=false;
 struct fpga_dmabuf_export r={.slot=4};assert(basic_export_dmabuf(&f,&r)==-EINVAL);
 r.slot=0;r.flags=1;assert(basic_export_dmabuf(&f,&r)==-EINVAL);
 r.flags=0;r.reserved=1;assert(basic_export_dmabuf(&f,&r)==-EINVAL);
 r.reserved=0;assert(basic_export_dmabuf(&f,&r)==0);const int first=r.fd;
 assert(d.io_mode==BASIC_IO_DMABUF && d.exported_slots==1 && f.refs==2);
 assert(basic_export_dmabuf(&f,&r)==-EALREADY);
 struct dma_buf *b=installed[first];struct device rga={.domain=0x80000000},gpu={.domain=0x50000000};
 struct dma_buf_attachment a={.dev=&rga,.dmabuf=b},a2={.dev=&gpu,.dmabuf=b};
 assert(basic_dmabuf_attach(b,&a)==0);
 struct sg_table *s=basic_dmabuf_map(&a,DMA_BIDIRECTIONAL),*s2=basic_dmabuf_map(&a2,DMA_TO_DEVICE);
 assert(!IS_ERR(s)&&!IS_ERR(s2)&&s!=s2 && s->sgl!=s2->sgl && s->nents==1);
 assert(s->sgl->dma==0x80100000 && s2->sgl->dma==0x50100000 && s->sgl->dma!=d.frame[0].dma_addr);
 struct basic_dmabuf *priv=b->priv;assert(priv->pages.sgl->dma==0); // backing 表未被污染。
 basic_dmabuf_unmap(&a,s,DMA_BIDIRECTIONAL);basic_dmabuf_unmap(&a2,s2,DMA_TO_DEVICE);assert(unmaps==2);
 fail_map=true;assert(PTR_ERR(basic_dmabuf_map(&a,DMA_TO_DEVICE))==-EIO);fail_map=false;
 assert(PTR_ERR(basic_dmabuf_map(&a,DMA_NONE))==-EINVAL);
 struct vm_area_struct v={.vm_start=0x10000,.vm_end=0x10000+FPGA_MAP_BYTES,.vm_flags=VM_MAYSHARE|VM_READ|VM_MAYWRITE};
 assert(basic_dmabuf_mmap(b,&v)==0 && !(v.vm_flags&VM_MAYWRITE));
 v.vm_flags|=VM_WRITE;assert(basic_dmabuf_mmap(b,&v)==-EACCES);v.vm_flags&=~VM_WRITE;
 v.vm_pgoff=1;assert(basic_dmabuf_mmap(b,&v)==-EINVAL);v.vm_pgoff=0;
 d.lock.held=true;assert(basic_dmabuf_mmap(b,&v)==-EAGAIN);d.lock.held=false;
 assert(basic_dmabuf_cpu_begin(b,DMA_FROM_DEVICE)==0 && barriers==1);
 assert(basic_dmabuf_cpu_begin(b,DMA_BIDIRECTIONAL)==0);
 assert(basic_dmabuf_cpu_begin(b,DMA_NONE)==-EINVAL);
 assert(basic_dmabuf_cpu_end(b,DMA_FROM_DEVICE)==0);
 assert(basic_dmabuf_kmap(b,0)==d.frame[0].cpu_addr && !basic_dmabuf_kmap(b,1));
 basic_dmabuf_kunmap(b,0,d.frame[0].cpu_addr);
 d.present=false;
 assert(basic_dmabuf_attach(b,&a)==-ENODEV && PTR_ERR(basic_dmabuf_map(&a,DMA_TO_DEVICE))==-ENODEV);
 assert(basic_dmabuf_mmap(b,&v)==-ENODEV && basic_dmabuf_cpu_begin(b,DMA_FROM_DEVICE)==-ENODEV);d.present=true;
 int fds[4]={first};
 for(unsigned i=1;i<4;++i){r.slot=i;assert(basic_export_dmabuf(&f,&r)==0);fds[i]=r.fd;}
 assert(d.exported_slots==15 && f.refs==5);
 // DMA-BUF 模式直接取帧/归还，不需要先建立原 misc mmap。
 d.state=BASIC_RUNNING;d.session=7;
 struct fpga_frame frame={0};assert(basic_get_frame(&f,&frame)==0);
 assert(frame.slot==2 && frame.session==7 && d.held_token[2]==frame.token);
 ready_slot=3;fault_out=true;assert(basic_get_frame(&f,&frame)==-EFAULT && !d.held_token[3]);fault_out=false;
 mutex_lock(&d.lock);struct fpga_frame wrong=frame;wrong.session=6;
 assert(basic_put_frame_locked(&d,&wrong)==-ESTALE);
 assert(basic_put_frame_locked(&d,&frame)==0 && !d.held_token[2] && frame_releases==1);
 assert(basic_put_frame_locked(&d,&frame)==-ESTALE);mutex_unlock(&d.lock);
 ready_slot=2;assert(basic_get_frame(&f,&frame)==0);
 d.state=BASIC_IDLE;mutex_lock(&d.lock);
 assert(basic_put_frame_locked(&d,&frame)==0 && frame_releases==1);mutex_unlock(&d.lock);
 d.io_mode=BASIC_IO_READ;assert(basic_get_frame(&f,&frame)==-EINVAL);d.io_mode=BASIC_IO_DMABUF;
 get_file(b->file); // 模拟 RGA 导入者继续持有 DMA-BUF 引用。
 fput(&f);assert(f.refs==4 && !closed && d.opened);
 for(unsigned i=0;i<4;++i)close_export(fds[i]);
 assert(f.refs==1 && !closed && d.opened); // 控制 fd 和导出 fd 全关，导入者还在用。
 dma_buf_put(b);assert(closed==1 && !d.opened && !f.refs);
 assert(sg_allocations==sg_frees);
 puts("DMABUF_DRIVER_PASS: ABI/fd rollback, coherent SG export, per-device mappings, readonly CPU access, importer lifetime, unplug guards");
}
'''

struct_start = source.index('struct basic_dmabuf {')
struct_text = source[struct_start:source.index('\n};', struct_start) + 3]
ops_start = source.index('static const struct dma_buf_ops basic_dmabuf_ops')
ops_text = source[ops_start:source.index('\n};', ops_start) + 3]
names = ['basic_dmabuf_attach', 'basic_dmabuf_map', 'basic_dmabuf_unmap',
         'basic_dmabuf_release', 'basic_dmabuf_mmap', 'basic_dmabuf_cpu_begin', 'basic_dmabuf_cpu_end',
         'basic_dmabuf_kmap', 'basic_dmabuf_kunmap']
with tempfile.TemporaryDirectory(prefix='board-v10-dmabuf-driver-') as folder:
    unit = Path(folder) / 'dmabuf.c'
    unit.write_text(prefix + struct_text + '\n'.join(function(name) for name in names) +
                    ops_text + function('basic_export_dmabuf') + function('basic_get_frame') +
                    function('basic_put_frame_locked') + tests)
    exe = Path(folder) / 'dmabuf-test'
    subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu11', '-O1', '-Wall', '-Wextra',
                    '-Wno-misleading-indentation', '-I', str(root), str(unit), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True, timeout=15)
