// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCIe 视频驱动学习版 7.0（2026-10-15 FPGA 单向调参）：read 双复制 / DMA mmap 两种模式。
 * Linux 4.19.232；FPGA PCIE_HS_20260919 四槽协议 v2；不使用 V4L2。
 *
 * v2 Qt 与本驱动一起使用；read/mmap 两条图像路径保留。
 *
 * 图像参数：五字暂存 + 0x214=1 提交，没有独立配置状态 DMA。
 * SUBMIT_IMAGE 只表示已执行 MMIO 写入，不证明像素域已经应用。
 * 旧 SET_IMAGE/GET_IMAGE 返回 -EOPNOTSUPP，不能伪造 APPLIED。
 * 四槽视频、STOP 状态区和 DMA 生命周期保护保持不变。
 * 新用户程序使用本文件下面的 UAPI 定义（不要包含整个驱动文件）：
 *   open(O_RDONLY) -> GET_INFO
 *   对每个槽 mmap(PROT_READ, MAP_SHARED, info.offset[i], info.map_bytes)
 *   START -> GET_FRAME -> 使用 mapped[frame.slot] -> PUT_FRAME
 *   STOP -> 结束所有像素访问并 PUT 所有持有帧 -> munmap -> close
 *   注：mmap 的实际参数顺序见用户态 mmap()；这里描述流程。
 *
 * 映射整槽只做一次，不要每帧 mmap/munmap。
 * mmap 成功不等于有完整帧：只有 GET_FRAME 成功后才允许访问该槽，
 * PUT_FRAME 前必须结束所有 CPU/Qt 消费者。不要发送给 GUI 后立即 PUT。
 * 用户只读 info.image_bytes 字节；尾部 token 和页填充不属于图像。
 * 整槽映射是只读的，但不按帧撤销访问权限，用户必须遵守所有权协议。
 *
 * 一次 open 只能选择 read 或 mmap，选定后不能混用；重新 open 才能切换。
 * mmap 只允许在 IDLE 时建立，四槽应全部映射后再 START。
 * GET_FRAME 最多等 5 秒；IDLE 返回 -EPIPE，四槽全被用户持有返回 -ENOBUFS。
 * STOP 可在持有帧时执行；它不撤销用户持有权。PUT 在 IDLE/STOPPING 时
 * 只结束软件持有，不写 RELEASE。还有未归还帧时禁止重新 START。
 *
 * 寿命：VMA 自带的 vm_file 引用使 .release 推迟到最后一个映射解除后。
 * 因而 close(fd) 不保证立即执行 .release，必须先显式 STOP。
 * 正常卸载前应 STOP -> PUT -> munmap -> close，让应用先退出。
 * 若仍有 mmap 会话打开就被强制解绑，即使 STOP 已确认也隔离 DMA。
 * 原因：4.19 在 remove 返回后会清除 DMA ops，不能晚些再 dma_free。
 * 本学习版以保留内存和模块引用处理这个异常，不提供自动恢复。
 * 不覆盖 DMA API 的 vm_ops，也不把 DMA 地址当 CPU 物理地址映射。
 *
 * mmap 和用户复制的锁序：mmap 持有 mm 的写锁，不能再阻塞等待 bdev->lock，
 * 否则可能与持有 bdev->lock 的 copy_to_user 缺页形成死锁。
 * 本版 mmap 使用 mutex_trylock，竞争时返回 -EAGAIN，用户稍后重试。
 *
 * STOP 超时后保留 DMA 和模块引用；重试继续等待同一个 cookie。
 * 解绑时仍无法确认停止则永久隔离，拒绝再次绑定相同 BDF，需可靠复位。
 * 本例不提供未经板级验证的自动复位、热拔插恢复或系统休眠支持。
 * 首次绑定前 FPGA 必须是可靠复位后的已知空闲态，BME=0 不能证明排空。
 *
 * BAR1 仅支持命令写，没有 BAR 读 Completion 和 MSI；严禁 readl 刷新。
 * 图像固定 1920x1080 RGB565，像素字节序仍需板级核对。
 */
#include <linux/types.h>
#include <linux/mm.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/limits.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <asm/byteorder.h>

#define DRIVER_NAME             "basic_pcie"
#define FPGA_VENDOR_ID          0x0755
#define FPGA_DEVICE_ID          0x0755
#define FPGA_BAR_INDEX          1

#define FPGA_REG_FRAME_ADDR     0x110
#define FPGA_REG_STOP           0x130
#define FPGA_REG_RELEASE        0x170
#define FPGA_REG_STATUS_ADDR    0x180
#define FPGA_REG_START          0x190
#define FPGA_REG_RESET_CONFIG   0x1a0
#define FPGA_REG_IMAGE_WORD0    0x200
#define FPGA_REG_IMAGE_COMMIT   0x214
#define FPGA_BAR_MAP_BYTES      (FPGA_REG_IMAGE_COMMIT + sizeof(u32))

#define FPGA_FRAME_WIDTH        1920U
#define FPGA_FRAME_HEIGHT       1080U
#define FPGA_IMAGE_BYTES        (FPGA_FRAME_WIDTH * FPGA_FRAME_HEIGHT * 2U)
#define FPGA_MARKER_BYTES       64U
#define FPGA_FRAME_BYTES        (FPGA_IMAGE_BYTES + FPGA_MARKER_BYTES)
/* 分配、范围检查、映射和释放统一使用页对齐后的真实长度。 */
#define FPGA_MAP_BYTES          PAGE_ALIGN(FPGA_FRAME_BYTES)
#define FPGA_STATUS_BYTES       64U
#define FPGA_DMA_ALIGNMENT      64U
#define FPGA_STOP_TIMEOUT_US    1000000U
#define FPGA_FRAME_TIMEOUT_US   5000000U

#include "fpga_uapi.h"
#include "image_protocol.h"

enum basic_io_mode {
	BASIC_IO_UNSET,
	BASIC_IO_READ,
	BASIC_IO_MMAP,
};

/* 所有状态修改都受 bdev->lock 保护。 */
enum basic_state {
	BASIC_IDLE,             /* 尚未启动，或已经收到 STOP ACK */
	BASIC_RUNNING,          /* FPGA 可以写入四个 DMA 槽 */
	BASIC_STOPPING,         /* STOP 已提交，尚未取得有效 ACK */
	BASIC_QUARANTINED,      /* 解绑时无法确认停止，资源保留 */
};

struct basic_dma_buffer {
	void *cpu_addr;         /* CPU 使用的内核虚拟地址 */
	dma_addr_t dma_addr;    /* FPGA 使用的 DMA/IOVA 地址 */
	size_t size;
};

struct basic_pcie_dev {
	struct pci_dev *pdev;
	void __iomem *bar;
	struct basic_dma_buffer frame[FPGA_FRAME_SLOTS];
	struct basic_dma_buffer status;

	/*
	 * lock：保护硬件命令、状态、DMA 槽及 snapshot。
	 * read_lock：把 read / GET_FRAME 等待者串行化。
	 * 固定锁顺序 read_lock -> lock；STOP/remove 不获取 read_lock。
	 */
	struct mutex lock;
	struct mutex read_lock;
	wait_queue_head_t waitq;

	struct miscdevice misc;
	char name[48];
	struct kref refs;       /* 绑定持有一份引用，每次成功 open 再持有一份 */
	bool present;
	bool opened;
	enum basic_io_mode io_mode; /* 每次 open 重新选择接收方式 */
	u32 held_token[FPGA_FRAME_SLOTS]; /* 0=未交付，非零=用户仍持有 */
	enum basic_state state;

	void *snapshot;        /* 普通内核副本；允许分多次 read 导出一帧 */
	bool snapshot_ready;
	size_t snapshot_pos;
	u32 last_frame_token[FPGA_FRAME_SLOTS];
	u32 last_sequence;     /* 防止按槽号扫描导致输出序号倒退 */

	/*
	 * START 前预留下一次 STOP 的 cookie，避免运行后发现编号已耗尽。
	 * STOP 超时后 cookie 不变，重试只轮询，不能再清状态区/换新编号。
	 */
	u32 cookie_counter;
	u32 stop_cookie;
	bool dma_guard;        /* 未确认 DMA 停止期间额外持有模块引用 */

	/*
	 * 每次 START 增加会话代数，使旧 read 不会误读重启后的新会话。
	 * wake_seq 只用来唤醒睡眠者；锁外读取使用 READ_ONCE。
	 * 真正的状态判断必须重新获取 lock。
	 */
	u64 session;
	unsigned int wake_seq;
	struct list_head quarantine_node;
};


/* ==================== 一、设备对象与异常隔离 ==================== */

static LIST_HEAD(basic_quarantine_list);
static DEFINE_MUTEX(basic_quarantine_lock);

/* 已隔离的 PCI 地址不可重新绑定，避免给未知状态的 FPGA 换 DMA 地址。 */
static bool basic_is_quarantined(struct pci_dev *pdev)
{
	struct basic_pcie_dev *entry;
	bool found = false;

	mutex_lock(&basic_quarantine_lock);
	list_for_each_entry(entry, &basic_quarantine_list, quarantine_node) {
		if (!strcmp(pci_name(entry->pdev), pci_name(pdev))) {
			found = true;
			break;
		}
	}
	mutex_unlock(&basic_quarantine_lock);
	return found;
}

/* STOP 未确认或 mmap 会话仍打开时解绑：保留 DMA、对象和模块。 */
static void basic_quarantine(struct basic_pcie_dev *bdev)
{
	/* mmap 解绑隔离可能发生在 STOP 成功之后，需重新保留模块引用。 */
	if (!bdev->dma_guard) {
		__module_get(THIS_MODULE);
		bdev->dma_guard = true;
	}
	bdev->state = BASIC_QUARANTINED;
	mutex_lock(&basic_quarantine_lock);
	list_add_tail(&bdev->quarantine_node, &basic_quarantine_list);
	mutex_unlock(&basic_quarantine_lock);
	dev_crit(&bdev->pdev->dev,
		 "DMA retained: STOP or mmap lifetime unresolved\n");
	/* 不 put 初始 kref，不释放 DMA，也不释放 DMA 模块引用。 */
}



/* ==================== 二、资源申请与成对释放 ==================== */

static void basic_free_buffer(struct basic_pcie_dev *bdev,
			      struct basic_dma_buffer *buffer)
{
	if (!buffer->cpu_addr)
		return;

	dma_free_coherent(&bdev->pdev->dev, buffer->size,
			  buffer->cpu_addr, buffer->dma_addr);
	buffer->cpu_addr = NULL;
}

/* 仅在图像 DMA 安全时使用，也可回滚分配失败。 */
static void basic_free_dma(struct basic_pcie_dev *bdev)
{
	int i;

	basic_free_buffer(bdev, &bdev->status);
	for (i = FPGA_FRAME_SLOTS - 1; i >= 0; i--)
		basic_free_buffer(bdev, &bdev->frame[i]);
}

/*
 * 初始引用属于 PCI 绑定；open 持有另一引用。
 * mmap 保留 vm_file，因而 open 的引用直到 fd 和全部 VMA 释放才会归还。
 * DMA 在安全的 remove 路径、DMA ops 仍有效时释放。
 * 对象最终析构只释放普通内存；异常隔离对象不会走到这里。
 */
static void basic_free_object(struct kref *ref)
{
	struct basic_pcie_dev *bdev;

	bdev = container_of(ref, struct basic_pcie_dev, refs);
	vfree(bdev->snapshot);
	pci_dev_put(bdev->pdev);
	kfree(bdev);
}

static int basic_alloc_buffer(struct basic_pcie_dev *bdev,
			      struct basic_dma_buffer *buffer, size_t size)
{
	u64 start;
	u64 limit = DMA_BIT_MASK(32);

	buffer->size = size;
	buffer->cpu_addr = dma_alloc_coherent(&bdev->pdev->dev, size,
					     &buffer->dma_addr, GFP_KERNEL);
	if (!buffer->cpu_addr)
		return -ENOMEM;

	/* 校验整个区间：非零、64 字节对齐、全部位于 32 位 DMA 空间。 */
	start = (u64)buffer->dma_addr;
	if (!size || size - 1 > limit || !start ||
	    (start & (FPGA_DMA_ALIGNMENT - 1)) ||
	    start > limit - (size - 1)) {
		basic_free_buffer(bdev, buffer);
		return -ERANGE;
	}
	memset(buffer->cpu_addr, 0, size);
	return 0;
}

/*
 * 一次准备好 BAR、DMA 地址限制、内核副本和四槽 DMA 缓冲区。
 * 成功：调用者负责 basic_release_resources()。
 * 失败：本函数回滚硬件资源；snapshot 随 basic_free_object() 释放。
 * 此函数不向 FPGA 提交地址，也不允许 FPGA 发起 DMA。
 */
static int basic_init_resources(struct basic_pcie_dev *bdev)
{
	struct pci_dev *pdev = bdev->pdev;
	int i, ret;

	ret = pci_enable_device_mem(pdev);
	if (ret)
		return ret;
	pci_clear_master(pdev);

	if (!(pci_resource_flags(pdev, FPGA_BAR_INDEX) & IORESOURCE_MEM) ||
	    pci_resource_len(pdev, FPGA_BAR_INDEX) < FPGA_BAR_MAP_BYTES) {
		ret = -ENODEV;
		goto disable;
	}
	ret = pci_request_region(pdev, FPGA_BAR_INDEX, DRIVER_NAME);
	if (ret)
		goto disable;
	bdev->bar = pci_iomap(pdev, FPGA_BAR_INDEX, FPGA_BAR_MAP_BYTES);
	if (!bdev->bar) {
		ret = -ENOMEM;
		goto release_region;
	}
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		goto unmap;

	bdev->snapshot = vzalloc(FPGA_IMAGE_BYTES);
	if (!bdev->snapshot) {
		ret = -ENOMEM;
		goto unmap;
	}
	/* 四个图像槽和独立状态区，失败时统一回滚已分配部分。 */
	for (i = 0; i < FPGA_FRAME_SLOTS; i++) {
		ret = basic_alloc_buffer(bdev, &bdev->frame[i],
					 FPGA_MAP_BYTES);
		if (ret)
			goto free_dma;
		dev_info(&pdev->dev, "slot %d DMA=%pad bytes=%zu\n",
			 i, &bdev->frame[i].dma_addr, bdev->frame[i].size);
	}
	ret = basic_alloc_buffer(bdev, &bdev->status, FPGA_STATUS_BYTES);
	if (ret)
		goto free_dma;
	return 0;

free_dma:
	basic_free_dma(bdev);
unmap:
	pci_iounmap(pdev, bdev->bar);
	bdev->bar = NULL;
release_region:
	pci_disable_device(pdev);
	pci_release_region(pdev, FPGA_BAR_INDEX);
	return ret;
disable:
	pci_disable_device(pdev);
	return ret;
}

/*
 * 与 init_resources 配对，仅处理成功初始化的一整套资源。
 * 进入前必须排除用户访问，且运行中的设备已经尝试过 STOP。
 * STOP 未确认或 mmap 会话仍打开则隔离 DMA。
 * 本函数不释放对象引用，旧 fd 仍可能持有它。
 */
static void basic_release_resources(struct basic_pcie_dev *bdev)
{
	struct pci_dev *pdev = bdev->pdev;

	pci_clear_master(pdev);
	if (bdev->state != BASIC_IDLE ||
	    (bdev->opened && bdev->io_mode == BASIC_IO_MMAP))
		basic_quarantine(bdev);
	else
		basic_free_dma(bdev);
	/* 不能把 dma_free_coherent 推迟到 remove 返回后的最后一次 close。 */

	pci_iounmap(pdev, bdev->bar);
	bdev->bar = NULL;
	pci_disable_device(pdev);
	pci_release_region(pdev, FPGA_BAR_INDEX);
}

/* ==================== 三、FPGA 命令与启停协议 ==================== */

/* 调用者持有 lock；唤醒因缺帧而睡眠的 read。 */
static void basic_notify_locked(struct basic_pcie_dev *bdev)
{
	WRITE_ONCE(bdev->wake_seq, bdev->wake_seq + 1);
	wake_up_interruptible(&bdev->waitq);
}

static void basic_write_reg32(struct basic_pcie_dev *bdev,
			      unsigned int offset, u32 value)
{
	u8 __iomem *base = bdev->bar;

	writel(value, base + offset);
	/* 本 FPGA 没有 BAR 读 Completion，绝不能用 readl() 做写后刷新。 */
}

/*
 * 每次新的 DMA 会话先预留 STOP cookie，再持有模块引用。
 * 将这两个前置条件放在一起，START 和初始化回显检查共用。
 * 超时重试不调用此函数：必须继续等待原 cookie。
 */
static int basic_prepare_dma(struct basic_pcie_dev *bdev)
{
	if (bdev->cookie_counter == U32_MAX)
		return -EOVERFLOW;
	bdev->stop_cookie = ++bdev->cookie_counter;

	if (!bdev->dma_guard) {
		if (!try_module_get(THIS_MODULE))
			return -ENODEV;
		bdev->dma_guard = true;
	}
	return 0;
}

/* 16 个 LE32 全部匹配才是 ACK；__le32 标注不能替代字节序转换。 */
static bool basic_stop_ack(struct basic_pcie_dev *bdev)
{
	const __le32 *status = bdev->status.cpu_addr;
	int i;

	for (i = 0; i < FPGA_STATUS_BYTES / sizeof(*status); i++) {
		if (le32_to_cpu(READ_ONCE(status[i])) != bdev->stop_cookie)
			return false;
	}

	/* 先确认完成标记，再建立对之前 DMA 写入的读取顺序。 */
	dma_rmb();
	return true;
}

/* 每个 cookie 只提交一次 STOP。超时重试不能再次清状态区。 */
static void basic_submit_stop_locked(struct basic_pcie_dev *bdev)
{
	bdev->state = BASIC_STOPPING;
	bdev->snapshot_ready = false;
	bdev->snapshot_pos = 0;
	memset(bdev->status.cpu_addr, 0, FPGA_STATUS_BYTES);
	dma_wmb();
	basic_write_reg32(bdev, FPGA_REG_STOP, bdev->stop_cookie);
	basic_notify_locked(bdev);
}

/* 持锁等待最多一秒；超时保持 STOPPING 和 DMA 资源，重试只等旧 ACK。 */
static int basic_stop_locked(struct basic_pcie_dev *bdev)
{
	ktime_t deadline;

	if (bdev->state == BASIC_IDLE)
		return 0;
	if (bdev->state == BASIC_QUARANTINED)
		return -EIO;
	if (bdev->state == BASIC_RUNNING)
		basic_submit_stop_locked(bdev);

	deadline = ktime_add_us(ktime_get(), FPGA_STOP_TIMEOUT_US);
	for (;;) {
		if (basic_stop_ack(bdev))
			break;
		if (ktime_compare(ktime_get(), deadline) >= 0) {
			if (basic_stop_ack(bdev))
				break;
			dev_err(&bdev->pdev->dev,
				"STOP timeout, cookie=%#x; DMA memory retained\n",
				bdev->stop_cookie);
			/* 保持 STOPPING、BME 和所有 DMA 分配，允许等待迟到 ACK。 */
			return -ETIMEDOUT;
		}
		usleep_range(500, 1000);
	}

	/* 新 FPGA 只有视频/STOP DMA；确认停止后不再保留配置通道引用。 */
	pci_clear_master(bdev->pdev);
	bdev->state = BASIC_IDLE;
	if (bdev->dma_guard) {
		bdev->dma_guard = false;
		module_put(THIS_MODULE);
	}
	basic_notify_locked(bdev);
	return 0;
}

/* IDLE -> RUNNING；已经 RUNNING 时重复 START 不影响正在接收的帧。 */
static int basic_start_locked(struct basic_pcie_dev *bdev)
{
	int i, ret;

	if (bdev->state == BASIC_RUNNING)
		return 0;
	if (bdev->state != BASIC_IDLE)
		return -EBUSY; /* 先重试 STOP 并收到 ACK，才能重新 START */
	/* STOP 不等于用户已用完帧，不能让重启覆盖仍被 Qt 持有的槽。 */
	for (i = 0; i < FPGA_FRAME_SLOTS; i++) {
		if (bdev->held_token[i])
			return -EBUSY;
	}
	if (bdev->session == U64_MAX)
		return -EOVERFLOW;

	ret = basic_prepare_dma(bdev);
	if (ret)
		return ret;

	/*
	 * 上次 STOP 已确认，且 lock 保证没有线程正在读取 DMA 槽。
	 * START 前清尾标记，防止旧标记被误判；不需要清整幅图像。
	 */
	for (i = 0; i < FPGA_FRAME_SLOTS; i++)
		memset((u8 *)bdev->frame[i].cpu_addr + FPGA_IMAGE_BYTES,
		       0, FPGA_MARKER_BYTES);
	bdev->snapshot_ready = false;
	bdev->snapshot_pos = 0;
	bdev->session++;
	dma_wmb();
	pci_set_master(bdev->pdev);
	bdev->state = BASIC_RUNNING;
	basic_write_reg32(bdev, FPGA_REG_START, 1);
	basic_notify_locked(bdev);
	return 0;
}

/*
 * 资源准备完后，只在 probe 中调用：
 * 登记四槽和状态区地址 -> 做一次 STOP 回显检查 -> 保持 IDLE。
 * 这是协议初始化，不启动视频，也不能证明像素内容正确。
 * 首次绑定仍要求 FPGA 已经处于已知复位后的空闲态。
 */
static int basic_init_protocol(struct basic_pcie_dev *bdev)
{
	int i, ret;

	mutex_lock(&bdev->lock);
	dma_wmb();
	basic_write_reg32(bdev, FPGA_REG_RESET_CONFIG, 1);
	for (i = 0; i < FPGA_FRAME_SLOTS; i++)
		basic_write_reg32(bdev, FPGA_REG_FRAME_ADDR,
				  lower_32_bits(bdev->frame[i].dma_addr));
	basic_write_reg32(bdev, FPGA_REG_STATUS_ADDR,
			  lower_32_bits(bdev->status.dma_addr));

	ret = basic_prepare_dma(bdev);
	if (!ret) {
		pci_set_master(bdev->pdev);
		basic_submit_stop_locked(bdev);
		ret = basic_stop_locked(bdev);
	}
	mutex_unlock(&bdev->lock);
	return ret;
}


/* ==================== 四、单向图像参数提交 ==================== */

/*
 * 调用者持有 bdev->lock：五字和 COMMIT 不能被其他配置穿插。
 * 只在 IDLE/RUNNING 接受提交，不需要配置 DMA、Bus Master 或额外模块引用。
 * reserved 必须为零；MMIO 写入之后没有 copy_to_user 等可能失败的步骤。
 * 返回 0 仅表示写操作已发出，当前 FPGA 没有软件可查询的配置 ACK。
 */
static int basic_cfg_submit(struct basic_pcie_dev *bdev,
                            const struct fpga_image_submit *config)
{
	u32 clean[5];
	int i;

	for (i = 0; i < ARRAY_SIZE(config->reserved); i++) {
		if (config->reserved[i])
			return -EINVAL;
	}
	if (bdev->state != BASIC_IDLE && bdev->state != BASIC_RUNNING)
		return -EBUSY;

	fpga_cfg_sanitize(config->words, clean);
	for (i = 0; i < ARRAY_SIZE(clean); i++)
		basic_write_reg32(bdev, FPGA_REG_IMAGE_WORD0 + 4 * i, clean[i]);
	basic_write_reg32(bdev, FPGA_REG_IMAGE_COMMIT, 1);
	/* 不使用 readl() 刷新：本 FPGA 没有 BAR 读 Completion。 */
	return 0;
}

/* ==================== 五、完整帧确认、复制与归还 ==================== */

/* 校验序号、槽号、重复 token 和全部 16 个 DWORD，再执行 dma_rmb。 */
static bool basic_frame_ready(struct basic_pcie_dev *bdev,
			       unsigned int slot, u32 *token_out)
{
	const __le32 *tail;
	u32 token;
	int i;

	tail = (const __le32 *)((u8 *)bdev->frame[slot].cpu_addr +
			       FPGA_IMAGE_BYTES);
	token = le32_to_cpu(READ_ONCE(tail[0]));
	if (!(token >> 2) || (token & 0x3U) != slot)
		return false;
	if (token == bdev->last_frame_token[slot])
		return false;

	for (i = 1; i < FPGA_MARKER_BYTES / sizeof(*tail); i++) {
		if (le32_to_cpu(READ_ONCE(tail[i])) != token)
			return false;
	}
	dma_rmb();
	*token_out = token;
	return true;
}

/* 最后一个 CPU 消费者用完后：清帧尾 -> dma_wmb -> RELEASE 原 token。 */
static void basic_release_frame_locked(struct basic_pcie_dev *bdev,
					unsigned int slot, u32 token)
{
	bdev->last_frame_token[slot] = token;
	memset((u8 *)bdev->frame[slot].cpu_addr + FPGA_IMAGE_BYTES,
	       0, FPGA_MARKER_BYTES);
	dma_wmb();
	basic_write_reg32(bdev, FPGA_REG_RELEASE, token);
	/* 从这一刻起不得再访问这个槽，直到出现新的有效帧尾。 */
}

/* 只选择序号最早的新帧；由 read 或 GET_FRAME 决定如何使用。 */
static bool basic_find_frame_locked(struct basic_pcie_dev *bdev,
				    unsigned int *slot_out, u32 *token_out)
{
	unsigned int slot, selected = FPGA_FRAME_SLOTS;
	u32 token, best_token = 0, best_sequence = U32_MAX;

	for (slot = 0; slot < FPGA_FRAME_SLOTS; slot++) {
		/* 已交给用户的槽不能再次交付，也不能被当成旧帧归还。 */
		if (bdev->held_token[slot])
			continue;
		if (!basic_frame_ready(bdev, slot, &token))
			continue;
		if ((token >> 2) <= bdev->last_sequence) {
			basic_release_frame_locked(bdev, slot, token);
			continue;
		}
		if ((token >> 2) < best_sequence) {
			best_sequence = token >> 2;
			best_token = token;
			selected = slot;
		}
	}
	if (selected == FPGA_FRAME_SLOTS)
		return false;

	*slot_out = selected;
	*token_out = best_token;
	return true;
}

/*
 * 将“等到可读的一帧”与“复制给用户”分开。
 * 返回 1：已有 snapshot 或选中完整槽；0：已正常停止；负数：错误。
 *
 * 进入和返回时都持有 lock。睡眠期间暂时释放 lock，
 * 让另一个线程的 STOP/remove 能执行；唤醒后重新检查状态和会话。
 * 调用者还应持有 read_lock，防止其他 read/GET 抢走所选帧。
 */
static int basic_wait_frame_locked(struct basic_pcie_dev *bdev,
				   unsigned int *slot, u32 *token)
{
	ktime_t deadline = ktime_add_us(ktime_get(), FPGA_FRAME_TIMEOUT_US);
	u64 session = bdev->session;
	unsigned int wake_seq;
	long waited;
	unsigned int i, held;

	for (;;) {
		if (!bdev->present)
			return -ENODEV;
		if (session != bdev->session)
			return -ECANCELED;
		if (bdev->state == BASIC_IDLE)
			return 0;
		if (bdev->state != BASIC_RUNNING)
			return -EIO;
		if (bdev->io_mode == BASIC_IO_READ && bdev->snapshot_ready)
			return 1;
		if (basic_find_frame_locked(bdev, slot, token))
			return 1;

		held = 0;
		for (i = 0; i < FPGA_FRAME_SLOTS; i++)
			held += !!bdev->held_token[i];
		if (held == FPGA_FRAME_SLOTS)
			return -ENOBUFS; /* 用户必须先归还某一帧 */
		if (ktime_compare(ktime_get(), deadline) >= 0)
			return -ETIMEDOUT;

		wake_seq = bdev->wake_seq;
		mutex_unlock(&bdev->lock);
		/*
		 * FPGA 无中断，因此按 tick 轮询帧尾。
		 * waitq 用于 STOP/remove 提前唤醒，不是 FPGA 帧中断。
		 */
		waited = wait_event_interruptible_timeout(bdev->waitq,
				READ_ONCE(bdev->wake_seq) != wake_seq,
				msecs_to_jiffies(1));
		mutex_lock(&bdev->lock);
		if (waited < 0)
			return waited;
	}
}

/* ==================== 六、用户接口：read / mmap / ioctl ==================== */

static int basic_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct basic_pcie_dev *bdev;
	int ret;

	bdev = container_of(misc, struct basic_pcie_dev, misc);
	if (!(file->f_mode & FMODE_READ))
		return -EACCES;
	if (file->f_flags & O_NONBLOCK)
		return -EOPNOTSUPP;

	mutex_lock(&bdev->lock);
	if (!bdev->present) {
		ret = -ENODEV;
		goto out;
	}
	if (bdev->opened) {
		ret = -EBUSY;
		goto out;
	}
	ret = nonseekable_open(inode, file);
	if (ret)
		goto out;

	kref_get(&bdev->refs);
	bdev->opened = true;
	bdev->io_mode = BASIC_IO_UNSET;
	memset(bdev->held_token, 0, sizeof(bdev->held_token));
	bdev->snapshot_ready = false;
	bdev->snapshot_pos = 0;
	file->private_data = bdev;
out:
	mutex_unlock(&bdev->lock);
	return ret;
}

/*
 * read 主流程：串行化读取 -> 等完整帧 -> 复制当前帧剩余部分。
 * 不跨帧复制；snapshot_pos 保存短读进度，文件位置 ppos 不参与定位。
 */
static ssize_t basic_read(struct file *file, char __user *buf,
			  size_t count, loff_t *ppos)
{
	struct basic_pcie_dev *bdev = file->private_data;
	size_t length, copied;
	unsigned int slot = 0;
	u32 token = 0;
	unsigned long not_copied;
	ssize_t ret;

	if (!count)
		return 0;
	if (file->f_flags & O_NONBLOCK)
		return -EOPNOTSUPP;
	if (mutex_lock_interruptible(&bdev->read_lock))
		return -ERESTARTSYS;

	mutex_lock(&bdev->lock);
	if (!bdev->present) {
		ret = -ENODEV;
		goto out;
	}
	if (bdev->io_mode == BASIC_IO_MMAP) {
		ret = -EBUSY;
		goto out;
	}
	bdev->io_mode = BASIC_IO_READ;
	ret = basic_wait_frame_locked(bdev, &slot, &token);
	if (ret <= 0)
		goto out;

	if (!bdev->snapshot_ready) {
		/* 兼容路径的第一次复制；mmap 路径不会执行这里。 */
		memcpy(bdev->snapshot, bdev->frame[slot].cpu_addr,
		       FPGA_IMAGE_BYTES);
		bdev->last_sequence = token >> 2;
		basic_release_frame_locked(bdev, slot, token);
		bdev->snapshot_pos = 0;
		bdev->snapshot_ready = true;
	}

	/* 第二次复制：snapshot -> 用户缓冲区；返回值是未复制的字节数。 */
	length = min_t(size_t, count, FPGA_IMAGE_BYTES - bdev->snapshot_pos);
	not_copied = copy_to_user(buf,
				 (u8 *)bdev->snapshot + bdev->snapshot_pos,
				 length);
	copied = length - not_copied;
	bdev->snapshot_pos += copied;
	if (bdev->snapshot_pos == FPGA_IMAGE_BYTES) {
		bdev->snapshot_ready = false;
		bdev->snapshot_pos = 0;
	}
	ret = copied ? (ssize_t)copied : -EFAULT;
out:
	mutex_unlock(&bdev->lock);
	mutex_unlock(&bdev->read_lock);
	return ret;
}

/*
 * 新 mmap 入口：offset = slot * map_bytes，length 必须等于 map_bytes。
 * dma_mmap_coherent 按“单个分配”解释 vm_pgoff，所以解析槽号后归零。
 * 不自行设置缓存属性、不覆盖 vm_ops、不替换 vm_file。
 * VMA 对 file 的引用自然保护 .release、对象和模块寿命。
 */
static int basic_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct basic_pcie_dev *bdev = file->private_data;
	unsigned long pages = FPGA_MAP_BYTES >> PAGE_SHIFT;
	unsigned long slot = vma->vm_pgoff / pages;
	int ret;

	if (slot >= FPGA_FRAME_SLOTS || vma->vm_pgoff % pages ||
	    vma->vm_end - vma->vm_start != FPGA_MAP_BYTES)
		return -EINVAL;
	/*
	 * 4.19 对 O_RDONLY + MAP_SHARED 会清 VM_SHARED，但保留 VM_MAYSHARE。
	 * 因此不能只检查 VM_SHARED，否则会误拒绝正常的只读映射。
	 */
	if (!(vma->vm_flags & VM_MAYSHARE) ||
	    !(vma->vm_flags & VM_READ) ||
	    (vma->vm_flags & (VM_WRITE | VM_EXEC)))
		return -EACCES;

	/* mmap 持有 mm 写锁；避免与 copy_to_user 的缺页等待形成 ABBA。 */
	if (!mutex_trylock(&bdev->lock))
		return -EAGAIN;
	if (!bdev->present) {
		ret = -ENODEV;
		goto out;
	}
	if (bdev->io_mode == BASIC_IO_READ || bdev->state != BASIC_IDLE) {
		ret = -EBUSY;
		goto out;
	}

	vma->vm_flags &= ~(VM_MAYWRITE | VM_MAYEXEC);
	vma->vm_flags |= VM_DONTCOPY | VM_DONTEXPAND | VM_DONTDUMP;
	vma->vm_pgoff = 0;
	ret = dma_mmap_coherent(&bdev->pdev->dev, vma,
				bdev->frame[slot].cpu_addr,
				bdev->frame[slot].dma_addr,
				bdev->frame[slot].size);
	if (!ret)
		bdev->io_mode = BASIC_IO_MMAP;
out:
	mutex_unlock(&bdev->lock);
	return ret;
}

/*
 * GET_FRAME 只复制 32 字节元数据，不复制像素，也不发送 RELEASE。
 * copy_to_user 失败时不提交持有记录，用户必须忽略部分输出并重试。
 * read_lock 防止多个 GET 等待者重复交付；STOP/PUT 不取 read_lock。
 */
static int basic_get_frame(struct file *file, void __user *arg)
{
	struct basic_pcie_dev *bdev = file->private_data;
	struct fpga_frame frame = { 0 };
	unsigned int slot = 0;
	u32 token = 0;
	int ret;

	if (file->f_flags & O_NONBLOCK)
		return -EOPNOTSUPP;
	if (mutex_lock_interruptible(&bdev->read_lock))
		return -ERESTARTSYS;
	mutex_lock(&bdev->lock);
	if (!bdev->present) {
		ret = -ENODEV;
		goto out;
	}
	if (bdev->io_mode != BASIC_IO_MMAP) {
		ret = -EINVAL;
		goto out;
	}
	ret = basic_wait_frame_locked(bdev, &slot, &token);
	if (!ret)
		ret = -EPIPE; /* ioctl 的 0 应表示成功，不能拿它表示“停止” */
	if (ret < 0)
		goto out;

	frame.session = bdev->session;
	frame.slot = slot;
	frame.token = token;
	frame.bytes_used = FPGA_IMAGE_BYTES;
	frame.sequence = token >> 2;
	if (copy_to_user(arg, &frame, sizeof(frame))) {
		ret = -EFAULT;
		goto out;
	}

	bdev->held_token[slot] = token;
	bdev->last_frame_token[slot] = token;
	bdev->last_sequence = token >> 2;
	ret = 0;
out:
	mutex_unlock(&bdev->lock);
	mutex_unlock(&bdev->read_lock);
	return ret;
}

/* 用户必须把 GET 返回的 session/slot/token 原样带回，且先结束像素访问。 */
static int basic_put_frame_locked(struct basic_pcie_dev *bdev,
				   const struct fpga_frame *frame)
{
	if (bdev->io_mode != BASIC_IO_MMAP)
		return -EINVAL;
	if (frame->reserved || frame->slot >= FPGA_FRAME_SLOTS ||
	    !(frame->token >> 2) || (frame->token & 3U) != frame->slot)
		return -EINVAL;
	if (frame->session != bdev->session ||
	    bdev->held_token[frame->slot] != frame->token)
		return -ESTALE;

	/*
	 * RUNNING：归还给 FPGA。
	 * STOPPING/IDLE：停止流程将/已回收硬件槽，只结束软件持有；
	 * 不重发 RELEASE，也不在停止途中擅自清 DMA 帧尾。
	 */
	if (bdev->state == BASIC_RUNNING)
		basic_release_frame_locked(bdev, frame->slot, frame->token);
	bdev->held_token[frame->slot] = 0;
	basic_notify_locked(bdev);
	return 0;
}

static long basic_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	struct basic_pcie_dev *bdev = file->private_data;
	void __user *user = (void __user *)arg;
	struct fpga_frame frame;
	struct fpga_info info = { 0 };
	struct fpga_image_submit config;
	struct fpga_caps caps = { 0 };
	int i, ret;

	/* GET 会等待完整帧，它自己按 read_lock -> lock 的顺序取锁。 */
	if (cmd == FPGA_IOC_GET_FRAME)
		return basic_get_frame(file, user);

	switch (cmd) {
	case FPGA_IOC_START:
	case FPGA_IOC_STOP:
	case FPGA_IOC_GET_INFO:
	case FPGA_IOC_GET_CAPS:
		break;
	case FPGA_IOC_SET_IMAGE:
	case FPGA_IOC_GET_IMAGE:
		/* 明确拒绝旧配置协议，不读取参数、不发旧地址、不等待回执。 */
		return -EOPNOTSUPP;
	case FPGA_IOC_SUBMIT_IMAGE:
		if (copy_from_user(&config, user, sizeof(config)))
			return -EFAULT;
		break;
	case FPGA_IOC_PUT_FRAME:
		if (copy_from_user(&frame, user, sizeof(frame)))
			return -EFAULT;
		break;
	default:
		return -ENOTTY;
	}

	if (mutex_lock_interruptible(&bdev->lock))
		return -ERESTARTSYS;
	if (!bdev->present) {
		ret = -ENODEV;
		goto out;
	}
	switch (cmd) {
	case FPGA_IOC_SUBMIT_IMAGE:
		ret = basic_cfg_submit(bdev, &config);
		break;
	case FPGA_IOC_GET_CAPS:
		/* 驱动能力声明，不是读取到的 FPGA 版本或当前硬件状态。 */
		caps.version = FPGA_CAPS_VERSION;
		caps.flags = FPGA_CAP_IMAGE_SUBMIT;
		ret = copy_to_user(user, &caps, sizeof(caps)) ? -EFAULT : 0;
		break;
	case FPGA_IOC_START:
		ret = basic_start_locked(bdev);
		break;
	case FPGA_IOC_STOP:
		ret = basic_stop_locked(bdev);
		break;
	case FPGA_IOC_GET_INFO:
		info.abi_version = FPGA_ABI_VERSION;
		info.width = FPGA_FRAME_WIDTH;
		info.height = FPGA_FRAME_HEIGHT;
		info.stride = FPGA_FRAME_WIDTH * 2;
		info.image_bytes = FPGA_IMAGE_BYTES;
		info.slot_count = FPGA_FRAME_SLOTS;
		info.pixel_format = FPGA_FORMAT_RGB565;
		info.map_bytes = FPGA_MAP_BYTES;
		for (i = 0; i < FPGA_FRAME_SLOTS; i++)
			info.offset[i] = (u64)i * FPGA_MAP_BYTES;
		ret = copy_to_user(user, &info, sizeof(info)) ? -EFAULT : 0;
		break;
	case FPGA_IOC_PUT_FRAME:
		ret = basic_put_frame_locked(bdev, &frame);
		break;
	default:
		ret = -ENOTTY;
		break;
	}
out:
	mutex_unlock(&bdev->lock);
	return ret;
}

#ifdef CONFIG_COMPAT
/* 新 ioctl 带用户指针，32 位进程需要 compat_ptr；结构体布局保持一致。 */
static long basic_compat_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg)
{
	return basic_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif

/* 最后一个 fd/VMA 引用释放时才到这里；mmap 用户必须先显式 STOP。 */
static int basic_close(struct inode *inode, struct file *file)
{
	struct basic_pcie_dev *bdev = file->private_data;
	int ret = 0;

	mutex_lock(&bdev->lock);
	if (bdev->present)
		ret = basic_stop_locked(bdev);
	bdev->snapshot_ready = false;
	bdev->snapshot_pos = 0;
	/* .release 发生时该 file 的所有映射已不再使用，才可撤销持有记录。 */
	memset(bdev->held_token, 0, sizeof(bdev->held_token));
	bdev->opened = false;
	mutex_unlock(&bdev->lock);

	/*
	 * close 不适合报告可重试控制错误。需要确认停止结果的应用应显式
	 * ioctl(STOP) 并检查返回值。若这里超时，可重新 open 后再 STOP；
	 * 模块额外引用和设备初始 kref 继续保护尚未停止的 DMA 资源。
	 */
	if (ret)
		dev_warn(&bdev->pdev->dev, "close: STOP not confirmed: %d\n",
			 ret);
	kref_put(&bdev->refs, basic_free_object);
	return 0;
}

static const struct file_operations basic_fops = {
	.owner = THIS_MODULE,
	.open = basic_open,
	.read = basic_read,
	.mmap = basic_mmap,
	.unlocked_ioctl = basic_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = basic_compat_ioctl,
#endif
	.release = basic_close,
	.llseek = no_llseek,
};


/* ==================== 七、PCI 入口：按阶段初始化，按阶段清理 ==================== */

static int basic_probe(struct pci_dev *pdev,
			const struct pci_device_id *id)
{
	struct basic_pcie_dev *bdev;
	u16 command;
	int ret;

	BUILD_BUG_ON(sizeof(struct fpga_info) != 72);
	BUILD_BUG_ON(sizeof(struct fpga_frame) != 32);
	BUILD_BUG_ON(sizeof(struct fpga_image_config) != 32);
	BUILD_BUG_ON(sizeof(struct fpga_image_status) != 96);

	/* 拒绝通过 new_id 将不符合寄存器协议的设备强行绑定到这个例子。 */
	if (pdev->vendor != FPGA_VENDOR_ID || pdev->device != FPGA_DEVICE_ID)
		return -ENODEV;
	if (basic_is_quarantined(pdev))
		return -EBUSY;

	ret = pci_read_config_word(pdev, PCI_COMMAND, &command);
	if (ret)
		return pcibios_err_to_errno(ret);
	if (command & PCI_COMMAND_MASTER) {
		dev_err(&pdev->dev, "device already bus mastering; reset first\n");
		return -EBUSY;
	}

	bdev = kzalloc(sizeof(*bdev), GFP_KERNEL);
	if (!bdev)
		return -ENOMEM;
	bdev->pdev = pci_dev_get(pdev);
	kref_init(&bdev->refs);
	mutex_init(&bdev->lock);
	mutex_init(&bdev->read_lock);
	init_waitqueue_head(&bdev->waitq);
	INIT_LIST_HEAD(&bdev->quarantine_node);
	bdev->state = BASIC_IDLE;
	/* 首个 STOP cookie 为 0x12345678，便于上板核对字节序。 */
	bdev->cookie_counter = 0x12345677U;

	/* 1. 准备 Linux 资源；失败时内部已经回滚。 */
	ret = basic_init_resources(bdev);
	if (ret)
		goto put_object;

	/* 2. 配置 FPGA 并验证 STOP 回显；成功后保持空闲。 */
	ret = basic_init_protocol(bdev);
	if (ret)
		goto release_resources;

	/* 3. 最后发布用户接口，避免用户看到尚未准备好的设备。 */
	snprintf(bdev->name, sizeof(bdev->name), "fpga_frame_%s",
		 pci_name(pdev));
	bdev->misc.minor = MISC_DYNAMIC_MINOR;
	bdev->misc.name = bdev->name;
	bdev->misc.fops = &basic_fops;
	bdev->misc.parent = &pdev->dev;
	bdev->misc.mode = 0600;
	bdev->present = true;
	pci_set_drvdata(pdev, bdev);
	ret = misc_register(&bdev->misc);
	if (ret) {
		bdev->present = false;
		pci_set_drvdata(pdev, NULL);
		goto release_resources;
	}

	dev_info(&pdev->dev, "/dev/%s ready; use ioctl START to capture\n",
		 bdev->name);
	return 0;

release_resources:
	basic_release_resources(bdev);
put_object:
	if (bdev->state != BASIC_QUARANTINED)
		kref_put(&bdev->refs, basic_free_object);
	return ret;
}

static void basic_remove(struct pci_dev *pdev)
{
	struct basic_pcie_dev *bdev = pci_get_drvdata(pdev);

	/*
	 * 先撤销入口，再取 lock，避免与 misc_open 的内部锁形成反向锁序。
	 * 已打开的 fd 仍持有对象引用，但 present=false 后不能访问硬件。
	 */
	misc_deregister(&bdev->misc);
	mutex_lock(&bdev->lock);
	bdev->present = false;
	basic_notify_locked(bdev);

	/*
	 * STOP 失败仍保留 STOPPING；
	 * release_resources 根据 STOP 状态和 mmap 会话决定释放还是隔离 DMA。
	 */
	basic_stop_locked(bdev);
	basic_release_resources(bdev);
	pci_set_drvdata(pdev, NULL);
	mutex_unlock(&bdev->lock);

	if (bdev->state != BASIC_QUARANTINED)
		kref_put(&bdev->refs, basic_free_object);
}

static const struct pci_device_id basic_ids[] = {
	{ PCI_DEVICE(FPGA_VENDOR_ID, FPGA_DEVICE_ID) },
	{ }
};
MODULE_DEVICE_TABLE(pci, basic_ids);

static struct pci_driver basic_driver = {
	.name = DRIVER_NAME,
	.id_table = basic_ids,
	.probe = basic_probe,
	.remove = basic_remove,
};

module_pci_driver(basic_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("FPGA PCIe video: leased DMA mmap and compatible read");
MODULE_VERSION("7.0");
