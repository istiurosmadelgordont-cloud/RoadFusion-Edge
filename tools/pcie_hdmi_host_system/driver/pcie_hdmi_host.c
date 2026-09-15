/*
 * pcie_hdmi_host.c —— PCIe HDMI 视频采集：主机端（Root Complex 侧）字符设备驱动
 *
 * 目标协议： FPGA 端 PCIE_FIX_20260912（即 fpga/pcie_video 里的 PGL50H HDMI 彩条工程）
 * 运行环境： RK3568 / Debian / Linux 4.19
 * 设备节点： /dev/pcie_hdmi_host（misc 设备，权限 0600）
 * 版本：     arm-fast-v2（2026-09-15）—— 取帧策略改为"优先选最新候选帧"，并加 perf 统计
 *
 * ===========================================================================
 * 一、这套协议的关键约束（决定了下面所有写法）
 * ===========================================================================
 *  FPGA 是 Endpoint，本机是 Root Complex。视频数据由 FPGA 主动以 Memory Write
 *  写进主机内存，主机只负责：下发 4 个缓冲区地址 → 轮询帧尾标志 → 读走一帧。
 *
 *  三条硬约束，缺一不可地影响了实现：
 *
 *  1) **没有 START 寄存器**。四个 DMA 地址提交后，FPGA 自动在下一个 VS 上升沿
 *     装载缓冲区基址、VS 下降沿开始采集。软件无法主动触发某一帧。
 *
 *  2) **没有停止确认（no stop ACK）**。RTL 提供 0x130 CLEAR，但硬件不会回报
 *     "我已经停了"。因此固定延时之后无法确认 FPGA 是否真的排空，本驱动
 *     索性**不写 CLEAR**——一旦开始流式接收就让它一直跑（见第三节）。
 *
 *  3) **没有 BAR 回读能力（no read completion）**。绝对不能 readl() 回读 BAR，
 *     所有"刷新已发出的 posted write"都只能靠读 PCI 配置空间实现。
 *
 *  另外 RTL 也没有中断、没有缓冲区归还机制、没有只读状态寄存器，
 *  所以只能轮询，且无法从硬件得知当前 FPGA 正在写哪个缓冲区。
 *
 * ===========================================================================
 * 二、BAR1 寄存器（32 位写，普通 writel，不做字节序转换）
 * ===========================================================================
 *   0x110  写 DMA 地址低 32 位。连续写 4 次 = buffer0..buffer3，第 4 次提交。
 *   0x130  CLEAR，停止 DMA 并清除四地址配置（本驱动刻意不使用，见第一节）
 *   0x140  图像增强：高亮压制参数（低 8 位有效）
 *   0x150  图像增强：暗部抬升参数（低 8 位有效）
 *   0x160  写任意值 = 清除两项增强参数
 *
 *  FPGA 只保存地址的低 32 位（没有高地址寄存器），所以必须强制 32 位 DMA mask，
 *  并且分配到的地址高 32 位必须为 0。
 *
 * ===========================================================================
 * 三、持续接收模式
 * ===========================================================================
 *  第一次 read() 时才开始：置 Bus Master → 下发 4 个地址 → 此后不再干预。
 *  之后每次 read() 只是等一个新的完整帧并复制出来。
 *
 *  为什么不每次 CLEAR 重来？
 *    - CLEAR 没有完成确认，重配地址存在"上一帧还在写、新地址已下发"的窗口；
 *    - 四缓冲区是 FPGA 侧的轮转帧缓存，本来就设计成持续覆盖，停停开开没有收益。
 *
 *  由此引出一个必须处理的安全问题：**FPGA 会持续写那 4 块内存**。所以：
 *    - 一旦进入 streaming，就用 __module_get(THIS_MODULE) 把模块钉住，
 *      不允许卸载——否则 remove 会释放 FPGA 仍在写的内存，直接损坏系统；
 *    - remove 路径若发现 streaming，**故意不释放** DMA 内存（宁可泄一点，
 *      也不能让硬件写野指针）。要换固件或重新加载，请重启。
 *    - .suppress_bind_attrs = true，禁止从 sysfs 手动 unbind 绕过上述保护。
 *
 * ===========================================================================
 * 四、arm-fast-v2 的取帧策略（与旧版最大的区别）
 * ===========================================================================
 *  旧版按 `next_buffer` 轮转，**第一个**发现"标志完整且未交付"的缓冲区就用，
 *  于是可能交付较旧的帧。新版改成：先把 4 个标志一次性读出来，再从中挑
 *  **帧号最新**的那个。
 *
 *  帧号是 1..255 的循环序列（0 表示标志不完整，无效）。比较新旧用
 *  "相差是否落在半周期内"来判断：
 *
 *      delta = (value + 255 - chosen) % 255      // value 比 chosen 新多少
 *      取 delta ∈ (0, 127] 的作为更新者
 *
 *  **这个判断有前提**：只有两个候选相差不到半个序列时才可靠。如果某个缓冲区
 *  长时间没被写入（丢帧、链路复位），它的帧号会显得"很旧"，此时先后顺序可能
 *  算错。所以：
 *    - 它**不是**可靠的时间戳，也不消除原协议本身的撕裂风险；
 *    - 保留了 `newest_first=0` 来回退到旧的轮转行为，便于对比与排障。
 *
 *  **不要把性能提升等同于完整性提升**——选最新帧只是让交付的帧更"新鲜"，
 *  并不能保证每一帧都完整无撕裂。
 *
 *  运行时可切换：
 *    echo N | sudo tee /sys/module/pcie_hdmi_host/parameters/newest_first   # 轮转
 *    echo Y | sudo tee /sys/module/pcie_hdmi_host/parameters/newest_first   # 选最新
 *
 * ===========================================================================
 * 五、perf 统计
 * ===========================================================================
 *  每约 5 秒（或在超时时）向内核日志输出一行 `perf`，各字段是**该统计窗口内的
 *  累计值**，输出后清零：
 *
 *    frames      成功交付给用户态的帧数
 *    attempts    实际执行图像复制的次数
 *    retries     复制前后标志发生变化（疑似被覆盖）而放弃的次数
 *    timeouts    超时次数
 *    read_us     整个 read() 函数体的累计耗时
 *    dma_copy_us 4 MB DMA 缓冲区 → 内核暂存区 的累计耗时
 *    user_copy_us 内核暂存区 → 用户缓冲区 的累计耗时
 *    markers     四个缓冲区当前的帧号
 *    newest      本窗口内 newest_first 的取值
 *
 *  注意：read_us **已经包含**两次复制的时间，三者不能相加。
 *  耗时数据受调度影响，只作趋势参考。
 *
 * ===========================================================================
 * 六、缓冲区布局（每个缓冲区 BUFFER_BYTES，共 4 个）
 * ===========================================================================
 *   0x000000 ~ 0x3F47FF   1920×1080 RGB565 图像，4,147,200 字节（0x3F4800）
 *   0x3F4800 ~ 0x3F483F   64 字节帧尾标志
 *
 *  标志格式：整 64 字节为**同一个非零字节**（即帧号）。只有标志 TLP 的最后一拍
 *  成功握手后帧号才 +1，0xFF 之后回绕到 0x01。因此"64 字节全相同且非零"
 *  就是一帧写完的可靠判据；若 64 字节不等，说明标志还没写完，不能采信。
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/dma-mapping.h>
#include <linux/sched/signal.h>
#include <linux/jiffies.h>
#include <linux/vmalloc.h>
#include <linux/ktime.h>

/* 单帧图像字节数：1920 × 1080 × 2（RGB565 每像素 2 字节）= 0x3F4800 */
#define FRAME_BYTES (1920U * 1080U * 2U)

/* 单个缓冲区总大小：图像区 + 64 字节标志区，向上取整到页 */
#define BUFFER_BYTES PAGE_ALIGN(FRAME_BYTES + 64U)

/* 串行化所有对设备与全局状态的访问（read/write/probe/remove 共用） */
static DEFINE_MUTEX(video_lock);

static struct pci_dev *video_pdev;	/* 绑定的 PCIe 设备，NULL 表示未绑定 */
static void __iomem *bar;		/* BAR1 映射后的虚拟地址（命令区） */
static void *buffers[4];		/* 4 块一致性缓冲区的 CPU 虚拟地址 */
static dma_addr_t addresses[4];		/* 对应交给 FPGA 的 DMA 总线地址 */

/* BAR1 中 DMA 地址寄存器的偏移：连写 4 次依次填入 buffer0~3 */
#define DMA_CMD_L_ADDR 0x110

static bool streaming;			/* 是否已进入持续接收（地址已下发） */
static u8 delivered[4];			/* 每个缓冲区最近一次已交付给用户的帧号 */
static unsigned int next_buffer;	/* 轮转起点：newest_first=0 时决定扫描顺序 */
static void *frame_copy;		/* 内核侧暂存区，避免持锁太久/撕裂 */

/*
 * 取帧策略开关（可运行时修改）：
 *   true （默认）—— 在候选帧里选帧号最新的那个；
 *   false        —— 退回按 next_buffer 轮转，取第一个命中的。
 * 详见文件头第四节。旧流启动后模块被钉住，但该参数仍可热切换。
 */
static bool newest_first = true;
module_param(newest_first, bool, 0644);
MODULE_PARM_DESC(newest_first, "Prefer newest candidate using 1..255 marker order; false uses round robin");

/* ---- perf 统计（窗口累计，输出后清零）---- */
static unsigned long stats_next;	/* 下次输出 perf 的时间点 */
static u64 stats_copy_ns, stats_user_ns, stats_total_ns;	/* 两段复制 / 总耗时 */
static unsigned int stats_frames, stats_retries, stats_attempts, stats_timeouts;


/*
 * 读一次 PCI 配置空间（PCI_COMMAND）。
 *
 * FPGA 没有实现 BAR read completion，不能靠 readl() 回读 BAR 来把之前的
 * posted memory write 顶出去；读配置空间同样能让前面的写真正到达链路对端，
 * 因此这里用它充当 flush。开销极小，但绝对安全。
 */
static void flush_commands(struct pci_dev *pdev)
{
    u16 command;
    pci_read_config_word(pdev, PCI_COMMAND, &command);
}

/*
 * 关闭 DMA：清 Bus Master 位并刷新。
 *
 * 注意这里**只**清总线主控，不写 BAR 的 CLEAR 寄存器。原因同文件头第一节：
 * RTL 无法确认停止完成，写 CLEAR 后我们既不能确保它已停，也无法重新安全下发
 * 地址。所以停止动作只在卸载/关机/异常移除时执行，且之后不再重新启动流。
 * 也就是说：流一旦开始，这个诊断模块就被"钉住"，直到重启。
 */
static void disable_dma(struct pci_dev *pdev)
{
    pci_clear_master(pdev);
    flush_commands(pdev);
}

/*
 * 读取第 index 个缓冲区的帧尾标志。
 *
 * 返回 0 表示"标志不完整或为 0"，即该缓冲区当前没有一帧完整的数据；
 * 返回非 0 即帧号。
 *
 * 判据：64 字节必须两两相同且整体非零。只用 READ_ONCE 逐字节读，
 * 避免编译器把循环优化掉或读出半更新的值。
 */
static u8 marker(int index)
{
    u8 *p = (u8 *)buffers[index] + FRAME_BYTES;
    u8 value = READ_ONCE(p[0]);
    int j;
    if (!value)
        return 0;
    for (j = 1; j < 64; j++)
        if (READ_ONCE(p[j]) != value)
            return 0;
    return value;
}

/*
 * read()：取一帧完整图像交给用户态。
 *
 * 语义是"一次 read = 一帧完整图像"，用户态绝不能把多次 read 的结果拼起来当
 * 一帧（见 frames.py 的注释）。
 *
 * 流程：
 *   首次调用 → 清空缓冲、开 Bus Master、下发 4 个地址、钉住模块、进入 streaming
 *   每次都 → 在 5 秒窗口内轮询：先一次性读出 4 个标志，按 newest_first 选中
 *            一个候选帧，复核标志未变（尽力检测拷贝期间被覆盖）后 copy_to_user
 */
static ssize_t video_read(struct file *file, char __user *out,
                          size_t count, loff_t *pos)
{
    int i, slot, selected;
    u8 observed[4], chosen;
    u64 started = 0, tick;
    bool measure = false;
    unsigned long deadline;
    ssize_t ret;
    u8 value;
    u16 command;

    /* 用户给的缓冲区至少放得下一帧，否则直接拒绝 */
    if (count < FRAME_BYTES)
        return -EINVAL;

    /* 用可中断加锁：卡在等待时 Ctrl-C 也能退出来 */
    if (mutex_lock_interruptible(&video_lock))
        return -ERESTARTSYS;

    if (!video_pdev) {
        ret = -ENODEV;
        goto unlock;
    }

    /* ---- 首次 read：启动持续接收 ---- */
    if (!streaming) {
        /* 清空 4 个缓冲区（尤其是末尾 64 字节标志区），
         * 这样"标志非零"才真正代表 FPGA 写过东西 */
        for (i = 0; i < 4; i++)
            memset(buffers[i], 0, BUFFER_BYTES);
        dma_wmb();

        /* 允许 FPGA 发起 Memory Write */
        pci_set_master(video_pdev);

        /* 复核配置空间：Memory Space 与 Bus Master 都必须已使能，
         * 否则后面的地址下发没有意义 */
        ret = pci_read_config_word(video_pdev, PCI_COMMAND, &command);
        if (ret || (command & (PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY)) !=
                   (PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY)) {
            disable_dma(video_pdev);
            ret = -EIO;
            goto unlock;
        }

        /*
         * 钉住模块计数，然后再公布地址。
         * 顺序很重要：一旦地址发出去，FPGA 随时可能开始写这 4 块内存，
         * 此时若允许卸载，remove 会 dma_free_coherent 掉 FPGA 仍在写的页，
         * 造成难以定位的内存损坏。宁可用户态拿到 EBUSY，也不能冒这个险。
         */
        __module_get(THIS_MODULE);
        streaming = true;

        /* 连续 4 次写 0x110，第 4 次由硬件提交整套配置 */
        for (i = 0; i < 4; i++) {
            writel(lower_32_bits(addresses[i]), bar + DMA_CMD_L_ADDR);
            dev_info(&video_pdev->dev, "stream address%d=%pad offset=0x110\n",
                     i, &addresses[i]);
        }
        flush_commands(video_pdev);
        dev_info(&video_pdev->dev, "stream started; no CLEAR; buffers retained until reboot\n");
    }

    /* 从这里开始计时：perf 的 read_us 覆盖整个等待+复制过程 */
    started = ktime_get_ns();
    measure = true;

    /*
     * 轮询等待一个新帧。
     * 硬件无 START 寄存器：地址下发后要等 FPGA 走到下一个完整帧边界才会出数据，
     * 因此超时不能设太短；文档建议至少 100ms，这里给 5 秒余量。
     */
    deadline = jiffies + msecs_to_jiffies(5000);
    ret = -ETIMEDOUT;
    do {
        /* 允许被信号打断，避免用户态 Ctrl-C 后内核线程卡住 */
        if (signal_pending(current)) {
            ret = -ERESTARTSYS;
            break;
        }

        /*
         * 先一次性把 4 个标志都读出来再挑，避免边读边比导致拿到不一致的快照。
         * 这里不持任何锁去读 FPGA 内存，读到的值随时可能变化，只做启发式判断。
         */
        selected = -1;
        chosen = 0;
        for (i = 0; i < 4; i++)
            observed[i] = marker(i);
        for (i = 0; i < 4; i++) {
            unsigned int delta;
            slot = (next_buffer + i) % 4;
            value = observed[slot];
            /* 标志不完整，或这一帧号已经交付过 → 不是候选 */
            if (!value || value == delivered[slot])
                continue;
            /* Heuristic for marker sequence 1..255 (zero is invalid).
             * Valid only for candidates less than half a sequence apart.
             * Stale/aborted buffers make age ambiguous; retain RR option. */
            delta = (value + 255U - chosen) % 255U;
            /*
             * newest_first=1 时，只有 delta 落在半个周期 (0,127] 内才认为
             * value 比当前 chosen 更新——超过半周期说明两者跨度太大，
             * 无法判断先后，此时保留先扫到的那个（即轮转语义）。
             * newest_first=0 时条件恒假，等价于"取第一个候选"。
             */
            if (selected < 0 || (newest_first && delta > 0 && delta <= 127)) {
                selected = slot;
                chosen = value;
            }
        }

        if (selected >= 0) {
            slot = selected;
            value = chosen;

            /* 统计第一次复制（DMA 缓冲区 → 内核暂存区）的耗时 */
            tick = ktime_get_ns();
            stats_attempts++;
            dma_rmb();
            memcpy(frame_copy, buffers[slot], FRAME_BYTES);
            dma_rmb();
            stats_copy_ns += ktime_get_ns() - tick;

            /* Detect completed overwrites during copying. The legacy RTL
             * does NOT invalidate the marker at write start, so this check
             * cannot exclude every torn frame. No ownership guarantee. */
            if (marker(slot) != value) {
                /* 复制期间标志变了 → 大概率被 FPGA 覆盖，本轮数据作废重来 */
                stats_retries++;
                cond_resched();		/* 连续重试时让出 CPU，避免独占 */
                continue;
            }

            /* 统计第二次复制（内核暂存区 → 用户缓冲区）的耗时 */
            tick = ktime_get_ns();
            ret = copy_to_user(out, frame_copy, FRAME_BYTES) ?
                  -EFAULT : FRAME_BYTES;
            stats_user_ns += ktime_get_ns() - tick;
            if (ret > 0) {
                stats_frames++;
                delivered[slot] = value;
                next_buffer = (slot + 1) % 4;	/* 下次从下一块开始找 */
            }
            goto unlock;
        }
        /* 让出 CPU；帧率远低于此，不会漏帧 */
        usleep_range(1000, 2000);
    } while (time_before(jiffies, deadline));

    if (ret == -ETIMEDOUT)
        dev_warn(&video_pdev->dev, "stream: no new frame in 5s; addresses not resent\n");
unlock:
    /* 累加本次 read 的总耗时，并按窗口输出 perf */
    if (measure) {
        stats_total_ns += ktime_get_ns() - started;
        if (ret == -ETIMEDOUT)
            stats_timeouts++;
        /* 每约 5 秒输出一次；超时则立即输出，便于快速定位卡在哪 */
        if (time_after_eq(jiffies, stats_next) || ret == -ETIMEDOUT) {
            dev_info(&video_pdev->dev,
                     "perf frames=%u attempts=%u retries=%u timeouts=%u read_us=%llu dma_copy_us=%llu user_copy_us=%llu markers=%u,%u,%u,%u newest=%u\n",
                     stats_frames, stats_attempts, stats_retries, stats_timeouts,
                     (unsigned long long)(stats_total_ns / 1000),
                     (unsigned long long)(stats_copy_ns / 1000),
                     (unsigned long long)(stats_user_ns / 1000),
                     marker(0), marker(1), marker(2), marker(3), newest_first);
            stats_next = jiffies + msecs_to_jiffies(5000);
            /* 窗口清零：下一行 perf 是新的统计区间，不是累计总量 */
            stats_frames = stats_attempts = stats_retries = stats_timeouts = 0;
            stats_total_ns = stats_copy_ns = stats_user_ns = 0;
        }
    }
    mutex_unlock(&video_lock);
    return ret;
}

/*
 * write()：下发图像增强参数。
 *
 * 用户态必须一次写入正好 8 字节：两个小端 u32 —— <偏移, 值>。
 * 只开放 0x140 / 0x150 / 0x160 三个增强寄存器，
 * **绝不**通过 write 暴露 0x110（地址）或 0x130（CLEAR），
 * 否则用户态一个误写就能破坏正在运行的地址环、甚至让 FPGA 写到错误内存。
 */
static ssize_t video_write(struct file *file, const char __user *in,
                           size_t count, loff_t *pos)
{
    __le32 words[2];
    u32 offset, value;
    u16 command;
    ssize_t ret;

    if (count != sizeof(words)) {
        pr_warn("pcie_hdmi_host: control-v1 rejected count=%zu expected=8\n", count);
        return -EINVAL;
    }
    if (copy_from_user(words, in, sizeof(words)))
        return -EFAULT;

    /* 显式做小端转换：协议是 PC 侧定义的小端，不依赖主机字节序 */
    offset = le32_to_cpu(words[0]);
    value = le32_to_cpu(words[1]);
    pr_info("pcie_hdmi_host: control-v1 write offset=0x%x value=%u\n", offset, value);

    /* 白名单校验：只有三个增强寄存器，且参数 0~255（0x160 必须为 0） */
    if ((offset != 0x140 && offset != 0x150 && offset != 0x160) ||
        value > 255 || (offset == 0x160 && value != 0))
        return -EINVAL;

    if (mutex_lock_interruptible(&video_lock))
        return -ERESTARTSYS;
    if (!video_pdev) {
        ret = -ENODEV;
        goto out;
    }

    /* 写入前确认 Memory Space 仍处于使能状态 */
    ret = pci_read_config_word(video_pdev, PCI_COMMAND, &command);
    if (ret || !(command & PCI_COMMAND_MEMORY)) {
        ret = -EIO;
        goto out;
    }

    writel(value, bar + offset);
    dev_info(&video_pdev->dev, "control-v1 BAR write issued offset=0x%x value=%u\n", offset, value);
    flush_commands(video_pdev);

    /* 已发出 posted write；FPGA 没有任何回读/应答手段，故只能报"已提交" */
    ret = sizeof(words); /* Posted write issued; no FPGA acknowledgement. */
out:
    mutex_unlock(&video_lock);
    return ret;
}

static const struct file_operations video_fops = {
    .owner = THIS_MODULE,
    .read = video_read,
    .write = video_write,
    .llseek = no_llseek,	/* 这是流式设备，不支持文件偏移 */
};

static struct miscdevice video_misc = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "pcie_hdmi_host",	/* → /dev/pcie_hdmi_host */
    .fops = &video_fops,
    .mode = 0600,		/* 仅 root 可读写 */
};

/*
 * probe()：绑定设备并预分配全部资源。
 *
 * 设计取舍：4 块一致性缓冲区和 frame_copy 都在 probe 时一次性分配、
 * 直到 remove 才释放，中途不再分配/释放。这样 read 的热路径里没有任何
 * 可能失败的内存操作，也不会有"边跑边分配导致地址变化"的风险。
 */
static int video_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    int ret, i;

    mutex_lock(&video_lock);

    /* 单实例：已绑定或已在流式接收时拒绝第二个设备 */
    if (video_pdev || streaming) {
        ret = -EBUSY;
        goto unlock;
    }

    ret = pci_enable_device_mem(pdev);
    if (ret)
        goto unlock;

    /* 初始不持有总线主控，等真正要开始接收时再打开 */
    pci_clear_master(pdev);

    /* FPGA 只保存地址低 32 位，必须强制 32 位且一致性 DMA */
    ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
    if (ret)
        goto disable;

    /* BAR1（资源索引 1）必须是内存空间且覆盖到 0x160 才够用 */
    if (!(pci_resource_flags(pdev, 1) & IORESOURCE_MEM) ||
        pci_resource_len(pdev, 1) < 0x164) {
        ret = -ENODEV;
        goto disable;
    }

    ret = pci_request_region(pdev, 1, "pcie_hdmi_host");
    if (ret)
        goto disable;

    bar = pci_iomap(pdev, 1, 0);
    if (!bar) {
        ret = -ENOMEM;
        goto region;
    }

    /* 内核侧整帧暂存区：read 时先复制到这里，再 copy_to_user */
    frame_copy = vmalloc(FRAME_BYTES);
    if (!frame_copy) {
        ret = -ENOMEM;
        goto free_buffers;
    }

    for (i = 0; i < 4; i++) {
        buffers[i] = dma_alloc_coherent(&pdev->dev, BUFFER_BYTES,
                                       &addresses[i], GFP_KERNEL);
        if (!buffers[i]) {
            ret = -ENOMEM;
            goto free_buffers;
        }
        /* 地址必须非零、高 32 位为 0、64 字节对齐——否则 FPGA 侧无法正确解码 */
        if (!addresses[i] || upper_32_bits(addresses[i]) ||
            (lower_32_bits(addresses[i]) & 63)) {
            ret = -ERANGE;
            goto free_buffers;
        }
        dev_info(&pdev->dev, "buffer%d DMA=%pad bytes=%lu\n", i,
                 &addresses[i], (unsigned long)BUFFER_BYTES);
    }

    dev_info(&pdev->dev, "control-v1 driver ready: enhancement write API=8 bytes\n");
    video_pdev = pdev;

    /* 注册字符设备；失败则回滚到错误处理分支 */
    ret = misc_register(&video_misc);
    if (!ret)
        goto unlock;
    video_pdev = NULL;

free_buffers:
    vfree(frame_copy);
    frame_copy = NULL;
    for (i = 0; i < 4; i++) {
        if (buffers[i])
            dma_free_coherent(&pdev->dev, BUFFER_BYTES, buffers[i], addresses[i]);
        buffers[i] = NULL;
    }
    pci_iounmap(pdev, bar);
region:
    pci_release_region(pdev, 1);
disable:
    pci_disable_device(pdev);
unlock:
    mutex_unlock(&video_lock);
    return ret;
}

/*
 * remove()：正常卸载或设备移除。
 *
 * 关键分支：如果已经 streaming，**不释放** DMA 内存。
 * 硬件没有任何"我已停止/我写完了"的反馈，释放就等于把仍可能被 FPGA
 * 写入的物理页还给系统，后果是随机的内存破坏。宁可保留少量内存，
 * 也要保证不出现野写。要恢复干净状态请重启机器。
 */
static void video_remove(struct pci_dev *pdev)
{
    int i;

    misc_deregister(&video_misc);
    mutex_lock(&video_lock);
    disable_dma(pdev);
    video_pdev = NULL;
    vfree(frame_copy);
    frame_copy = NULL;
    for (i = 0; i < 4; i++) {
        /* Unexpected device removal: retain DMA memory because this RTL
         * supplies no drain acknowledgement. Normal unload is pinned. */
        if (!streaming)
            dma_free_coherent(&pdev->dev, BUFFER_BYTES, buffers[i], addresses[i]);
        buffers[i] = NULL;
    }
    pci_iounmap(pdev, bar);
    pci_release_region(pdev, 1);
    pci_disable_device(pdev);
    mutex_unlock(&video_lock);
}

/*
 * shutdown()：关机/重启时被调用。
 * 只做最保守的动作——关掉总线主控，尽量让 FPGA 在下电前停止发起写。
 */
static void video_shutdown(struct pci_dev *pdev)
{
    disable_dma(pdev);
}

/* 匹配紫光 PCIe IP 的默认 ID；必须与 FPGA 端 ipcore 配置一致 */
static const struct pci_device_id video_ids[] = {
    { PCI_DEVICE(0x0755, 0x0755) }, { 0, }
};
MODULE_DEVICE_TABLE(pci, video_ids);

static struct pci_driver video_driver = {
    .name = "pcie_hdmi_host",
    .id_table = video_ids,
    .probe = video_probe,
    .remove = video_remove,		/* 注意：pci_driver.remove 返回类型是 void */
    .shutdown = video_shutdown,
    /* 禁止通过 /sys/bus/pci/drivers/.../unbind 手动解绑：
     * 流式接收中解绑会绕过 remove 的保护逻辑，直接释放 FPGA 仍在写的内存 */
    .driver = { .suppress_bind_attrs = true },
};

module_pci_driver(video_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PCIE_FIX_20260912 RGB565 continuous diagnostic receiver");

MODULE_VERSION("2026.09.15-arm-fast-v2");
