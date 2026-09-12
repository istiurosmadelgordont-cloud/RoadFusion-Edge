# ARM 端 PCIe 彩条视频传输操作说明（修改版 RTL）

## 1. 适用版本

本文只适用于带有以下源码标记的 FPGA 版本：

```text
PCIE_FIX_20260912
```

该版本已经修正 BAR 命令背靠背接收、发送 AXIS 背压、帧边界地址竞争和帧尾计数方式。旧版文档中“每次 BAR 写之间强制延时”和“帧尾为 N/N+1/N+2/N+3”不再适用。

FPGA 是 PCIe Endpoint，ARM 是 Root Complex。控制方向和视频方向分别是：

```text
ARM → BAR1 MWr → FPGA 控制寄存器
FPGA → PCIe MWr → ARM DMA 一致性缓冲区
```

视频数据不会存放在 BAR1 空间。BAR1 只负责下发 DMA 地址和控制命令。

## 2. 固定数据格式

| 项目 | 数值 |
|---|---:|
| 分辨率 | 1920 × 1080 |
| 像素格式 | RGB565 |
| 单像素 | 2 字节 |
| 图像区大小 | 4,147,200 字节，`0x3F4800` |
| 单个 MWr payload | 64 字节 |
| 图像 MWr 数量 | 64,800 |
| 帧尾标志大小 | 64 字节 |
| 单缓冲区最低大小 | 4,147,264 字节，`0x3F4840` |
| 缓冲区数量 | 4 |
| FPGA DMA 地址宽度 | 32 bit |

每个缓冲区布局：

```text
0x000000 ～ 0x3F47FF：1920×1080 RGB565 图像
0x3F4800 ～ 0x3F483F：64 字节帧尾标志
```

建议驱动分配时向页大小取整，但传给 FPGA 的仍是该缓冲区首地址。

## 3. BAR1 命令

| BAR1 偏移 | ARM 写入内容 | FPGA 行为 |
|---:|---|---|
| `0x110` | DMA 地址低 32 位 | 连续 4 次分别写入 buffer0～buffer3 |
| `0x130` | 任意 32 位值 | 停止 DMA并清除四地址配置 |
| `0x140` | 参数低 8 位 | 高亮压低参数；当前顶层未接实际图像处理 |
| `0x150` | 参数低 8 位 | 暗部抬升参数；当前顶层未接实际图像处理 |
| `0x160` | 任意 32 位值 | 清除图像增强参数 |

地址配置规则：

```text
第1次 writel(addr0, BAR1+0x110) → dma_addr0
第2次 writel(addr1, BAR1+0x110) → dma_addr1
第3次 writel(addr2, BAR1+0x110) → dma_addr2
第4次 writel(addr3, BAR1+0x110) → dma_addr3，并提交配置
```

配置提交后，额外的 `0x110` 写入会被硬件忽略。必须先写 `0x130`，才能重新写一组地址。

## 4. Linux PCI 驱动初始化顺序

建议严格按以下顺序执行：

1. 使能 PCIe Memory Space。
2. 申请并映射 BAR1。
3. 设置 32 bit coherent DMA mask。
4. 分配 4 个 DMA 一致性缓冲区。
5. 检查四个 DMA/IOVA 地址高 32 位均为 0，并至少 64 字节对齐。
6. 清零缓冲区，尤其是末尾 64 字节标志区。
7. `pci_set_master()` 允许 FPGA 发起 MWr。
8. 向 `BAR1+0x130` 写 CLEAR，并刷新 posted write。
9. 对 `BAR1+0x110` 连续写入 4 个地址低 32 位。
10. 再次刷新 posted write。
11. 启动内核轮询线程或用户态等待队列，检测 4 个缓冲区的帧尾标志。

### 4.1 BAR1 映射

```c
#define REG_DMA_ADDR   0x110
#define REG_DMA_CLEAR  0x130

void __iomem *bar1;

ret = pci_enable_device_mem(pdev);
if (ret)
    return ret;

ret = pci_request_region(pdev, 1, "pgl50h-video");
if (ret)
    goto err_disable;

if (pci_resource_len(pdev, 1) < 0x164) {
    ret = -ENODEV;
    goto err_release_bar;
}

bar1 = pci_iomap(pdev, 1, 0);
if (!bar1) {
    ret = -ENOMEM;
    goto err_release_bar;
}
```

BAR 号必须以实际 `lspci -vv` 和驱动资源为准。本文按当前 FPGA 协议使用 BAR1，即 Linux 资源索引 1。

### 4.2 强制 32 bit DMA

当前 FPGA 只保存和发送目标地址低 32 位，没有 `DMA_CMD_H_ADDR`。驱动必须：

```c
ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
if (ret) {
    dev_err(&pdev->dev, "32-bit coherent DMA is unavailable\n");
    goto err_iounmap;
}
```

不要把 CPU 虚拟地址或物理地址直接写给 FPGA。必须写 Linux DMA API 返回的 `dma_addr_t`。

## 5. 四缓冲区分配

```c
#define VIDEO_WIDTH       1920U
#define VIDEO_HEIGHT      1080U
#define BYTES_PER_PIXEL   2U
#define FRAME_BYTES       (VIDEO_WIDTH * VIDEO_HEIGHT * BYTES_PER_PIXEL)
#define MARKER_BYTES      64U
#define DMA_BUFFER_BYTES  (FRAME_BYTES + MARKER_BYTES) /* 0x3F4840 */
#define DMA_BUFFER_COUNT  4

struct video_dma_buffer {
    void       *cpu_addr;
    dma_addr_t  dma_addr;
    u8          last_marker;
};

struct video_dma_buffer buf[DMA_BUFFER_COUNT];

for (i = 0; i < DMA_BUFFER_COUNT; i++) {
    buf[i].cpu_addr = dma_alloc_coherent(&pdev->dev,
                                          DMA_BUFFER_BYTES,
                                          &buf[i].dma_addr,
                                          GFP_KERNEL);
    if (!buf[i].cpu_addr) {
        ret = -ENOMEM;
        goto err_free_buffers;
    }

    if (upper_32_bits(buf[i].dma_addr) != 0 ||
        (lower_32_bits(buf[i].dma_addr) & 0x3f) != 0) {
        dev_err(&pdev->dev, "unsupported DMA address %pad\n",
                &buf[i].dma_addr);
        ret = -ERANGE;
        goto err_free_buffers;
    }

    memset(buf[i].cpu_addr, 0, DMA_BUFFER_BYTES);
    buf[i].last_marker = 0;
}
```

`dma_alloc_coherent()` 通常返回页对齐地址，因此天然满足 64 字节对齐，但驱动仍应检查。

## 6. 向 FPGA 提交地址

### 6.1 刷新 posted write

FPGA 当前没有实现 BAR read completion，所以不要用 `readl(BAR1+offset)` 做回读刷新。可以使用 PCI Configuration Space 读取迫使前面的 posted write 完成：

```c
static void pgl50h_flush_posted_writes(struct pci_dev *pdev)
{
    u16 command;

    pci_read_config_word(pdev, PCI_COMMAND, &command);
}
```

### 6.2 配置函数

```c
static void pgl50h_program_buffers(struct pgl50h_dev *d)
{
    int i;

    /* Clear a possible configuration left by a previous driver instance. */
    writel(0, d->bar1 + REG_DMA_CLEAR);
    pgl50h_flush_posted_writes(d->pdev);

    for (i = 0; i < DMA_BUFFER_COUNT; i++)
        writel(lower_32_bits(d->buf[i].dma_addr),
               d->bar1 + REG_DMA_ADDR);

    /* Keep CPU/MMIO ordering explicit before the polling worker starts. */
    wmb();
    pgl50h_flush_posted_writes(d->pdev);
}
```

修改版 RX 状态机支持背靠背 TLP，因此四次 `writel()` 之间不需要 `udelay()` 或 `usleep_range()`。PCIe 同一路径的 posted Memory Write 保序，硬件按收到顺序填充四地址。

地址写入不需要 `cpu_to_be32()`、`swab32()` 或手工交换字节；使用普通 `writel()` 即可。

## 7. 自动启动行为

硬件没有 START 寄存器。四地址提交后：

1. FPGA 等待下一个 VS 上升沿；
2. 在该边沿选择一个 DMA 缓冲区基址；
3. 随后的 VS 下降沿打开本帧采集；
4. FIFO 每积累够 64 字节，FPGA 就向 ARM 发送一个 MWr；
5. 发送完 4,147,200 字节图像后，再写 64 字节帧尾标志；
6. 下一帧切换到下一个缓冲区。

如果第四个地址刚好在 VS 上升沿之后到达，硬件会等待下一次完整帧边界，绝不会用地址 0 启动。因此软件在配置后最多允许约两个帧周期再判断“没有数据”；60 fps 时建议启动超时不少于 100 ms。

## 8. 帧尾标志检测

### 8.1 修改后的标志格式

每个完整帧的 64 字节标志区全部写为同一个非零值：

```text
01 01 01 ... 01   // 64 bytes
02 02 02 ... 02
...
FF FF FF ... FF
01 01 01 ... 01
```

只有 64 字节标志 TLP 的最后一个 AXIS beat 成功握手后，FPGA 才把帧号加 1。

### 8.2 内核轮询示例

```c
static bool marker_is_complete(const u8 *marker, u8 *value)
{
    u8 v = READ_ONCE(marker[0]);
    int i;

    if (v == 0)
        return false;

    for (i = 1; i < MARKER_BYTES; i++) {
        if (READ_ONCE(marker[i]) != v)
            return false;
    }

    *value = v;
    return true;
}

static int find_completed_buffer(struct pgl50h_dev *d)
{
    int i;

    for (i = 0; i < DMA_BUFFER_COUNT; i++) {
        u8 *marker = (u8 *)d->buf[i].cpu_addr + FRAME_BYTES;
        u8 value;

        if (!marker_is_complete(marker, &value))
            continue;

        if (value == d->buf[i].last_marker)
            continue;

        /* PCIe writes to image data precede its marker write. Ensure CPU image
         * reads happen after observing the completed marker. */
        dma_rmb();

        d->buf[i].last_marker = value;
        return i;
    }

    return -EAGAIN;
}
```

驱动应该轮询所有 4 个缓冲区，而不是只假定“下一帧必定是当前索引+1”。如果某一帧因 FIFO 满、链路复位或未能在 VS 前发完而被放弃，该缓冲区标志不会更新，硬件可能在后续帧继续轮转。

轮询周期建议 1～5 ms。帧号 255 帧后回绕，所以不能长时间停止轮询后仅靠 8 位帧号推算丢了多少帧；应同时记录软件时间戳和已交付帧计数。

## 9. 向用户态交付图像

当前 FPGA 没有“buffer busy/归还”寄存器，四缓冲区会按视频节奏持续循环覆盖。因此驱动不能真正阻止 FPGA 覆盖用户仍在读取的缓冲区。

两种实现方式：

### 9.1 安全方式：内核复制

检测到完整标志后，将 `FRAME_BYTES` 拷贝到 V4L2 `vb2` 缓冲区或普通字符设备队列，再立即允许 FPGA 环继续使用原缓冲区。

优点是不会发生用户态持有过久导致撕裂；缺点是每帧复制约 4 MB。

### 9.2 零拷贝方式：mmap DMA 缓冲区

将四个 coherent buffer 映射给用户态，驱动返回：

```text
buffer_index
frame_marker
timestamp
width=1920
height=1080
stride=3840
format=RGB565
```

用户必须在约 4 帧时间内处理完，即 60 fps 下约 66.7 ms；否则同一缓冲区可能再次被 FPGA 覆盖。应用读取前后可再次比较标志值，若变化则丢弃该次图像。

## 10. RGB565 显示检查

理论上每行 8 条彩条，每条 240 像素：

```text
白 FFFF → 黄 FFE0 → 青 07FF → 绿 07E0
→ 品红 F81F → 红 F800 → 蓝 001F → 黑 0000
```

ARM CPU 通常是小端。若应用把缓冲区解释为 `uint16_t`，首像素应读到 `0xFFFF`，红条应为 `0xF800`。如果颜色通道正确但每个 16 位像素字节颠倒，应先检查用户态图像格式设置，不要直接修改 DMA 地址或 BAR 写入字节序。

## 11. 停止、卸载和重新配置

正常停止建议：

```c
static void pgl50h_stop(struct pgl50h_dev *d)
{
    writel(0, d->bar1 + REG_DMA_CLEAR);
    wmb();
    pgl50h_flush_posted_writes(d->pdev);

    /* No status/readback register exists. Allow an already presented 64-byte
     * TLP to finish and confirm markers are no longer changing. */
    msleep(40);

    pci_clear_master(d->pdev);
}
```

然后再执行：

1. 停止轮询线程和唤醒等待者；
2. 解除用户态映射；
3. `dma_free_coherent()` 释放四个缓冲区；
4. `pci_iounmap()`；
5. `pci_release_region()`；
6. `pci_disable_device()`。

不要先释放 DMA 内存再发 CLEAR，否则 FPGA 可能向已经归还给系统的页面继续写入。

如需重新配置四地址，必须完整执行：

```text
CLEAR → posted-write flush → 写4个新地址 → posted-write flush
```

## 12. 故障定位

| 现象 | 优先检查 |
|---|---|
| BAR 写入后四个地址没有配置 | 是否写 BAR1；是否为 32 bit `writel`；PCIe IP 的 `TUSER[5:4]` 是否为 1 |
| 完全没有 DMA | `pci_set_master()`；32 bit DMA mask；四地址是否非零；链路是否 up |
| 第一次启动晚一帧 | 属正常帧边界保护；等待 100 ms 再判超时 |
| 标志始终为 0 | 图像包没有完成；检查 FIFO 水位、`TREADY`、带宽和 VS 是否过早到来 |
| 标志区不是 64 字节同值 | FPGA/ARM 文档版本不一致，或标志 TLP尚未完整可见 |
| 图像缺行但标志更新 | 检查 FIFO 读延迟、输入 `de` 像素计数和硬件逻辑分析信号 |
| 图像颜色字节颠倒 | 检查用户态 RGB565 解释和 CPU 端字节序 |
| 每约 4 帧图像被撕裂 | 用户持有零拷贝缓冲区太久；改为复制模式或缩短处理时间 |
| 卸载后系统内存损坏 | CLEAR 未刷新/未等待就释放 DMA buffer，或先清除了 Bus Master |

## 13. 当前仍存在的软件限制

- 没有硬件中断，ARM 必须轮询帧尾标志。
- 没有状态寄存器或 BAR read completion，不能读取 FPGA 的配置状态、当前 buffer、溢出或停止完成状态。
- 没有 64 位 DMA 地址支持，只能使用低于 4 GiB 的 DMA/IOVA。
- 没有生产者/消费者握手，FPGA 不会等待用户归还 buffer。
- CLEAR 会让已呈现给 PCIe Core 的当前 TLP完成后再停，但软件无法从寄存器得到停止确认。

这些限制不影响基本彩条采集验证，但若要做稳定的视频采集驱动，后续建议增加只读状态寄存器、完成中断、写指针/读指针和 64 位 DMA 地址协议。

## 14. 上板检查清单

- [ ] `lspci` 能看到 PGL50H Endpoint，Memory Space 和 Bus Master 已使能。
- [ ] BAR1 长度覆盖到 `0x160`。
- [ ] `dma_set_mask_and_coherent(..., 32 bit)` 成功。
- [ ] 四个 DMA 地址均非零、高 32 位为零、64 字节对齐。
- [ ] CLEAR 后只写 4 次 `BAR1+0x110`。
- [ ] 配置后允许至少 100 ms 启动时间。
- [ ] 任一 buffer 的 `0x3F4800～0x3F483F` 出现 64 字节同值非零标志。
- [ ] 标志每个完整帧加 1，`0xFF` 后回到 `0x01`。
- [ ] 图像区大小严格为 `0x3F4800`，stride 为 3840 字节。
- [ ] 八条彩条的宽度、顺序和 RGB565 数值正确。
- [ ] 卸载顺序为 CLEAR/刷新/等待，然后停 Bus Master，最后释放 DMA 内存。

