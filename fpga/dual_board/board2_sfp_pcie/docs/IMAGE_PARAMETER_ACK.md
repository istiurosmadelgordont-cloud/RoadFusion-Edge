# 图像参数两阶段回传接口（2026-10-17）

> 工程以ZIP提供：[下载20261017两阶段回传版](../sfp_ddr_image_pice_handshake_20261017_image_ack.zip)。源码路径相对解压后的工程根目录。

本版在手写 `source/source/pcie_dma_ctrl.v` 内加入可选回传。ARM 发送一次完整图像配置，FPGA 分两次回复：**参数已接收**、**参数已生效**。两次回复携带相同的 32 位请求编号 `cookie`。不增加 BAR 读回、MSI 或新的底层驱动模块。

## 1. 回传内容

ARM 为当前 PCIe 设备分配独立的 **128 字节 DMA coherent 内存**，非零、64 字节对齐、完整落在 32 位 DMA 地址空间，不能与四个图像缓冲或原 STOP 状态区重叠。传给 FPGA 的是 DMA/IOVA 地址，不是 CPU 指针。

| 内存偏移 | 长度 | 回复含义 | FPGA 写入内容 |
| --- | --- | --- | --- |
| `+0x00` | 64 字节 | RECEIVED：5 个参数字已全部接收，非零 COMMIT 已接受，完整快照进入待提交队列 | 16 个相同的 LE32 cookie |
| `+0x40` | 64 字节 | APPLIED：该 cookie 对应的快照已被像素域在帧边界采用，帧 ACK 已同步回来 | 16 个相同的 LE32 cookie |

这是两笔独立的 64 字节 PCIe MWr，沿用当前 TX 通道、相同的字节序和非松散排序属性。若参数在传输 RECEIVED 期间已生效，APPLIED 仍排在该事务的 RECEIVED 后面。没有视频帧边界时可以收到 RECEIVED，但不会提前回复 APPLIED。APPLIED 证明配置在流水线入口生效；它不表示整条流水线、PCIe 图像采集或显示器已显示出新画面。图像模块原有参数限幅/清理规则继续生效。

## 2. BAR1 命令

| 地址 | 写入内容 | 说明 |
| --- | --- | --- |
| `0x218` | 128 字节回传区 DMA 地址低 32 位 | 新增。四个图像地址和 STOP 地址配置完成后、采集尚未 START 时登记；停止且 TX 空闲时有效 |
| `0x200` | WORD0 | 开始本次完整参数包，同时重新开始统计 5 字接收掩码 |
| `0x204` | WORD1 | 同原接口 |
| `0x208` | WORD2 | 同原接口 |
| `0x20C` | WORD3 | 同原接口 |
| `0x210` | WORD4 | 同原接口 |
| `0x214` | 本次唯一非零 cookie | 原 COMMIT 地址；新用法以请求编号替代固定值 1，提交语义不变 |

所有命令仍为有序的 **1 DW MWr32、32 位对齐、First BE=0xF / Last BE=0**。本工程图像命令的 IP 用户接口要求命令头 `tkeep=0111`、下一数据拍 `tkeep=0001` 且 TLAST=1；逻辑命令值沿用 `endian32()`，不要自行改变旧 DMA 的字节映射。每笔参数必须独立写，不能合并成一个 5 DW TLP。

只发旧接口、不登记 `0x218` 时，不产生任何图像确认 DMA 包。旧的 `0x140/0x150/0x160` 和局部 WORD 更新仍保留原有行为，但**不产生本协议的完整配置确认**；要显示两阶段反馈，请把暗光、高光开关及阈值放入完整五字快照，通过 `0x200..0x214` 提交。参数位定义见 `PCIE_IMAGE_INTEGRATION.md`。

## 3. ARM 发送与轮询顺序

1. 完成原四缓冲和 STOP 区登记；若需要原 STOP 自检，先完成自检及 STOP ACK。
2. 在停止状态把回传区 DMA 地址写入 `BAR1+0x218`。地址非法会被忽略，不能靠 BAR readl 检查；第一次成功回传才证明整条配置链路有效。
3. 可按原流程 START 采集。完整图像参数也可在停止状态提交，只是生效仍需要视频输入的帧边界。
4. 在同一个驱动锁/控制线程下，确保上一事务已经 APPLIED，清零回传区的 128 字节，执行 DMA 写屏障。
5. 按 `0x200、0x204、0x208、0x20C、0x210` 顺序写全部 5 个 WORD，最后向 `0x214` 写本次非零 cookie。
6. 轮询 `+0x00` 的全部 16 个 LE32；全部等于本次 cookie 才置“参数已接收”，随后执行 DMA 读屏障并复核。
7. 独立轮询 `+0x40` 的全部 16 个 LE32；全部等于本次 cookie 且已确认 RECEIVED，才置“参数已生效”。随后才允许下一次参数提交。

例如 cookie=`0x12345678`：先看到前 64 字节为重复的 `78 56 34 12`，表示已接收；随后后 64 字节也出现相同内容，表示已生效。仅第一个 DWORD 匹配不算确认，其他 cookie 不算本次回复。

参考代码为 `rc/image_ack_reference.h`：

```c
/* ack_state 先按零初始化；ops 负责 coherent 内存读/清零。
 * frame_state 已按原握手配置，addr/status 是已经登记的 DMA 地址。 */
rf_image_ack_bind(&ack_state, &frame_state, &ack_ops, ctx,
                  addr, status_dma, image_ack_dma);

/* 完整构造 config，所有调用与其他 BAR 命令使用同一把驱动锁。
 * 实际调用必须检查返回值。 */
rf_image_submit_ack(&ack_state, &frame_state, &config, &cookie);
flags = rf_image_poll_ack(&ack_state, &frame_state);
/* flags==0 等待；flags==1 已接收；flags==3 已接收且已生效；-1 使用条件错误。
 * 采用非阻塞轮询或带超时工作队列，不要无限阻塞控制线程。 */
```

参考头文件提供 C99 回调接口，不是已经安装的 ARM Linux 驱动。Linux 接线需使用当前 PCIe 设备的 DMA API、`READ_ONCE()`、`le32_to_cpu()`、`dma_rmb()/dma_wmb()` 和现有经过验证的单 DW BAR 写适配；非 coherent 内存还要执行正确的 DMA sync，不能用普通 volatile 代替。

## 4. 并发、超时与停止

- 同时只保留 **一个待确认的图像请求**。cookie 在本 FPGA 会话内单调递增、不重复，0 无效。参考接口到 `UINT32_MAX` 拒绝回绕；STOP 后也保留软件 cookie 序号。
- 现有图像队列会合并中间提交。如果绕过参考接口连续提交 A、B、C，B 可能只收到 RECEIVED、没有 APPLIED；不能把 C 的 APPLIED 当成 B 的回复。回传发送器保留最新状态，不承诺为任意高速突发保存全部 RECEIVED 消息。因此使用一问一答串行提交。
- 漏写、被拒绝的参数字或未被接受的 COMMIT 不会获得本次完整 RECEIVED 确认。超时表示尚未确认，不能宣称参数接收/生效成功。接收确认超时查 `image_rx_word_mask`、`image_write_valid` 和地址配置；只有生效确认超时再查视频 ready、帧边界和像素域 ACK。
- 发送有效原 STOP 后，回传地址立即禁用；已经对 IP 呈现的回传 TLP 完整排完，然后才发送原 STOP ACK。STOP ACK 后不会再向该回传区自动写入。收到 STOP ACK 前不得释放或替换回传区。
- STOP ACK 后要再次使用本协议，重新写 `0x218`。不允许在旧地址仍启用时直接改成新地址。参考接口用 `rf_image_ack_poll_stop()` 包装原 STOP 轮询，只有确认 STOP 完成后才解除本地绑定。
- 超时保持内存映射和当前事务记录；先完成 STOP/排空或平台已经验证的复位恢复，再重试完整五字配置。FPGA 复位意味着新会话，须同步结束旧 DMA 会话。
- 图像回传包在 TLP 边界与原图像包仲裁，原 STOP 优先。回传不推进图像 DMA 地址/计数，不改变四缓冲 token、帧尾、RELEASE 或原 STOP 状态区格式。少量控制回传仍占用同一条 PCIe 带宽。

## 5. 源码观察点与验证

新增保持型寄存器：`image_return_addr_valid`、`image_rx_word_mask[4:0]`、`image_received_cookie`、`image_applied_cookie`。cookie 寄存器表示内部确认状态，ARM 只有收到目标 DMA 区完整回写才算观察到回复。这些新增寄存器已标记 PAP_MARK_DEBUG，但本次没有增加 FIC 端口或改动现有 FIC 探针布局。

`tests/image_pcie/image_ack_tb.sv` 在外部 TX AXIS 上逐包核对两种回复，覆盖地址重叠/溢出、未启用兼容、缺字/错误 TKEEP、帧生效、视频复位、队列合并、背压、VS、STOP 排空和图像 DMA 中插入回复，并对16次串行请求施加随机Payload背压，确认每次两笔回复没有丢失或重复。`tests/image_ack_reference_test.c` 覆盖 ARM 两阶段轮询、半写标记、旧 cookie、序号耗尽与 STOP。

本次验证记录在 `docs/verification/20261017/image_ack/`。仅完成 RTL 仿真和 C99 参考代码测试；未运行新的 PDS 综合/布局布线、未生成新 bitstream、未重新烧录，也尚未由真实 ARM 驱动验证字节序与 DMA 一致性。历史资源/时序结果不代表本次新增逻辑的资源/时序结果。当前硬件必须使用之后由本源码重新编译得到的位流，才会支持 `0x218` 和两阶段回传。
