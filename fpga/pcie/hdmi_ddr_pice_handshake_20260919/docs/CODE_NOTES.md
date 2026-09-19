# 四缓冲握手 RTL 代码说明与改动记录

完成日期：2026-09-19。工程目录的 20260916 是任务起始日期。原始控制器保存在 `archive/pcie_dma_ctrl.before_handshake.v`；当前控制器保留相同端口，可直接由现有顶层例化。

## 改动位置

搜索 `PCIE_HS_20260919` 或下列 `[Hn]` 标记。此版本仅改 `source/source/pcie_dma_ctrl.v` 的功能 RTL；PDS 相对路径和先前整合的 DDR IP 依赖继续沿用。

| 标记 | 代码区 | 原行为 → 新行为 |
| --- | --- | --- |
| H1 | localparam 命令表 | 四地址自动开始 → 明确配置、START、RELEASE、STOP、清配置 |
| H2 | 所有权寄存器 | 固定 addr_page 轮转 → 每槽状态和令牌 |
| H3 | address_valid / ranges_separate / status_separate | 非零检查 → 64B 对齐、32 位范围、不重叠检查 |
| H4 | free_found / free_slot | 取下一个编号 → 从轮转起点搜索空闲槽 |
| H5 | overflow_pix / overflow_sync | full 时仅不写 FIFO → 同时将本帧判为失败 |
| H6 | RX 状态机 | 加强 BE/keep/对齐检查；不支持的 TLP 排到 TLAST |
| H7 | TX 主状态机 | VS/STOP 分支跳过握手处理 → 每拍仍消费真实 AXIS 握手 |
| H8 | RELEASE 匹配 | 无归还机制 → 只释放状态、slot 和完整 token 都匹配的槽 |
| H9 | VS 分配 | 可能覆盖读者 → 无空闲跳整帧；旧包未排空时跳过本帧 |
| H10 | PK_STOP_DONE | 无停止确认 → 独立状态区回写 STOP cookie |
| H11 | PK_FRAME_DONE | 重复 8-bit 帧号 → 16 个重复 LE32 token |

## 所有权寄存器

`buffer_state[0..3]` 只有 TX always 块写入，避免多个时序过程同时驱动。FREE=0 表示可以分配，WRITING=1 表示 FPGA 已预留或正在发送，HELD=2 表示完整帧已交给 RC、等待归还。READY/READING 是 RC 自己的细分状态，FPGA 无需区分。

`allocation_sequence` 为 30 位，`active_token={sequence,slot}`。每次 VS 预留时产生一个新令牌，复制到 `buffer_token[slot]`。STOP/START 不清 sequence；到最大值后停止分配，避免 ABA，即非常迟的旧归还恰好释放重用后的新帧。

`addr_page` 现在是下次搜索起点。`free_found` 为零时不启动 FIFO 采集，`dropped_frames` 加一。搜索不依赖 RC 按编号归还；槽 2 先空闲即可先写槽 2。

`dropped_frames` 统计 VS 上未能分配的次数，不是所有可能的数据丢失总数；`invalid_releases` 统计不匹配归还。这两个计数器供 ILA 观察，未映射到 BAR 读寄存器。

## RX 和 TX 如何交接

RX 解析命令，发出一个时钟宽度的 `start_pulse`、`stop_pulse` 或 `release_pulse`，附带稳定 token/cookie。TX 在下一拍处理。只有停止且 TX_WAIT、没有 stop_pending 时 `idle_control` 才允许配置/启动。

收到第四个合法图像地址后，验证六对地址范围不重叠，再置 `rc_cfg_ep_flag`。状态地址必须在四地址提交后登记，并验证与四个范围不重叠。控制端写错时无错误回包，需重新清配置并登记，或通过 ILA 定位。

RX_DROP 把非本协议包消耗到 TLAST，避免把长包的后续 Payload 当作 Header。协议仍依赖当前 IP 的 Header/Payload 分拍接口，没有新增 MRd Completion 实现。

## 一帧的发送过程

1. VS 上升沿：取消上一帧未完成部分；只有没有已呈现的旧 TLP、存在空闲槽、令牌未耗尽时才预留新槽。
2. VS 下降沿：确认 TX_WAIT 且配置/运行仍有效，置 frame_active；同步到像素域后允许写 FIFO。
3. 按原时序执行 FETCH_REQUEST → FETCH_WAIT → FETCH_CAPTURE，读取四个 128-bit 数据拍，形成 512-bit Payload。
4. TX_HEADER 呈现独立头拍，随后 TX_DATA 发送四个 Payload 拍。只在 valid && ready 时推进。
5. 64,800 个图像包后，再发送一个 64 字节帧尾包；token 先 endian32 后在 512-bit 暂存中重复 16 次。
6. 帧尾最后一拍握手完成，槽变为 HELD，等待 RC 归还。

Header 使用固定 16 DW Length；这 16 DW 都是数据，头拍不占图像 Payload。地址以 64B 对齐递增，因此本版本每个 64B 图像包不会跨越 4 KB 边界。仿真参数 IMAGE_TLP_COUNT 可以缩短帧；上板默认值及 RC 尾偏移必须保持一致。

## 背压、VS 和 STOP 的优先关系

`cancel_frame = vs_rising || stop_pulse || overflow_sync` 停止继续取新数据，但不会撤回已经拉高的 TVALID。`abort_pending` 记住“当前 TLP 完成后丢掉剩余帧”；`stop_pending` 记住“还需要发送停止 ACK”。

旧代码在 VS 分支中可能不执行 TX case。如果 VS 与 valid && ready 同拍，IP 已收走一拍而本地未前进，就可能重发。本版先记录事件，再始终执行对应 TX 状态的握手逻辑。仿真专门覆盖 Header/VS、TLAST/VS 同拍。

若取消发生在尚未呈现 Header 的取数阶段，直接回 TX_WAIT，释放 WRITING 槽；若 Header 已呈现，即使 ready=0 也必须保持数据并排完四拍。如果已经呈现的是帧尾，则完整发完并交付 HELD，RC 仍可以收到有效完成。

STOP 后 running=0，不再预留新帧。当前 TLP 排完后，用同一个发送器发 PK_STOP_DONE，地址为 status_addr，内容为 16 个 STOP cookie。它最后一拍被 IP 接受后 stop_pending 清零。RC 必须等这个包真正到达内存；不能用内部 stop_pending=0 或固定延时代替。

## 测试文件

- `tests/handshake_tb.sv`：外部 AXIS scoreboard，检查包地址、每拍数据顺序、TLAST、标记格式、HELD 不被覆盖、背压稳定性。
- `tests/fifo_model.v`：控制器用 FIFO 行为替身，可制造 empty/full；不验证厂商 FIFO CDC 或真实视频采集。
- `tests/run.ps1`：默认短帧压力测试；`-ImagePackets 64800` 使用完整 1080p 包数。
- `rc/handshake_reference.h`：平台无关 RC 协议辅助实现；DMA 屏障、字节序、MMIO 由驱动回调提供。
- `tests/rc_reference_test.c` / `tests/run_rc.ps1`：C99 参考代码的模拟回调测试，检查持有状态及清标记→屏障→MMIO 的顺序。

`docs/pcie_dma_ctrl_changes.patch` 可用于审阅或对原始 source/source/pcie_dma_ctrl.v 应用差异；当前工程已经包含这些修改，不要重复应用。

## 未包含在本次修改里的问题

此版没有提高单包发送流水效率、增大 Payload、实现 MSI 或真实 Linux PCI 驱动。DDR 写页避让逻辑、yuv2rgb 饱和高位丢失、增强参数跨时钟原子更新仍是旧工程待修项。RC 读写所有权握手解决的是接收缓冲覆盖，不能修复这些上游画面问题。

详细仿真和 PDS 结果见 `BUILD_STATUS.md`。真实板上需要验证 VS 消隐长度、FIFO 复位、DMA 字节序、PCIe 排序及显示消费者释放时机。
