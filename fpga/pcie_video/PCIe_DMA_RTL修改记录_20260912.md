# PCIe DMA RTL 修改记录（2026-09-12）

## 1. 修改目标

本次修改针对 `05_hdmi_test` 工程中 PCIe 彩条视频传输路径，目标是修正 AXIS 背压时 FIFO 被多读、TLP 计数提前推进、BAR 命令依赖 `TVALID` 间隔、DMA 地址配置与帧边界竞争，以及帧尾标志每 128 bit 加 1 等问题。

所有 RTL/约束改动均可搜索统一标记：

```text
PCIE_FIX_20260912
```

## 2. 修改前归档

- 原工程：`C:\Users\hero\Desktop\pds_prj\05_hdmi_test\05_hdmi_test`
- 归档目录：`C:\Users\hero\Documents\ChatGPT\pcie\archives\05_hdmi_test_before_pcie_fixes_20260912_2210`
- 源码快照：`05_hdmi_test_source_snapshot.zip`
- ZIP SHA-256：`8E7AAC4E2BA416D0FAA6C0E75D029700940A34DD0EAEA98785DA8B72A545CC1C`
- 归档清单：`ARCHIVE_MANIFEST.md`

快照包含 `src/source/ipcore/sim`、工程 `.pds` 和 `impl.tcl`，排除了 PDS 正在写入的锁、日志以及可重新生成的构建目录。

## 3. 修改文件

| 文件 | 修改后 SHA-256 |
|---|---|
| `src/pcie_dma_ctrl.v` | `9B2BC9CC12186BFD9FFF27658A4D37A6EA377783F8BBFC878DB31205777D391C` |
| `src/hdmi_test.v` | `6390D98EAD0A631E99147BA0E7F53CBC473E547BE470EA9488B61C927A3C4771` |
| `source/hdmi_test.fdc` | `782976D6B1E430DC188FF9D639EEC75FCBA09E875E202C9E296E50D9CB5C68AF` |

## 4. `pcie_dma_ctrl.v` 改动

### 4.1 BAR 命令接收改为两状态 FSM

旧代码仅检测 `axis_master_tvalid` 上升沿，默认 header 后紧跟 payload，并要求两个 TLP 之间出现 `TVALID=0`。连续有效的背靠背 TLP 可能漏掉下一包。

新代码使用：

```text
RX_HEADER → RX_PAYLOAD → RX_HEADER
```

每个 beat 都由 `axis_master_tvalid && axis_master_tready` 确认接收。命令只有同时满足以下条件才执行：

- 3DW MWr32；
- Length=1DW；
- `axis_master_tuser[5:4]=2'b01`，即 BAR1；
- payload 的 `TKEEP[0]=1`；
- payload 的 `TLAST=1`。

### 4.2 四地址配置提交更安全

- `BAR1+0x110` 仍按顺序写 `dma_addr0～dma_addr3`。
- 第 4 个非零地址收到后才置 `rc_cfg_ep_flag`。
- 四地址配置完成后，额外的 `0x110` 写入会被忽略。
- 如需重新配置，必须先写 `BAR1+0x130` 清除。

这样避免意外的第 5 次写入覆盖 `dma_addr0`、破坏正在使用的地址环。

### 4.3 VS 跨时钟采样

增加 `vs_in_meta → vs_in_sync → vs_in_prev`，在 `pclk_div2` 域检测 VS 上升/下降沿。`frame_active` 再同步回 `pix_clk_out` 域，用于控制 FIFO 写入。

四缓冲基址只在 VS 上升沿选择；配置若错过该边沿，会自动等待下一完整帧，不会用地址 0 启动 DMA。

### 4.4 发送前缓存完整 64 字节

旧代码在 HEADER 状态持续拉高 FIFO `rd_en`，PCIe Core 一旦拉低 `TREADY`，就可能在 header 未被接收时连续读走多个 FIFO 字。

新代码先把 4 个 128 bit FIFO 字读入本地 512 bit `payload_buffer`，然后才发送 TLP。FIFO 读取使用：

```text
TX_FETCH_REQUEST → TX_FETCH_WAIT → TX_FETCH_CAPTURE
```

额外的 WAIT 拍用于匹配寄存后的 `pcie_rd_en` 与同步 FIFO 读数据延迟。

### 4.5 发送状态严格遵守 AXIS 握手

发送状态机现在为：

```text
TX_WAIT
  → TX_FETCH_REQUEST
  → TX_FETCH_WAIT
  → TX_FETCH_CAPTURE（循环4次）
  → TX_HEADER
  → TX_DATA（4个128 bit beat）
  → TX_WAIT
```

只有 `TVALID && TREADY` 时才会：

- header 切换到 payload；
- payload 切换到下一 beat；
- 结束 TLP；
- 增加 `dma_cnt`；
- DMA 目标地址增加 64 字节。

`TREADY=0` 时 `TVALID/TDATA/TLAST` 保持不变，FIFO、地址和包计数器均不前进。

### 4.6 帧边界不再截断已呈现的 TLP

VS 上升沿或 CLEAR 到来时：

- 尚未把 header 呈现给 PCIe Core的本地包可立即丢弃；
- 已经呈现 header 或正在发送 payload 的 TLP会完成到成功握手的 `TLAST`；
- 完成该 TLP 后停止本帧其余传输。

这样不会在背压期间撤销已经拉高的 `TVALID`。

### 4.7 帧尾标志格式修正

旧代码在标志包的每个 128 bit beat 都增加 `fram_cnt`，所以 64 字节内出现 `N/N+1/N+2/N+3` 四段。

新代码中整个标志包均为同一个非零字节 `N`：

```text
buffer + 0x3F4800 ～ buffer + 0x3F483F：64 字节全部等于 N
```

只有标志包最后一个 beat 成功握手后帧号才增加一次；`0xFF` 后回到 `0x01`。

### 4.8 FIFO 满空保护

- 写：`de_in && frame_active_pix && !fifo_wr_full`
- 读：发出 `pcie_rd_en` 前检查 `!fifo_rd_empty`

正常带宽下满/空不应成为常态。这些保护防止非法访问；如果写满导致像素丢失，该帧通常无法达到 64800 个图像包，也不会产生新的完整帧标志，ARM 应忽略该帧。

## 5. `hdmi_test.v` 改动

- 显式声明 `pclk`、`pclk_div2`、`pcie_ref_clk`、`core_rst_n`、`s_pclk_div2_rstn` 等 PCIe 时钟/复位网络。
- 保持 `pcie_dma_ctrl.rstn = s_pclk_div2_rstn`，并添加修改标记说明。
- 删除顶层输出 `smlh_link_up/rdlh_link_up` 的重复内部声明。
- 将未使用的 PCIe DBI/APB 输入 `p_sel/p_strb/p_addr/p_wdata/p_ce/p_we` 明确绑 0，不再悬空。

## 6. `hdmi_test.fdc` 改动

原约束引用不存在的 `free_clk` 时钟对象，PDS 报告：

```text
Nothing implicitly matched 'free_clk'
```

PCIe wrapper 的 `free_clk` 端口实际由顶层 `ref_clk` 驱动，因此异步时钟组改为引用 `ref_clk`。

## 7. ARM 软件可见的协议变化

| 项目 | 修改前 | 修改后 |
|---|---|---|
| 连续 BAR 写 | 依赖 `TVALID` 出现低间隔 | 支持背靠背 TLP，不需要人为延时 |
| BAR 识别 | 只比较地址低 12 位 | 同时要求 BAR1 命中 |
| 第 5 次地址写 | 覆盖地址环开头 | 配置完成后忽略，CLEAR 后才能重配 |
| 启动边界 | 存在 `alloc_addrl=0` 竞争 | 只在完整 VS 边界装载有效缓冲地址 |
| 帧尾 64 字节 | 四段递增值 | 64 字节同一个帧号 |
| 帧号递增 | 每个 128 bit beat 加 1 | 每个完整帧加 1 |
| 背压 | 可能多读 FIFO/提前计数 | 输出保持，握手成功才推进 |

## 8. 验证记录

验证在归档解压出的隔离工程中执行，使用：

```text
C:\pango\PDS_2022.2-SP6.4\bin\pds_shell.exe
```

结果：

- PDS `compile`：通过；
- `rx_state`：识别为 2 状态 FSM；
- `tx_state`：识别为 6 状态 FSM；
- 修改后的 `pcie_dma_ctrl.v`、`hdmi_test.v`：无编译警告；
- `free_clk` 未匹配警告：已消除；
- PDS `synthesize`：通过。

尚未执行：

- PCIe/视频联合行为仿真；
- 当前工程的完整布局布线与时序收敛；
- 上板 ARM DMA 实测。

因此本次结果表示“RTL 编译和综合通过”，最终硬件正确性仍需上板检查 BAR 命令、AXIS 背压和 ARM 内存中的彩条/帧标志。

## 9. 回退方法

1. 关闭或停止当前 PDS 构建任务。
2. 将归档 ZIP 解压到新目录。
3. 从新目录取回 `src/pcie_dma_ctrl.v`、`src/hdmi_test.v` 和 `source/hdmi_test.fdc`。
4. 不建议直接把整个 ZIP 覆盖到当前工程，以免覆盖后续新生成的约束或工程设置。

