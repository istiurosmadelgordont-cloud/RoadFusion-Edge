# FPGA 端 PCIe 彩条工程代码说明（修改版）

## 1. 版本和入口

工程：

```text
C:\Users\hero\Desktop\pds_prj\05_hdmi_test\05_hdmi_test
```

本说明对应带有以下注释标记的版本：

```text
PCIE_FIX_20260912
```

重点文件：

| 文件 | 作用 |
|---|---|
| `src/hdmi_test.v` | 顶层视频、PCIe IP、时钟和复位连接 |
| `src/pcie_dma_ctrl.v` | BAR 命令、四缓冲、FIFO 和 MWr 发送 |
| `src/sync_vg.v` | 1080p 视频时序 |
| `src/pattern_vg.v` | 8 条彩条 |
| `ipcore/hdmi_pcie_fifo/hdmi_pcie_fifo.v` | 16→128 bit 异步 FIFO |
| `source/hdmi_test.fdc` | 时钟和管脚约束 |

## 2. 总体数据流

```text
                    ARM / RC
                       │
       BAR1 MWr命令 ───┤
                       ▼
                axis_master_*
                       │
                       ▼
               RX_HEADER/PAYLOAD
                       │
                 4个DMA基址
                       │
                       ├──────────────────────────────┐
                       │                              │
50MHz ─ PLL ─ 148.5MHz │                              │
           │           │                              │
           ▼           │                              │
 sync_vg → pattern_vg  │                              │
           │ RGB888    │                              │
           ▼           │                              │
        RGB565         │                              │
           │ 16bit     │                              │
           ▼           │                              │
       异步 FIFO ──────┘                              │
           │ 128bit                                   │
           ▼                                          │
  512bit payload_buffer → MWr Header/Data FSM         │
           │                                          │
           ▼                                          │
     axis_slave2_* → PCIe IP → MWr到ARM四缓冲区 ◄────┘
```

## 3. `hdmi_test.v`

### 3.1 视频侧

PLL 产生：

- `pix_clk=148.5 MHz`：1920×1080@60 视频像素时钟；
- `cfg_clk=10 MHz`：HDMI 芯片 I²C 配置。

`sync_vg` 参数：

```text
H_TOTAL=2200, H_SYNC=44, H_BP=148, H_ACT=1920, H_FP=88
V_TOTAL=1125, V_SYNC=5,  V_BP=36,  V_ACT=1080, V_FP=4
```

`pattern_vg` 将一行分成 8 段，每段 240 像素，输出白、黄、青、绿、品红、红、蓝、黑。

顶层送入 PCIe DMA 的像素为：

```verilog
{r_out[7:3], g_out[7:2], b_out[7:3]}
```

即：

```text
RGB565[15:11] = R5
RGB565[10:5]  = G6
RGB565[4:0]   = B5
```

### 3.2 PCIe 接口方向

接口名称是从 PCIe IP 自身视角定义的：

| 接口 | 数据方向 | 功能 |
|---|---|---|
| `axis_master_*` | PCIe IP → `pcie_dma_ctrl` | 接收 ARM 发来的 BAR MWr |
| `axis_slave2_*` | `pcie_dma_ctrl` → PCIe IP | FPGA 主动向 ARM 内存发送 MWr |
| `axis_slave0/1_*` | 用户逻辑 → PCIe IP | 当前固定为 0 |

### 3.3 复位

PCIe DMA 复位链为：

```text
rst_board / pcie_perst_n
    → 去抖/同步
    → pcie_test.core_rst_n
    → u_pclk_div2_core_rstn_sync
    → s_pclk_div2_rstn
    → pcie_dma_ctrl.rstn
```

`pcie_dma_ctrl` 不直接使用裸 `rst_board`，其复位释放与 PCIe Core 的 `pclk_div2` 域一致。

本次还显式声明了所有 PCIe 时钟/复位 net，避免 Verilog 隐式网络掩盖拼写或连接错误。

### 3.4 未使用 APB/DBI 端口

PCIe IP 的 `p_sel/p_strb/p_addr/p_wdata/p_ce/p_we` 当前不用，修改后显式绑 0。`p_rdy/p_rdata` 是输出，可以保持不连接。

## 4. `pcie_dma_ctrl.v` 常量

```verilog
TLP_LENGTH_DW   = 16;       // 16 DW = 64 bytes
IMAGE_TLP_COUNT = 64800;
TOTAL_TLP_COUNT = 64801;
```

计算关系：

```text
1920 × 1080 × 2 = 4,147,200 bytes = 0x3F4800
4,147,200 / 64  = 64,800 image TLPs
64,800 + 1 marker TLP = 64,801 total TLPs
```

## 5. RC→EP BAR 命令状态机

### 5.1 接收握手

```verilog
axis_master_tready = 1'b1;
rx_handshake = axis_master_tvalid && axis_master_tready;
```

硬件始终声明可接收，但只在 `rx_handshake` 成功的时钟沿解释当前 beat。

### 5.2 `RX_HEADER`

本状态读取 128 bit TLP header，只接受：

```text
Fmt/Type = 8'h40，3DW MWr32
Length   = 1DW
TUSER[5:4] = 2'b01，BAR1
TLAST    = 0，后面还有payload
```

满足后保存地址低 12 位 `TDATA[75:64]`，进入 `RX_PAYLOAD`。其他 TLP 不作为控制命令执行。

### 5.3 `RX_PAYLOAD`

只有 `TKEEP[0]=1` 且 `TLAST=1` 时才执行 1DW 命令，然后返回 `RX_HEADER`。

与旧版只检测 `TVALID` 上升沿不同，该 FSM 能正确处理：

```text
header0, payload0/TLAST, header1, payload1/TLAST
```

即便 `TVALID` 从头到尾连续为 1，也不会漏掉第二个命令。

### 5.4 命令表

| 偏移 | 功能 |
|---:|---|
| `0x110` | 依次写入 `dma_addr0～dma_addr3` |
| `0x130` | CLEAR，清地址和运行状态 |
| `0x140` | 高亮压低参数 |
| `0x150` | 暗部抬升参数 |
| `0x160` | 清除增强参数 |

payload 地址数据使用 `endian32()` 做 32 位字节顺序转换。ARM 端应使用普通 `writel()`，不再额外交换。

## 6. 四缓冲配置

`alloc_addr_state` 为 0～3：

```text
0：写 dma_addr0
1：写 dma_addr1
2：写 dma_addr2
3：写 dma_addr3并提交
```

第 4 次写入时，只有 `dma_addr0～2` 和当前 payload 都非 0，才拉高 `rc_cfg_ep_flag`。

提交后 `0x110` 写入被忽略，直到收到 CLEAR。这样一次已经运行的四缓冲配置不会被意外第 5 次写破坏。

## 7. VS 和帧控制

### 7.1 `pclk_div2` 域边沿检测

```text
vs_in → vs_in_meta → vs_in_sync → vs_in_prev
```

```verilog
vs_rising  =  vs_in_sync && !vs_in_prev;
vs_falling = !vs_in_sync &&  vs_in_prev;
```

### 7.2 缓冲区选择

VS 上升沿时：

```text
addr_page=0 → current_addr=dma_addr0
addr_page=1 → current_addr=dma_addr1
addr_page=2 → current_addr=dma_addr2
addr_page=3 → current_addr=dma_addr3
```

然后页号加 1。只有已经完成四地址配置时才设置 `frame_addr_valid`。

VS 下降沿时，如果地址有效、发送状态空闲且没有停止请求，`frame_active` 拉高。

这种顺序解决了旧版中“第四地址在 VS 高电平期间才写完，下降沿直接以 `alloc_addrl=0` 启动”的竞争。错过上升沿的配置会等下一帧。

### 7.3 `frame_active` 回到像素域

`frame_active` 经过两级寄存器同步到 `pix_clk_out`，形成 `frame_active_pix`。FIFO 写使能为：

```verilog
fifo_wr_en = de_in && frame_active_pix && !fifo_wr_full;
```

`de_in` 距 VS 下降沿还有垂直后沿消隐，因此同步延迟不会漏掉有效区开头。

## 8. 异步 FIFO

| 端口 | 参数 |
|---|---|
| 写时钟 | `pix_clk_out` |
| 写数据 | 16 bit RGB565 |
| 读时钟 | `pclk_div2` |
| 读数据 | 128 bit，即 8 个像素 |

修改后连接 `wr_full`、`rd_empty`：

- 满时不继续写，避免非法溢出；
- 空时不发出 `pcie_rd_en`，避免非法读。

如果满导致像素丢失，DMA 不会凑够 64800 个图像 TLP，本帧不会写出新的完整标志，软件应忽略。

## 9. 64 字节 payload 预取

一个 TLP payload 需要 4 个 128 bit FIFO 字。发送状态机必须先看到：

```text
rd_water_level >= 4 && !fifo_rd_empty
```

每个 FIFO 字使用三步读取：

```text
TX_FETCH_REQUEST：寄存 pcie_rd_en=1
TX_FETCH_WAIT：FIFO 在本拍看到 rd_en并更新输出
TX_FETCH_CAPTURE：把稳定的 rd_data锁存到 payload_buffer
```

重复 4 次后得到 512 bit `payload_buffer`。

这个本地缓冲有两个作用：

1. 匹配 FIFO 同步读延迟；
2. PCIe Core 即使中途 `TREADY=0`，也无需再推进 FIFO。

`endian128()` 保留了旧版每个 32 bit 范围内的字节重排，用来适配 PCIe AXIS 字节序。

## 10. EP→RC MWr 发送状态机

完整状态：

```text
TX_WAIT
   ├─ 图像包 → FETCH_REQUEST → FETCH_WAIT → FETCH_CAPTURE，循环4次
   └─ 标志包 → 直接填充512bit payload_buffer
                    │
                    ▼
                TX_HEADER
                    │ header handshake
                    ▼
                 TX_DATA
                    │ 4个payload handshake，最后一拍TLAST
                    ▼
                 TX_WAIT
```

### 10.1 Header

`make_mwr32_header()` 生成：

```text
Fmt/Type       = 3DW MWr32
Length         = 16 DW
First/Last BE  = 0xF
Requester ID   = 当前 EP Bus/Device/Function 0
Address        = packet_addr
```

`packet_addr` 在发 header 前锁存，所以即使 VS 到来并选择下一缓冲区，已经开始的 TLP 地址也不会改变。

### 10.2 Payload

Header 只有在：

```text
r_axis_s_tvalid && AXIS_S_TREADY
```

时才切换到 payload。Payload 也只有握手成功才切换到下一 128 bit。

第 4 个 payload beat带 `TLAST=1`。其握手成功后才：

- `dma_cnt+1`；
- `current_addr+64`；
- 返回 `TX_WAIT`。

### 10.3 背压

`TVALID=1,TREADY=0` 时，本 always 块不会修改当前 payload、`TLAST`、地址或计数器。PCIe Core 恢复 `TREADY=1` 后从原 beat继续。

这是本次最主要的修正。

## 11. 帧边界和中止

如果 VS 上升沿或 CLEAR 到来：

- 尚在 WAIT/FETCH、没有向 PCIe Core 显示 header：立即回 WAIT；
- Header 已经 `TVALID=1` 或处于 DATA：设置 `abort_pending`；
- 当前 TLP 一直保持协议，直到带 `TLAST` 的最后一拍成功握手；
- 随后清包计数并停止本帧。

这种处理只保证“不截断已呈现的 TLP”，不保证来不及发送的整帧能够补传。若一帧未在下一 VS 前完成，该帧会被放弃，ARM 看不到新的完整帧标志。

## 12. 帧尾标志

`dma_cnt=0～64799`：图像包。

`dma_cnt=64800`：标志包，内容为：

```verilog
payload_buffer <= {64{frame_counter}};
```

因此：

```text
buffer+0x3F4800 ～ buffer+0x3F483F
```

64 字节全部相同。标志包最后一拍成功握手后，`frame_counter` 才加 1；`0xFF` 后回到 `0x01`。

与旧版相比，不再出现一个标志包中的 `N/N+1/N+2/N+3`。

## 13. 理论状态机吞吐

在不背压时，一个图像 TLP 大约需要：

```text
4次FIFO读取 × 3拍 + 1拍header传输 + 4拍payload传输
≈ 17个pclk_div2周期
```

当前约束中 `pclk_div2=125 MHz`，64800 个图像 TLP 的内部处理量级约为：

```text
64800 × 17 / 125 MHz ≈ 8.81 ms
```

低于 60 fps 的 16.67 ms 帧周期，理论上有余量。实际还受 PCIe Flow Control、Core `TREADY`、链路速率和系统内存写入能力影响。

## 14. 约束修改

PCIe wrapper 的 `free_clk` 端口实际连接顶层 `ref_clk`。原 FDC 查找不存在的 `free_clk` 时钟对象，修改后使用：

```tcl
set_clock_groups -name group_3 -asynchronous -group [get_clocks {ref_clk}]
```

该修改消除了 `Nothing implicitly matched 'free_clk'` 警告。

## 15. 调试建议

推荐抓取：

```text
RX：
axis_master_tvalid, axis_master_tready, axis_master_tdata,
axis_master_tkeep, axis_master_tlast, axis_master_tuser,
rx_state, rx_cmd_addr, alloc_addr_state,
dma_addr0, dma_addr1, dma_addr2, dma_addr3, rc_cfg_ep_flag

帧/FIFO：
vs_in_sync, vs_in_prev, frame_addr_valid, frame_active,
frame_active_pix, fifo_wr_full, fifo_rd_empty,
rd_water_level, pcie_rd_en, fetch_index

TX：
tx_state, packet_addr, current_addr, dma_cnt, frame_counter,
AXIS_S_TVALID, AXIS_S_TREADY, AXIS_S_TLAST, AXIS_S_TDATA,
abort_pending, stop_pending
```

建议断言：

```text
TVALID && !TREADY 时，TDATA/TLAST保持不变
pcie_rd_en拉高时，fifo_rd_empty必须为0
图像TLP的packet_addr每包增加64
dma_cnt=64800时目标地址为base+0x3F4800
完整标志包64字节完全相同
```

## 16. 当前仍未实现的功能

- BAR 只写控制，没有 BAR read completion。
- 没有帧完成 MSI/MSI-X/INTx 中断。
- 没有可读状态、错误、溢出、当前 buffer 索引。
- 只有 32 位 DMA 地址。
- 四缓冲没有 ARM 归还/FPGA 等待机制。
- FIFO 满时只能放弃像素，不能对实时视频源反压。
- 尚未加入完整的自检 testbench。

所以当前版本适合做稳定的“FPGA 主动写四缓冲、ARM 轮询帧标志”验证。若进入产品化，建议下一步设计真正的 DMA descriptor/status/interrupt 协议。

## 17. 验证结果

- PDS 2022.2-SP6.4 `compile`：通过；
- RX/TX 状态机均被综合器识别；
- 修改过的 `pcie_dma_ctrl.v`、`hdmi_test.v`：无编译警告；
- PDS `synthesize`：通过；
- 未执行完整 place & route、时序报告和上板联合测试。

详细修改与回退方法见 `PCIe_DMA_RTL修改记录_20260912.md`；ARM 端操作见 `ARM端_PCIe彩条视频传输操作说明_修改版.md`。

