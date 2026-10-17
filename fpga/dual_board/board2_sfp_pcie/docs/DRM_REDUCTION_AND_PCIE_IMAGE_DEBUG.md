> 2026-10-17：FIC已扩展为每核两个Trigger Port，接收/生效参数均可观察。当前配置与加载步骤见 [参数探针说明](IMAGE_PCIE_FIC.md)。下方126/134及133/172位记录对应之前未接入参数总线的FIC。

> 工程以ZIP提供：[下载20261017两阶段回传版](../sfp_ddr_image_pice_handshake_20261017_image_ack.zip)。源码路径相对解压后的工程根目录。

> 已生成并加入工程的配置：image_pcie_debug.fic（`debug/image_pcie_debug.fic`）。两组256点，插核后DRM为126/134；见 [FIC使用说明](IMAGE_PCIE_FIC.md)。下方121.5/134为未插核的行缓存优化结果。

# DRM优化与PCIe图像入口调试

日期：2026-10-16。适用工程：`sfp_ddr_image_pice_handshake`。

## 本次修改

`source/src/common/video_line_delay.v` 对非2次幂长度的行缓存分成两个独立RAM。
1080p模式的2200拍行延时使用2048项主存储和152项尾存储，避免整组像素位宽被统一扩展成4096项。
两路读端仍为同步读，读RAM的同一拍记录银行选择，循环地址、填充屏蔽和后级寄存延时保持一致。
像素数据、消隐期间的数据及控制位均保持原有逐拍延时；复位仍只清地址、有效状态及短寄存流水，不清RAM阵列。
所有使用本模块的图像对齐及HS对齐路径自动生效，图像处理功能和参数不变。

`source/source/pcie_dma_ctrl.v` 增加只用于观察的像素计数、RGB565采样和溢出探针，并标记原有FIFO信号。
去掉新增观察块和调试属性后，DMA的功能文本与修改前一致。RX/TX状态机、四缓冲所有权、TLP布局、地址、token、STOP/RELEASE、寄存器和端序全部沿用现有协议。
上位机、RC参考头、SFP/PCIe生成IP、DDR和FIFO IP均未修改。
顶层 `source/hdmi_ddr_pcie.v` 只给图像输入/输出与复位、配置握手信号增加 `PAP_MARK_DEBUG` 标记。

## 资源结果

PDS 2022.2-SP6.4，PGL50H/FBG484/-6，Compile、ADS Synthesize、Device Map通过。
对比同一工程上一版、不插入FLA采样核的Device Map结果：

| 资源 | 修改前 | 修改后 | 器件容量 |
| --- | ---: | ---: | ---: |
| DRM（18K等效） | 133 | 121.5 | 134 |
| LUT | 16198 | 16234 | 42800 |
| FF | 20071 | 20071 | 64200 |
| APM | 51.5 | 51.5 | 84 |

DRM减少11.5，剩余12.5；LUT增加36。以上为未连接抓波核时的资源值。
`PAP_MARK_DEBUG` 供ADS/Inserter识别RTL信号；未连接调试核的观察寄存器可能在后续Device Map中被裁剪。
给FLA接线后会保留所用观察逻辑，采样RAM和调试逻辑需另计资源，并重新执行映射。

## 像素时钟域的探针

采样时钟使用 `pix_clk_out`。下表DMA内部信号的层次为 `u_pcie_dam_ctrl/`，保留工程原有实例拼写。

| 信号 | 位宽 | 用途 |
| --- | ---: | --- |
| `hdmi_data_in`、`vs_in`、`de_in` | 16/1/1 | 真正送入DMA的处理后RGB565、高有效VS、像素有效 |
| `pix_rstn_sync` | 1 | DMA像素侧复位同步释放状态 |
| `frame_active_pix` | 1 | 当前帧是否被DMA选中接收 |
| `fifo_wr_en`、`fifo_wr_rst`、`fifo_wr_full` | 各1 | 写入请求、写侧复位及满状态；实际写入为 `fifo_wr_en && !fifo_wr_rst` |
| `overflow_pix` | 1 | 原DMA溢出状态，参与原有取消帧流程 |
| `dbg_image_unselected` | 1 | `de_in && !frame_active_pix`，有图像但该帧没有被DMA接收 |
| `dbg_image_fifo_blocked` | 1 | `de_in && frame_active_pix && fifo_wr_full`，接收中遇满 |
| `dbg_image_pixels` | 22 | 当前VS间隔内输入DE有效拍数 |
| `dbg_image_pixels_last` | 22 | 上一个VS间隔的输入DE有效拍数 |
| `dbg_fifo_pixels` | 22 | 当前VS间隔内FIFO实际写入拍数 |
| `dbg_fifo_pixels_last` | 22 | 上一个VS间隔内FIFO实际写入拍数 |
| `dbg_first_rgb565`、`dbg_last_rgb565` | 各16 | 当前接收帧首个/最近一个实际写入像素；消隐期间保持，下一接收帧开始更新 |
| `dbg_input_overflow` | 1 | 本VS间隔内是否遇过FIFO满，不因原DMA提前取消帧而丢失 |
| `dbg_input_overflow_last` | 1 | 上一VS间隔内的FIFO满记录，便于触发后检查 |

计数在 `vs_in` 上升沿锁存上一帧并清当前计数。正常视频VS与DE互斥，锁存该拍不累计DE；计数器22位，1080p一帧2073600像素可完整表示。
`dbg_image_pixels_last` 可在DMA停止或无空闲槽时仍为2073600，而 `dbg_fifo_pixels_last` 为0；这是未接收整帧，不应当作FIFO溢出。
正常接收完整1080p帧时，两者均应为2073600。中途STOP/取消的帧，写入计数可以小于输入计数。

顶层还可选择 `image_rst_n`、`image_stage_rgb/vs/de`、`image_output_rgb/vs/de/hs` 和 `image_cfg_pending/ack`。
模块内部VS低有效，顶层输出和DMA入口VS高有效，比较时留意反相及输出寄存器的一拍延时。
图像输出先经过共用RGB888输出寄存器，再截取R[7:3]、G[7:2]、B[7:3]进入DMA；原有像素布局不变。

## PCIe时钟域的探针

采样时钟使用DMA端口 `clk` 对应的PCIe用户时钟。不要用它直接采样像素域计数总线。

| 信号 | 位宽 | 用途 |
| --- | ---: | --- |
| `rd_water_level` | 13 | 原FIFO读侧水位 |
| `fifo_rd_empty`、`pcie_rd_en`、`fifo_rd_rst` | 各1 | 原FIFO读侧请求、空及复位 |
| `pcie_dma_data` | 128 | FIFO读出的图像数据，按原预取流程观察 |
| `tx_state`、`dma_cnt` | 3/16 | 原发送状态机和图像TLP进度 |
| `AXIS_S_TVALID/READY/LAST` | 各1 | 原PCIe发送握手 |
| `AXIS_S_TDATA` | 128 | 原发送TLP数据 |
| `running`、`frame_active`、`cancel_frame` | 各1 | 原DMA运行/接收/取消状态 |
| `image_cfg_apply` | 1 | 配置提交toggle，不能当单拍脉冲 |

`image_cfg_ack`、`image_ready` 为像素侧返回信号，已标记供检查，归属像素域；PCIe域需结合现有同步器观察，不能把异步返回与PCIe采样边沿做逐拍等价判断。

`tx_state` 沿用0=WAIT、1=FETCH_REQUEST、2=FETCH_WAIT、3=FETCH_CAPTURE、4=HEADER、5=DATA。
`pcie_dma_data` 是同步FIFO读口，不能仅凭 `pcie_rd_en` 当拍就解释为该次返回数据；按原FETCH状态等待。

## Inserter/Debugger使用

1. 在PDS中打开工程并使用ADS综合，打开Fabric Inserter。标记方式沿用源码的 `/* synthesis PAP_MARK_DEBUG="1" */`。
2. 当前工程已启用 `debug/image_pcie_debug.fic`，旧的大容量FIC引用继续停用。若Inserter按标记自动连接全部信号，先缩减连接；已有大量DDR/PCIe总线标记，不适合全部高深度抓取。
3. 为像素域和PCIe域分别设置采样核及时钟。先选一组关键探针，以256或512采样深度开始，按重新映射的资源结果调整。
4. 像素侧先看 `de_in/vs_in/hdmi_data_in/frame_active_pix/fifo_wr_en/fifo_wr_full`。异常触发用 `dbg_image_fifo_blocked==1`；观察整帧计数时在VS上升沿附近取样，锁存值从该边沿后更新。
5. PCIe侧先看水位、empty、读使能、TX状态和发送握手；需要核对像素排列时再接128位数据，避免同时接入全部大总线。
6. 保存本次FIC配置，重新编译、综合、映射、布局布线并生成匹配的位流和调试配置，再下载后使用Fabric Debugger。仅有RTL标记不会在旧位流中产生可抓取的新信号。

不新增BAR读取、状态DMA、MSI或上位机命令，已有上位机按原接口工作。

## 验证与复跑

本次增加的两个用例：

- `tests/image_pcie/line_delay_banked_tb.sv`：58/31/27/1等位宽，2200、2048、2049、4095、16、17、3等行长度，共10组；以独立逐拍历史对比完整延时，覆盖填满前及填满后的复位。
- `tests/image_pcie/dma_input_debug_tb.sv`：未选中像素、实际写入、FIFO满、跨帧锁存、首末RGB565采样及复位。

复跑新增用例：

```powershell
./tests/image_pcie/run_drm_debug.ps1 -Iverilog <iverilog.exe路径> -Vvp <vvp.exe路径>
```

原有四缓冲握手、配置CDC、32帧DMA与配置并发、DDR后顶层集成、暗光/高光穷举、亮度流水线和串行图像处理回归使用同一份改后RTL验证。
结果和资源报告摘录放在 `docs/verification/20261016/`。

本次验证到器件映射。尚未执行本版布局布线后的时序签核、真实FLA插核后的资源验证或板上验收；未生成/替换位流。
