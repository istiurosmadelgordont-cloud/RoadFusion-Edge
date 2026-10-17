# 图像参数接收与生效探针

> 工程以ZIP提供：[下载20261017两阶段回传版](../sfp_ddr_image_pice_handshake_20261017_image_ack.zip)。源码路径相对解压后的工程根目录。

2026-10-17，FIC标识：`IMAGE_CONFIG_RX_ACTIVE_20261017`。

工程当前引用 **`debug/image_pcie_debug.fic`**，引用路径不变。
`synthesize/hdmi_ddr_pcie_syn.fic` 同步为相同内容，避免从旧路径打开时混用旧探针。
本次只补充FIC连接和手写DMA模块中的调试属性，没有改DMA功能、寄存器、端序、握手方式、SFP/PCIe生成IP或上位机接口。
**按要求，本次没有启动编译、布局布线或生成bit文件。**

## FIC分组

| 核 | 采样时钟 | Trigger Port 0 | Trigger Port 1 | 深度 |
| --- | --- | --- | --- | ---: |
| Core 0 | `nt_pix_clk_out`（像素时钟） | 原图像入口/FIFO写入观察，133位 | 生效参数160位、图像模块输入VS 1位 | 256 |
| Core 1 | `pclk_div2`（PCIe用户时钟） | 原FIFO读出/发送观察，172位 | 接收参数、地址、数据和接收状态，213位 | 256 |

每核两个Trigger Port，Data Same As Trigger；保留原有两组图像/FIFO探针。
逐位接线见 probe_manifest.json（`docs/verification/20261017/config_probes/probe_manifest.json`）。

## 判断是否收到

在 **Core 1 / Trigger Port 1** 看：

| 信号（DMA内部层次前缀为 `u_pcie_dam_ctrl/`） | 含义 |
| --- | --- |
| `rx_cmd_addr[11:0]` | 最近解析的寄存器地址 |
| `command_value[31:0]` | 已按现有协议转换端序的写入值 |
| `rx_state[1:0]` | 0=HEADER，1=PAYLOAD，2=DROP |
| `rx_handshake`、`axis_master_tlast`、`image_write_valid` | 配合 `rx_state==1`，确认当前拍是有效的图像命令payload |
| `image_rx_header_valid` | 图像命令Header格式检查结果 |
| **`image_shadow_config[159:0]`** | PCIe端已收到并暂存的完整五字参数 |
| `image_queued_valid` | 有完整提交快照排队等待发往像素域 |
| `image_ready_sync` | PCIe域内同步后的像素模块就绪状态 |
| `image_ack_sync` | PCIe域内同步后的配置应用ACK |

`command_value` 在Header和无效拍也会变化，必须结合有效payload条件解释。
`image_shadow_config` 在接收配置字的有效拍之后更新。
Core 1 / Trigger Port 0保留 `u_pcie_dam_ctrl/image_cfg_apply` 和顶层 `image_cfg_busy`：实际发出快照时apply翻转，等待完成期间busy有效。

## 判断是否生效

在 **Core 0 / Trigger Port 1** 看 **`image_cfg_active[159:0]`**，它是图像模块最终生效的完整参数。
同组的 `image_stage_vs` 是图像模块输入VS，低有效；配置在输入帧边界应用。
Core 0 / Trigger Port 0已有 `image_cfg_ack`、`image_cfg_pending` 和 `image_rst_n`，分别看应用确认、待应用和图像模块复位释放。

apply/ack是翻转握手，0→1与1→0都有效，不是单拍脉冲。
完整过程：有效payload接收 → shadow更新 → 提交排队 → apply翻转 → 像素域pending → 输入帧边界active更新、ack跟随该次提交 → PCIe域busy回0。
图像输出VS和DMA入口VS经过图像流水线延时，不能要求它们与配置应用ACK同拍。

## 五字对应关系

shadow和active使用相同位布局；active会把无效参数钳位并清除保留位，因此非法输入不一定原样出现在active。

| 写地址 | shadow/active位段 | 内容 |
| --- | --- | --- |
| `0x200` | `[31:0]` | word0，开关 |
| `0x204` | `[63:32]` | word1，DCP参数 |
| `0x208` | `[95:64]` | word2，降噪和原暗光/高光阈值 |
| `0x20C` | `[127:96]` | word3，CLAHE参数 |
| `0x210` | `[159:128]` | word4，CLAHE与高光保护参数 |

| 生效位段 | 含义 |
| --- | --- |
| `[0]`、`[1]`、`[2]`、`[3]` | DCP、Gamma、CLAHE、高光保护开关 |
| `[4]`、`[5]` | Y降噪、透射率双边滤波开关 |
| `[6]`、`[7]`、`[8]` | 天空、防光晕、阴影保护开关 |
| `[11]`、`[12]` | 原高光压制、原暗光增强开关 |
| `[39:32]`、`[47:40]`、`[55:48]` | TX_MIN、SKY_TX_MIN、OMEGA |
| `[72:64]`、`[78:73]` | 降噪混合量、SIGMA |
| `[87:80]`、`[95:88]` | 原暗光阈值、原高光阈值 |
| `[111:96]`、`[114:112]`、`[121:115]` | CLAHE裁剪阈值、强度、阴影增益 |
| `[135:128]`、`[143:136]`、`[152:144]` | CLAHE增益上限、高光膝点、高光斜率 |

写完五字必须写 `0x214` 非零值提交；仅修改shadow不会更新active。原 `0x140/0x150` 命令会自动提交。
例如原暗光96和原高光192均开启，最终active应满足 `[12:11]=2'b11`、`[87:80]=96`、`[95:88]=192`。

## 加载与验证边界

关闭旧的Inserter/Debugger会话，重新打开本工程引用的 `debug/image_pcie_debug.fic`。
新版本应有两个核、每核两个Trigger Port，并能看到shadow和active完整160位总线；旧窗口不会因磁盘文件更新自动刷新。
新增总线的RTL调试标记需要重新综合。使用这版RTL和FIC重新生成、下载匹配位流后，再用本FIC抓取新增信号；旧位流不能只换FIC就看到新探针。

同一RTL和同一FIC接线已在2026-10-16完成过Compile、综合、插核和Device Map：DRM **131.5/134**、LUT 24153/42800、FF 30872/64200、APM 51.5/84。
本次只复用该记录并静态核对文件/逐位接线，不重新运行编译；该资源结果没有包括后续用户自行修改，不等同于板上抓波或时序签核。
