> 当前工程：[主工程ZIP](../sfp_ddr_image_pice_handshake_20261019_header_keep_fix.zip)、[PCIe仿真库ZIP](../sfp_ddr_image_pice_handshake_20261019_pcie_sim.zip)。两包解压到同一父目录，合并同名工程文件夹；下述源码路径相对工程根目录。

> 2026-10-19发布：本FIC已包含在此次工程ZIP中；下面“仅桌面、未上传”的描述是2026-10-17更新时的历史状态。包头修正没有改变本FIC探针布局。

# 图像参数接收与生效探针

2026-10-17，FIC标识：`IMAGE_ACK_RX_APPLIED_TX_20261017`。

工程当前引用 **`debug/image_pcie_debug.fic`**，引用路径不变。
`synthesize/hdmi_ddr_pcie_syn.fic` 同步为相同内容，避免从旧路径打开时混用旧探针。
本次保留已有参数/视频探针，在PCIe核加入两阶段回传状态及发送完成事件。手写DMA模块只增加观察信号和调试属性，不改功能状态机、端序、握手或生成IP。仅修改桌面工程，不上传GitHub。
**本次未启动PDS综合、布局布线或bit生成；只运行了发送事件的RTL仿真核对。**

## FIC分组

| 核 | 采样时钟 | Trigger Port 0 | Trigger Port 1 | 深度 |
| --- | --- | --- | --- | ---: |
| Core 0 | `nt_pix_clk_out`（像素时钟） | 原图像入口/FIFO写入观察，133位 | 生效参数160位、图像模块输入VS 1位 | 256 |
| Core 1 | `pclk_div2`（PCIe用户时钟） | 原172位 + 回传状态/发送事件72位，共244位 | 原213位 + 本次回包cookie32位，共245位 | 256 |

每核两个Trigger Port，Data Same As Trigger；保留原有两组图像/FIFO探针。
逐位接线见 probe_manifest.json（`docs/verification/20261017/ack_probes/probe_manifest.json`，位于工程ZIP内）。

## 两阶段回传的新增探针

所有新增信号均在Core 1 / `pclk_div2`，层次前缀为`u_pcie_dam_ctrl/`。

| 端口 | 位段 | 信号 | 判读 |
| --- | --- | --- | --- |
| TP0 | `[203:172]` | `image_received_cookie[31:0]` | 完整五字接收并接受COMMIT后的请求编号；内部RECEIVED状态 |
| TP0 | `[235:204]` | `image_applied_cookie[31:0]` | 对应像素帧ACK返回后的请求编号；内部APPLIED状态 |
| TP0 | `[240:236]` | `image_rx_word_mask[4:0]` | WORD0开始新包，收到各WORD置对应位；COMMIT后清零。收齐前为11111，之后清零是正常现象 |
| TP0 | `[241]` | `image_return_addr_valid` | 0x218回传地址已启用；有效STOP后清零 |
| TP0 | `[242]` | `dbg_image_received_sent_event` | RECEIVED回包最后一拍在PCIe TX AXIS握手的单拍事件 |
| TP0 | `[243]` | `dbg_image_applied_sent_event` | APPLIED回包最后一拍在PCIe TX AXIS握手的单拍事件 |
| TP1 | `[244:213]` | `image_packet_cookie[31:0]` | 当前/最近一次回包快照编号；应在上述发送事件为1的采样点解释 |

`image_received_cookie`变化不等于ARM已经收到；`image_applied_cookie`变化也不等于APPLIED回包已经发完。两个sent_event证明该回包已被PCIe核用户TX接口完整接受，仍不能代替ARM目标DMA区16个DW全部匹配的检查。

推荐先触发`image_received_cookie`等于本次cookie，检查完整参数确实接收；再分别触发两个sent_event=1，查看同一采样点`image_packet_cookie`是否为本次编号。**这几项状态/事件在TU0，当前激活Trigger Condition要包含TU0，例如直接设为TU0；Function为==，未关心位填X。** TP1原接收条件仍属于TU1，比较值需要按新245位宽重新生成，旧213位整口掩码不能直接复用。不要同时要求sent_event=1与rx_handshake=1，两者是独立时刻。

原有所有triggerChannel索引保持不变，新信号在各端口末尾追加；Core 1的全核busInserter编号已按新宽度重算。图像确认区的登记、唯一cookie、两次回传和STOP重绑定规则见 [ARM两阶段接口](IMAGE_PARAMETER_ACK.md)。

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
新版本应有两个核、每核两个Trigger Port；Core 1宽度为244/245位，并能看到新增cookie/mask/valid/发送事件。旧窗口不会因磁盘文件更新自动刷新。
新增总线的RTL调试标记需要重新综合。使用这版RTL和FIC重新生成、下载匹配位流后，再用本FIC抓取新增信号；旧位流不能只换FIC就看到新探针。

原385位PCIe核配置的历史映射为DRM 131.5/134，仅供旧版本参考；本次PCIe核总宽增加104位到489位，没有重新综合/插核/映射，新增布局的资源和时序须由后续PDS构建确认。当前只做逐位FIC静态核对及发送事件仿真。

本次静态核对与仿真证据见验证摘要（`docs/verification/20261017/ack_probes/validation_summary.json`，位于工程ZIP内）：逐位通道/总线映射通过，24次RECEIVED与22次APPLIED发送事件均与外部TX包记分板一致；原有背压、VS、STOP和配置恢复用例通过。
