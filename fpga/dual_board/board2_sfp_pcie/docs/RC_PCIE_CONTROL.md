> 当前工程：[主工程ZIP](../sfp_ddr_image_pice_handshake_20261019_header_keep_fix.zip)、[PCIe仿真库ZIP](../sfp_ddr_image_pice_handshake_20261019_pcie_sim.zip)。两包解压到同一父目录，合并同名工程文件夹；下述源码路径相对工程根目录。

> 2026-10-19 最新源码快照：已修复 Logos PCIe IP 的3DW包头 `tkeep=1111` 被图像命令过滤的问题。图像写入仍要求单DW数据拍 `tkeep=0001`，视频DMA原有解析条件不变。参数接收、生效和回传通过9项离线回归；此发布未运行PDS位流构建或硬件下载，不附带位流。需要自行重建匹配位流。当前FIC为 `IMAGE_ACK_RX_APPLIED_TX_20261017`，Core0端口133/161位，Core1端口244/245位，均256点。下方较早记录保留为历史说明。

# RC端控制PCIe与图像处理指南


适用版本：2026-10-17，`sfp_ddr_image_pice_handshake`，默认1920×1080 RGB565。工程中RC命令、五字参数与快照控制均在 pcie_dma_ctrl.v（`source/source/pcie_dma_ctrl.v`） 内，图像处理位于DDR读出后，HDMI和PCIe共用处理结果。

本文以当前RTL和 handshake_reference.h（`rc/handshake_reference.h`）、image_control_reference.h（`rc/image_control_reference.h`） 为准。新增 image_ack_reference.h（`rc/image_ack_reference.h`） 提供两阶段回传参考；这些C99头通过回调接入已有RC驱动，不包含PCIe枚举、DMA分配、BAR映射或现成用户态ioctl。

## 1. 先区分本版和旧快照

| 项目 | 本版接口 |
|---|---|
| 图像参数控制 | 已合入 `pcie_dma_ctrl`，没有独立 `pcie_image_control` |
| 原暗光/高光 | `0x140/0x150/0x160` 继续有效并自动提交 |
| 完整图像参数 | `0x200~0x210` 五字暂存，`0x214` 写非零提交 |
| 图像DMA完成 | 每个图像缓冲尾部64字节token |
| 停止完成 | `0x180` 登记的独立64字节STOP状态区 |
| 图像参数确认DMA | 可选 `0x218` 登记128字节，分 RECEIVED/APPLIED；旧 `0x220/0x224` 不使用 |
| BAR读回/MSI | 当前控制器未实现；不要用 `readl(BAR1)` 查询或flush |

像素域ACK继续用于内部跨域握手；启用新增 `0x218` 后，FPGA另向独立图像确认区回传 RECEIVED/APPLIED，见 [两阶段接口](IMAGE_PARAMETER_ACK.md)。图像确认不会写入STOP状态区。收到STOP cookie只说明DMA按停止协议排空，不能据此确认某次图像配置已在像素域生效。

## 2. RC驱动需要提供的基础能力

RC先完成设备枚举、申请资源、映射BAR1、启用Bus Master，并检查32位DMA mask设置。使用该PCIe设备的DMA API分配缓冲，提供设备可访问的DMA/IOVA地址。

当前RTL使用32位目的地址，不可直接截断64位DMA地址；CPU虚拟地址、普通malloc指针和随意选择的物理地址均不能代替DMA地址。每块缓冲在设备地址空间必须是连续范围，Scatter-Gather列表不能只登记第一页。

参考头要求这些平台回调：

| `rf_hs_ops` 回调 | 平台实现含义 |
|---|---|
| `bar_write32(ctx,offset,value)` | 按原IP端序映射发出有序的单DW BAR1写；value是本文逻辑值 |
| `read_word(ctx,slot,dw)` | 读取LE32完成标记并转换为逻辑32位值；slot 4代表STOP状态区 |
| `clear_marker(ctx,slot)` | 在软件仍拥有对应区域时清零恰好64字节标记 |
| `read_barrier(ctx)` | 完成有效标记检查后执行对应DMA读屏障 |
| `write_barrier(ctx)` | 清标记和移交所有权时执行对应DMA写屏障 |

Linux coherent适配通常需要 `READ_ONCE`、`le32_to_cpu`、`dma_rmb/dma_wmb` 和正确的MMIO写访问。streaming/non-coherent映射还需配套DMA sync，不能仅用volatile代替。

现有 `board_v8` 应用的HS2 ioctl接口与较早的host驱动不能假定ABI相同；本次只更新FPGA板卡二与说明。实际应用若没有任意图像参数写入口，需要在其已有驱动/控制接口中适配上述写回调。

## 3. BAR1所有已实现寄存器

所有偏移均为字节偏移，值为逻辑32位值。每次发对齐的1 DW MWr32，FirstBE=F、LastBE=0；不要将四次地址登记合成Length=4的TLP。

| 偏移 | 写入值 | 用途 |
|---|---|---|
| `0x110` | 一个图像缓冲DMA地址 | 停止时依次登记slot 0、1、2、3；第四次检查范围后锁定 |
| `0x130` | 本次非零STOP cookie | 停止采集，结束已呈现TLP，向独立状态区回写cookie |
| `0x140` | 低8位高光阈值 | 开启原高光压制并自动提交 |
| `0x150` | 低8位暗光阈值 | 开启原暗光增强并自动提交 |
| `0x160` | 通常写0 | 关闭上述两路并清阈值，保留其他暂存算法参数，自动提交 |
| `0x170` | 读到的完整帧token | 归还匹配的HELD槽 |
| `0x180` | 独立STOP状态区DMA地址 | 四个地址登记成功后、停止状态下设置 |
| `0x190` | 1 | START，等待下一完整视频帧 |
| `0x1A0` | 1 | 停止且发送空闲时清DMA地址配置，不重置token序号，不清图像参数 |
| `0x200` | word0 | 图像开关暂存 |
| `0x204` | word1 | DCP参数暂存 |
| `0x208` | word2 | Y降噪及原暗光/高光阈值暂存 |
| `0x20C` | word3 | CLAHE参数暂存 |
| `0x210` | word4 | CLAHE/高光保护参数暂存 |
| `0x214` | 非零；回传模式用唯一请求编号 | 提交五字快照，写0不提交；回传模式先收齐全部5字 |
| `0x218` | 独立128字节图像确认区DMA地址 | 新增；原地址配置后、STOP自检完成后、START前登记；STOP后须重新登记 |

未定义寄存器没有有效控制行为，非法命令也没有错误BAR读回。

## 4. 先验证字节序

接收逻辑为 `command_value=endian32(axis_master_tdata[31:0])`。逻辑START=1在IP用户RX口应对应 `0x01000000`；逻辑地址 `0x12340000` 对应 `0x00003412`。

应在 `bar_write32` 适配层统一验证实际 `writel()` 与IP字节通道的关系，不能在所有调用处再盲目加一次bswap。第一次联调用非对称STOP cookie `0x12345678`，并核对token=4在ARM内存中的字节序为 `04 00 00 00`，重复16次。全0或全1不能证明端序正确。

像素字节序也要用RGB565彩条单独核对，不能用控制token的检查代替像素颜色检查。

## 5. 四缓冲内存布局

```text
每个slot：
 DMA基址 + 0x000000 ┌────────────────────────┐
                   │ RGB565图像             │ 1920×1080×2 = 0x3F4800
 DMA基址 + 0x3F4800 ├────────────────────────┤
                   │ 16个相同LE32 token     │ 64字节
 DMA基址 + 0x3F4840 └────────────────────────┘

另外分配独立64字节STOP状态区，保存16个相同LE32 cookie。
```

每个slot最少4147264字节，可向平台页大小取整。四个图像范围与STOP范围均非零、64字节对齐、互不重叠，整个范围不能超过32位DMA地址空间。

`token=(allocation_sequence<<2)|slot`。slot为0～3，sequence从1开始，首次slot 0的token为4。序号可因丢帧跳号，不等于成功帧计数；STOP/START/清配置不重置序号。达到 `0x3fffffff` 后需要受控复位恢复，不允许自然回绕。

## 6. 首次配置和启动

1. 确保设备在已知复位后的空闲状态；如果旧会话可能仍在DMA，先完成旧会话STOP或平台验证的复位/排空。
2. 分配四块图像缓冲与独立STOP状态区，初始化软件token和消费者持有表。
3. 清零四个尾标记与STOP区，执行必要DMA写屏障。
4. 同一驱动锁内依次发 `0x1A0=1`、四次 `0x110=addr[0..3]`、`0x180=stop_addr`。
5. 可在空闲态发一次STOP(cookie)，等待完整cookie回写，验证地址配置、EP→RC写通路和端序。
6. 若启用图像确认，先登记 `0x218`；再下发完整五字参数，以唯一非零cookie提交。首次调试建议原图旁路。
7. 写 `0x190=1`，等待下一个完整视频帧，轮询帧尾标记。

配置失败后在空闲态清配置并重新登记全部地址，不要猜测登记到了第几个槽。图像参数可以在DMA未START时设置；没有输入视频时会保持待处理，直到恢复帧边界。

## 7. 收帧与归还

RC只检查软件当前未持有的slot。16个LE32标记必须全部相同，sequence非零、低两位等于slot，且token不是该槽上次已接收的值；不能只看到第一个DW更新就读取图像。

识别完整标记后执行DMA读屏障，再向CPU、RGA、NPU或显示模块交付 `slot+token+图像地址`。所有异步消费者都应计入持有关系，最后一个消费者结束后才能归还。

归还顺序：

```text
全部消费者完成
 → 软件仍持有时清零64字节尾标记
 → DMA写屏障/必要sync
 → 写0x170=完整token
 → 软件不再访问该槽，等待下次新token
```

归还可乱序。若四槽都未归还，FPGA跳过新帧而不覆盖HELD槽，这是正常保护。DRM提交返回或NPU入队返回不等于消费完成，需要等待对应完成事件/fence。

`rf_hs_poll_frame()` 返回1才授予软件帧所有权；`rf_hs_release()` 应在最后一个消费者完成后调用。不要在RELEASE后再次清该槽标记。

## 8. 只控制原暗光和高光

通过已验证端序的原BAR写入口发逻辑值：

```c
bar_write32(ctx, 0x150, 96);   /* 原暗光增强，暗阈值96，自动提交 */
bar_write32(ctx, 0x140, 192);  /* 原高光压制，亮阈值192，自动提交 */
/* 关闭这两路： */
bar_write32(ctx, 0x160, 0);
```

暗阈值越大，参与提亮的暗像素越多；亮阈值越小，参与压暗的亮像素越多。原曲线增减量除以8，并饱和到0～255。

这两条开启命令各自自动提交，可能在不同帧生效。要求两路同一帧切换时，使用下一节完整五字配置。原高光压制与高光保护是不同功能。

## 9. 完整五字配置

| 字 | 位 | 功能 | 默认/处理范围 |
|---|---|---|---|
| word0 | 0 | DCP去雾 | 0/1，默认0 |
| word0 | 1 | Gamma | 0/1，默认0，原固定LUT |
| word0 | 2 | CLAHE | 0/1，默认0 |
| word0 | 3 | 高光保护 | 0/1，默认0 |
| word0 | 4 | Y双边降噪 | 0/1，默认0 |
| word0 | 5 | DCP透射率双边滤波 | 0/1，默认0 |
| word0 | 6/7/8 | 天空/防光晕/阴影保护 | 默认均1，仅随所属主算法起作用 |
| word0 | 10:9 | 保留 | 固定清0 |
| word0 | 11 | 原高光压制 | 0/1，默认0 |
| word0 | 12 | 原暗光增强 | 0/1，默认0 |
| word1 | 7:0 | TX_MIN | 默认52，0钳为1 |
| word1 | 15:8 | SKY_TX_MIN | 默认128，小于TX_MIN时钳到TX_MIN |
| word1 | 23:16 | OMEGA | 默认243，0～255 |
| word2 | 8:0 | DENOISE_MIX | 默认256，最大256；0取原Y |
| word2 | 14:9 | DENOISE_SIGMA | 默认32，支持16/32/48，其余按32 |
| word2 | 23:16 | 原暗光阈值 | 默认0，0～255 |
| word2 | 31:24 | 原高光阈值 | 默认0，0～255 |
| word3 | 15:0 | CLAHE_CLIP_LIMIT | 默认400，0钳为1 |
| word3 | 18:16 | CLAHE_STRENGTH | 默认3，最大4；0取原Y |
| word3 | 25:19 | CLAHE_SHADOW_GAIN | 默认64，最大64 |
| word4 | 7:0 | CLAHE_BOOST_LIMIT | 默认64，0～255 |
| word4 | 15:8 | HIGHLIGHT_KNEE | 默认192，0～255 |
| word4 | 24:16 | HIGHLIGHT_SLOPE | 默认128，最大256 |

其余位写0。分辨率、行消隐、CAPABLE、CLAHE_DONOR_MODE、BILATERAL_GUIDE_ENABLE和Gamma表属于编译结构，不能通过这些BAR字改变。

提交顺序必须在同一锁/控制线程下完成。下面固定写1是旧兼容模式；需要两次回复时以唯一非零cookie替代1，完整流程见第14节：

```c
bar_write32(ctx, 0x200, word[0]);
bar_write32(ctx, 0x204, word[1]);
bar_write32(ctx, 0x208, word[2]);
bar_write32(ctx, 0x20c, word[3]);
bar_write32(ctx, 0x210, word[4]);
bar_write32(ctx, 0x214, 1);       /* 整包提交，不是主算法使能位 */
```

单独改word0不会立即生效，最后必须提交。提交会复制整包，后续编辑不会改变在途包。源域等待像素域帧边界ACK，期间最多再留一份最新待处理快照，中间连续提交可以被合并。

旧暗光/高光命令也基于当前暂存整包提交，所以不能让其他线程在五字写入中间插入旧命令。正常单次调参后至少观察3个完整视频帧，让CLAHE重新建立统计；等待固定帧数只是画质观察策略；软件确认应使用新128字节确认区里的匹配cookie。

## 10. 可直接填入的参数起点

以下均是逻辑十六进制值，为联调起点，未作本版板上画质验收。每行写完五字，再提交；旧兼容模式写 `0x214=1`，两阶段回传模式写本次唯一非零cookie。

| 场景 | word0 | word1 | word2 | word3 | word4 |
|---|---|---|---|---|---|
| 原图旁路 | `000001C0` | `00F38034` | `00004100` | `02030190` | `0080C040` |
| 仅原暗光96 | `000011C0` | `00F38034` | `00604100` | `02030190` | `0080C040` |
| 仅原高光192 | `000009C0` | `00F38034` | `C0004100` | `02030190` | `0080C040` |
| 原暗光96+原高光192 | `000019C0` | `00F38034` | `C0604100` | `02030190` | `0080C040` |
| 雾天，仅DCP | `000001C1` | `00F38034` | `00004100` | `02030190` | `0080C040` |
| DCP+Gamma+CLAHE+高光保护 | `000001CF` | `00F38034` | `00004100` | `02030190` | `0080C040` |
| 夜间，Y降噪+Gamma+高光保护 | `000001DA` | `00F38034` | `00004080` | `01020100` | `0080C018` |
| 夜间，再开温和CLAHE | `000001DE` | `00F38034` | `00004080` | `01020100` | `0080C018` |

夜间先比较CLAHE开关，避免增强后噪声过强；Gamma和原暗光叠加会累计提亮。输入已饱和的高光细节不能由压暗处理恢复。

## 11. 接入现有C参考接口

下面是驱动控制流程片段，`ops`、`ctx`、`dma_addr[4]`、`stop_dma` 均由实际平台适配层提供；DMA地址不能使用示例常数代替。

```c
#include "handshake_reference.h"
#include "image_control_reference.h"

struct rf_hs session = {0};
struct rf_image_config cfg = rf_image_default();

/* 已知设备空闲，已有合法DMA分配和平台回调；全流程由同一驱动锁串行。 */
if (rf_hs_init(&session, ops, ctx) != 0)
    return -1;
if (rf_hs_configure(&session, dma_addr, stop_dma) != 0)
    return -1;

rf_image_set_tone(&cfg, 1, 96, 1, 192);
if (rf_image_submit(&session, &cfg) != 0)
    return -1;  /* 返回0仅代表写入回调已调用，非硬件配置ACK */

if (rf_hs_start(&session) != 0)
    return -1;
```

收帧循环对slot调用 `rf_hs_poll_frame(&session,slot,&token)`；返回1后启动消费者，最后一个消费者完成时调用 `rf_hs_release(&session,slot,token)`。

STOP用 `rf_hs_request_stop(&session,cookie)` 后轮询 `rf_hs_poll_stop()`，返回1才完成设备侧停止确认。所有消费者也结束后，才允许释放/更换DMA内存。

## 12. 停止、重启和故障恢复

1. 停止提交新的显示/推理任务，保留所有DMA映射。
2. 清STOP状态区、执行写屏障，发 `0x130=本次非零且不重复cookie`。
3. 等16个DW全部匹配cookie，再执行读屏障。
4. 等已有CPU/RGA/NPU/显示消费者结束。
5. 同地址重启时清标记并START；换地址时清配置、重新登记所有地址再START。

STOP超时不能当成停止完成，也不能只等固定毫秒数就释放映射。链路故障时走平台验证过的复位/禁用/排空流程。已经呈现的旧TLP可以先结束，帧尾可能先于STOP应答到达。

视频暂时消失、PLL失锁或DDR读侧复位时，像素链路暂停；只要PCIe应用控制器没有复位，最近一次提交的图像配置会在恢复后重送。PCIe核复位则重建DMA会话和图像配置，禁止重放旧会话的RELEASE。

## 13. 常见现象与检查位置

| 现象 | 先检查 |
|---|---|
| 只有四帧后停更 | 四个HELD槽是否都未RELEASE，异步消费者是否一直持有 |
| 写word0没有变化 | 是否写0x214，是否有恢复视频帧，是否误用旧0x220/224协议 |
| 暗光/高光没变化 | word0[12]/[11]及word2阈值，或旧命令的端序；阈值是否覆盖当前Y |
| 调参后不是立刻生效 | 帧边界提交、在途/待处理槽和固定流水延迟 |
| 图片颜色不对 | RGB565字节序、R/G/B截取和接收端重建方式 |
| 首尾像素错位 | DDR读响应与ddr_stream_*、HS等长延迟、DE像素数 |
| STOP不返回 | 状态地址登记、cookie端序、TX反压、链路状态 |
| 无法读取BAR状态 | 本版没有BAR读完成器，使用规定的DMA标记和FPGA调试信号 |

FPGA端可观察 `running/stop_pending/buffer_state/buffer_token/dropped_frames/invalid_releases`；图像控制看 `image_shadow_config/image_queued_valid/image_cfg_data/image_cfg_apply/image_cfg_busy`，像素域看 `image_cfg_active/image_cfg_pending/image_cfg_ack/image_ready`。当前PDS引用 `debug/image_pcie_debug.fic`，Core0端口为133/161位、Core1为244/245位、各256点；现有FIC已包含接收/生效cookie、掩码、回传地址使能及发送事件。见 [FIC说明](IMAGE_PCIE_FIC.md)。

2026-10-17版本通过7项RTL仿真和2项C99测试；本次未运行新的PDS综合/映射、位流生成或下载，未完成本版ARM回传联调和板上画质验证。历史PDS结果仅对应旧版本。详细证据见 合并验证记录（`docs/PCIE_CONTROL_MERGE_VALIDATION.md`）。

## 14. 参数已接收与已生效，两次独立回复

完整适配契约以 [IMAGE_PARAMETER_ACK.md](IMAGE_PARAMETER_ACK.md) 为准。ARM另分配独立128字节DMA区并写 `0x218`；先清区、执行DMA写屏障，顺序写全5字，再向 `0x214` 写本次唯一非零cookie。

| 地址 | 必须完整匹配的内容 | UI可报告状态 |
|---|---|---|
| 图像确认区 `+0x00` | 16个LE32全部等于本次cookie | 参数已接收 |
| 图像确认区 `+0x40` | 16个LE32全部等于本次cookie，且已确认RECEIVED | 参数已生效 |

RECEIVED只在完整五字被接收并接受COMMIT后产生，APPLIED只在对应像素帧ACK返回后产生。串行提交，上一请求APPLIED后再发下一次；超时保留DMA映射，STOP确认排空后再恢复。原STOP一旦被接受会禁用图像确认区，STOP ACK后重新写0x218才能继续回传。旧暗光/高光命令不产生完整五字确认，应改为五字快照控制同一开关/阈值。

仓库 `tools/pcie_v2` 的现有7.0驱动是单向提交参考；本次不修改该驱动或Qt程序。接入新协议时，BAR1映射必须覆盖到0x21B（至少0x21C字节），增加128字节DMA分配、cookie及两阶段轮询，并扩展相应UAPI能力/状态返回。不要把现有SUBMIT_IMAGE成功返回当作RECEIVED或APPLIED；无回传能力时继续显示“仅提交”。新接口不是旧0x220/0x224状态页。
