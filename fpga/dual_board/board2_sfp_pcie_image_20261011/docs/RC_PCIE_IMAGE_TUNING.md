# RC 控制 PCIe 与图像处理调参指南

适用版本：2026-10-11 板卡二 `sfp_ddr_img_pice_handshake`，PDS 顶层 `hdmi_ddr_pcie`。本文依据本版顶层接线、`pcie_dma_ctrl.v`、`pcie_image_control.v`、`image_runtime_control.v`、处理链 RTL、两个 RC 参考头及用户提供的 [PCIE_IMAGE_CONTROL.md](PCIE_IMAGE_CONTROL.md) 整理。寄存器行为来自工程；场景建议是待上板验证的起始配置。

## 1. RC 控制什么

RC 是 PCIe Root Complex，这里是鲁班猫二 Linux 主机；板卡二 FPGA 是 EP。RC 控制两件独立的事：图像 DMA 缓冲所有权，以及图像处理参数。用户程序应通过实际 PCIe 驱动提交命令、取得 DMA 帧；本工程只提供适配参考头，没有新增现成 ioctl 驱动或命令行工具。

板卡二把自身 HDMI 通过 SFP 发送给板卡一；反向接收板卡一拼好的 1920×1080 RGB565 视频。DDR 存原图，DDR 读出后才做图像处理。HDMI 与 PCIe 使用同一处理结果；关闭图像 DMA 不等于关闭 HDMI 或图像参数回写。四个 DMA 缓冲是同一合成视频的四个槽，参数作用于整帧，不能按槽或象限分别设置。

```text
申请 DMA 内存 → 登记四槽 / STOP 页 / 配置状态页
       ├─ START → 完整帧 token → 消费 → RELEASE(token) → 下一帧
       └─ 写五个参数字 → COMMIT(cookie) → 帧边界生效 → DMA 状态确认
```

所有 BAR 命令都是 **BAR1 字节偏移、对齐的 32 位单 DW Memory Write**，FirstBE=0xF、LastBE=0。不能把四个地址或五个参数字合并为多 DW TLP。本工程没有 BAR Memory Read Completion，禁止用 `readl(BAR1)` 查状态或做 posted-write flush；状态从主机 DMA 内存读取。

## 2. RC 内存与驱动准备

驱动枚举并申请 PCIe 资源、映射 BAR1、启用 Bus Master，设置并检查 `dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32))`。登记的是该设备 DMA API 返回的 `dma_addr_t` / IOVA，不是 CPU 指针；必须先检查完整地址范围，再转换为 32 位。首版适配建议采用 coherent 分配。

| 内存 | 数量 | 最小有效大小 | FPGA 用途 |
| --- | --- | --- | --- |
| 图像缓冲 | 4 | 每块 `0x3F4840` = 4,147,264 B | 前 `0x3F4800` 为 1920×1080 RGB565，尾部 64 B 为完成 token |
| STOP 应答页 | 1 | 64 B | 16 个相同的 LE32 STOP cookie |
| 图像配置状态页 | 1 | 64 B | 16 DW 的配置记录，与 STOP 页格式不同 |

六块有效范围互不重叠、非零、64 B 对齐，末地址也不能越过 4 GiB。分配尺寸可以按页向上取整。每个图像范围需要连续 DMA/IOVA，不能拿普通 SG 链第一页地址代替整帧映射。配置页即使图像 STOP 也可能继续被 QUERY/配置回写，不能随 STOP 应答就释放。

coherent 只解决缓存一致性，不省略完成标记校验和 DMA 内存屏障。Linux 地址与 coherent 分配规则见 [DMA API 官方说明](https://docs.kernel.org/core-api/dma-api-howto.html)；有序 BAR 写及平台映射见 [MMIO 官方说明](https://docs.kernel.org/driver-api/device-io.html)。

## 3. BAR1 命令表

| 偏移 | 逻辑值 | 本版作用 |
| --- | --- | --- |
| `0x110` | 单个图像 DMA 地址 | 停止时依次写四次，登记 slot 0、1、2、3 |
| `0x130` | 非零且此次唯一 STOP cookie | 停止图像 DMA，完成已呈现 TLP 后向 STOP 页回写 |
| `0x170` | 完整帧 token | 归还对应 HELD 槽；旧值、重复值或未完成槽归还无效 |
| `0x180` | STOP 页 DMA 地址 | 四图像地址登记完成后，在停止态登记 |
| `0x190` | 1 | START，等待完整帧边界开始图像 DMA |
| `0x1A0` | 1 | 停止且发送空闲时清图像配置；不重置 token 序号 |
| `0x200/204/208/20C/210` | word0～4 | 图像参数暂存区 |
| `0x214` | 非零 commit cookie | 复制五字整包，等待图像核心帧边界锁存 |
| `0x220` | 配置状态页 DMA 地址 | 独立状态页登记，不与图像/STOP 页重叠 |
| `0x224` | 非零 query cookie | 查询当前配置状态，结果 DMA 到配置页 |

旧 `0x140/150/160` 不再控制新版增强链。旧握手文档保留这些行用于历史参考，实际调用以本表为准。

RX RTL 对 payload 执行 `endian32()`。驱动的 `writel()` 是否已经符合 IP 用户口的字节映射，必须在首次联调验证，不能默认再做一次交换。用非对称 STOP cookie `0x12345678` 检查 ARM 内存为 `78 56 34 12` 重复 16 次；首槽首次 token 通常为 4，完成标记为 `04 00 00 00` 重复 16 次。

## 4. 首次启动、收帧和停止

1. 确保 EP 是本会话已知空闲状态；若驱动重载但旧 DMA 状态未知，先完成旧会话 STOP 或执行平台已验证的复位/排空流程。
2. 申请并保持六块 DMA 内存，清四个图像尾标记、STOP 页和配置页，执行初始化所需 DMA 写屏障。
3. 在串行控制路径写 `0x1A0=1`，再四次 `0x110=addr[0..3]`，再 `0x180=stop_dma`。地址登记失败时停态清配置并完整重新登记，不能猜登记到了第几个槽。
4. 写 `0x220=cfg_ack_dma`；用新 query cookie 写 `0x224`，等相同 cookie 的完整配置状态记录，检查登记 flag、拒绝状态与错误计数。
5. 可先 STOP 自检并等待 16 DW cookie 完整匹配，验证 EP→RC 写入和端序，再提交全旁路参数。没有视频帧时参数可能等待输入，不能把它当作提交丢失。
6. 写 `0x190=1`。完整视频帧到来后，RC 轮询未被软件持有槽的尾部标记；四槽全部 HELD 且不归还时，FPGA 跳帧，这是正常保护行为。

完整 token 为 `(allocation_sequence << 2) | slot`，slot=0～3，sequence 非零且允许跳号。认领一帧前读满尾部 16 个 LE32，要求全部相等、非零、槽号正确且不是该槽已接收的旧 token；随后 `dma_rmb()`，再读像素并交给消费者。只看到首 DW 更新不能判定完整帧。

每槽维护 CPU/RGA/NPU/显示消费者引用或 fence。全部消费者结束后，**仍拥有该槽时清尾部 64 B → `dma_wmb()` / 必要同步 → 写 `0x170=原 token` → 放弃软件所有权**。归还后不得继续读写该槽；显示提交返回不代表已经退出扫描。可以立即归还尚未开始消费的旧 READY 帧，不能归还仍在显示或推理的帧。

停止时暂停新增消费任务，清 STOP 页并做写屏障，写新 STOP cookie，等 16 DW 全部匹配后做读屏障，再等消费者退出。超时保持映射并进入恢复流程，不以固定等待时间代替 ACK。图像地址可在已停止后清配置重登记；普通 START/STOP 不重置 token sequence。

卸载还需停止配置命令并完成平台支持的 EP 复位/总线 DMA 排空，再释放配置页；图像 STOP 应答只确认图像 DMA 停止。token sequence 到 `0x3fffffff` 后需要受控复位换新会话；按每秒 60 次分配约 207 天，不能依赖自然回绕。

## 5. 参数提交与生效确认

用一个控制线程或 mutex 串行写五字和 COMMIT，防止不同调用者混包；只允许一项未完成提交。不要在持有 spinlock 时等待帧边界。cookie 本会话非零且不复用；建议由会话统一分配器分配 commit/query/STOP cookie，记录各事务类型。

```c
/* ordered_bar1_write 接收逻辑值；实际端序在驱动适配层验证。
 * cfg_ack_dma 已由 DMA API 分配、检查并登记，不能用示例常量地址。 */
ordered_bar1_write(0x200, 0x000001E1u);
ordered_bar1_write(0x204, 0x00E0A060u);
ordered_bar1_write(0x208, 0x00004100u);
ordered_bar1_write(0x20C, 0x02030190u);
ordered_bar1_write(0x210, 0x0080C040u);
ordered_bar1_write(0x214, new_commit_cookie);
```

这组只开启 DCP、天空保护与 DCP t 滤波。参考 [rc/image_control_reference.h](../rc/image_control_reference.h) 的 `rf_img_sanitize`、`rf_img_register_ack`、`rf_img_submit`、`rf_img_query`、`rf_img_poll`、`rf_img_is_applied`。回调接入实际驱动：单 DW 读取用 `READ_ONCE` 加 `le32_to_cpu`，读屏障用 `dma_rmb`，BAR 写用经过端序验证的有序 32 位 MMIO。

ACCEPTED 只说明接收整包，APPLIED 才说明帧边界锁存。每条完整记录按以下顺序判定：读 DW1 sequence → `dma_rmb()` → 复制 16 DW → `dma_rmb()` → 重读 DW1；要求前后及快照 sequence 一致、DW15 为 sequence 反码、magic 正确。不要把同 cookie 的 ACCEPTED 尾标记误认作下一条 APPLIED 完成。

| DW | 配置状态页字段 |
| --- | --- |
| 0 | `0x494D4743`，布局 v1 |
| 1 | 记录 sequence |
| 2 / 3 / 4 | 本记录 cookie / status / flags |
| 5～9 | 当前已生效并钳位的 word0～4 |
| 10 / 11 | accepted_cookie / applied_cookie |
| 12 / 13 / 14 | rejected_count / reply_overflow_count / last_rejected_cookie |
| 15 | `~sequence` |

status：0 RESET、1 ACCEPTED、2 APPLIED、3 BUSY、4 BAD_COMMAND、5 BAD_ACK_ADDRESS、6 REPLY_OVERFLOW、7 WAITING_INPUT、8 RESTORING。flags：bit0 图像复位释放，bit1 首帧曾就绪，bit2 pending，bit3 提交在途，bit4 配置页登记，bit5 曾溢出，bit6 曾提交，bit7 恢复中。

确认本次配置时检查 `applied_cookie==commit_cookie`、active 五字等于软件钳位后的目标，且 flags[0:1] 为 1、flags[2/3/7] 为 0。状态页可能被后续回复覆盖，超时用**新的** query cookie 查询，并匹配查询记录 DW2 和原提交的 DW11；不要反复重交同 cookie，也不要仅凭 sequence 变化宣布成功。轮询应有超时和退避，避免把八条回复队列灌满。

参数在下一次处理核心输入 VS 下降沿整组生效；从该边界到公共输出为 4497 个像素时钟，通知还经过 CDC 与 PCIe 回写，RC 收到 ACK 的时刻不能精确代表某个输出像素。算法统计还需要稳定，改变 DCP/Gamma/降噪/clip 后至少观察三个完整视频帧。

首帧未到时查询可显示 WAITING_INPUT；首帧后的 ready 是粘滞标记，不是实时 SFP 存活检测。SFP 断流时 DDR 可能重放旧图，不能由 ready=1 判定光口仍传输。图像域复位会恢复最后接受的整包；PCIe 会话复位则重新登记全部 DMA 控制地址、cookie 状态与参数，不重放旧 RELEASE。

## 6. 开关和参数怎么选

固定顺序为 DCP → RGB/YCbCr → Y 双边降噪 → Gamma → CLAHE → 原暗光/高光曲线 → 高光保护 → RGB。主开关全关时绕过颜色转换往返，输出保持还原 RGB565 对应的 RGB888 低位补零，流水延迟不变。

| word0 位 | 开关 | 使用说明 |
| --- | --- | --- |
| 0 | DCP 去雾 | 有雾、灰白低对比时先单独试；夜间默认关 |
| 1 | Gamma | 固定 LUT 提亮暗部；没有运行时 gamma 指数参数 |
| 2 | CLAHE | 局部对比；暗噪声或光晕明显时减强度/增益 |
| 3 | 高光保护 | 混回增强前 Y 后使用 soft knee；减少增强造成的亮部过冲 |
| 4 | Y 降噪 | 在 Gamma/CLAHE 前处理；过强会损失纹理 |
| 5 / 6 | DCP t 联合滤波 / 天空保护 | DCP 开启时使用；天空保护限制天空区域去雾增益 |
| 7 / 8 | CLAHE 防光晕 / 阴影保护 | CLAHE 开启时建议先保持开 |
| 9 / 10 | 保留 | 写 0；旧 A 冻结/原始 A 旁路已移除 |
| 11 / 12 | 原高光压制 / 原暗光增强 | 简单亮度曲线，必须同时设置阈值；与高光保护独立 |

word0 有效掩码 `0x000019FF`。默认 `0x000001C0` 的三个保护位没有独立增强效果，所以它是主算法全旁路基线。

| 参数 | 范围 / 默认 | 建议调节方向 |
| --- | --- | --- |
| DCP TX_MIN | 1～255 / 52 | 先 96；噪声、色偏、过增强时往 112/128 提高下限 |
| SKY_TX_MIN | TX_MIN～255 / 128 | 天空多时先 160；天空过暗/不均时试 176～192 |
| OMEGA | 0～255 / 243 | 先 224；降低去雾强度可试 208，重雾再逐步提高 |
| DENOISE_MIX | 0～256 / 256 | 日间先 0～128；夜间先 128～192；256 全滤波 |
| DENOISE_SIGMA | 16/32/48 / 32 | 日间轻噪 16，夜间先 32；48 更易跨较大亮度差平滑，检查边缘损失 |
| CLAHE_CLIP_LIMIT | 1～65535 / 400 | 先 200；这是硬件直方图 bin 的计数阈值，不是 OpenCV 浮点 clipLimit |
| CLAHE_STRENGTH | 0～4 / 3 | 先 1；0 保持原 Y，画面平淡可逐步试 2 |
| CLAHE_SHADOW_GAIN | 0～64 / 64 | 先 32；暗部颗粒明显时试 16，不盲目提高 |
| CLAHE_BOOST_LIMIT | 0～255 / 64 | 先 32；约束提亮，夜间噪声大可试 16 |
| HIGHLIGHT_KNEE | 0～255 / 192 | 先 192；越低越早开始软压，车灯影响大可试 176 |
| HIGHLIGHT_SLOPE | 0～256 / 128 | 128 为膝点后半斜率，256 为单位斜率；过暗可提高到 192 |
| dark_threshold | 0～255 / 0 | bit12 开时先 96，阈值以下最多提升约阈值差的 1/8 |
| light_threshold | 0～255 / 0 | bit11 开时先 192，阈值以上最多压低约阈值差的 1/8 |

原曲线使用进入该级的 Y（已过 Gamma/CLAHE），不是传感器原始亮度；阈值 0 打开高光压制会作用到几乎全部亮度，应先写好阈值。Gamma 与原暗光增强会累计提亮，先选一种再评估。任何这些算法都不能恢复采集端已经饱和丢失的细节，也不替代曝光调节。

## 7. 按场景选起点

以下十六进制值均为逻辑 32 位值，先写五字再 COMMIT。建议先旁路确认链路正常，再只改一个功能，保存素材、参数和处理前后对比。推荐值未经本次板级画质或识别准确率验证。

| 场景 / 配置 | word0 | word1 | word2 | word3 | word4 |
| --- | --- | --- | --- | --- | --- |
| 日间正常、链路基线：全旁路 | `000001C0` | `00F38034` | `00004100` | `02030190` | `0080C040` |
| 阴天、逆光：温和 CLAHE + 高光保护 | `000001CC` | `00F38034` | `00004100` | `010100C8` | `0080C020` |
| 阴天/背光且暗部仍不足：上一行加 Gamma | `000001CE` | `00F38034` | `00004100` | `010100C8` | `0080C020` |
| 雾天第一步：仅 DCP + 天空 + t 滤波 | `000001E1` | `00E0A060` | `00004100` | `02030190` | `0080C040` |
| 雾天综合增强，逐项确认后再用 | `000001FF` | `00E0A060` | `00004080` | `010100C8` | `0080C020` |
| 夜间/隧道：降噪 + Gamma + 温和 CLAHE + 高光保护，DCP 关 | `000001DE` | `00F38034` | `000040C0` | `010100C8` | `0080C020` |
| 日间轻噪/雨天先试降噪，DCP 关 | `000001D0` | `00F38034` | `00002080` | `010100C8` | `0080C020` |
| 单独验证暗光增强，阈值 96 | `000011C0` | `00F38034` | `00604100` | `02030190` | `0080C040` |
| 单独验证高光压制，阈值 192 | `000009C0` | `00F38034` | `C0004100` | `02030190` | `0080C040` |
| 简单暗提亮 + 亮压低，阈值 96/192 | `000019C0` | `00F38034` | `C0604100` | `02030190` | `0080C040` |

雾天配置的 DCP 为 TX_MIN=96、SKY_TX_MIN=160、OMEGA=224；综合配置的降噪 mix=128、sigma=32。夜间 mix=192、sigma=32。温和 CLAHE 为 clip=200、strength=1、shadow gain=32、boost limit=32。雨天降噪起点 mix=128、sigma=16，**它不是去雨滴/雨线算法**；只有确实有雾才另外试 DCP。

四宫格可能同时包含白天、夜间和逆光视角。当前是一套全局参数与整幅统计，不能把某一路的最强去雾或提亮参数直接视为四路最优。以最容易出现噪声/饱和的象限约束全局强度；需要独立四路处理时应修改 FPGA 结构和控制协议。

用于 NPU 时，除肉眼观感，还应对比交通灯颜色、细线/小目标保留、误检漏检。不要仅因画面更亮就宣布识别变好。跨隧道/夜间切换时先验证温和档，避免依据一帧亮度频繁切换导致视觉闪动；本版没有自动场景分类控制器。

## 8. 参数字打包

```c
/* 参数已按范围钳位，所有保留位为 0。 */
word1 = tx_min | (sky_tx_min << 8) | (omega << 16);
word2 = denoise_mix | (sigma << 9)
      | (dark_threshold << 16) | (light_threshold << 24);
word3 = clip_limit | (strength << 16) | (shadow_gain << 19);
word4 = boost_limit | (knee << 8) | (slope << 16);
```

实现中使用 `uint32_t` 操作数，避免有符号左移；优先直接调用参考头的 sanitize。TX_MIN=0 钳到 1，SKY_TX_MIN 至少等于 TX_MIN；mix/slope 上限 256，sigma 非 16/32/48 时变 32；clip=0 变 1，strength/gain 上限 4/64。状态页读回的是硬件钳位值，应与软件钳位后的值比较。

分辨率、行消隐、CAPABLE、Gamma LUT、CLAHE_DONOR_MODE、BILATERAL_GUIDE_ENABLE 是编译结构，不是 BAR 参数。不要用本表去动态改帧大小；DMA 尾部偏移及缓冲大小也必须与实际编译的分辨率一致。

## 9. 联调排查

| 现象 | 优先检查 |
| --- | --- |
| RC 不收到帧 | PCIe 枚举/Bus Master、六块地址范围、START、完整 VS、DDR 初始化、SFP 输入；不读 BAR 状态 |
| 收四帧后停 | 四槽是否 HELD；消费者是否结束并按原 token RELEASE |
| 提交后没有画质变化 | word0 主开关、ACCEPTED 与 APPLIED 区别、active 五字、cookie、核心 VS 是否仍到来 |
| WAITING_INPUT / RESTORING | 首帧、图像 reset、恢复状态；不重交旧 cookie |
| 噪声放大、白边/光晕 | 先旁路排除数据错误；减 DCP 强度、CLAHE 强度/boost，适当开启 Y 降噪 |
| 天空过暗、颜色失真 | 天空保护、提高 TX_MIN/SKY_TX_MIN、降低 OMEGA；校验 RGB565 字节序 |
| 水平错位、短行、整屏抖动 | VS/HS/DE 对齐、SFP 水位/ful、`frame_error`、DDR/FIFO 及 DMA 丢帧；不能仅靠算法参数解决 |
| 配置应答超时 | 配置页存活、完整记录校验、队列溢出计数；新 QUERY 恢复 applied_cookie，不忙等灌队列 |
| STOP 后仍有配置页写入 | 配置状态 DMA 独立；卸载必须另行停止控制事务和执行平台复位/排空 |

FLA 可抓 `image_cfg_active`、pending/ack、`image_rst_n`、`post_frame_vsync/de`、`image_output_vs/de`。DCP A 固定使用平滑结果，原始 A 只作为逐帧统计输入；旧 a_freeze/a_bypass 信号不再适用。SFP ready 粘滞，链路存活需另查输入包/帧活动。

先全旁路验证 token、归还、STOP、字节序和长时间传输，再做图像对比。附带仿真与 PDS 报告见 [BUILD_IMAGE_PCIE_20261010.md](BUILD_IMAGE_PCIE_20261010.md)；本次只核对源码和归档完整性。当前保存位流与附带报告哈希不一致，见[工程 README](../README.md)，不能以旧记录代替当前硬件验收。
