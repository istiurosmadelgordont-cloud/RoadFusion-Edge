# 板卡二：SFP 接收、图像处理与 PCIe 四缓冲

2026-10-30 按用户指定归档 **`sfp_ddr_image_pice_handshake`** 当前桌面工程与现有 bit；用户确认该版已上板测试通过。不是名称相近的旧 `sfp_ddr_img_pice_handshake`，也没有接入暂停中的 HSV / 金字塔暗光融合实验。

- [完整工程 ZIP](sfp_ddr_image_pice_handshake_tested_20261030.zip)：单个 ZIP 包含 PDS 输入、RTL/IP、约束、bit、FIC、RC 参考、测试源码及历史说明；不需要再合并旧版两个 ZIP。
- [bit](hdmi_ddr_pcie_tested_20261030.sbit) · [配套 FIC](hdmi_ddr_pcie_tested_20261030.fic) · [SHA256](SHA256SUMS.txt)。
- 解压 `sfp_ddr_image_pice_handshake`，用 **PDS 2022.2-SP6.4** 打开 `hdmi_ddr_pice.pds`，顶层 `hdmi_ddr_pcie`，器件 **PGL50H / FBG484 / -6**。从 Compile 开始重建，使用包内手工整合的全部 IP，不能用重新生成的原始 PCIe IP 覆盖共享 SFP / HSST 接口。

## 码流来源与状态

本次直接收录桌面 `generate_bitstream/hdmi_ddr_pcie.sbit`，本地生成日志结束于 **2026-10-18 20:56:54**。独立下载文件增加 `tested_20261030` 后缀只是本次归档 / 用户确认日期，不代表 10 月 30 日重建。ZIP 中保留通用文件名与同字节的有日期副本。

bit SHA256：`d33c267f4e2caa739a00c9a019687e00e052c7234967922f38d5ded198db57f5`。

FIC SHA256：`940e659a73ab1b1149f0b01e9eb8327eb3346d56104b91cf07960673b300e34d`。`debug/image_pcie_debug.fic` 与 `synthesize/hdmi_ddr_pcie_syn.fic` 相同；独立 FIC 也取自此配置。

本次未重新编译、重新下载或复跑 RTL；已核对当前 PDS 引用齐全、构建输入没有比 bit 更新、ZIP CRC 与逐文件哈希。文件时间只能辅助判断，完整源码和 bit 的证明边界仍以现有日志、归档哈希与用户上板反馈为准。根目录旧 README / BUILD_STATUS 中“未编译 / 未上板”的文字保留在 `docs/history`，代表当时阶段，不是本次发布状态。

## 视频与处理链

本机 HDMI 经 SFP 发往板卡一作为第四路；反向接收板卡一的 1920×1080 合成画面，RGB565 写 DDR，再经统一图像处理输出本地 HDMI 与 PCIe。断开两板光纤后，板卡二和上位机停留末帧属于输入链路中断。

当前处理包括 DCP 去雾、Y 通道双边去噪、Gamma、CLAHE、原暗部 / 高光调整和软膝高光压缩等，按参数开关。默认五字配置：`000001C0 00F38034 00004100 02030190 0080C040`，算法开关默认关闭；保护标志保留。字0 bit1 为 Gamma，bit2 为 CLAHE。新增多尺度暗光融合尚未实施。

## PCIe 与参数回执

保持 RGB565、64 字节图像 Payload、四缓冲所有权、帧尾 token、RELEASE / STOP 协议。

| BAR1 偏移 | 用途 |
| --- | --- |
| 0x200 / 0x204 / 0x208 / 0x20C / 0x210 | 完整五字图像参数 |
| 0x214 | 提交唯一非零 cookie |
| 0x218 | 可选 128 字节独立 DMA 确认区地址 |

RECEIVED 位于确认区 +0，APPLIED 位于 +64，均携带同一 cookie；APPLIED 表示已在帧边界生效。TX 完成、STOP 回执或寄存器写成功不能替代 APPLIED。具体缓存同步、字节序和寄存器语义以包内 `docs/IMAGE_PARAMETER_ACK.md`、`docs/RC_HANDSHAKE_GUIDE.md`、`rc` 参考为准。

当前 `source/source/pcie_dma_ctrl.v` 保留 3DW Posted Header 对 `tkeep[2:0]==111` 的兼容处理，支持 0111 / 1111；其 SHA256 与仓库 2026-10-19 HEADER_KEEP_FIX 记录一致：`f0411b2aba80385ba66849006127d2a46f3b399165bcb78a76b336514c68ccf3`。

FIC 标识 `IMAGE_ACK_RX_APPLIED_TX_20261017`；Core0 端口 133 / 161 位，Core1 244 / 245 位，各 256 点。导入 FIC 前请下载本版 bit，避免和旧固件探针不匹配。

## 时序与验证边界

用户确认当前版本上板测试通过；功能可运行不等于物理时序闭合。现有 2026-10-18 报告显示 **pclk_div2 Slow Setup −2.866 ns**、像素 PLL CLKOUT1 Slow Setup **−1.149 ns**，报告标记 `Some Constraints Violated`。完整报告保留在 `report_timing/hdmi_ddr_pcie.rtr`，本次只归档，不改 RTL 或约束修复这些路径。

历史 RTL / RC 验证记录及源码随包保留，不能把历史离线测试视作本次新跑的板级验收。上位机 / RK3568 驱动源码不在本次工程上传范围。见 [双板说明](../README.md) 与 [权利通知](../DISCLAIMER.md)；旧版本可从 Git 历史恢复。
