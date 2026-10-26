# 板卡二：光口接收、图像处理与 PCIe 输出

当前归档为 **2026-10-19 / HEADER_KEEP_FIX**，工程名 `sfp_ddr_image_pice_handshake`。接收板卡一的 SFP 合成视频，写 DDR、恢复时序，经统一图像处理后输出到 HDMI 与 PCIe；同时将自身 HDMI 输入通过 SFP 送板卡一。完整链路见 [双板说明](../README.md)。

## 下载与还原

| 文件 | 内容 |
| --- | --- |
| [主工程 ZIP](sfp_ddr_image_pice_handshake_20261019_header_keep_fix.zip) | 2,425,732 字节、560 个文件：RTL/IP、约束、FIC、RC 参考及验证记录 |
| [PCIe 仿真库 ZIP](sfp_ddr_image_pice_handshake_20261019_pcie_sim.zip) | 4,978,960 字节、18 个文件 |
| [SHA256SUMS.txt](SHA256SUMS.txt) / [归档检查](SNAPSHOT_CHECKS.json) | 两包哈希、文件数及完整性记录 |

**两个 ZIP 解压到同一父目录，合并同名工程文件夹，共 578 个文件。** IP 内容按目录拆包，不能只解压主工程就认为仿真库齐全。

用 **PDS 2022.2-SP6.4** 打开 `hdmi_ddr_pice.pds`；顶层 `hdmi_ddr_pcie`，器件 **PGL50H / FBG484 / -6**。已检查 215 个 PDS 设计输入存在。此归档不附 bit；须重建、检查时序并使用匹配的 FIC。

## 使用步骤

1. 合并两个 ZIP，核对哈希和工程输入，保留工程内的生成 IP、RTL 与约束。
2. 完成编译、综合、映射、布局布线和时序检查，再生成/下载位流；工具与下载方式按板卡环境配置。
3. SFP RX 接板卡一 TX，板卡二 TX 接板卡一 RX；自身 HDMI 源用于送往板卡一的单路输入。
4. 先检查 DDR、SFP 与 HDMI 输出，再连接 RK3568 PCIe、加载匹配驱动并开始取帧。
5. 修改图像草稿后，点击 ARM 界面的“应用设置”；核对对应 cookie 的 RECEIVED 和 APPLIED，详情见 [图像控制窗口](../../../board_v10/docs/PCIE_V3_IMAGE_CONTROLS.md)。

重新下载匹配位流后，应停止并重新开始 ARM 采集，以重新登记图像回执邮箱。

## 视频与图像控制

SFP 原始 RGB565 写 DDR，恢复的视频经过 `image_processing_top`，公共结果送 HDMI 和 `pcie_dma_ctrl`。后者同时管理图像参数与原视频四缓冲 DMA。默认输出 1920×1080 RGB565，图像负载 64 字节、每帧 64,800 个图像包；帧尾 token、RELEASE 与 STOP 生命周期见 [RC 指南](docs/RC_PCIE_CONTROL.md)。

| BAR1 地址 | 用途 |
| --- | --- |
| `0x200 / 0x204 / 0x208 / 0x20C / 0x210` | 五个完整图像参数字 |
| `0x214` | 提交唯一 cookie |
| `0x218` | 登记 128 字节独立图像确认区 |

RECEIVED 表示参数接收回执，APPLIED 表示对应配置在帧边界应用；提交函数成功、TX 完成事件或 STOP cookie 均不等于 APPLIED，也不保证主机已显示对应新画面。旧 `0x220/0x224` 配置状态 DMA 不用于本版。

当前推荐配套软件是 [board_v10 及其 driver](../../../board_v10/CURRENT.md)，其 pcie_v3 8.0/8.1 接口支持两阶段回执。[tools/pcie_v2](../../../tools/pcie_v2/README.md) 保留驱动 7.0 单向提交，不能将其消息当作本协议已确认生效。

## 本版包头修复

修正图像路径把 3DW Posted 包头只按 `tkeep=0111` 识别的问题：Logos IP 的合法包头可以是 `1111`，原外层解析器已接受它，图像路径又将其拒绝，造成“视频可收但参数无效/回执超时”。本版复用已验证的外层包头判断；单 DW 数据拍仍要求 `0001`。

详细修改和复现见 [PCIE_HEADER_KEEP_FIX.md](docs/PCIE_HEADER_KEEP_FIX.md)。未修改 PCIe/SFP 生成 IP 和原四缓冲视频协议；ARM 无须因该包头修复改变合法命令格式，但仍须匹配位流和驱动。

## 文档与调试入口

- [代码解说](docs/CODE_WALKTHROUGH.md)：模块关系与数据流。
- [RC 控制指南](docs/RC_PCIE_CONTROL.md)：寄存器、端序、缓冲与启停。
- [RECEIVED/APPLIED 协议](docs/IMAGE_PARAMETER_ACK.md)：参数生命周期。
- [FIC 探针](docs/IMAGE_PCIE_FIC.md)：`IMAGE_ACK_RX_APPLIED_TX_20261017`，Core0 端口 133/161 位、Core1 244/245 位，各 256 点。
- [DDR/输入 debug](docs/DRM_REDUCTION_AND_PCIE_IMAGE_DEBUG.md)：调试细节。

## 验证状态

9 项离线仿真通过，覆盖两种包头、参数跨域、两阶段回包、视频 DMA 并发控制与 DDR 后完整集成。日志在工程包内 `docs/verification/20261019/header_keep_fix/`；哈希与归档检查见本页下载表。

本次工程发布未运行 PDS 位流构建或下载，不附 bit；以前生成的文件不能证明这次快照已通过时序或上板验收。2026-10-26 的 README 整理不改变该验证边界，也未改写工程 ZIP、IP 或位流。

旧版本由 Git 历史保留，第三方声明沿用 [系统权利通知](../DISCLAIMER.md)。
