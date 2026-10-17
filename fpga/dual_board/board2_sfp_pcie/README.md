# 板卡二：DDR后图像处理与参数两阶段回传

当前版本 **2026-10-17 / IMAGE_ACK_20261017**，来自桌面`sfp_ddr_image_pice_handshake`。工程先压缩后上传，用这一版替换双板方案板卡二；板卡一、ARM驱动/应用和以太网上位机文件保留仓库当前内容。

- **[下载完整工程ZIP](sfp_ddr_image_pice_handshake_20261017_image_ack.zip)**：7385937字节，解压后553个文件。
- **[参数已接收/已生效：ARM接口说明](docs/IMAGE_PARAMETER_ACK.md)**。
- [代码解说](docs/CODE_WALKTHROUGH.md)、[RC控制PCIe指南](docs/RC_PCIE_CONTROL.md)。
- [FIC与参数探针](docs/IMAGE_PCIE_FIC.md)、[DRM优化与图像入口调试](docs/DRM_REDUCTION_AND_PCIE_IMAGE_DEBUG.md)。
- [SHA-256](SHA256SUMS.txt)、[归档检查](SNAPSHOT_CHECKS.json)、[双板接线](../README.md)。

```text
板卡一合成视频 → SFP RX → DDR原始RGB565 → 恢复时序 → 图像处理
                                                    ├ HDMI
                                                    └ PCIe四缓冲 → ARM
ARM完整五字配置 → 0x214提交cookie → 已接收回复 → 像素帧ACK → 已生效回复
```

新增可选`0x218`：ARM在停止状态登记独立128字节DMA区；前64字节回复RECEIVED、后64字节回复APPLIED，分别写16个相同的本次LE32 cookie。每次完整写`0x200..0x210`，以唯一非零cookie写`0x214`，串行等待两次回复。STOP禁用回传区，STOP ACK后须重新登记。旧`0x140/0x150/0x160`继续可用，但要两阶段确认应使用完整五字提交；旧`0x220/0x224`协议不使用。

图像控制和回传逻辑位于手写`pcie_dma_ctrl.v`；沿用原64字节MWr通道，四缓冲token、RELEASE、STOP区格式不变，PCIe/SFP生成IP未改。保留DRM行缓存优化和当前FIC（Core0端口133/161位、Core1端口172/213位，各256点）。新增确认寄存器标记可调试，但没有扩大本次FIC布局。

**ARM需配合扩展。** 仓库`tools/pcie_v2`现有7.0版本仍是单向提交；本次不修改该驱动/Qt。新接收需扩展BAR映射至0x21B、分配128字节确认区、提交cookie、验证两种完整回写，再把状态提供给应用。参考头在ZIP内`rc/image_ack_reference.h`。

用PDS 2022.2-SP6.4打开解压目录的`hdmi_ddr_pice.pds`，顶层`hdmi_ddr_pcie`，器件PGL50H / FBG484 / -6。PDS的215个设计输入均存在，FIC引用可恢复。ZIP包含RTL、IP、约束、RC参考、测试、验证记录和更新后的完整指南；排除了构建缓存、旧位流及实时采样文件。

本次7项RTL仿真、2项C99测试通过，包含两阶段回复、随机背压、视频复位、STOP排空、完整1080p和DDR后整机。**本次未运行新的PDS综合/映射/布局布线，未生成或烧录新bit，也未完成真实ARM回传验证。** 历史资源/时序结果不代表本版；使用新协议须由本源码重建并检查时序后下载。

SHA-256：`58ae167c029de5131836a5b3f86e1d23c5d705867c4b68dfec28a8ebcb98a3c4`。ZIP内`SOURCE_MANIFEST.csv`校验全部项目文件，`ORIGINAL_COPY_MANIFEST.csv`记录源文件复制，当前证据在`docs/verification/20261017/image_ack/`。旧版本由Git历史保留。

第三方IP声明保持原样，沿用[私有工程权利通知](../DISCLAIMER.md)。
