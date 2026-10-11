# 板卡二：SFP 接收、DDR 原图缓存与 PCIe 图像处理控制

2026-10-11 保存自用户本地 `sfp_ddr_img_pice_handshake`。工程入口为 [hdmi_ddr_pice.pds](hdmi_ddr_pice.pds)，顶层 [source/hdmi_ddr_pcie.v](source/hdmi_ddr_pcie.v)，器件 PGL50H / FBG484 / -6，工具 PDS 2022.2-SP6.4。源码、生成 IP、约束、RC 参考接口和测试均按原字节保存；没有修改功能 RTL。

## 使用入口

- **[RC 控制 PCIe 与场景调参指南](docs/RC_PCIE_IMAGE_TUNING.md)**：从启动、收帧、归还、停止，到图像处理整包提交、状态确认、场景参数与故障排查。
- [用户原始 PCIe 图像控制说明](docs/PCIE_IMAGE_CONTROL.md)：详细位域、状态页、流水对齐与联调信号。
- [四缓冲握手说明](docs/RC_HANDSHAKE_GUIDE.md)：图像所有权与 STOP/RELEASE；其中旧 `0x140/150/160` 增强接口属于历史设计，新版应使用 `0x200~0x224`。
- [RC 图像配置参考头](rc/image_control_reference.h)和[四缓冲参考头](rc/handshake_reference.h)：需要接到实际内核驱动，不是可直接运行的 ARM 上位机。
- [本地附带的构建与仿真记录](docs/BUILD_IMAGE_PCIE_20261010.md)：记录日期以正文及各 JSON 为准，最新实现记录为 2026-10-11。

## 双板关系与处理位置

板卡二自身 HDMI 输入仍经光口发送给板卡一作为一路输入；板卡一返回的 1920×1080 RGB565 合成图经光口接收、DDR 保存，再处理后共同送 HDMI 和 PCIe。返回方向是一张合成帧，四个 PCIe 缓冲表示同一视频流的所有权轮转，不表示四路独立视频。

```text
板卡二 HDMI IN → SFP TX → 板卡一一路输入
板卡一合成图 → SFP RX → DDR 原始 RGB565
 → DDR 读出 / 时序对齐 → DCP → Y 降噪 → Gamma → CLAHE
 → 原亮度曲线 → 高光保护 → 公共输出
                              ├─ HDMI RGB888
                              └─ PCIe RGB565 四缓冲 → RC
```

主处理开关默认关闭；天空保护、CLAHE 防光晕/阴影保护只在相应主算法开启时起作用。处理在 DDR 读侧，DDR 中仍保存原图。参数是一整张合成帧的全局配置，不能分别给四个象限选择独立算法。图像尺寸、行消隐和 Gamma LUT 不能由本版 BAR 参数修改。

## 文件完整性与位流

本次归档检查 214 个 PDS 设计输入引用，均存在；源码/IP 与附带 `implementation.json` 中的输入哈希一致。复制记录见 [ORIGINAL_COPY_MANIFEST.csv](ORIGINAL_COPY_MANIFEST.csv)，上传目录的完整哈希见 [SOURCE_MANIFEST.csv](SOURCE_MANIFEST.csv)，本次检查结果见 [SNAPSHOT_CHECKS.json](SNAPSHOT_CHECKS.json)。原 README 保存在 `README.original.md`，其中部分中文在来源文件中已有编码问题；当前使用本页和新版指南。

`release/` 保留本地现有 `.sbit/.sfc/.smsk/.bgr`。**当前 `.sbit` 不匹配附带构建记录中的位流哈希**：

- 实际复制文件：`2f70f91179b0f0c4b71076020885462f1b77a5ed415711766f1c4f214c5c353a`。
- 附带记录值：`1370b510a03c24f299c47411669a0317396efeb497f1776e64804a8b9808883f`。

它是原文件留存，不作为已与报告对应的合格下载文件。恢复后用本目录源码重新生成位流、检查时序并登记新的哈希，再用于本次硬件验收；本次没有重跑 PDS 或上板测试。

附带实现记录说明在已有约束下通过时序，功能 DRM 128 / 134，Debugger 增加 0.5；同时明确外部 HDMI I/O 延时约束不完整、硬件图像效果未验证。这些是随工程保存的历史证据，不是本次上传重新取得的测试结果。

## 恢复与回退

本目录是普通工程目录，无需解压。旧 `../board2_sfp_pcie/` 保留，可以分别打开两个版本的 PDS。若只恢复本版，在 Git 检出保存此目录的提交；不要将两版 IP 或位流混用。构建缓存与 ModelSim 工作库未提交，PDS 中的阶段输出可重新生成；`device_map/hdmi_ddr_pcie.pcf` 作为原引脚映射留存。`impl.tcl` 是来源工程的历史操作记录，含旧绝对路径，不作为恢复脚本执行；请从本目录 PDS 重新构建。

原 `BUILD_STATUS.md`、`docs/IMG_BEFORE_DDR.md`、`docs/README_DVIO.md` 与 `archive/` 是历史资料，不能替代本版 PCIe 控制说明。DVIO 不再是本版参数入口，旧 A 冻结/原始 A 旁路已删除。

工程随用户授权保存到私有仓库；原第三方 IP 声明保持原样，适用已有[权利通知](../DISCLAIMER.md)。
