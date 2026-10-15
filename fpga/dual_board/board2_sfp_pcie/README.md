# 板卡二：SFP、DDR后图像处理与PCIe控制

2026-10-15，用本地 `sfp_ddr_image_pice_handshake` 当前整合版替换双板方案板卡二。工程以ZIP压缩包上传；板卡一、ARM应用/驱动和以太网上位机保持原版本。

## 下载与说明

- **[下载完整工程ZIP](sfp_ddr_image_pice_handshake_20261015.zip)**（7335149字节，解压后516个文件）。
- **[代码解说](docs/CODE_WALKTHROUGH.md)**：SFP、DDR、图像处理、PCIe命令及四缓冲发送状态机。
- **[RC控制PCIe指南](docs/RC_PCIE_CONTROL.md)**：寄存器、端序、DMA生命周期、暗光/高光和五字参数提交。
- [SHA-256](SHA256SUMS.txt)、[归档检查](SNAPSHOT_CHECKS.json)、[双板接线](../README.md)。

下载后核对ZIP的SHA-256：

```text
dd5384b5ae59721d9458f8eaf75ab9a231b3735b4644d2ae6e3ce258c39c7445
```

解压得到 `sfp_ddr_image_pice_handshake/`，使用PDS 2022.2-SP6.4打开其中 `hdmi_ddr_pice.pds`。顶层为 `source/hdmi_ddr_pcie.v` 的 `hdmi_ddr_pcie`，器件PGL50H / FBG484 / -6。压缩包包含完整RTL、IP、PDS、约束、RC参考头、测试、验证记录及两份新指南。

## 本版行为

```text
本板HDMI IN → SFP TX → 板卡一的一路输入
板卡一合成视频 → SFP RX → DDR原始RGB565
 → 时序对齐 → DCP → Y降噪 → Gamma → CLAHE
 → 原暗光增强/高光压制 → 高光保护 → 公共输出 → HDMI和PCIe
```

图像参数控制已合入手写 `pcie_dma_ctrl.v`，共用原RX解析器，独立 `pcie_image_control.v` 已删除。原SFP/PCIe IP核及生成文件保持来源原样。

原0x140/0x150/0x160暗光/高光命令仍有效；完整图像参数使用0x200~0x214。**本版没有旧日期快照的0x220/0x224图像配置状态DMA接口**，STOP状态区只用于DMA停止应答。

代码整合阶段已通过RTL回归和PDS Compile、综合、器件映射；资源为LUT 16198/42800、FF 20071/64200、APM 51.5/84、DRM 133/134。旧FLA自动插核引用已取消，调试标记保留。尚未执行布局布线后时序签核或板上验收，ZIP不包含旧位流和构建缓存；恢复后从PDS重建。

ZIP内 `SOURCE_MANIFEST.csv` 校验项目文件，`ORIGINAL_COPY_MANIFEST.csv` 记录本地来源复制，`docs/verification/20261015` 保存同次测试日志。旧板卡二由Git历史保留，旧20261011目录仅保留迁移入口，避免混用控制协议。

第三方IP声明保持原样，沿用 [私有工程权利通知](../DISCLAIMER.md)。
