# 板卡二已迁移到当前统一工程

2026-10-15，双板方案板卡二已由 `sfp_ddr_image_pice_handshake` 替换，统一入口为 [board2_sfp_pcie](../board2_sfp_pcie/README.md)。

请使用新版 [代码解说](../board2_sfp_pcie/docs/CODE_WALKTHROUGH.md) 与 [RC控制PCIe指南](../board2_sfp_pcie/docs/RC_PCIE_CONTROL.md)。当前图像控制已合入 `pcie_dma_ctrl`，没有旧版0x220/0x224配置状态DMA协议，旧暗光/高光命令继续有效。

本目录仅保留迁移入口；旧源码、文档和生成位流可从Git历史中的2026-10-11提交恢复，不与当前版本混用。
