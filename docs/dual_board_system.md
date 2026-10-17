# 双板系统当前入口

双板接线、板卡一四路映射、以太网上位机和启动顺序统一见 [双板系统说明](../fpga/dual_board/README.md)。

2026-10-17已用 `sfp_ddr_image_pice_handshake` 替换 [板卡二](../fpga/dual_board/board2_sfp_pcie/README.md)：DDR后统一图像处理，HDMI/PCIe共源，图像控制合入 `pcie_dma_ctrl`。

- [板卡二代码解说](../fpga/dual_board/board2_sfp_pcie/docs/CODE_WALKTHROUGH.md)
- [RC控制PCIe指南](../fpga/dual_board/board2_sfp_pcie/docs/RC_PCIE_CONTROL.md)
- [板卡一](../fpga/dual_board/board1_capture/README.md)
- [board_v8 PCIe输入与驱动ABI](../board_v8/PCIE_INPUT.md)

板卡一、ARM应用和以太网上位机保持仓库现有版本。新版图像参数使用0x200~0x214，旧暗光/高光0x140/0x150/0x160仍有效，新增0x218独立128字节参数确认区，分别回复RECEIVED/APPLIED；旧0x220/0x224接口不使用。ARM现有单向驱动需另行适配。新版本仍须重新生成位流并完成板上联调。

- [参数两阶段回传接口](../fpga/dual_board/board2_sfp_pcie/docs/IMAGE_PARAMETER_ACK.md)
