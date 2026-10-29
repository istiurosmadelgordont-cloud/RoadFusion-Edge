# PCIe / RGA 输入

board_v12 的 PCIe 实时输入固定为 driver 8.2 DMA-BUF + 同步 RGA。RGA 将 RGB565 转成完整 BGR888 识别帧，并完成实时预览和模型前处理中的缩放/通道变换。详情见 [图像处理路径](PCIE_RGA_PREVIEW.md)。
