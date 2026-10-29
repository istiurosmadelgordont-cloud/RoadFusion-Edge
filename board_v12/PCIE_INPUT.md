# board_v12 PCIe/RGA 输入

PCIe 实时输入固定使用 driver 8.2 DMA-BUF + 同步 RGA。RGB565 由 RGA 转换成完整 BGR888 图像；预览、YOLO/MO/UFLDv2 前处理及实时显示缩放均使用 RGA。运行时没有 CPU/read 输入对照切换。

启动：`sh scripts/run_pcie.sh`。多设备时设置 `PCIE_DEVICE=/dev/fpga_frame_<BDF>`；板端需加载支持 DMA-BUF 导出的 driver 8.2，并使用本项目配套 librga。

数据路径、缓冲池及验证边界见 [RGA 处理说明](docs/PCIE_RGA_PREVIEW.md)。
