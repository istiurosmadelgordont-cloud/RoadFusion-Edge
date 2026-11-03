# board_v12 PCIe实时输入

当前实时入口为 `sh scripts/run_pcie.sh`，固定使用PCIe驱动8.2、四槽DMA-BUF和同步RGA；RGB565转换为完整BGR888，预览及模型输入由RGA做适配。没有read/CPU实时切换选项，独立pcie_v4查看器的对照选项不属于本应用。

默认原始槽顺序左/前/右/后，四宫格为左上左视、右上前视、左下右视、右下后视。内部语义索引仍为front/rear/left/right。实际素材和FPGA输出模式须与应用布局匹配；融合布局不替代FPGA KEY2。

多设备用`PCIE_DEVICE=/dev/fpga_frame_<实际BDF>`指定；脚本自动准备驱动及权限，默认显示模拟导航，车道检测、FCW和危险模拟停车需按需开启。`--pcie-buffers`是应用BGR池大小，不增加FPGA四槽。

详见 [部署](README.md)、[当前状态](CURRENT.md)、[RGA路径](docs/PCIE_RGA_PREVIEW.md)与[双板接线](../fpga/dual_board/README.md)。
