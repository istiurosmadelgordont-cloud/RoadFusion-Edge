# DDR读出后的图像处理

打开`hdmi_ddr_pice.pds`，顶层仍为`hdmi_ddr_pcie`。

当前数据顺序为：SFP接收RGB565 → DDR保存原图 → DDR读FIFO恢复规则视频流 → 完整图像处理 → 公共输出 → HDMI / PCIe图像DMA。

处理核心仅实例化一套。DDR原图写入与算法开关独立；HDMI使用处理后的RGB888，PCIe保持RGB565格式并取同一处理结果的高位。
全主开关关闭时，输出为原DDR还原RGB的等延迟旁路。

PCIe五参数字、整帧提交、配置回执与场景预设见[PCIE_IMAGE_CONTROL.md](PCIE_IMAGE_CONTROL.md)。
A冻结及原始A旁路已删除，归一化和恢复固定使用平滑A；原高光压制与暗光增强保留在Y增强链路中。

核心4496拍延迟，DDR读出的RGB和同步先对齐，HS使用等长RAM延迟；HDMI和PCIe共用同一组最终RGB/VS/DE寄存。

时钟沿约定：将FIFO实际受理读请求的上升沿记为R0，入口寄存在R1后有效、核心在R2采入；公共输出寄存在R4498后有效，PCIe DMA等下游在R4499采样。4499按流水级及下游采样口径计数，相对原还原输出（R1后有效）新增4497拍。
实际时序和资源见[本次验证](BUILD_IMAGE_PCIE_20261010.md)。
