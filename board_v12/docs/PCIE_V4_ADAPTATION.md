# board_v12 的 PCIe/RGA 接入

board_v12 由 board_v11 演进，保留原识别、交互和预警功能，将 PCIe 实时图像输入固定到 DMA-BUF + RGA。当前图像链路、启动命令和回归说明见 [PCIe RGA 图像处理路径](PCIE_RGA_PREVIEW.md)。

FPGA 与驱动协议不变：driver 8.2 导出四个 DMA-BUF 槽，RGA 同步读取 RGB565 并转换为 BGR888。完整 BGR 帧进入现有识别链；界面预览、YOLO/MO/UFLDv2 的可加速缩放和通道转换也使用 RGA。

RGA 的变换并不取代 OpenCV 的 `cv::Mat`、模型输出解码、HSV/灯态分析、车道拟合、跟踪或绘制。OpenCV 继续处理这些算法和界面任务；本版将 PCIe 实时路径中可由 RGA 执行的像素缩放和格式转换切换为硬件实现。

ARM64 默认链接 `third_party/librga` 中与板端内核匹配的 librga 1.10.6 静态库，不替换系统库。板端完整视频帧率和模型准确率需要在目标硬件上验证；当前 Windows 源码副本不包含板端模型和运行时素材。
