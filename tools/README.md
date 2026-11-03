# 工具与独立工程导航

| 目录 | 用途 | 版本与适用范围 |
| --- | --- | --- |
| [udp_video_sender](udp_video_sender/README.md) | Windows视频/图片/彩条转RGB565，以UDP送FPGA | 当前双板以太网输入配套工具 |
| [pcie_v4](pcie_v4/README.md) | Qt独立视频预览与图像参数控制 | 驱动8.2，DMA-BUF/RGA按需缩放，支持对照read/mmap；不运行识别模型 |
| [pcie_v2](pcie_v2/README.md) | 旧Qt采集/调参查看器 | 驱动7.0、单向五字参数提交，不确认RECEIVED/APPLIED |
| [pcie_hdmi_host_system](pcie_hdmi_host_system/README.md) | 早期Python查看、彩条及持续采集对照 | 历史协议，不能直接替代当前四槽所有权驱动 |
| [pango_pcie_dma_alloc](pango_pcie_dma_alloc/README.md) | 厂商参考DMA例程 | 接口学习与独立基线，不是当前ADAS程序 |

模型转换与数据准备脚本也保留在本目录，其版本按各脚本参数及模型名称判断。历史21类模型的脚本不自动成为当前14类YOLO的转换流程；当前部署文件见 [模型说明](../models/README.md)。

ADAS运行入口为 [board_v12](../board_v12/README.md)。Qt独立查看器与ADAS不能同时打开同一个PCIe设备。当前应用固定RGA，不继承pcie_v4中的read/mmap对照选项。
