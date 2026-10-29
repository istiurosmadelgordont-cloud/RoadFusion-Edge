# 第一阶段：PCIe DMA-BUF -> 同步 RGA -> BGR

本文件记录 board_v12 独立联通工具的来源。主程序的实时 PCIe 输入和预览统一见 [board_v12 PCIe/RGA 处理路径](PCIE_RGA_PREVIEW.md)。该 probe 仍是独立诊断程序，使用 DRM 输出，不参与主程序启动，也不作为 CPU/read 对比入口。

probe 可测量 DMA-BUF -> RGA 转换本身，并用首帧验证 RGB565 通道顺序；它不能代替固定 BGR 池、并发消费、完整识别链或板端端到端帧率验证。
