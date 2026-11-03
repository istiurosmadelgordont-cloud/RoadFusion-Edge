# FPGA工程导航

当前整机使用 [dual_board](dual_board/README.md) 下的两套PGL50H工程。两套均有独立工程ZIP及配套bit/FIC，使用PDS 2022.2-SP6.4；连接、模式和协议先读双板说明。

| 目录 | 用途 | 当前定位 |
| --- | --- | --- |
| [dual_board/board1_capture](dual_board/board1_capture/README.md) | 多源采集、三路融合、后视画布/四宫格、SFP输出 | 当前板卡一，10月30日归档 |
| [dual_board/board2_sfp_pcie](dual_board/board2_sfp_pcie/README.md) | SFP/DDR/图像增强/PCIe及参数回执 | 当前板卡二，10月30日已测试归档 |
| [ethernet_video](ethernet_video/README.md) | 单路以太网RGB565接收与显示 | 独立接口基线，Windows发送器仍可用于当前输入 |
| [hdmi_ddr_pice](hdmi_ddr_pice/README.md) | 早期HDMI/DDR/增强/PCIe | 历史基线，无完整当前所有权/回执能力 |
| [pcie/hdmi_ddr_pice_handshake_20260919](pcie/hdmi_ddr_pice_handshake_20260919/README.md) | RC四缓冲所有权与STOP的早期实现 | 历史握手基线 |
| [pcie_video](pcie_video/) | PCIe彩条和DMA调试 | 独立接口验证，不代表当前整机链路 |
| [dual_board/board2_sfp_pcie_image_20261011](dual_board/board2_sfp_pcie_image_20261011/README.md) | 图像参数早期阶段 | 历史说明入口 |

旧目录中的“当前”“尚未上板”只描述当时版本，不覆盖现行双板归档状态。新增HSV/金字塔多尺度暗光融合实验没有加入当前发布件。功能上板通过和时序闭合分开记录，详见 [性能与验证](../docs/PERFORMANCE.md)。
