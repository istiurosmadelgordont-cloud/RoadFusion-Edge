# 当前版本与工程分支

整理日期：2026-11-03。最近功能代码为主分支2026-10-31提交 `06da0df8d50c750740f39de7f4c898eb1ea183f5`；本次更新只整理文档。

## 当前可组合组件

| 部分 | 当前版本/下载入口 | 说明 |
| --- | --- | --- |
| ARM应用 | [board_v12](../board_v12/README.md) | 10月31日板端源码同步；PCIe驱动8.2、视频ABI2、DMA-BUF/RGA |
| FPGA一 | [board1_canvas_sfp_20261030.zip](../fpga/dual_board/board1_capture/board1_canvas_sfp_20261030.zip) | 三路共用LUT、后视下半区画布、KEY2四宫格；配套`test_ddr_canvas_sfp_20261030.sbit/.fic` |
| FPGA二 | [sfp_ddr_image_pice_handshake_tested_20261030.zip](../fpga/dual_board/board2_sfp_pcie/sfp_ddr_image_pice_handshake_tested_20261030.zip) | SFP、DDR、增强、四槽PCIe及两阶段参数回执；bit生成于10月18日，10月30日归档 |
| 查看器 | [pcie_v4](../tools/pcie_v4/README.md) | 独立Qt预览/调参；同设备不能与ADAS同时打开 |
| 发送器 | [udp_video_sender](../tools/udp_video_sender/README.md) | Windows以太网视频/图片输入 |

两块FPGA均为PGL50H / FBG484 / −6，工具PDS 2022.2-SP6.4。bit与FIC按各板SHA256配对，不以相似文件名混用。协议ABI相同也不代表旧固件支持DMA-BUF驱动所需的所有控制能力或参数回执。

## Git分支与目录

核对时仓库有两个Git分支：`main`是当前整机工程；`video-download/silk-road-20261017`是素材下载历史分支，不作为当前软硬件发布入口。本次工程说明统一在main维护。

`board_v12`、`fpga/dual_board`和`tools`是主分支中的工程目录，不是独立Git分支。旧`board_v7`至`board_v11`应用已从当前工作树移除，可由[历史索引](HISTORY.md)及固定提交查看。

## 运行配置与测量版本

用户技术文档列出的板卡一文件为`test_ddr_shared_lut_20261027.sbit`，仓库当前下载件为10月30日画布版。板卡二文档使用通用文件名`hdmi_ddr_pcie.sbit`，仓库通过归档哈希识别。没有重新逐一核对技术文档测试位流与归档件哈希，不能把其中的性能数字直接标成10月30日位流的复测结果。

源码、归档bit、实机运行及文档测量分别记录；性能来源与时序状态见 [PERFORMANCE.md](PERFORMANCE.md)。
