# 板卡一：四路采集、三路融合与四宫格切换

当前工程为 **2026-10-24** 版：HDMI、双以太网和板卡二 SFP 视频写入四路 DDR 缓存，输出三路融合或完整四宫格。系统接线见 [双板说明](../README.md)。

## 下载入口

| 文件 | 用途 |
| --- | --- |
| [board1_fusion_quad_20261024.zip](board1_fusion_quad_20261024.zip) | 完整 PDS 工程、RTL/IP、LUT/BRAM、约束、bit/FIC 与仿真输入 |
| [test_ddr_mode_switch_20261024.sbit](test_ddr_mode_switch_20261024.sbit) | 当前集成调试 bit，可单独下载；验证限制见下文 |
| [SHA256SUMS.txt](SHA256SUMS.txt) | 压缩包和 bit 校验；包内另有逐文件 `SOURCE_MANIFEST.csv` |

解压进入 `board_3_oneboard_design`，用 **PDS 2022.2-SP6.4** 打开 `3_ddr_test.pds`。器件 **PGL50H / FBG484 / -6**；顶层 `test_ddr`，源码 `source/ddr_test_top.v`。Debugger 使用配套 `generate_bitstream/test_ddr_mode_switch_20261024.fic`。

## 快速操作

1. 按完整工程重建或准备与工程配套的调试文件，核对下方时序/板级验证状态。
2. 接入所需四路视频，确认 DDR 初始化和源帧有效；以太网输入设 960×540，参数见 [Windows 上位机](../../../tools/udp_video_sender/README.md)。
3. 上电默认显示三路融合：1920×540 位于 1080p 上半区，下半区黑色。
4. 按 **KEY2** 切完整 1920×1080 四宫格，再按返回融合；按键后等待完整结果帧和垂直消隐切换，HDMI/SFP 同时生效。
5. RK3568 按四路象限运行 ADAS 时，使用四宫格并核对前/后/左/右的实际素材顺序。

KEY2：**L15 / 3.3 V / LVCMOS33 / 低电平按下**；KEY1 保留复位功能。原 `Key_anti_shake.v` 字节未改，外层 `video_mode_key.v` 将消抖下降沿转换为一次切换；50 MHz、MCNT=999999，约 20 ms。长按或松开不连续切换。

## 输入映射

| DDR 通道 | 输入 | 四宫格位置 | 融合用途 |
| --- | --- | --- | --- |
| 0 | 板卡一 HDMI | 左上 | 左路 |
| 1 | 以太网1 | 右上 | 中路 |
| 2 | 以太网0 | 左下 | 右路 |
| 3 | 板卡二 HDMI 经 SFP 返回 | 右下 | 不参与 |

每路源图 960×540。以太网0：`192.168.1.10 / A0-B1-C2-D3-E4-E4`；以太网1：`192.168.1.11 / A0-B1-C2-D3-E1-E1`，UDP 默认 1234。OV5640 配置/采集模块仍在，当前未接入上述四路。

HDMI 与 SFP 编码器读取同一套模式输出。当前融合保留原 128 像素过渡，未启用窄羽化试验版本。更换相机视角、位姿、分辨率或裁剪需重新生成 LUT；现有 LUT 为测试视角的离线估计，不是官方通用标定。

## 实现与缓存

`source/fusion/quad_frame_renderer.v` 读取锁定源页，按象限生成完整四宫格结果；`video_mode_key.v` 接入原消抖，顶层 `mode_key_n` 在 FDC/PCF 绑定 L15。`panorama_output_top.v`、`fusion_ddr_pipeline.v`、`fusion_frame_writer.v`、`fusion_frame_reader.v` 包含模式选择和像素打包/拆包。

融合使用 RGB888/32bit，四宫格使用 RGB565/16bit；两种结果都是 1,036,800 个 32bit 字，共用两个 4 MiB 结果槽，字地址 `0x1000000 / 0x1100000`。请求先等待当前渲染完成，再生成新模式完整结果，缓冲与像素格式在垂直消隐期共同切换。

融合锁前三路、四宫格锁全部四路；第四路尚无有效帧时继续显示上一完整结果。源页锁保证单路帧内一致，不保证独立相机曝光同步。

保留旧融合读修复版的 FIFO 补水策略：低于等于 96 字时优先显示读，补到 192 字并减少突发间隙。HDMI 名义时序 150 MHz、2200×1125，约 60.606 Hz；新图生成受渲染限制，未有新结果时重复上一完整帧。

## 仿真与重建

在解压后的工程根目录运行：

```tcl
do sim/panorama/run_key_unit.do
do sim/panorama/run_quad_unit.do
do sim/panorama/run_mode_switch.do
```

Mode 仿真第四路是彩色测试图；参考向量和日志位于包内 `docs/verification`。不同安装环境需要调整本机 `modelsim.ini` 库路径。重建从 Compile 开始，使用包内全部 RTL/IP/LUT/FDC/PCF，生成配套 bit/FIC；旧 PDS 窗口和原构建机器路径不能代替本机重建配置。

## 验证与限制

- 消抖/切换单测覆盖抖动、长按、松开和再次按下；四宫格取样逐像素检查 2,073,600 像素，覆盖跨行/突发、源页、反压和读延迟。
- 100 MHz DDR / 150 MHz 视频仿真完成融合→四宫格→融合：9 帧融合、9 帧四宫格、2 帧返回融合，6 次渲染、2 次翻转、欠读 0；包含后台写竞争及 24–31 周期随机 DDR 返回延迟。
- 综合、映射、布局布线和 bit 生成已完成，三个 FLA 核的端口/深度已核对。
- **物理时序未收敛，属于上板调试版**：DDR Slow Setup −3.606 ns、最差 Hold −0.172 ns；SFP 缩放像素路径 Slow Setup −2.107 ns。报告在包内 `report_timing/test_ddr.rtr`。
- 新按键/四宫格集成 bit 尚未完成板级验收；此前上半区完整显示和零欠读的用户验证属于旧融合读修复版。

旧源码可由 Git 历史恢复。第三方 IP/例程声明保留在工程包内，见 [系统权利通知](../DISCLAIMER.md)。
