# 板卡一：共用 LUT 融合、下半区 SFP 与四宫格

2026-10-30 归档当前桌面工程；用户确认本版已上板测试通过。本次上传保留测试过的工程与码流，没有重新编译或下载硬件。

- [工程 ZIP](board1_canvas_sfp_20261030.zip)：完整 PDS 输入、RTL/IP、12 份 LUT、约束、匹配 bit/FIC、时序与验证记录。
- [bit](test_ddr_canvas_sfp_20261030.sbit) · [配套 FIC](test_ddr_canvas_sfp_20261030.fic) · [SHA256](SHA256SUMS.txt)。
- 解压 `board_3_oneboard_design`，以 **PDS 2022.2-SP6.4** 打开 `3_ddr_test.pds`；器件 **PGL50H / FBG484 / -6**，顶层 `test_ddr`。重建从 Compile 开始，保留包内全部 IP、RTL、LUT 和约束。

## 本次增加

融合模式：上半区为原三路 1920×540 全景；第四路 SFP 的原始 960×540 图像放在下半区中间，即 `x=480..1439, y=540..1079`，两侧黑色。SFP 不参与三路融合。KEY2（L15，低有效，原消抖）继续切换融合画布 / 完整四宫格；HDMI 与 SFP 共用输出，完整结果生成后在 VS 边界切换。

整张结果画布以 **RGB565** 保存，沿用原双缓冲地址。融合内部插值、混合仍按 RGB888 计算，写 DDR 前量化为 RGB565；相较旧版融合结果 DDR 的 RGB888 存储，HDMI 融合区会少一些颜色精度。SFP 输出原本就是 RGB565。

为新增画布及供数调整了读取、写入和坐标计算流水线，并降低原来明显的物理时序违例；没有放宽时钟周期或增加掩盖路径的 false path。

## 输入顺序与 LUT

| DDR 通道 | 输入 | 当前融合视角 | 四宫格 |
| --- | --- | --- | --- |
| 0 | HDMI | 左前 | 左上 |
| 1 | 以太网1 | 正前 / 前中广角 | 右上 |
| 2 | 以太网0 | 右前 | 左下 |
| 3 | SFP（板卡二回传） | 下半区原图，不参与融合 | 右下 |

每路 960×540 RGB565，合成输出 1920×1080。输入改序提议没有实施。

保留 2026-10-27 **三组视频共用的一套 LUT**：`02_sunny_dashed_solid_473423e3`、`03_sunny_middle_lane_fdab85ad`、`04_daylight_cloudy_backup_47e8da5f`。left/center/right 各有自己的 4 个 BRAM bank，位于 `fusion_maps/grid16_bram`；共用是三组素材共用映射，不是三相机使用相同坐标。

固定接缝 x=640 / 1280，128 像素羽化；动态接缝已回退，窄带羽化未启用。换镜头、安装位姿、分辨率或裁剪需重新验证 LUT，当前参数仍可能出现近景视差重影。

## 缺路与光纤

初始没有 SFP 完整帧时，下半区或四宫格第四格显示黑色，其他画面可更新；前三路仍需各收到完整帧才能启动。`source_valid` 是曾有完整帧的粘滞状态，已经运行后断开输入可能保留末帧，并非实时断线变黑。

同一光纤链路承载两个方向。拔掉后，板卡二及其 PCIe 上位机收不到板卡一合成视频而停留末帧，是传输中断；用户已澄清这不是板卡一本地四路全部卡住。接线见 [双板说明](../README.md)。

## 验证与物理时序

2026-10-30 用户反馈本版上板测试通过；本次归档只检查工程引用、文件哈希与 ZIP 完整性，不新增硬件测试结论。

已有逐像素 / 页面 / 模式切换回归随包保留：坐标 17,142 个样本；采样器 240 项；最终画布 `fusion=8, missing_quad=4, missing_updated=6, full_quad=10, returned=2, renders=11, underruns=0`。详见 `docs/SIMULATION_CANVAS_20261030.json`。

**物理时序未完全闭合**。最终报告仍有 DDR Slow Setup **−0.210 ns、−0.002 ns** 两个端点（TNS −0.212 ns），用户接受保留小违例；DDR Slow/Fast Hold 为 +0.200 / +0.115 ns，pix_clk_in/out 的报告内 setup/hold 均为正。外部 I/O 预算不完整，不能把上板通过或这些正值视作全部环境签核。完整报告在 ZIP `report_timing/test_ddr.rtr`。

bit SHA256：`808b5f97b2362fccecc630e9c3d911a13eb5fdfd01738839edbd852ec7fdb617`。

FIC SHA256：`82701ffb836baf9568c6e39799712f1451bfc7290371c2944a0424520a6e2aa1`。通用 `generate_bitstream/test_ddr.sbit/.fic` 与有日期版本相同，Debugger 请使用配套 FIC。

本包未加入暂停中的暗光 HSV / 金字塔或去雾串联实验。第三方声明保留；见 [权利通知](../DISCLAIMER.md)。旧工程可从 Git 历史恢复。
