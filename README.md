# PG2L50H-RK3568-ADAS

面向道路环境感知与辅助驾驶的 FPGA + ARM/NPU 异构边缘视觉系统。

本项目是“紫光同创 FPGA + RK3568 ARM/NPU”异构高级辅助驾驶系统工程，
仓库同时保存 FPGA 工程、ARM 端软件和模型部署工具。FPGA 负责视频采集、图像预处理、
帧缓存和低延迟数据传输；RK3568 ARM 负责系统调度、车道分析和场景逻辑，
内置 NPU 运行 INT8 量化目标检测模型。ARM 将决策结果反馈给 FPGA，最终
输出图像叠加、LED 指示和车辆控制信号。

## 最新板端源码（2026-10-23 上传）

[`board_v9`](board_v9/CURRENT.md) 保存 2026-10-22 已在 RK3568 PCIe 程序部署验证的版本：虚实线画法平滑过渡、普通目标残影修复、按类别着色、模拟导航与测试四。自动车道偏离报警已关闭；测试一红绿灯与测试二强制实线变道报警保留。模型未更换，导航为人工模拟输入。构建、启动及验证范围见 [当前版本](board_v9/CURRENT.md) 和 [验证摘要](board_v9/docs/VALIDATION_20261022.md)。

## 上次同步：测试二与检测框闪烁修复（2026-10-21）

[`board_v9`](board_v9/README.md) 已同步 2026-10-20 板端验证通过的版本：保留四宫格、测试一红绿灯匹配及方向/行驶/停止按钮，增加测试二虚线变道、实线直行、强制实线变道报警，以及本车道显示与虚实线区分。盲区联动暂缓。

修复开启车道后 YOLO 红绿灯框闪烁：一次 YOLO＋MO 观测连续占用 NPU，获准执行后再读取最新 PCIe 帧，父灯组与子灯使用统一且有上限的显示续接。灯态决策仍只使用真实观测，未延长原证据时限。

运行方式、模型区别及验证范围见 [board_v9 说明](board_v9/README.md) 和 [验证记录](board_v9/docs/VALIDATION_20261020.md)。UFLDv2 1600×320 QAT 续训、真实 INT8 PC 测试脚本及局限见 [QAT 记录](training/UFLDV2_QAT.md)。本次上传源码、配置与摘要；模型、原视频、训练集和本地测试视频独立保存。

## 板卡二PCIe图像处理工程（2026-10-19）

[板卡二源码与仿真库](fpga/dual_board/board2_sfp_pcie/README.md) 已更新为3DW包头修正版，包含DDR后统一图像处理、PCIe五字控制、RECEIVED/APPLIED两阶段回传和最新FIC探针。修复Logos IP包头tkeep=1111被图像命令过滤的问题；视频DMA和PCIe/SFP生成IP保持原有协议/源码。

见[修正说明](fpga/dual_board/board2_sfp_pcie/docs/PCIE_HEADER_KEEP_FIX.md)、[代码解说](fpga/dual_board/board2_sfp_pcie/docs/CODE_WALKTHROUGH.md)及[RC指南](fpga/dual_board/board2_sfp_pcie/docs/RC_PCIE_CONTROL.md)。9项离线仿真通过；本次发布未构建/下载位流，不附带bit，需自行重建匹配位流。ARM提供的pcie_v3无需因本修正改变命令；仓库ARM程序、板卡一和以太网程序保持当前内容。

## board_v9 初始接入记录（2026-10-17）

[`board_v9`](board_v9/README.md) 在 board_v8 的「模拟板端测试一」基础上，把 PCIe 实时输入接到 [`tools/pcie_v2`](tools/pcie_v2/README.md) 的 read 路径（`/dev/fpga_frame_<BDF>`），回放与实时共用同一套车道、选灯和按钮逻辑：

- **用途车道选灯**：左转车道只看左箭头、直行车道优先直行箭头（无箭头时才用圆灯）、右转车道只看右箭头；不再依赖 `signal_demo.txt` 的场景 ROI 人工绑定
- **方向按钮单击切换**：第一次单击开始模拟行驶，再点同一按钮停车；行驶中点另一方向即切换方向
- **报警分两类**：确认红灯时任意方向行驶报「闯红灯」；确认绿灯但方向与用途车道不符报「违规操作」，红灯优先
- 新增 `lane_signal_simulator.hpp`、`momentary_input.hpp`、`tests/input_mode_regression.cpp`；`four_view.cpp` 大改
- `driver/` 随附与 `tools/pcie_v2` 同源的驱动（4 个文件逐字节一致），驱动已加载时无需重编

仍使用 YOLO 14 类 QAT INT8（960×544）＋MobileNet 灯态 FP16（128×128），四路整图单次 YOLO，车道线与 TTC 默认关闭。说明见 [PCIe 输入说明](board_v9/PCIE_INPUT.md) 与 [启动指令](board_v9/启动指令.txt)。板端验证记录见 [验证记录](board_v9/docs/VALIDATION_SIMULATED_TEST_1_20261015.json) 与 [模拟板端测试一](board_v9/docs/SIMULATED_BOARD_TEST_1.md)：板端完成 CMake 配置与 `adas_four_view` 构建，`perception_regression`、`overlay_regression`、`avm_regression`、`preview_resize_regression`、`ufld_decode_regression`、`ui_upload_regression` 六项全部通过。**未实现自动车道/控制灯组归属判断**，夜间素材仍有拒识、漏判与灯形误分类。

## 上一版：模拟板端测试一（board_v8，2026-10-15）

[`board_v8`](board_v8/README.md) 已同步当前 RK3568 正在使用的 YOLO 14 类 QAT INT8（960×544）＋MobileNet 灯态 FP16（128×128）版本，包含四路整图单次 YOLO、中文紧凑标签、图形叠加开关、ByteTrack、指定灯组的方向判断和模拟报警。车道线与 TTC 默认关闭。

“模拟板端测试一”采用视频回放和控件模拟方向/行驶状态，YOLO 与 MO 实际在 NPU 推理。保留当前 11 组菜单素材的配置和哈希清单，夜间拒识与误分类限制如实记录。源码、构建脚本与配置随本次提交；模型、视频和本地七路原视频独立保存。见 [版本说明与验证](board_v8/docs/SIMULATED_BOARD_TEST_1.md)。

Windows PC 鸟瞰预览：双击根目录的 [启动PC鸟瞰.cmd](启动PC鸟瞰.cmd)，
可切换八组 NVIDIA 七视角拼接视频。操作和环境说明见
[PC_AVM_START.md](board_v8/PC_AVM_START.md)。该入口是 PC 视频预览，板端启动流程另见 board_v8 文档。

## 数据流

```text
四路摄像头/HDMI
       ↓
PG2L50H FPGA：同步、缓存、拼接、缩放和基础预处理
       ↓ 高速视频/控制接口
RK3568 ARM：任务调度、车道检测、交通规则与风险判断
       ↓
RK3568 NPU：INT8 道路目标检测
       ↓
ARM 决策反馈 → FPGA 图像叠加 / LED / 控制信号
```

## 双板采集与光口/PCIe 链路（2026-09-27）

新增[双板系统说明](fpga/dual_board/README.md)，包含可直接打开的两份 PDS 工程：

- [板卡 1：四路采集与光口回传](fpga/dual_board/board1_capture/README.md)：当前接入 HDMI、板卡 2 的 SFP 视频、以太网 0、以太网 1，合成 1920×1080 四宫格。
- [板卡 2：DDR后图像处理与PCIe输出](fpga/dual_board/board2_sfp_pcie/README.md)：HDMI 送板卡 1，反向接收四宫格，经 DDR/增强/PCIe 四缓冲握手送鲁班猫 2。
- [最新版 Windows 以太网上位机](tools/udp_video_sender/README.md)：可配置本机 IP、目标 IP/MAC、输入/输出尺寸与帧率、VS 消隐及包间隔。
- [鲁班猫 PCIe 采集与图像参数控制](tools/pcie_v2/README.md)：驱动 7.0（四槽视频 + STOP 安全 + 单向参数提交）+ Qt 上位机，对应板卡二 20261015 位流；五字整包提交，无 0x220/0x224 配置状态 DMA。

板卡一按源码目录保存；板卡二按ZIP提供，下载解压后打开PDS。旧版本由Git历史保留。当前接线、OV5640 保留状态、通道 1 VS 连接、HS2 驱动依赖与验证边界见系统说明。此次整合不等同于双板链路已完成时序签核及板级验收。

该快照位于私有仓库，仅作内部备份和调试；第三方材料仍受其原许可约束。详见
[免责声明与权利通知](fpga/dual_board/DISCLAIMER.md)。

## FPGA 工程内容

仓库的 [`fpga/`](fpga/) 目录包含可恢复的 Pango PDS 工程、关键 RTL、协议说明和验证记录：

| 目录 | 内容 | 当前验证边界 |
| --- | --- | --- |
| [`fpga/pcie/hdmi_ddr_pice_handshake_20260919`](fpga/pcie/hdmi_ddr_pice_handshake_20260919/README.md) | PGL50H HDMI/DDR/增强/PCIe 工程，增加 RC 四缓冲所有权握手、完成标记、归还 token 和 STOP 应答 | 控制器仿真、1080p 完整帧回归、RC 参考代码、PDS Compile/Synthesize 已通过；尚未完成布局布线、真实驱动和板级压力测试 |
| [`fpga/hdmi_ddr_pice`](fpga/hdmi_ddr_pice/README.md) | 2026-09-15 HDMI → DDR → 图像增强 → PCIe 开发基线 | 保存源码和依赖；固定四缓冲轮转没有 RC 所有权反馈，不能保证零拷贝处理期间不被覆盖 |
| [`fpga/pcie_video`](fpga/pcie_video/) | 1080p RGB565 PCIe 彩条基线及 ARM 操作说明 | 用于先验证 BAR 命令、TLP/DMA 和四缓冲传输，不代表摄像头链路完成 |
| [`fpga/ethernet_video`](fpga/ethernet_video/README.md) | 千兆以太网接收 960×540 RGB565、写 DDR 并显示，配套 Windows UDP 发送器 | 已有彩条显示现场观察；协议没有包序号、确认或重传 |

早期 PDS 基线以 ZIP 或分卷形式保存；双板版本位于 `fpga/dual_board`：板卡一为源码目录，板卡二为工程ZIP。
恢复方式、SHA-256、工具版本、已知 RTL/CDC 问题见各目录 README 和验证记录。综合成功不等于板级数据完整性、
缓存一致性和时序已经签核。

`board_v7` 保留同步视频演示；[`board_v8`](board_v8/PCIE_INPUT.md) 增加 PCIe 实时合成帧输入；
[`board_v9`](board_v9/PCIE_INPUT.md) 改用 pcie_v2 的 read 路径，并接入用途车道选灯与方向按钮切换。
双板工程提供四路采集/拼接、SFP 互传和 PCIe 输出源码；完整联合链路仍需对应位流、
HS2 驱动及鲁班猫板级验收，不能将旧版历史验证记录视为本次链路验证结果。

## 功能

- 单个 21 类道路目标检测模型，交通灯细分为红/绿两色的圆灯、左箭头、右箭头和直行箭头。
- 有效信号灯区域筛选和连续帧状态判断。
- 四点车道标定、IPM 鸟瞰变换、二值特征、滑动窗口搜索和二次曲线拟合。
- 可选 UFLDv2 CULane RKNN 车道检测；当前精度配置由前视独占完整
  1600×320 ResNet18，后视车道关闭，YOLO 与前视 UFLD 共享 NPU 调度队列。
- 车道偏移、道路曲率、安全距离、相对速度和 TTC 风险估计。
- 集成可视化界面，可导入视频、暂停、重新标定、切换检测并显示实时帧率。

## 当前可演示版本：RK3568 四路 V7

`board_v7` 是当前推荐版本，保留了现有 V7 训练权重。这里的 **V7 是第七版训练模型**，
检测网络仍为 YOLOv8n-P2，类别数为 21，并不是 YOLOv7 或 23 类模型。八个交通灯类别为：

```text
traffic_red_circle    traffic_red_left     traffic_red_right    traffic_red_straight
traffic_green_circle  traffic_green_left   traffic_green_right  traffic_green_straight
```

当前四路版本包括：

- 前、后、左、右四路同步视频和可拖动的统一进度条；支持按钮或 `[`、`]` 切换场景。
- 四路画面先按 FPGA 接口拼成一个 1920×1080 四宫格，再整体 letterbox 到固定
  640×640 输入；一次调用 RK3568 NPU 完成检测并映射回各个视角。
- ByteTrack 风格的目标关联、短时预测和检测间隔补偿，降低框体滞后与闪烁。
- 前后视角独立四点标定；标定时暂停画面，标定结果写入 `board_v7/config`。
- IPM 鸟瞰空间二次曲线拟合、逐帧插值、车辆遮挡保持，以及车道宽度突变抑制。
- 前后碰撞/TTC 预警、左右盲区提示、交通灯状态与真实显示帧率。预警逻辑使用
  ByteTrack 目标锁定、连续帧确认和进入/退出回差，避免检测间隔预测框造成单帧误报。
- 中文化 1920×1080 车载界面；保留 FPS、NPU、TTC、YOLOv8 和模型类别名等技术术语。
- 1920×1080 OpenGL ES 界面；进程绑定 CPU 2、3，RKNN Runtime 独立调用 NPU。

模型文件位于 `models/unified21_p2_v7_640_int8.rknn`。训练权重、Rockchip ONNX
和 INT8 RKNN 一并保留，便于重新转换及核对模型来源。测试视频、NVIDIA PhysicalAI
原始素材、量化图片、运行日志和构建产物体积较大，不进入 Git 仓库。

四路版本的详细操作、已验证性能和当前模型限制见 [`board_v7/README.md`](board_v7/README.md)，
九层软件框架与模块完成边界见 [`board_v7/ARCHITECTURE.md`](board_v7/ARCHITECTURE.md)，
当前及后续代码的扩展性、健壮性、安全性与验证要求见
[`board_v7/ENGINEERING.md`](board_v7/ENGINEERING.md)。

## 软件模块

The PC conversion pipeline exports the current 21-class detector with Rockchip's
YOLOv8 output layout and builds an RK3568 INT8 RKNN model. The board application
is C++11 and keeps inference, lane detection, traffic-signal selection, distance/TTC
estimation, and visualization in separate modules.

- `RknnDetector`: RKNN Runtime, NPU execution, YOLOv8 DFL decoding and NMS.
- `LaneDetector`: configurable four-point ROI, color/edge extraction, bird-eye transform, line fitting and temporal smoothing.
- `SignalLogic`: selects the forward-facing signal and requires temporal agreement.
- `RiskEstimator`: selects the obstacle in the ego corridor and estimates distance, closing speed and TTC.
- `overlay`: renders detections, lane area, signal state, risk and performance.

## PC 端模型转换

```text
python tools/select_calibration.py --count 300
python tools/export_rockchip_onnx.py
python tools/inspect_onnx.py
python tools/convert_int8.py
```

21 类模型完成训练后，将选中权重复制到 `models/unified21_light_focus_v4_selected_640.pt`，
再执行上述三步。运行脚本会优先选择 21 类 RKNN；在新模型尚未部署时自动回退到旧 17 类模型。

The RKNN Toolkit step runs in the `.rknn-env` Linux environment under WSL.

量化校准图片由脚本从本地数据集中选取，不提交到代码仓库。

## 21 类训练流水线

`training/scripts` 保存公共 ATLAS 静态图像的断点下载、八类灯整理、19→21 类
标签和检测头迁移、数据门槛检查、训练及独立评估代码。圆灯与直行箭头始终使用
不同类别；训练仍输出一个检测模型。数据集、缓存和训练结果由 `.gitignore` 排除。

```powershell
powershell -ExecutionPolicy Bypass -File training/scripts/run_unified21_full_pipeline.ps1 -Epochs 24 -Batch 16
```

## RK3568 编译与运行

当前四路版本：

```sh
cd ~/rk3568_adas/board_v7
sh scripts/build.sh
sh scripts/run_four_view.sh
```

四路样例视频是本地素材，默认目录为 `board_v7/four_view_sample/66b5fa4b_30fps`。
也可以通过命令行传入其他同步场景。单路 V7 程序可这样运行：

```sh
sh scripts/run.sh --source /path/to/video.mp4
```

旧版单路程序仍保存在 `board`：

```sh
cd ~/rk3568_adas/board
sh scripts/build.sh
sh scripts/run.sh --source /path/to/video.mp4
```

无桌面环境运行并保存结果：

```sh
sh scripts/run.sh --source /path/to/video.mp4 \
  --output output/result.avi --headless
```

With a desktop attached, omit `--headless`. The integrated dashboard shows the
annotated video on the left, bird-eye binary features at top-right, and sliding
windows plus quadratic fitting at bottom-right. Its toolbar and keys provide:

- `OPEN VIDEO [O]`: select a new video with the desktop file picker.
- `PAUSE [P]`: pause or resume playback.
- `CALIBRATE [C]`: click the four lane trapezoid corners in any order.
- `DETECT [D]`: enable or disable NPU recognition after calibration.
- `SIZE - [-]` and `SIZE + [+]`: switch among three dashboard sizes.

Importing a video freezes its first frame for calibration. Recognition starts
after four valid points are accepted. Calibration markers are hidden afterward,
and the normalized points are saved to `config/lane_roi.txt`. Playback drops
intermediate source frames when necessary to preserve the video's original speed.

## 运行依赖

- RK3568 / aarch64 Linux
- RKNN Runtime 1.5.x 与匹配的 NPU 驱动
- OpenCV 4、CMake、支持 C++11 的编译器

模型文件位于 `models/`。第三方 RKNN SDK、Python 虚拟环境、量化图片和
测试视频未纳入版本控制。

