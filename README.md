# PG2L50H-RK3568-ADAS

面向道路环境感知与辅助驾驶的 FPGA + ARM/NPU 异构边缘视觉系统。

本项目是“紫光同创 FPGA + RK3568 ARM/NPU”异构高级辅助驾驶系统的
ARM 端软件与模型部署工程。FPGA 负责多路视频同步采集、图像预处理、
帧缓存和低延迟数据传输；RK3568 ARM 负责系统调度、车道分析和场景逻辑，
内置 NPU 运行 INT8 量化目标检测模型。ARM 将决策结果反馈给 FPGA，最终
输出图像叠加、LED 指示和车辆控制信号。

> 当前仓库包含 RK3568 端 C++ 程序、RKNN 模型与转换工具，以及 FPGA 开发基线。
> HDMI → DDR → 图像增强 → PCIe 工程见 [fpga/hdmi_ddr_pice](fpga/hdmi_ddr_pice/README.md)，
> 原 PCIe 彩条基线保留于 [fpga/pcie_video](fpga/pcie_video/)。当前 FPGA 版本的已知问题与验证边界见各目录说明。
> 当前以太网 960×540 RGB565 接收工程及 Windows 视频/图片发送器见 [fpga/ethernet_video](fpga/ethernet_video/README.md) 和 [tools/udp_video_sender](tools/udp_video_sender/README.md)。

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

## 功能

- 单个 21 类道路目标检测模型，交通灯细分为红/绿两色的圆灯、左箭头、右箭头和直行箭头。
- 有效信号灯区域筛选和连续帧状态判断。
- 四点车道标定、IPM 鸟瞰变换、二值特征、滑动窗口搜索和二次曲线拟合。
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
- 四路画面拼成一个 640×640 输入，一次调用 RK3568 NPU 完成检测，再映射回各个视角。
- ByteTrack 风格的目标关联、短时预测和检测间隔补偿，降低框体滞后与闪烁。
- 前后视角独立四点标定；标定时暂停画面，标定结果写入 `board_v7/config`。
- IPM 鸟瞰空间二次曲线拟合、逐帧插值、车辆遮挡保持，以及车道宽度突变抑制。
- 前后碰撞/TTC 预警、左右盲区提示、交通灯状态与瞬时显示帧率。
- 1920×1080 OpenGL ES 界面；进程绑定 CPU 2、3，RKNN Runtime 独立调用 NPU。

模型文件位于 `models/unified21_p2_v7_640_int8.rknn`。训练权重、Rockchip ONNX
和 INT8 RKNN 一并保留，便于重新转换及核对模型来源。测试视频、NVIDIA PhysicalAI
原始素材、量化图片、运行日志和构建产物体积较大，不进入 Git 仓库。

四路版本的详细操作、已验证性能和当前模型限制见 [`board_v7/README.md`](board_v7/README.md)。

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
