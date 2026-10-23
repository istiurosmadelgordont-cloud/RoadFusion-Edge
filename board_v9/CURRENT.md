# 当前板端版本：四宫格 ADAS、车道显示过渡与 LDW 关闭

本目录保存 2026-10-22 已部署并验证、2026-10-23 上传的源码。输入为 PCIe 四宫格实时视频，也保留文件回放；`board_v8` 保留历史测试一版本及 PC 工具。

## 功能

- 保留四宫格、中文紧凑标签、叠加开关及 ByteTrack。
- 测试一：左/直/右用途车道选灯；方向按钮单击行驶，再点同方向停车；确认红灯优先报警，绿灯下方向不符报警。保留原行驶/停止状态与按钮。
- 测试二：左变道、保持直行、右变道、强制左变道、强制右变道五个按钮。请求侧虚线且证据有效时显示对应箭头；实线保持直行；强制实线请求红色报警；未知线型保持直行。盲区条件按当前演示要求暂不参与。
- UFLDv2 1600×320 本车道边界、绿色区域与箭头；虚线分段、实线连续，线型待确认但当前几何可靠时显示绿色参考边界；无效或过期观测不补画旧线，也不回退画白线。虚实线画法使用独立显示渐变，UNKNOWN 仍不允许变道。测试二前视指示框为 220×58，两侧邻接车道暂不显示。
- 两个准备线程共享一个串行 NPU 队列。保留几何/卡尔曼跟踪与有限保持，显示保持不冒充新观测。
- 修复 YOLO 红绿灯框闪烁：YOLO＋MO 完成整组观测后再让出 NPU；获得 NPU 使用权后再取最新 PCIe 输入；父子灯组统一显示期限。换色、连续真实漏检和绝对采集年龄过期仍清除旧框。

- 已关闭自动车道偏离报警（LDW），包括偏离红框、边界告警着色和 TLC 接近边界提示；车道检测、虚实线识别、强制实线变道报警与测试一红绿灯规则保留。
- 检测框按类别区分颜色；真实漏检后隐藏普通目标旧框，保留内部跟踪 ID 供重新关联；显示运动补偿不参与安全决策。
- 右上支持模拟导航、测试四与 AVM 切换。导航方向/距离由按钮模拟，未实现视频导航 OCR；前视不显示原导航悬浮小窗。详见 [模拟导航](docs/NAVIGATION_DEMO.md)。
- 测试四复用行人/前车观测演示模拟停车与人工解除，默认关闭；详见 [测试四](docs/TEST4.md)。

## 模型

| 用途 | 默认文件 | 说明 |
| --- | --- | --- |
| 目标定位 | `yolo14_960x544_qat_int8.rknn` | 960×544、14 类、QAT INT8，NPU 推理 |
| 灯形与颜色 | `mo_fp16_baseline.rknn` | 128×128、9 类含拒识、FP16，NPU 推理 |
| 车道几何 | `ufldv2_culane_res18_1600x320_int8.rknn` | 原 UFLDv2 INT8 部署模型，NPU 推理；未替换为新续训 QAT |

文件哈希见 [模型清单](config/model_manifest.json)。YOLO 与 MO 必须放在同一目录。模型位于本目录 `models/` 或仓库根目录 `models/`，不随本次源码提交上传。RKNN 是部署格式，INT8 是量化精度，QAT 是训练方式，不能视为同一个含义。

新车道 QAT 候选在板端独立测试中比当前原模型慢，未切换默认模型；数字输出名候选也需严格头格式适配，不能只改路径。详见 [QAT 训练记录](../training/UFLDV2_QAT.md)。

## 构建与运行

依赖 CMake、C++11、OpenCV、FreeType、RKNN Runtime、MPP/GStreamer、RGA、EGL/GLES2、X11；中文字体为 `/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc`。

```sh
cd /home/cat/rk3568_adas/board_v9
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target adas_four_view -- -j1
export DISPLAY=:0
export XAUTHORITY="$HOME/.Xauthority"
sh scripts/run_pcie.sh --enable-lanes

# 默认关闭车道，也可在界面顶部或用 L 键开启
sh scripts/run_pcie.sh

# 回放视频独立准备
sh scripts/run_four_view.sh --scene four_view_sample/SCENE --enable-lanes
```

PCIe 使用 pcie_v2 的 `/dev/fpga_frame_<BDF>` read 节点；本次应用修复不要求更换位流或驱动。多节点时显式设置 `PCIE_DEVICE`。启动脚本仅在硬件支持、上限允许且有非交互权限时恢复 NPU 最低 800MHz，不改 governor 和最高频率。

按键：`L` 切车道；`1/2/3` 切方向行驶/停车；`4/5/6` 选用途车道；`H` 切叠加；`V` 环视放大；`Q/Esc` 退出。测试二使用右侧五个按钮。

2026-10-22 板端构建与 7 项回归检查通过，部署后 PCIe 实时 YOLO 和 UFLD 均产生新结果，车道偏离报警关闭。车道更新约 3.5 次/秒，近期显示样本约 7 FPS；这是特定素材下的实测样本，不是帧率保证。记录与部署二进制哈希见 [最新验证摘要](docs/VALIDATION_20261022.md)。2026-10-23 上传前已恢复 SSH 并核对板端 116 个源码、配置和脚本文件，部署二进制及三份模型哈希与记录一致；未重新进行模型准确率测试。

没有人工标签，几何有效占比不等于准确率；夜间、阴影、磨损线仍可能漏检。测试二为视觉与按钮规则演示，未连接车辆执行机构。旧 README 下方测试一记录、场景清单与 PCIe 说明保留作历史参考，当前功能以本页为准。
