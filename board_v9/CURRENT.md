# 当前板端版本：测试一、测试二与闪烁修复

本目录保存 2026-10-20 板端验证通过、2026-10-21 上传的最新源码。输入为 PCIe 四宫格实时视频，也保留文件回放；`board_v8` 保留历史测试一版本及 PC 工具。

## 功能

- 保留四宫格、中文紧凑标签、叠加开关及 ByteTrack。
- 测试一：左/直/右用途车道选灯；方向按钮单击行驶，再点同方向停车；确认红灯优先报警，绿灯下方向不符报警。保留原行驶/停止状态与按钮。
- 测试二：左变道、保持直行、右变道、强制左变道、强制右变道五个按钮。请求侧虚线且证据有效时显示对应箭头；实线保持直行；强制实线请求红色报警；未知线型保持直行。盲区条件按当前演示要求暂不参与。
- UFLDv2 1600×320 本车道边界、绿色区域与箭头；虚线分段、实线连续，未知或不可靠观测不回退画白线。测试二前视指示框为 220×58，两侧邻接车道暂不显示。
- 两个准备线程共享一个串行 NPU 队列。保留几何/卡尔曼跟踪与有限保持，显示保持不冒充新观测。
- 修复 YOLO 红绿灯框闪烁：YOLO＋MO 完成整组观测后再让出 NPU；获得 NPU 使用权后再取最新 PCIe 输入；父子灯组统一显示期限。换色、连续真实漏检和绝对采集年龄过期仍清除旧框。

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

构建、按钮逻辑与闪烁回归均通过，实机车道开/关/再开短测记录见 [验证摘要](docs/VALIDATION_20261020.md)。车道开启时 YOLO、UFLD 各约 3–3.5 次更新/秒、显示约 11–12 FPS，NPU 72–81%；数值随素材和负载变化，不能视为保证值。

没有人工标签，几何有效占比不等于准确率；夜间、阴影、磨损线仍可能漏检。测试二为视觉与按钮规则演示，未连接车辆执行机构。旧 README 下方测试一记录、场景清单与 PCIe 说明保留作历史参考，当前功能以本页为准。
