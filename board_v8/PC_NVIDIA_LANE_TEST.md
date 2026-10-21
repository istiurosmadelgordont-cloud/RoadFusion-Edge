# NVIDIA 多车道素材 PC 测试

2026-10-17 新增清晰标线素材：用 128 段原始运动数据预筛后，下载 14 段新前视视频（175.3 MB），挑选 5 段完整 PC 测试，共 2999 帧。`启动PC清晰车道素材.cmd` 按 1–5 观看：意大利清晰白线高速、美国宽阔多车道、美国邻车并行、美国跟车与桥下、荷兰出口弯道。详细结果在 `board_v8/output/pc_nvidia_clear_lanes_20261017/REPORT.md`，前三段约 60 秒合集为该目录 `clear_lanes_60s.mp4`。第 1 段原画白线对比明显，第 2、3 段适合多车道展示，第 5 段保留作复杂出口测试。

2026-10-17：PC 播放器已更新相邻车道显示稳定性优化。可见部分通过检查时保留该部分，左右分别跟踪和平滑；短暂缺失最多保留 180 ms，并淡化、标为 HOLD。六组主演示的右侧显隐切换合计从 508 次降到 74 次，新增处理平均约 1 ms。显示结果不写回用于决策的车道几何。本次是 PC 显示更新，未部署至板端。

双击 `启动PC车道稳定性对照.cmd` 可观看左侧优化前、右侧优化后的同帧对照。普通播放器仍显示左侧原始检测点、右侧优化区域。详细记录见 `board_v8/output/pc_lane_stability_20261017/REPORT.md`，包含六组主演示和一组较差素材的 4200 帧顺序验证；9 项行为回归通过。

原始点线模式：`启动PC车道原始点线预览.cmd`。将同批 PC 实测保存的四槽原始点放大为单个前视画面，保留橙、绿、紫配色；直接逐帧显示，不进行区域填充、曲线拟合或时序平滑。清单当前有 8 组，1–5 是新增清晰标线素材，6–8 是原来的 3 组。清单在 `board_v8/output/pc_lane_raw_points_20261017/`；原有 `raw_points_60s.mp4` 仍为旧 3 段合集。空格暂停。生成工具为 `board_v8/tools/pc_lane_point_preview.py`，使用原始相机视频和 `decoded_slots.npz`，逐帧校验源帧号、时间戳。

2026-10-16：首轮下载 6 段原始前视片段，选择 2 段新高速素材和 1 段已有高速并入素材；第二轮又下载 18 段、247.9 MB，逐段抽帧检查，5 段完成完整顺序测试，新增 3 段高速素材。播放器现在有 6 组，默认先显示第二轮的新素材。

双击仓库根目录 `启动PC英伟达车道测试.cmd`，或原来的 `启动PC车道测试.cmd`。

| 按键 | 场景 | 原始片段 ID |
| --- | --- | --- |
| 1（默认，新增） | 晴天开阔多车道高速 | d473d337-3079-477d-826d-aebcbdc075f0 |
| 2（新增） | 高速邻车超越与并行 | f806f6e6-24f3-4fc2-b166-f26f8d1e48f0 |
| 3（新增） | 高速右侧车辆并入 | ed5a7be3-259b-4165-9f67-d01b29544c22 |
| 4 | 首轮白天多车道高速 | cac16498-c454-455b-9946-22787e574843 |
| 5 | 高速货车并行，观察遮挡和边界恢复 | 3bdafdf3-fa71-486c-825e-03f2d788f72f |
| 6 | 首轮高速右侧车辆并入 | 3107d317-e466-4b8d-a90b-ff993342e77a |

空格暂停/继续，Q 或 Esc 退出。左图显示模型原始锚点，右图显示车道候选区域及短暂保持，底部提供 partial、held 与实际测得耗时。窗口播放已完成真实推理的导出回放，保持原视频时间尺度。

本轮采用原 UFLDv2 CULane ResNet18 的 1600×320 模型，PyTorch FP32、RTX 4070 Laptop。原始 1920×1080 画面直接进入预处理，仅显示缩至 640×360。与原 ONNX CPU 同一图像核对，四槽坐标最大差异小于 0.0001 个显示像素。

六组主演示各 600 帧，累计 3600 次独立推理。首轮三组检测平均约 28 ms，第二轮三组本次实测分别为 29.2、46.0、49.8 ms；完整几何观测比例分别为 96.3%、95.3%、100%，完整三候选区域比例分别为 40.7%、30.5%、49.5%。这些比例反映几何门限覆盖，不是人工标注准确率。不同测试时机器负载不同，耗时不能直接作为素材间性能比较。导出文件 30 FPS 不等于实时推理达到 30 FPS。

另一个新片段 `1dd8feff` 也完成 600 帧测试，完整几何观测比例仅约 25.5%，且保持比例较高，未加入主演示；该诊断结果仍保留。完整几何观测比例、三候选区域比例是模型门限统计，不能当作有标注真值上的准确率。PC FP32 结果也不能替代板端 INT8/QAT 验证。

当前优化版结果目录：`board_v8/output/pc_lane_stability_20261017/`。播放器清单仍为 `board_v8/output/pc_nvidia_lane_test_20261016/test_manifest.json`，已指向优化版；旧视频、原始报告和清单备份继续保留。

优化前结果目录：`board_v8/output/pc_nvidia_lane_test_20261016/`。

第二轮结果、全部 18 段筛选记录及 5 段完整测试保存在 `board_v8/output/pc_nvidia_lane_test_20261016_round2/`；详情见该目录 `REPORT.md`。其中 `8158dbdd` 完整几何仅 25%，`1435e257` 为 68% 且保持约 11.2%，未加入主演示。第二轮只下载和测试前视画面；通过元数据确认入选片段还具备原始后、左、右相机，另外三路尚未下载、同步。

- `test_manifest.json`：播放器清单、源片段、测试参数及选择记录。
- `REPORT.md`：本轮测试统计和测量范围。
- 每组 `ufld_pc.mp4`、`frames.csv`、`report.json` 和 3/8/14 秒截图。
- `three_lanes_confirmed.jpg`：有完整锚点覆盖且非保持/身份不确定时的三候选区域截图。

新原始素材与元数据、SHA256 保存在 `D:/brige/multi_camera_material/nvidia_physical_ai_av/candidate/lane_pc_20261016/`。下载工具为 `tools/fetch_nvidia_lane_material.py`，只读取选定 ZIP 成员，使用已有 Hugging Face 登录。官方来源：[NVIDIA PhysicalAI-Autonomous-Vehicles](https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles)。

重新推理可运行：

```powershell
& D:/brige/adas_training/.venv-train/Scripts/python.exe board_v8/tools/pc_ufld_video.py --backend torch-cuda --videos D:/brige/multi_camera_material/nvidia_physical_ai_av/candidate/lane_pc_20261016/cac16498-c454-455b-9946-22787e574843/front.mp4 --output board_v8/output/pc_lane_stability_20261017 --stride 1 --max-seconds 20 --compare --cache-slots
```

没有 CUDA 时可使用默认 `--backend onnx-cpu`。播放器只需 OpenCV 和 NumPy，不要求安装训练环境。
