# PC UFLDv2 车道测试

2026-10-16 新增三组 NVIDIA 多车道精选测试。默认双击入口优先打开新清单，说明见 [NVIDIA PC 测试](PC_NVIDIA_LANE_TEST.md)。以下保留旧三组与 CPU 测试方法；当前推理已改为从原始分辨率预处理，避免预先缩图损失细车道线。

在仓库根目录双击 `启动PC车道测试.cmd`，打开已完成推理的三组测试回放。
键盘 `1`：侧方车辆，`2`：跟车，`3`：开阔道路；空格暂停，`Q` 或 Esc 退出。
左侧是模型四个 slot 的原始检测点，右侧是拟合、几何校验和短暂保持后的车道区域。

重新执行真实模型推理（在仓库根目录）：

```powershell
& D:/python/python.exe board_v8/tools/pc_ufld_video.py --videos D:/brige/multi_camera_material/adas_demo_groups/11_side_vehicle_bsd/original_four_view/front.mp4 D:/brige/multi_camera_material/adas_demo_groups/03_daylight_following/original_four_view/front.mp4 D:/brige/multi_camera_material/adas_demo_groups/04_clear_open_road/original_four_view/front.mp4 --stride 6 --max-seconds 20
```

使用本地 `models/ufldv2_culane_res18_1600x320.onnx`，OpenCV DNN、CPU、FP32。
预处理归一化与 RKNN 转换配置对应，四 slot 混合解码、拟合及保持规则对应板端当前代码。
本工具未增加卡尔曼滤波；不评测实虚线语义、LDW 决策或 YOLO/UFLD 共享 NPU 调度。
ONNX FP32 与板端 RKNN INT8 数值和耗时可能不同，不能把 PC 结果当作板端实测。

默认每六帧检测一次，即 30 FPS 素材按每秒五次取样；导出保持原始视频时间尺度。
窗口播放的是测试结果，不会再次运行推理。底部的 `completed results` 是生成视频时按
实际识别完成时刻统计的速率，不是视频回放帧率。`detect` 包含预处理、模型前向、解码与拟合，
`CPU forward` 仅为模型前向耗时。输出在 `board_v8/output/pc_ufld_test`，每组包含
`ufld_pc.mp4`、逐次记录 `frames.csv`、摘要 `report.json`、3/8/14 秒截图。
`summary.json` 汇总本次指定的素材；所有生成结果均不提交到 Git。

有效车道比例和三车道显示比例仅是诊断统计，不是标注数据上的检测准确率。
