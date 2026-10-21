# PC 鸟瞰预览启动

在当前电脑双击 `D:\brige\rk3568_adas\启动PC鸟瞰.cmd`。
该入口打开八组 NVIDIA 七视角拼接预览。再次启动会把已有窗口调到前台。

也可以在 PowerShell 执行：

```powershell
& 'D:\brige\rk3568_adas\启动PC鸟瞰.cmd'
```

无需进入 scripts 目录，无需运行 Linux 的 `.sh` 文件。
`board_v8\scripts\run_pc_surround_review.cmd` 是同一个入口；
`run_pc_avm_preview.cmd` 保留原来的单组实验，用于对照。

## 窗口操作

| 操作 | 方式 |
| --- | --- |
| 选择素材 | 点击顶部编号，或按 1–8 |
| 上一组 / 下一组 | 点击 PREVIOUS / NEXT，或左右方向键、A / D |
| 暂停 / 恢复 | 点击 PAUSE / RESUME，或空格 |
| 放大鸟瞰 | 点击 ENLARGE，或 V |
| 四路 / 七路 / 补纹理对比 | 点击 COMPARE，或 B；显示固定第 8 秒样本 |
| 检查拼接位置 | S 显示/隐藏各相机的彩色接缝线；配合 V 放大、空格暂停查看 |
| 从头播放 | R |
| 退出 | Q、Esc 或关闭窗口 |

使用快捷键前先点击播放器窗口。放大模式保留完整鸟瞰区域，
不会被顶部素材按钮遮住。当前 FPS export 表示导出视频帧率；
这是 1 倍速拼接视频回放，未运行实时 YOLO / UFLD 推理。

## 素材与 Python

预览视频位于 `board_v8\output\pc_surround_groups\各组名称\nvidia_seven_camera_avm.mp4`。
启动只读取已生成的视频，不会重新下载或重新拼接，也不需要板子在线。
播放器仅在新视频帧或操作状态改变时重画画面。

当前电脑优先使用 `D:\python\python.exe`；其他电脑可指定：

```powershell
$env:ROADFUSION_PYTHON = 'C:\你的Python目录\python.exe'
& 'D:\brige\rk3568_adas\启动PC鸟瞰.cmd'
```

Python 需要 `opencv-python` 和 `numpy`。失败时窗口会保留错误信息。
如果没有已导出视频，按 `AVM.md` 中的下载和生成流程准备素材；
生成还需要各组原始七路视频、时间戳和对应标定文件，以及 pandas、pyarrow、SciPy。

## 尚需优化的画面问题

- 路沿、车辆跨接缝时可能有错位：当前映射假定地面平坦，存在高度的物体不能直接对齐。
- 近处车辆、杆件会拉伸：需要高度/深度信息或更适合俯视的相机布置进一步修正。
- 缺失区域的纹理补位属于显示效果，不能当作真实地面覆盖。
- 最初八组导出为 10 FPS，快速运动有顿挫。现在八组均已从原始七路视频重新生成，
  使用 30 FPS 采样并启用鸟瞰区域抗锯齿，减轻纹理闪动。重复刷新已有视频不会补出新帧。

在工程根目录可重新生成指定组的 30 FPS 视频：

```powershell
& 'D:\python\python.exe' board_v8/tools/build_pc_surround_groups.py --groups 02_daylight_side_traffic --video-fps 30
```

省略 `--groups` 可重新生成全部八组。各组旧版视频及报告保存在
`output\pc_surround_groups\各组名称\previous_10fps`，便于对照。

生成前退出播放器，避免视频文件被占用。该步骤从原始七路图像重新采样，
并保持 1 倍速；不会用插帧假装原始识别帧率。生成器现在默认使用 30 FPS。

播放器按下一视频帧的时间调整等待，减少播放间隔忽长忽短。
接缝处的车辆变形仍受平面投影限制。

接缝已使用主视角窄范围融合和有限亮度校正；当前结果仍是近似 2D BEV 原型。
`S` 只标出实际相机分区，不能自动修正错位。检查时优先看道路标线和地面纹理；
车辆、路沿、树木等具有高度的物体，在不同相机投影中可能不重合。
