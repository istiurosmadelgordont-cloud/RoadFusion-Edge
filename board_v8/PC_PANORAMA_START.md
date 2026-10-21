# PC 前方三路融合预览

双击仓库根目录 `启动PC前方融合.cmd`。按 `1`／`2` 切换路口、跟车两组素材，
空格暂停，`B` 切换同帧前后对比，`Q` 或 Esc 退出。对比模式上方是原固定接缝，
下方是根据画面内容优化后的接缝。

使用 NVIDIA 同一 clip 的左交叉、前宽角、右交叉视频、各自相机标定和采集时间戳。
预处理沿用 FTheta 标定投影到 240° 柱面画布，重叠处优先切过画面差异小、边缘较少的位置。
这不是车辆检测或深度重建，无法保证接缝避开每辆车。

接缝搜索范围为原边界左右 120 个全景像素；连续视频中的位移限制为每秒 20 个像素，
根据视频时间戳计算。跳转、倒放时间戳或相隔超过 0.5 秒时重新初始化。
仅在两路都有真实像素的位置改变来源，接缝窄带融合，不延长或生成缺失区域。
车身遮罩导致重叠断开时分段求解，保留有效区域已得到的接缝。

默认导出视频每秒取 10 帧，PC 窗口播放已经完成的融合结果；导出 FPS 不表示实时融合性能。
输出在 `board_v8/output/pc_front_panorama_v2`，包含融合视频、`comparison.mp4`、截图、
处理耗时 `report.json` 和逐帧接缝差异及位移 `seams.json`。原先固定接缝版本保留在
`board_v8/output/pc_front_panorama`。

重新生成两组视频：

```powershell
& D:/python/python.exe board_v8/tools/pc_front_panorama.py --groups D:/brige/multi_camera_material/adas_demo_groups/01_daylight_intersection D:/brige/multi_camera_material/adas_demo_groups/03_daylight_following --duration 5 --fps 10
```

验证接缝行为：

```powershell
& D:/python/python.exe board_v8/tests/panorama_seam_regression.py
```

统计的接缝颜色差异只衡量切换处的画面一致程度，不是图像整体的配准精度。
相机安装位置、高度造成的近物视差、道路标线错位及未观测区域仍可能存在。
本原型没有接入 YOLO、UFLD 或板端 UI。
