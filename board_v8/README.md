# RK3568 四路 YOLO + MobileNet 板端工程

本目录同步自 2026-10-10 板端实际运行版本：**YOLO 14 类定位 QAT INT8（960×544）+ MobileNet 9 类灯态 FP16（128×128）**，两者通过 RKNN Runtime 在 RK3568 NPU 上运行。

四路默认输入为整幅 1920×1080 四宫格，一次任务调用一次 YOLO；YOLO 灯组框再生成裁剪，由 MO 判断红/绿圆灯、左转、右转、直行箭头及拒识。黄色不属于当前 MO 的输出类别。

## 构建和启动

依赖板端已有的 C++11 编译器、CMake、OpenCV、FreeType、RKNN Runtime、RGA、EGL/GLES2、X11、GStreamer（含 app/video 开发库及 Rockchip MPP 解码插件）。当前验证环境：aarch64、OpenCV 3.2、RKNN Runtime 1.5.0、NPU 驱动 0.8.2。

```sh
cd /home/cat/rk3568_adas/board_v8
sh scripts/build.sh
export DISPLAY=:0
export XAUTHORITY="$HOME/.Xauthority"
sh scripts/run_four_view.sh
```

将两个模型放在同一目录 `board_v8/models/`（或仓库根目录 `models/`）：

- `yolo14_960x544_qat_int8.rknn`
- `mo_fp16_baseline.rknn`

精确 SHA-256 和类别顺序见 [模型清单](config/model_manifest.json)。模型权重、视频和编译产物不随这次源码同步提交；现有板端文件继续使用。MO 尺寸契约是 128，不能直接替换成 160 输入而不修改预处理和张量检查。

默认回放目录为 `four_view_sample/signal_ru_20261010`。场景目录可提供单个预拼接文件 `sync_720p.mp4` 或 `fpga_1080p.mp4`，程序优先读取前者；没有预拼接文件时需要 `front.mp4`、`rear.mp4`、`left.mp4`、`right.mp4` 四个视频。读取规则见 `src/four_view.cpp` 中的 `open_scene()`。这些是测试素材，不表示四路均为同一车辆真实同步采集。

```sh
# 指定其他四路素材目录；默认车道检测和 TTC 均关闭。
sh scripts/run_four_view.sh --scene four_view_sample/my_scene

# 单次完整回放，无图形窗口，可用于板端检查。
sh scripts/run_four_view.sh --headless --once --max-frames 90 --dump-detections
```

## UI 和模拟行为

| 操作 | 效果 |
| --- | --- |
| `1` / `2` / `3` 或右侧按钮 | 选择左转 / 直行 / 右转，所有场景可用 |
| `H` 或“图形叠加”按钮 | 切换四路左上角的小状态叠加，检测继续运行 |
| `A` / `G` / `S` | 已绑定主灯组场景：自动 / 强制行驶 / 停车 |
| 空格 | 已绑定主灯组场景：切换模拟行驶与停车 |
| `[` / `]` 或底部按钮 | 上一场景 / 下一场景 |
| `Q` | 退出 |

主视角场景目录含 `signal_demo.txt` 时启用明确灯组的通行演示，文件格式：

```text
roi 0.30 0.05 0.65 0.40
```

四个数是**左上视角图像**内归一化的 `x1 y1 x2 y2`；该示例仅说明格式，实际 ROI 必须按素材选择。绑定主灯组后红灯停车、绿灯确认后起步，红灯时按 `G` 强制行驶会触发模拟报警。未绑定时仍可查看灯态和选择方向，但 UI 显示“未选主灯组”，不提示车辆通行。其他三路显示其各自视角灯态。

当前没有自动判断本车道受哪组灯控制，也没有把 UI 模拟报警直接接入真实车辆制动。PCIe 接入方式保留在 [PCIe 输入说明](PCIE_INPUT.md) 和 `scripts/run_pcie.sh` 中。

## 关键处理逻辑

- `src/rknn_detector.cpp`：YOLO 张量解码、坐标还原、NMS、14 类与内部 ID 映射；每次结果最多分类 8 个灯组。
- `src/lamp_classifier.cpp`：亮灯候选、外扩 1.2 倍方形裁剪、128 输入预处理和 MO 推理；最多 4 个候选/灯组。分类概率至少 0.60、前两名差值至少 0.15，并校验颜色证据。
- `include/adas/signal_demo.hpp`：方向筛选、红绿冲突处理、连续新检测确认、短时漏检保持和超时复位。红灯至少 2 次且 0.2 秒，绿灯至少 3 次且 0.6 秒；红灯保持 2 秒，绿灯保持 1.5 秒。
- `src/four_view.cpp`：MPP 回放/PCIe 输入、一次整图检测、异步调度、四路拆分、UI 和按键；UI 重绘不增加灯态确认票数。
- `src/gl_presenter.cpp`：X11/GLES 显示和按键转发。

车道检测和 TTC/FCW/RCW 当前默认关闭，可按需求用 `--enable-lanes` / `--enable-ttc` 启用；普通场景的盲区逻辑保留。历史单模型及车道测试记录见 [旧版说明](docs/LEGACY_README.md)，其精度和速度数字不能当作当前双阶段版本的实测结果。

## 校验

源码同步清单见 [板端快照](docs/BOARD_SNAPSHOT_20261010.json)，记录注释补充前板端 66 个文本文件的 SHA-256；提交中的关键文件增加了中文注释。此次构建与测试结果见 [验证记录](docs/VALIDATION_20261010.md)。shell 脚本必须使用 LF 换行，仓库 `.gitattributes` 已约束 `*.sh`。
