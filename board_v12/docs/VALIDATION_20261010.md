# 2026-10-10 板端源码同步验证

本次同步来自 `/home/cat/rk3568_adas/board_v8` 的实际板端文件，保留仓库其他模块。发布前在板端独立目录 `.github_publish_validation_20261010` 编译和运行回归，未覆盖运行中的程序、模型或素材。

环境：RK3568 / aarch64，GNU C++ 8.3，OpenCV 3.2，板端现有 RKNN Runtime 1.5.0、NPU 驱动 0.8.2、FreeType、RGA、EGL/GLES2、X11 和 GStreamer 开发库。

## 源码和脚本

- 对快照内全部 C/C++ 文件进行词法比较，去除注释与空白后和发布版本一致；补充中文注释未改变其程序语句。
- 所有 `scripts/*.sh` 使用 LF，逐个执行 `sh -n` 通过。
- 增补已有灯态/方向测试为 `signal_demo_regression`，保持 Release 模式断言开启；这不改变默认应用构建和运行逻辑。
- 模型清单记录板端现有两个 RKNN 文件的大小、SHA-256 和类别顺序；权重、视频、运行日志不提交。

## 构建和回归

以下命令在板端验证目录执行并通过；也可在配置相同依赖的 `board_v8` 目录重现：

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
make -C build -j2 adas_four_view adas_demo \
  perception_regression overlay_regression ufld_decode_regression \
  avm_regression preview_resize_regression ui_upload_regression \
  signal_demo_regression

./build/perception_regression
./build/overlay_regression
./build/ufld_decode_regression
./build/avm_regression
./build/preview_resize_regression
./build/ui_upload_regression
./build/signal_demo_regression tests/fixtures/signal_demo_roi.txt
```

| 验证 | 结果及范围 |
| --- | --- |
| `adas_four_view` / `adas_demo` | 编译、链接通过 |
| `perception_regression` | 通过：调度、新结果确认、灯色核验、目标关联、告警优先级 |
| `overlay_regression` | 通过：告警确认、叠加颜色与邻道提示 |
| `ufld_decode_regression` | 通过：输出解码与坐标、缺失处理；保留模块的回归，不表示启用车道检测 |
| `avm_regression` | 通过：投影、重叠、缺失帧和缩放复位 |
| `preview_resize_regression` | 通过：双线性预览缩放最大通道误差为 1 |
| `ui_upload_regression` | 通过：可见像素、缺失相机、裁剪和区域重叠 |
| `signal_demo_regression` | 通过：红/绿确认、重复时间戳、漏检保持及超时、颜色冲突、ROI 筛选、方向切换和强制行驶报警 |

构建有一个既存的未使用函数警告：`draw_surround_map`（`src/four_view.cpp`）。应用和全部测试正常链接、运行。

本次验证针对源码发布的可编译性和逻辑回归。未重新测量模型精度、显示 FPS 或端到端延迟，旧版文档中的性能数字不能视作当前双阶段版本的测试结果。
