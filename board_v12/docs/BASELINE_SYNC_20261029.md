# board_v12 非 RGA 基线同步

2026-10-29。源基线为板端 `/home/cat/rk3568_adas/board_v11.before-new-baseline-20261027-195936` 的实际内容，目标仍为 `board_v12`。

## 恢复的非 RGA 差异

| 范围 | 同步结果 |
| --- | --- |
| 相机/界面 | 左/前/右/后槽映射；对应检测、HSV、运动参考、车道 ROI；标题栏放大、视角切换及 Z 键 |
| 鸟瞰 | 本地素材、手动时钟、播放/停止、复位和时间微调入口 |
| 预警 | 基线 FCW 控件和位置、测试四行人提前门槛、多目标红框及持续新观测后自动解除 |
| 车道 | 近场车身遮挡保护、虚实线采样与显示、强制变道及状态日志 |
| 运行 | 基线模型清单、启动默认开关、原串行运动校正、GLES 呈现、测试及文档 |

## RGA 例外

保留最新固定 DMA-BUF/RGA 采集、颜色修复、预览、YOLO/MO/UFLDv2 前处理、页对齐有界池、同步调用及原 STOP 行为；read/CPU 预览开关仍移除。
仅为适配恢复的相机放大，显示 RGA 池同时支持 672×378 和 1356×762；对应回归验证池饱和、复用、数据保护，并增加真实 RGA 放大颜色检查。
源码相对指定基线只保留上述 RGA 差异。版本标识和现行操作文档使用 board_v12，历史文档保留其原出处。
编译输出、旧运行日志和内核模块编译产物不作为源码同步对象；模型/视频单独核对。

## 验证

- ARM64 完整构建成功：`sh scripts/build.sh`，包含 `adas_four_view`、`adas_demo` 及默认构建目标。
- 14 项回归全部通过：相机映射、布局/放大、鸟瞰时钟、真实控件、虚实线、行人安全、感知、叠加、FCW、HSV 视图、RGA 采集、RGA 预览/放大、真实 RGA 颜色、图像参数控制。
- 真实 RGA 遍历 65536 种 RGB565 值，转 BGR 最大通道误差 0；BGR→RGB 最大通道误差 0。带步长 ROI、672×378 四宫格及 1356×762 放大颜色检查通过。
- PCIe 实际采集短测正常退出 30 帧，打开车道、FCW、测试四及模拟导航；YOLO、MO 灯色识别和 UFLD 均运行。截图确认左/前/右/后布局及检测叠加，采集设备已释放。
- 该短测为无窗口运行，验证了 RGA 软件合成显示、模型处理和恢复的布局；未进行完整交互/GLES 窗口测试及长时间性能或准确率复测。短测 FPS 不作为完整视频性能结论。
- 196 个交付源码/配置/文档文件逐文件 SHA256 校验；8 个模型及 386 个素材文件与指定基线内容一致。驱动保留板上 8.2，未重载驱动或提交 FPGA 图像参数。

主程序 SHA256：`6171d0360706b76a4bac3cba4407a6ad5186b624ed7183a7cc26b0114b16a827`。

## 备份和证据

板端变更前源码及 build 备份：`/home/cat/rk3568_adas/board_v12.before-baseline-sync-20261029-164323.tar.gz`。
备份 SHA256：`37d2d6a8aa5ed59622de5a782fac0a56d363c93c9256897d74e007c96e2ebb37`。
板端备份排除已校验一致且未修改的 models/、four_view_sample/。
PC 本地变更前目录：工作区 `_board_sync/local-board_v12-before-sync-20261029.tar.gz`。
PC 证据目录：工作区 `_board_sync/evidence/`，包含构建日志、14 项测试输出、模型/素材比对、真实采集日志及截图。
板端对应日志保存在 `/tmp/board-sync-*-20261029.*`。

启动仍为 `cd /home/cat/rk3568_adas/board_v12 && sh scripts/run_pcie.sh`。


## 相对基线保留的差异文件

- `CMakeLists.txt`
- `CURRENT.md`
- `PCIE_INPUT.md`
- `README.md`
- `docs/PCIE_RGA_INPUT.md`
- `docs/PCIE_RGA_PREVIEW.md`
- `docs/PCIE_RGA_STAGE1.md`
- `docs/PCIE_V4_ADAPTATION.md`
- `include/adas/lamp_classifier.hpp`
- `include/adas/pcie_capture.hpp`
- `include/adas/pcie_rga.hpp`
- `include/adas/pcie_rga_preview.hpp`
- `include/adas/rga_image_ops.hpp`
- `include/adas/rknn_detector.hpp`
- `include/adas/ufld_lane_detector.hpp`
- `scripts/run_pcie.sh`
- `src/four_view.cpp`
- `src/lamp_classifier.cpp`
- `src/pcie_rga_probe.cpp`
- `src/rknn_detector.cpp`
- `src/ufld_lane_detector.cpp`
- `tests/input_mode_regression.cpp`
- `tests/pcie_rga_capture_regression.cpp`
- `tests/pcie_rga_capture_regression.py`
- `tests/pcie_rga_color_hardware.cpp`
- `tests/pcie_rga_preview_regression.cpp`
- `third_party/librga/README.md`
- `启动指令.txt`
