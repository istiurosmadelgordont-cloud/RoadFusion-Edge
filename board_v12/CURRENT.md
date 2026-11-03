# board_v12 当前版本与运行状态

文档整理：2026-11-03。代码基线：2026-10-31 的 [`06da0df`](https://github.com/istiurosmadelgordont-cloud/RoadFusion-Edge/commit/06da0df8d50c750740f39de7f4c898eb1ea183f5)。部署命令见 [README](README.md)，整机工程版本见 [版本表](../docs/VERSIONS.md)。

## 最新变化

- 10月31日同步板端界面、相机布局、控件、启动脚本、回放合成脚本及 AVM 标定工具。保留四路均分、主视角优先、单路放大、三路融合显示和七套主题。
- “车道保持辅助”已有独立开关，默认关闭；只门控前视偏离提醒。它不替代车道检测开关，不影响测试二的实虚线规则，也不控制方向盘。此前文档中的“LDW已关闭”是旧阶段状态。
- `run_pcie.sh` 现已附带 `--demo-navigation`，普通 PCIe 启动默认显示模拟导航；方向与距离仍人工输入。
- 实时图像路径继续固定为 PCIe driver 8.2 → 四槽 DMA-BUF → 同步 RGA → 有界 BGR/RGB 图像池。回放与实时输入分开，当前应用没有 read/CPU 实时输入切换。

## 默认值与开关

| 功能 | `run_pcie.sh` 的默认状态 | 使用方式 |
| --- | --- | --- |
| 模拟导航页 | 开 | 方向、距离按钮；[说明](docs/NAVIGATION_DEMO.md) |
| 车道检测 | 关 | `--enable-lanes`，或界面/`L` |
| 车道保持辅助 | 关 | 顶栏独立开关，需有有效车道检测才可提醒偏离 |
| FCW | 关 | `--enable-fcw`，或独立控件 |
| 危险模拟停车 | 关 | `--enable-test4` / `scripts/run_test4.sh`，或危险监测开关/`B` |
| TTC | 关 | 脚本传入 `--no-ttc`；额外参数可覆盖 |
| 后视 UFLD | 非默认 | `--ufld-rear` 是可选路径，须单独验证 |

裸二进制与启动脚本的默认参数不同：模拟导航由脚本显式打开。脚本末尾追加的同名开关按命令行顺序覆盖，完整参数以 `./build/adas_four_view --help` 及 `src/four_view.cpp` 为准。

## 相机与显示布局

PCIe 默认原始槽顺序为左、前、右、后：左上左视、右上前视、左下右视、右下后视。内部语义索引仍为 front/rear/left/right，检测、HSV、车道和运动补偿共用相机映射。

界面支持四路均分、主视角优先、单路放大，以及把合成帧行半幅重新组合显示的融合布局；`Z` 在当前布局中切换放大/返回。ARM 的“三路融合”按钮只改变显示组织，不会经该按钮切换 FPGA KEY2，也不会计算新的空间融合。源端已是三路融合画布时，选择对应布局；四路独立画面应选择四路布局。具体输入位置见 [双板说明](../fpga/dual_board/README.md)。

FPGA 的三视角融合与 ARM 的 AVM 鸟瞰是两套功能。AVM 可读取特定本地素材，手动时钟与实时 PCIe 输入不构成自动同步；更换素材或相机需要匹配标定。

## 模型与数据通路

当前模型为 YOLO 14类 QAT INT8（960×544）、MO灯态 FP16（128×128）及 UFLDv2原始部署 INT8（1600×320）。YOLO与MO放在同一模型目录，哈希见 [manifest](config/model_manifest.json)。NPU驱动版本 `0.8.2` 与项目PCIe驱动 `8.2` 是两个不同版本号。

源槽在同步 RGA 读取完成后才归还，输出图像由有界池和 `cv::Mat` 引用保护。`--pcie-buffers` 调整应用 BGR 输出池，不增加 FPGA 的四个 DMA 槽。详见 [PCIe/RGA](docs/PCIE_RGA_PREVIEW.md)。

## 完成范围与验证

灯态匹配、实虚线变道提示、FCW、模拟停车和图像调参已在代码中接入。导航方向与距离人工提供；车辆行驶、转向与停车为模拟状态，未接 CAN 或车辆执行器。

10月31日源码同步提交记录了 ARM `adas_four_view` 构建、`perception_regression` 和 `input_mode_regression` 通过。10月29日的14项回归、RGA颜色和30帧PCIe短测保留在 [基线验证](docs/BASELINE_SYNC_20261029.md)，其范围不自动扩大为最新全部功能验收。用户提供的后续整机显示时差、帧率和资源统计见 [性能记录](../docs/PERFORMANCE.md)。

本次只整理文档，没有重建/下载位流或新增板端性能测试。两块FPGA归档的时序违例仍以各板README和原报告为准。
