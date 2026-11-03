# 板端专题与验证索引

当前功能见 [CURRENT](../CURRENT.md)，部署见 [README](../README.md)，架构见 [ARCHITECTURE](../ARCHITECTURE.md)。本页整理于2026-11-03；日期记录保持原始测试范围。

## 当前使用说明

| 主题 | 说明 |
| --- | --- |
| [架构与流程图](../../docs/DIAGRAMS.md) | 整机、双板、ARM、增强、参数与缓冲生命周期 |
| [PCIe/RGA图像路径](PCIE_RGA_PREVIEW.md) | 驱动8.2、同步转换、池寿命、预览及模型前处理 |
| [图像参数与回执](PCIE_V3_IMAGE_CONTROLS.md) | 草稿/应用、整包参数、RECEIVED/APPLIED；文件名沿用历史，正文已更新 |
| [模拟导航](NAVIGATION_DEMO.md) | 脚本默认开启；人工方向/距离，不含视频OCR |
| [测试四危险模拟停车](TEST4.md) | 多目标、新观测确认、断流保持停车与安全解除 |
| [AVM](../AVM.md) | 本地素材、标定与独立手动时钟 |
| [工程约定](../ENGINEERING.md) | 分层、资源生命周期与源码变更验证 |
| [整机性能与版本](../../docs/PERFORMANCE.md) | 用户技术文档转录、测量方法、资源和版本差异 |

## 验证与演进记录

| 日期/阶段 | 记录 | 范围 |
| --- | --- | --- |
| 2026-10-31 | [源码同步提交](https://github.com/istiurosmadelgordont-cloud/RoadFusion-Edge/commit/06da0df8d50c750740f39de7f4c898eb1ea183f5) | 界面/布局/控件/脚本与AVM工具；提交记录包含主程序构建及两项回归 |
| 2026-10-29 | [基线同步](BASELINE_SYNC_20261029.md) | 14项回归、RGA颜色、30帧PCIe短测，未覆盖完整交互及长期性能 |
| 2026-10-29 | [车道审查](LANE_AUDIT_20261029.md)、[行人与车道修复](PEDESTRIAN_LANE_20261029.md)、[提前预警](PEDESTRIAN_EARLIER_20261029.md) | 对应素材与原基线的行为检查 |
| 2026-10-29 | [启动调整](STARTUP_20261029.md) | 当时桌面会话与驱动准备变更；最新启动以scripts/run_pcie.sh为准 |
| 2026-10-27 | [部署](DEPLOYMENT_20261027.md) | 原board_v11构建与环境 |
| 早期RGA | [阶段一probe](PCIE_RGA_STAGE1.md)、[接入说明](PCIE_V4_ADAPTATION.md) | 独立诊断与接入过程，主程序以当前RGA路径为准 |
| 原驱动8.1 | [性能修改](DRIVER_PERFORMANCE.md) | read等待与锁改动，不能当作当前8.2/RGA性能报告 |
| 2026-10-22 | [验证](VALIDATION_20261022.md) | 原board_v9/read版本 |
| 2026-10-20 | [验证](VALIDATION_20261020.md) | 原灯态/车道规则版本 |
| 2026-10-15 | [测试一](SIMULATED_BOARD_TEST_1.md) | 原board_v8文件回放 |
| 2026-10-10 | [源码同步验证](VALIDATION_20261010.md) | 早期环境与构建 |

历史架构、叠加设计及旧应用介绍见 [历史资料索引](../../docs/HISTORY.md)。软件回归、模型准确率、视频性能、位流功能和物理时序是不同证据，不能互相替代。
