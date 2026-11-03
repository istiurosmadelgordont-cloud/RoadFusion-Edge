# 历史资料索引

当前部署请使用 [board_v12](../board_v12/README.md) 和 [双FPGA工程](../fpga/dual_board/README.md)。以下资料保留原日期、指标及阶段问题；正文内“当前”指原记录当时。

## 版本沿革

- [2026-10-26项目说明快照](PROJECT_HISTORY_20261026.md)：旧V7至V10、21类模型及早期整合过程。相关旧应用目录通过固定提交链接读取。
- [早期双板整合说明](dual_board_system.md)：保留板间接口建立过程，现行相机顺序与驱动以双板README为准。
- [测试二设计计划](test2_adas_plan.md)：原board_v8时期的方案，后续实虚线与控件实现见board_v12。
- [原board_v10基线](../board_v12/docs/BOARD_V10_BASELINE.md)与[旧单模型说明](../board_v12/docs/LEGACY_README.md)。

## 软件设计与测试

- [Overlay V2](../board_v12/OVERLAY_V2.md)、[Overlay V3](../board_v12/OVERLAY_V3.md)、[感知优化](../board_v12/PERCEPTION_OPTIMIZATION.md)、[UI参考](../board_v12/UI_REFERENCE_NOTES.md)：阶段设计与参考范围。
- [板端验证索引](../board_v12/docs/README.md)：按日期查看构建、规则回归、RGA与实机短测。
- [V7模型](../models/V7_MODEL.md)与[训练资料](../training/README.md)：旧21类训练和UFLD候选，不代表当前模型已经替换。

## FPGA与工具

- [FPGA工程导航](../fpga/README.md)区分当前双板、单路以太网、HDMI/DDR、早期所有权握手及PCIe彩条基线。
- [工具导航](../tools/README.md)区分当前pcie_v4、7.0单向参数查看器、早期Python查看器与厂商参考例程。
- 板卡二协议专题保留历史设计记录，但包下载已统一为10月30日完整ZIP；不再合并已移除的10月19日两份ZIP。

旧分支`video-download/silk-road-20261017`用于素材下载历史，不是当前整机发布分支。当前功能代码基线与位流清单见 [VERSIONS.md](VERSIONS.md)。
