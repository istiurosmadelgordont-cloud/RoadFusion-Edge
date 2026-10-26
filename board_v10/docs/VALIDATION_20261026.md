# board_v10 同步验证：2026-10-26

本页记录当天首次同步的版本。后续测试四多人危险框、自动解除与区域调整已部署，最新二进制和验证以 [测试四部署验证](TEST4_VALIDATION_20261026.md) 及 [CURRENT](../CURRENT.md) 为准；本页旧哈希保留用于追溯。

源码来自实际板端 `/home/cat/rk3568_adas/board_v10`。导出后再次核对所有快照文件哈希，采集期间源码未变化；应用二进制与 2026-10-25 已部署版本一致。

当前二进制 SHA256：`143109522d401a0a5e8e2ebf56a502b8244e95ab77a6f44d2cf2455b1dac6f2c`。

## 上传前检查

待上传工作目录从 GitHub `main` 的 `d6219d9d9ffb92e686565a18047c02025833bd1b` 创建。新增 `board_v10`，补充忽略规则及版本入口；保留该提交中的板卡一工程/位流更新。另一个本地工作目录的未提交导航和以太网上位机修改没有被改写。

直接导出 151 个有效源码/配置/文档文件，排除生成的 `driver/my_pciedriver.mod.c`；模型、二进制、日志、视频、下载缓存和凭据不提交。发布时刷新 README、CURRENT 和模型清单中的版本日期，并新增本页及源文件哈希记录；应用、头文件、测试、启动脚本和有效驱动源文件与板端逐字节匹配。

板端复跑通过：

- `lane_paint_regression`
- `fcw_regression`
- `overlay_regression`
- `input_mode_regression`
- `perception_regression`

从本次待上传源码在 x86 Linux/OpenCV/FreeType 环境独立编译并运行通过：

- `lane_paint_regression`
- `fcw_regression`
- `overlay_regression`
- `perception_regression`
- `test4_safety_regression`
- `box_motion_regression`
- `track_ghost_regression`
- `navigation_demo_regression`
- `image_control_regression`
- `pcie_capture_regression`

CPU 测试不代表 RKNN 或 FPGA 硬件验收。PCIe 采集回归使用 read/ioctl 模拟，图像控制回归不向 FPGA 发送参数。上传期间未替换板端模型、未重启应用、未更换驱动或位流。

## 已部署功能与限制

2026-10-25 的 FCW/顶部等级/按钮与车道线修复保持原样。修正线型取样窗口后，历史同图探针左侧实线证据从 0 变为 1.0；部署后的 26 个周期报告为 23 次左实线、3 次未知，没有左虚线。该统计不是有人工标签的模型准确率。

保持测试一/二控件、LDW 关闭、独立 FCW、淡色车道风险覆盖和小号距离。测试四模块与 RCW 基础代码仍在，但默认关闭；最新行人横穿/后车素材的 PC FP32 演示不能替代板端 INT8 验证。导航为人工模拟输入。模型、图片/视频、数据集、构建产物和本地素材筛选过程不属于这次源码发布。
