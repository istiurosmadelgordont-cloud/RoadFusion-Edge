# 板卡二：DDR后图像处理、PCIe参数控制与两阶段回传

当前工程为 **2026-10-19 / HEADER_KEEP_FIX**，来自桌面 `sfp_ddr_image_pice_handshake`。

- [主工程ZIP](sfp_ddr_image_pice_handshake_20261019_header_keep_fix.zip)：2425732字节，560个文件。
- [PCIe仿真库ZIP](sfp_ddr_image_pice_handshake_20261019_pcie_sim.zip)：4978960字节，18个文件。
- **完整还原：两个ZIP解压到同一父目录，合并同名工程文件夹，共578个文件。** IP文件字节内容未改，按目录分包传输。
- [本次包头修正与ARM使用说明](docs/PCIE_HEADER_KEEP_FIX.md)。
- [代码解说](docs/CODE_WALKTHROUGH.md)、[RC控制指南](docs/RC_PCIE_CONTROL.md)。
- [参数RECEIVED/APPLIED协议](docs/IMAGE_PARAMETER_ACK.md)、[FIC探针](docs/IMAGE_PCIE_FIC.md)。
- [DRM与输入debug说明](docs/DRM_REDUCTION_AND_PCIE_IMAGE_DEBUG.md)、[SHA-256](SHA256SUMS.txt)、[归档检查](SNAPSHOT_CHECKS.json)。

SFP接收原始视频写DDR，恢复时序后经统一图像处理，公共结果送HDMI和原PCIe四缓冲DMA。图像开关、参数、提交和回传整合在手写 `pcie_dma_ctrl`；PCIe/SFP生成IP和原视频DMA协议保持不变。

本次修复3DW包头tkeep=1111被图像路径拒收的问题，解决该条件造成的“视频可接收、0x218未登记、参数无效/回执超时”。数据拍仍要求0001。ARM提供的pcie_v3无需为本修正改变命令；重新下载匹配位流后，停止并重新开始采集以登记邮箱，再点“应用参数”。参数完整五字通过0x200..0x210写入，0x214提交唯一cookie；0x218登记128字节确认区，分别返回RECEIVED和APPLIED。

包含最新 `IMAGE_ACK_RX_APPLIED_TX_20261017` FIC：Core0端口133/161位，Core1端口244/245位，各256点。含接收/生效cookie、参数字掩码、回传地址使能和两种回包发送事件。

**9项离线仿真通过**，包括两种包头格式、参数跨域、两阶段回包、视频DMA并发控制及DDR后完整集成。本次发布未运行PDS位流构建或下载，不附带位流；仍需自行构建、检查时序并上板验证。ARM程序、板卡一及仓库其他目录不在此次工程替换范围内。

两包合并解压后用PDS 2022.2-SP6.4打开 `hdmi_ddr_pice.pds`，顶层 `hdmi_ddr_pcie`，器件PGL50H / FBG484 / -6。215个PDS设计输入均已检查存在。ZIP含RTL、生成IP、约束、FIC、RC参考、测试及验证记录，排除构建缓存、实时采样和位流。

两个ZIP的SHA-256见[校验文件](SHA256SUMS.txt)。旧工程包可从Git历史获取。第三方IP声明保持原样，沿用[工程权利通知](../DISCLAIMER.md)。
