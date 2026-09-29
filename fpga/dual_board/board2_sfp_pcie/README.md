# 板卡 2：HDMI 光口发送与四宫格 PCIe 输出

使用 PDS 2022.2-SP6.4 打开 [hdmi_ddr_pice.pds](hdmi_ddr_pice.pds)，顶层 `source/hdmi_ddr_pcie.v` 中的 `hdmi_ddr_pcie`。

本板 HDMI 输入经 SFP 发送给板卡 1，作为其一个象限。反向接收板卡 1 的 1920×1080 RGB565 四宫格，经 DDR、图像增强和 PCIe 四缓冲握手传给鲁班猫 2。完整数据流与接线见[双板系统说明](../README.md)。

SFP 与 PCIe 共享修改后的 PCIe IP 内 HSST 资源；请使用此目录提供的 `ipcore/pcie_test`，不要用旧版同名 IP 覆盖。顶层 `sfp_*` 端口是当前连接的一部分。

- [RC 握手指南](docs/RC_HANDSHAKE_GUIDE.md)：地址配置、START、token、RELEASE、STOP。
- [参考代码](rc/handshake_reference.h)：驱动平台回调适配。
- [原始 README](README.original.md)、[原验证记录](BUILD_STATUS.md)：2026-09-19 四缓冲握手工作的历史资料。
- `tests/`：已有握手控制器仿真和 RC 参考测试，运行方法见原 README。

本次复制保持功能源码不变，仅补充系统说明。历史测试结果不能代表 2026-09-27 双板光口和 PCIe 已完成板级验证。工程设计输入路径已核对，生成位流及构建数据库需重新产生。文件校验见 `SOURCE_MANIFEST.csv`。
