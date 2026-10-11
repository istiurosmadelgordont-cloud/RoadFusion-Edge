# 新版图像处理与PCIe控制

当前打开 `hdmi_ddr_pice.pds`，top=`hdmi_ddr_pcie`。

2026-10-11：DDR保存原图，图像处理移到DDR读出恢复视频流之后，HDMI和PCIe共用处理结果；平滑A固定启用，删除A冻结/原始A旁路；替换旧DVIO图像处理为PCIe整帧参数控制；保留原高光压制、暗光增强的Y曲线，整合到CLAHE后、高光保护前。

- [DDR后接入位置](docs/IMG_AFTER_DDR.md)
- [参数与上位机控制](docs/PCIE_IMAGE_CONTROL.md)
- [本次验证和时序状态](docs/BUILD_IMAGE_PCIE_20261010.md)
- [原4缓冲图像DMA协议](docs/RC_HANDSHAKE_GUIDE.md)

以下是2026-09-19至2026-10-07的历史传输工程说明。当前处理位置、参数入口及验证状态以上面两份新版文档为准。

> 历史记录，已由新版PCIe控制替换。2026-10-07更新：图像处理及DVIO已接到光口接收后、DDR写入前。工程入口不变，连接与调参见 [IMG_BEFORE_DDR.md](docs/IMG_BEFORE_DDR.md)。

# HDMI → DDR → 图像增强 → PCIe：四缓冲握手版

本版在 PGL50H 工程上加入 RC 四缓冲所有权握手，标记 `PCIE_HS_20260919`。已占用的帧缓冲不会被固定轮转覆盖；RC 用完一帧后，写回该帧 token 才允许 FPGA 重用。无空闲槽时跳过整帧。

## 打开工程

使用 PDS 2022.2-SP6.4 打开本目录 `hdmi_ddr_pice.pds`，顶层仍为 `source/hdmi_ddr_pcie.v`。DDR、PCIe、FIFO、PLL IP 已包含在工程内，相对路径继承此前整合版。归档名称中的 20260916 是任务起始日期，完成日期为 2026-09-19。

## 文件入口

- [代码说明与改动记录](docs/CODE_NOTES.md)：H1～H11 改动位置及状态机解释。
- [RC 配合操作指南](docs/RC_HANDSHAKE_GUIDE.md)：寄存器、token、缓存同步、启动、归还、停止。
- [已注释的 PCIe 控制器](source/source/pcie_dma_ctrl.v)：本次唯一功能 RTL 修改。
- [RC 协议参考代码](rc/handshake_reference.h)：通过回调适配 MMIO 与 DMA，不是完整 PCI 驱动。
- [验证记录](BUILD_STATUS.md)：控制器仿真、C 参考测试、PDS 编译/综合与验证边界。
- [逐行 RTL 差异](docs/pcie_dma_ctrl_changes.patch)：与 archive 中原控制器比较。
- `archive/pcie_dma_ctrl.before_handshake.v`：改动前版本；该目录不参加当前工程编译。
- `SOURCE_MANIFEST.csv`：当前源码包文件哈希；archive 下 before_handshake 清单仅供历史对照。

## 上位机必须适配

旧版上位机不能直接使用新协议。默认仍为 1080p RGB565、64 字节图像 Payload，但需要：

1. 登记四个图像 DMA 地址和独立 64 字节状态区。
2. 写 START 后等待完整帧。
3. 校验帧尾的 32 位 token，处理/显示全部结束后清标记并 RELEASE。
4. 结束时发送 STOP(cookie)，收到独立状态区的匹配 ACK 后，再等待显示消费者退出并释放 DMA 内存。

`0x130` 的语义已变成 STOP，`0x170` 归还，`0x180` 状态地址，`0x190` 启动，`0x1A0` 清配置。详细步骤和字节序约定以 RC 指南为准。

## 复跑验证

在 Icarus Verilog 和 GCC 可执行文件已在 PATH 的环境：

```powershell
./tests/run.ps1
./tests/run.ps1 -ImagePackets 64800
./tests/run_rc.ps1
```

也可通过 `-Iverilog`、`-Vvp`、`-Gcc` 指定路径。短帧用例覆盖竞态；64800 用例单独检查完整一帧、帧尾及停止。仿真使用 FIFO 行为替身，不是厂商 FIFO 或板级仿真。

## 历史验证边界（2026-09-19）

已完成 PDS Compile/Synthesize；未重新布局布线、签核时序或上板验证。IP/未约束 I/O 等工程警告仍需处理。DDR 页避让、增强 RGB 饱和高位及增强参数 CDC 是此前已有问题，本次未修复。握手不改变 PCIe 链路带宽，也未加入 RGA/DRM 显示驱动。

厂商 IP 的版权和授权声明保持原样，使用及再分发仍须遵守对应授权。
