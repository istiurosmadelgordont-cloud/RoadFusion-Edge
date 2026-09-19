# PGL50H HDMI/DDR/PCIe 四缓冲握手工程

该目录发布 `hdmi_ddr_pice_handshake_20260919` 完整工程及配套文档。工程以
PGL50H、PDS 2022.2-SP6.4 为目标，将 1920×1080 RGB565 图像经 DDR、图像增强和
PCIe MWr 发送到 ARM RC，并增加四缓冲所有权握手，防止 FPGA 覆盖 RC 正在显示或
推理的页面。

本次上传使用刚完成排版的 `pcie_dma_ctrl.v`；排版调整没有改变四缓冲状态机、命令
寄存器、TLP 数据路径或时序逻辑。

## 直接阅读

- [PCIe 控制器 RTL](rtl/pcie_dma_ctrl.v)
- [代码说明与 H1～H11 改动记录](docs/CODE_NOTES.md)
- [ARM RC 驱动改动与操作指南](docs/RC_HANDSHAKE_GUIDE.md)
- [控制器逐行差异](docs/pcie_dma_ctrl_changes.patch)
- [验证范围与结果](docs/BUILD_STATUS.md)
- [RC 协议参考头文件](rc/handshake_reference.h)
- [完整工程文件哈希清单](SOURCE_MANIFEST.csv)

## 下载和还原完整 PDS 工程

完整 ZIP 因连接器上传限制拆成 28 个分卷，Git 克隆后执行：

Windows PowerShell：

```powershell
powershell -ExecutionPolicy Bypass -File tools/reassemble_project.ps1
```

Linux：

```sh
sh tools/reassemble_project.sh
```

生成文件：

```text
hdmi_ddr_pice_handshake_20260919.zip
SHA-256: 1766B7285FCAFEC8C9E39230B77E2E83CE771B5F0E5C773DFB33D1F14680C576
```

解压后用 PDS 2022.2-SP6.4 打开 `hdmi_ddr_pice.pds`。压缩包根目录中包含443个
文件（442个清单文件加 `SOURCE_MANIFEST.csv`），不含 PDS 临时锁、日志和构建输出。

## RC 必须完成的适配

1. 使用该 PCIe 设备的 DMA API 分配四块图像缓冲，每块至少 `0x3F4840` 字节；另分配
   一个独立、64字节对齐的状态区。
2. 依次向 BAR1 `0x110` 登记四个32位 DMA 地址，再用 `0x180` 登记状态区地址，最后
   写 `0x190=1` 启动。
3. 每块缓冲的 `0x3F4800` 偏移是64字节完成标记。只有16个 LE32 DW 全部相同、
   sequence非零且slot匹配，才能发布该帧。
4. `token=(allocation_sequence<<2)|slot`。RC可丢弃尚未处理的旧READY帧，只显示
   sequence最大的完整帧；正在被CPU、RGA、NPU或DRM使用的帧不能提前归还。
5. 最后一个消费者完成后，清零尾标记并执行必要DMA屏障，然后向 `0x170` 写回原token。
6. 停止时向 `0x130` 写非零cookie，并等待独立状态区16个DW全部等于该cookie；确认
   消费者退出后才能释放DMA内存。

完整的字节序、一致性、STOP/重启、异常恢复和上板验收步骤以
[RC操作指南](docs/RC_HANDSHAKE_GUIDE.md)为准。

## 已验证边界

控制器短帧/完整1080p仿真、随机背压、STOP竞态、RC参考代码测试以及PDS Compile/
Synthesize均已通过。尚未完成布局布线、时序签核、真实ARM内核驱动编译和板级长期
压力测试；综合通过不能替代真实DMA字节序、缓存一致性及DRM/RGA/NPU fence验证。

