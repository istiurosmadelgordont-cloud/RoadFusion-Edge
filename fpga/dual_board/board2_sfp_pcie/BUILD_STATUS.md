# 四缓冲握手版验证记录

日期：2026-09-19。功能源码：`source/source/pcie_dma_ctrl.v`，`PCIE_HS_20260919`。

## 已完成的验证

- Icarus Verilog 10.1：短帧控制器回归通过，42 个完成帧、139 个 TLP（包括被取消帧留下的完整 TLP 和 STOP 通知）。
- 32 帧固定种子随机背压，逐个图像 Payload DW 核对内容与顺序。
- 空闲分配、四块占满跳帧、乱序归还、错误/旧/重复 token、WRITING 槽拒绝归还、FIFO empty 等待、full 取消。
- 定向检查 VS 与 Header 握手同拍、VS 与 TLAST 同拍、STOP 与 TLAST 同拍；测试内断言确认这些同拍条件确实发生。
- STOP 在 Header 背压和帧尾途中到达时，已呈现 TLP 仍排完，随后回写正确 cookie。
- 序号耗尽时拒绝分配，不自然回绕。
- 完整 1080p 配置：64,800 个图像包 + 1 个帧尾 + 1 个 STOP ACK，合计 64,802 个 TLP；检查像素 Payload 顺序、尾地址、token、归还。
- RC 参考 C 代码使用 GCC 的 `-std=c99 -Wall -Wextra -Werror` 编译，模拟回调测试通过；覆盖地址溢出/重叠、半写入标记、重复完成、清零/屏障/MMIO 顺序、持有者未退出时拒绝重启。
- PDS 2022.2-SP6.4 完整工程 Compile 与 Synthesize 通过。验证在单独工程副本运行，交付包不含旧 bitstream/构建数据库。
- 当前桌面原工程 source 与起始快照逐文件比对一致；本次功能差异只在 PCIe 控制器。

## 结果摘要

```text
ALL TESTS PASSED image_packets=2 completions=42 packets=139
FULL FRAME TEST PASSED image_packets=64800 completions=1 packets=64802
RC REFERENCE TESTS PASSED: ranges, partial/stale marker, ownership, barriers, STOP/restart
Process "Compile" done.
Process "Synthesize" done.
```

## 工具警告和验证边界

Icarus 10.1 对原有带 synthesis 注释的标量 ANSI 端口输出 inherits dimensions 提示，以及数组 @* 敏感列表提示。仿真没有连接位宽错误；PDS 能完成分析和综合。

PDS 仍有厂商 IP 的常量/零次重复展开提示、constant probe loop 提示和未约束 I/O 时序警告。本记录不把综合成功等同于时序签核。

未做布局布线、真实 FIFO 时序/CDC 仿真、ARM 内核驱动编译、真实 PCIe DMA 字节序及一致性测试、DRM/RGA/NPU 联调、板级吞吐和长期压力测试。RC 参考代码的回调测试不能替代 Linux DMA API 与显示完成事件的板级验证。

已有的 DDR 页避让和增强算法/参数 CDC 问题继续保留，详见代码说明。本次只保证所测试控制器场景的缓冲所有权与 TLP 发送行为。
