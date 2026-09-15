# 归档与验证状态：2026-09-15

## 版本范围

本包保存 `hdmi_ddr_pice` 的当前开发版本。没有修复 RTL；没有添加 RC 缓冲所有权、MSI、新 DMA 描述符或更大 Payload 功能。

## 本次整理动作

- 复制当前 source/IP 文件，排除构建目录、日志与锁文件。
- 将原外部引用的 `ddr_test` IP 纳入工程内 `ipcore/ddr_test`。
- 整理版 PDS 只重定位 DDR IP 的路径；原配置和历史 Tcl 原样保存于 archive。
- 补充 README，区分已实现功能、未实现优化和已知风险。
- 对归档源文件进行 SHA-256 来源核对，并检查 PDS 引用的 HDL/IDF/FDC 是否存在。

核对结果：425 个保留文件与原始来源的 SHA-256 一致；整理版 PDS 的 145 个独立 HDL/IDF/FDC 引用全部存在。DDR 仿真生成的 work 数据库、transcript 和 vsim.wlf 未纳入源码包。

## 现场已有构建结果

原工程 `pds.log` 记录 Compile、Synthesize、Device Map、Place & Route、Report Timing、Generate Bitstream 完成。原 bitstream 文件时间为 2026-09-15 20:06:37，大小 2,101,696 字节。

这些是现场原工程的既有记录，不是归档迁移后的重新构建证明。源码包不包含旧 bitstream/构建数据库，以避免把旧实现结果当作移植验证。

## 本次检查边界

已做：RTL 静态检查、增强运算数值核算、当前约束文本检查、文件来源核对和工程依赖整理。

未做：迁移工程全流程重新构建、HDL 动态仿真、板级视频/DDR/PCIe 实测、ARM 驱动性能测试或长时间帧一致性测试。

仍需修正：RGB 饱和高位丢失、增强参数 CDC、DDR 页避让、RC 缓冲覆盖风险。详情见 README。

## 上传位置

目标为 RoadFusion-Edge 仓库的 `fpga/hdmi_ddr_pice/` 独立目录，不覆盖已有 `fpga/pcie_video/` 彩条基线。实际上传状态以仓库提交记录为准，本地生成此文档不表示已经上传。
