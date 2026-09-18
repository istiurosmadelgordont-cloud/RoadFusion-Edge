# 以太网接收视频 → DDR → HDMI 工程快照

本目录保存 2026-09-16 的 Pango `board_3_oneboard_design` 工程以及对应的 Windows 上位机。FPGA 从千兆以太网接收 RGB565 视频，按当前 RTL 固定的 960×540 帧格式写入 DDR 并输出显示。用户已在板上看到上位机发送的彩条；本次归档只复制现有文件、核对工程输入并编写说明，没有修改 RTL、重新实现或重新上板验证。

## 文件与打开方式

- [`ethernet_video_fpga_20260916.zip`](ethernet_video_fpga_20260916.zip)：从当前 `.pds` 提取的全部 `source/`、`ipcore/` 输入，加上 `3_ddr_test.pds`、`ddr_test.fdc` 和当时的 `generate_bitstream/test_ddr.sbit`。保留文件的工程相对路径。
- [`SOURCE_MANIFEST.csv`](SOURCE_MANIFEST.csv)：压缩包内每个文件的字节数和 SHA-256；可用于确认快照未变。
- [`build_snapshot.py`](build_snapshot.py)：按 `.pds` 输入列表重新生成源码包和清单的脚本。
- [Windows 上位机](../../tools/udp_video_sender/README.md)：视频、图片、彩条经 UDP 发送到该接收器。

解压到单独目录后，用 Pango PDS 2022.2-SP6.4 打开 `3_ddr_test.pds`。工程配置为 Logos PGL50H、FBG484、速度等级 -6，顶层模块 `test_ddr` 在 `source/ddr_test_top.v`。约束文件为 `ddr_test.fdc`。压缩包仅保存工程输入和现成 bitstream，不包含旧日志、综合/布局数据库或完整实现目录；重新编译时让 PDS 重新生成这些文件。工具版本、IP 许可和实际板卡引脚须与此工程相容。

`.pds` 中列出的 27 个 Verilog 源文件、10 个 IP 配置文件和 101 个 IP 源文件，以及工程、约束和 bitstream，合计 141 个文件已逐一检查存在，并在 ZIP 中重新读取校验 SHA-256。部分 IP 是厂商加密文件；保留快照不改变其许可条件。

## 当前协议和固定参数

| 项目 | 当前值 |
| --- | --- |
| FPGA IP / 目标 UDP 端口 | `192.168.1.10:1234` |
| FPGA MAC | `A0:B1:C2:D3:E4:E4` |
| 输入画面 | 固定 `960×540`、RGB565、大端字节序 |
| 每帧第一个 UDP 负载 | `F0 5A A5 0F 03 C0 02 1C`，后 4 字节为宽高 |
| 后续 UDP 负载 | 连续像素流，每包最多 1200 字节，无序号和确认 |
| 每帧像素数据 | `960×540×2 = 1,036,800` 字节，即 864 个满载 UDP 包 |

上位机可任选输入和输出分辨率及采样/发送帧率，以便以后修改 FPGA 时继续测试；**这版 FPGA 的 `eth_img_rec.v` 只接受 960×540 帧头，并按固定 518400 像素结束一帧**。使用此 bitstream 时，将上位机“输出分辨率（实际发送）”设为 960×540。帧率不写入帧头，只由包发送节奏决定。上位机的“实际平均 fps”是本机发包速率，不是 FPGA 确认的显示帧率。

FPGA 目前没有 UDP 确认、重传、包序号或自动丢包恢复。应使用独立千兆网口，配置本机静态 IPv4（如 `192.168.1.102/24`）及指向 FPGA MAC 的静态邻居；操作步骤见上位机 README。不要仅凭网口灯或 Wireshark 发包计数判断整帧已进入 DDR。

## 帧率与带宽

960×540 RGB565 的纯图像数据为每帧 1,036,800 字节。30 fps 需要约 248.8 Mbps 的**净像素带宽**，另加 Ethernet/IP/UDP 开销和帧边界等待。若上位机限速 120 Mbps，30 fps 无法实现；提高至约 800 Mbps 后，还受视频解码、缩放、Windows 发包、网卡以及 FPGA/DDR 接收能力限制。上位机显示解码/转换与发包耗时，便于区分瓶颈。更高限速不等于 FPGA 必然能完整接收，须用 ILA/显示画面检查错帧和丢包。

## 验证边界与来源

该包中的 bitstream 是源工程现存的 `test_ddr.sbit`，不是本次重新生成的。上位机单元测试通过；压缩包完整性和所有 `.pds` 源文件引用已核对，但本次未运行 PDS 全流程，也未记录 ILA 统计或连续帧的 FPGA 端吞吐率。用户此前反馈彩条已经在板上显示，这属于现场观察，不应代替逐帧完整性验证。

此工程基于 `FPGA-Video-Capture-main` 的 `board_3_oneboard_design`，原项目说明提及“小眼睛半导体”和“正点原子”的部分参考代码；RGMII 接收部分也参照了用户所用的开发板官方以太网 Demo。保留原文件及厂商 IP 声明；使用或再分发这些部分时应遵守各自许可。
