> 当前驱动已改为持续接收：请先阅读 [持续视频说明](持续视频说明.md)。下文逐次 CLEAR/停止 DMA 的快照流程是旧版描述。

# PCIe HDMI Linux 上位机

针对相邻 pcie_hdmi 工程的 PCIE_FIX_20260912 协议。支持 RK3568 Debian 4.19 的诊断快照采集、图像预览、连续快照、标准彩条逐像素校验、PNG/原始帧/JSON 保存、设备与内核日志、离线文件和演示模式。界面使用 Tkinter，视频处理使用 NumPy/Pillow。

## 快速启动

将整个 pcie_hdmi_host_system 文件夹复制到 Linux，例如 /home/cat/PCIE/pcie_hdmi_host_system。

```sh
sudo apt-get update
sudo apt-get install -y build-essential python3-tk python3-numpy python3-pil
cd /home/cat/PCIE/pcie_hdmi_host_system
chmod +x run.sh load_driver.sh
./run.sh --demo
```

演示模式不需要 FPGA 或 root，明确显示来源 demo，不代表 PCIe 测试通过。真实采集要求与运行内核一致的 headers，且 recordmcount/modpost 等为 ARM64 本机可执行版本。保留此前修复好的厂商 4.19.232 headers，不能任意替换成其他内核版本。

FPGA 先配置新版 HDMI 位流，再启动 ARM，确认 lspci 可见 0755:0755。若旧 pango_pci_driver/pango_video_test 已加载，关闭旧程序并重启后直接运行新项目；脚本不会强制抢占旧驱动。

```sh
./run.sh
```

run.sh 自动 sudo 并保留当前用户的 DISPLAY/XAUTHORITY，适合 MobaXterm 的 cat 登录会话。普通 Tk 界面仍需要有效 X11 会话。root 图形程序仅作为开发调试入口。

无显示器/无 X11 时：

```sh
./run.sh --headless --count 3
```

默认输出 captures/*.rgb565、*.png、*.json。非彩条图像使用 --no-check（GUI 中取消“校验标准彩条”）。保存文件包含数据来源，禁止把 demo/file 的 PASS 当作设备采集成功。GUI 的“连续快照”循环执行独立采集，不是60fps流式视频，不计算或声称实时视频帧率。

## 使用界面

- hardware：读取 /dev/pcie_hdmi_host，点击“采集一帧”或“连续快照”。
- demo：合成标准彩条用于检查界面和处理流程。
- file：选择恰好 4,147,200 字节的小端 RGB565 原始帧，再采集。
- 停止：停止下次快照，等待当前驱动 read 和诊断结束；不强杀 DMA 线程。关闭窗口同样等待。
- 保存：保存当前完整帧及其来源、时间、校验结果。
- 设备/内核诊断：读取 lspci -nnk 与最近100行 dmesg，权限错误会显示在日志中。

## FPGA 协议与范围

单路1920×1080 RGB565。每块缓冲区图像区 0x3F4800 字节，随后64字节同值非零标志。四块缓冲区是同一路的轮转帧缓存，不是四路图像。BAR1资源索引1，0x130 CLEAR，0x110 连续写四个32位 DMA 地址。驱动分配页对齐 coherent 内存，强制32位DMA mask，不使用 /dev/mem 或虚构物理地址。地址下发后自动等待帧边界。

每次 read 等待最多5秒的完整标志，然后 CLEAR、等待、关闭 Bus Master、再等待，复制一帧。全部缓冲区在 probe 分配、remove 释放。超时打印非零字节数及帧尾，errno 110 表示未收到完整帧，不是SSH连接失败。原始RTL没有停止ACK、硬件缓冲区归还、BAR回读完成或中断；用PCI配置空间读刷新命令，不做BAR读。

协议缺少停止确认，固定等待不能保证异常链路下完全排空；此项目是上板诊断上位机，不能用于生产视频采集、热拔插或转向/制动闭环。连续覆盖可能使静态彩条检测不到跨帧问题。没有接入AI模型，也没有宣称解决原先板端DMA全零问题。

## 更新驱动

退出本项目所有采集程序后：

```sh
sudo rmmod pcie_hdmi_host
sudo make -C driver clean
./run.sh
```

模块已加载时脚本复用旧模块，不自动卸载。修改源码后必须按以上步骤重新加载。

## 测试

```sh
python3 -m unittest -v test_frames.py
python3 host.py --source demo --headless --count 1 --output demo_output
```

测试涵盖彩条/坏像素/字节序/截断读取/保存回读。Windows 可验证离线处理；真实内核模块和 PCIe 必须在目标 Debian 上构建验证。
