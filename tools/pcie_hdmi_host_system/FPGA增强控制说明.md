# FPGA 图像增强控制（替代软件预览增强）

当前 GUI 使用 FPGA 命令，不再调用 lowlight.py。

BAR1+0x150：暗光增强；BAR1+0x140：高光压制；参数为 0～255 的整数，由 writel(value) 发送 1DW。按当前 RTL 的字节转换约定，其 AXIS payload[31:24] 得到该参数。
BAR1+0x160，值 0：关闭两项增强，不是 DMA CLEAR（0x130）。设置数值 0 仍将对应增强开关置 1；关闭必须点击关闭两项增强。目前不支持单独关闭其中一项。

用户空间向字符设备写入两个 little-endian u32（偏移、值），驱动验证白名单后发送一次 BAR 写。这 8 字节是驱动 API，PCIe payload 只有参数 1DW。驱动仍只首次发送四个 DMA 地址，不发送 DMA CLEAR。

命令与 read 使用同一互斥锁，正在等待图像时命令可能延后约一个读超时或更久（受调度影响）。控制在后台线程执行。命令成功只表示主机提交写入，没有 FPGA 生效确认。

当前工作区 hdmi_test.v 的增强输出仅连接到 pcie_dma_ctrl 输出，未见接入像素处理模块；请 FPGA 同事确认实际烧录版本是否接好了增强模块及 DMA 数据路径。控制寄存器存在不等于图像效果已实现。

部署：复制 host.py、fpga_control.py、driver/pcie_hdmi_host.c 到 ARM，保留其他文件。驱动本次发生变化；已经启动连续传输时不能强制卸载。双方断电，先启动并配置 FPGA 再启动 ARM，然后 sudo make -C driver clean，再 ./run.sh。选择 hardware 后发送命令。

Python 语法、命令编码/白名单/短写/关闭资源检查通过。内核编译、TLP 字节序和硬件增强效果尚待 ARM/FPGA 联调验证。
