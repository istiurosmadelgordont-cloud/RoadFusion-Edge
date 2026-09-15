"""通过 PCIe 字符设备下发 FPGA 图像增强参数（受校验的写接口）。

只要三个增强寄存器，绝不放行 0x110（DMA 地址）和 0x130（CLEAR）：
那两个寄存器一旦被用户态误写，会破坏正在运行的地址环，
甚至让 FPGA 往错误的内存地址写数据。

协议固定一次写 8 字节：两个小端 u32 —— <寄存器偏移, 参数值>。
内核驱动会再校验一遍，这里是第一道防线。
"""
import os
import struct


def send_enhancement(device, offset, value):
    """把 <偏移, 值> 写进设备节点，由内核模块落到 BAR1。

    参数在用户态先校验一次（内核里还会再校验一次，双重防线）。
    """
    # 白名单：只有这三个是增强寄存器；参数必须是 8 位无符号
    if offset not in (0x140, 0x150, 0x160) or not 0 <= value <= 255:
        raise ValueError("Invalid enhancement command")

    # 0x160 的语义是"清除两项增强"，只接受 0
    if offset == 0x160 and value != 0:
        raise ValueError("Enhancement disable value must be zero")

    # "<II" = 两个小端 uint32，正好 8 字节；驱动会校验长度必须为 8
    payload = struct.pack("<II", offset, value)

    fd = os.open(device, os.O_WRONLY)
    try:
        # 短写意味着内核没有完整接收这条命令，必须按失败处理
        if os.write(fd, payload) != len(payload):
            raise OSError("Short control write")
    finally:
        os.close(fd)
