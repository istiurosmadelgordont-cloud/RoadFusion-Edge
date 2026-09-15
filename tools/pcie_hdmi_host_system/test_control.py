"""图像增强控制接口的测试（脚本式，不需要 pytest）。

覆盖四件事：
  1. 合法命令的报文编码正确（8 字节小端 <偏移, 值>）
  2. 非法偏移/越界参数必须被拒绝——尤其是 0x130/0x110 这类危险寄存器
  3. 短写要当失败处理
  4. 无论成功失败，句柄都要关闭

全部用 mock 替换系统调用，所以不需要真实设备。
"""
import struct
from unittest.mock import patch
from fpga_control import send_enhancement

# ---- 1) 合法命令：编码必须与协议一致，且正常关闭句柄 ----
for offset,value in ((0x150,32),(0x140,255),(0x160,0)):
    with patch('fpga_control.os.open',return_value=8),patch('fpga_control.os.write',return_value=8) as w,patch('fpga_control.os.close') as c:
        send_enhancement('dev',offset,value)
        assert w.call_args[0][1]==struct.pack('<II',offset,value)
        c.assert_called_once_with(8)

# ---- 2) 非法命令：包含 CLEAR(0x130)、地址(0x110)、参数越界、0x160 非零 ----
for offset,value in ((0x130,0),(0x110,1),(0x150,256),(0x160,1)):
    try: send_enhancement('dev',offset,value)
    except ValueError: pass
    else: raise AssertionError('accepted invalid command')

# ---- 3) 短写：内核没完整接收，必须抛 OSError 且仍然关闭句柄 ----
with patch('fpga_control.os.open',return_value=8),patch('fpga_control.os.write',return_value=4),patch('fpga_control.os.close') as c:
    try: send_enhancement('dev',0x150,32)
    except OSError: pass
    else: raise AssertionError('accepted short write')
    c.assert_called_once_with(8)

print('Packet encoding, validation, short-write and descriptor-close tests passed')
