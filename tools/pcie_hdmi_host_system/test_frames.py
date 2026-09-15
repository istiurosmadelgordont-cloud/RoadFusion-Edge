"""frames 模块的单元测试：彩条校验、坏像素定位、字节序、短读、保存回读。

全部在内存或临时目录里运行，不需要 FPGA、不需要 root，
因此在 Windows 上也能验证处理链路本身是否正确。
"""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import frames

class FrameTests(unittest.TestCase):
    def test_colorbars(self):
        """合成彩条应通过校验，且第 5 段的起点应是纯红。"""
        raw = frames.demo_frame()
        self.assertEqual(len(raw), frames.FRAME_BYTES)
        self.assertTrue(frames.check(raw)["passed"])
        # 段下标 5 = 红色 0xF800，转 RGB888 后应为 (255, 0, 0)
        self.assertEqual(frames.rgb(raw)[0, 5*240].tolist(), [255, 0, 0])
    def test_bad_pixel_and_size(self):
        """改坏一个像素要能精确定位；帧长不符必须报错而不是静默截断。"""
        raw = bytearray(frames.demo_frame()); raw[0:2] = b'\0\0'
        result = frames.check(raw)
        self.assertEqual(result["mismatched_pixels"], 1)
        self.assertEqual(result["first"]["x"], 0)
        with self.assertRaises(ValueError): frames.check(raw[:-1])
    def test_endian(self):
        """整体字节序颠倒必须被判为失败，不能误判成通过。"""
        import numpy as np
        raw = np.frombuffer(frames.demo_frame(), dtype='<u2').byteswap().tobytes()
        self.assertFalse(frames.check(raw)["passed"])
    def test_short_device_read_is_not_retried(self):
        """设备短读要直接失败：绝不能把两次 read 的结果拼成一帧。"""
        with patch("frames.os.open", return_value=3), patch("frames.os.close") as close, patch("frames.os.read", return_value=b'123') as read:
            with self.assertRaises(ValueError): frames.acquire("hardware", "/dev/test", "")
            read.assert_called_once()            # 只读一次，不重试
            close.assert_called_once_with(3)     # 失败路径同样必须关闭句柄
    def test_save_reload(self):
        """保存出的三个文件要齐全，且 .rgb565 能被原样读回。"""
        with tempfile.TemporaryDirectory() as folder:
            raw = frames.demo_frame()
            base = frames.save(raw, folder, {"source": "demo"})
            loaded, _ = frames.acquire("file", "", base + ".rgb565")
            self.assertEqual(loaded, raw)
            self.assertTrue(Path(base + ".png").is_file())
            self.assertIn('demo', Path(base + ".json").read_text())

if __name__ == "__main__": unittest.main()
