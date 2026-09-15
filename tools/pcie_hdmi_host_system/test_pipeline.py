"""FramePipeline / FrameReader 的单元测试：有界丢弃、异常传递、停止、句柄管理。

用假 Reader 替代真实设备，所以不需要 FPGA 或内核模块。
"""
import threading
import unittest
from unittest.mock import patch
import capture_pipeline as cp
from frames import FrameReader, FRAME_BYTES

class Reader:
    """假读取器：产生 10 帧后抛 OSError，用来模拟采集结束或设备出错。"""
    def __init__(self,*a): self.n=0
    def __enter__(self): return self
    def __exit__(self,*a): pass
    def read(self):
        self.n+=1
        if self.n>10: raise OSError('capture ended')
        return bytes([self.n]),0.001

class Tests(unittest.TestCase):
    def test_bounded_overload_and_error(self):
        """队列满了要丢最旧的帧（保留最新 4 帧），异常要由 take() 抛出。"""
        with patch.object(cp,'FrameReader',Reader):
            with cp.FramePipeline('hardware','x',None,threading.Event(),True) as p:
                p.thread.join(2)
                self.assertFalse(p.thread.is_alive())
                self.assertEqual(p.received,10)
                self.assertEqual(p.dropped,6)                                   # 10 帧中丢了 6 帧
                self.assertEqual([p.take()[3] for _ in range(4)],[7,8,9,10])    # 留下的是最新 4 帧
                with self.assertRaises(OSError): p.take()                       # 队列空后把异常抛回
    def test_single(self):
        """单帧模式：取到一帧即结束，且不应自行置位 stop。"""
        stop=threading.Event()
        with patch.object(cp,'FrameReader',Reader):
            with cp.FramePipeline('hardware','x',None,stop,False) as p:
                self.assertEqual(p.take()[3],1)
                self.assertIsNone(p.take())
        self.assertFalse(stop.is_set())
    def test_stop(self):
        """已请求停止时 take() 立刻返回 None，且线程能正常退出。"""
        stop=threading.Event(); stop.set()
        with patch.object(cp,'FrameReader',Reader):
            with cp.FramePipeline('hardware','x',None,stop,True) as p:
                self.assertIsNone(p.take())
        self.assertFalse(p.thread.is_alive())
    def test_descriptor_reused_and_closed(self):
        """同一个会话内复用设备句柄（只 open 一次），退出时关闭。"""
        with patch('frames.os.open',return_value=7) as op, patch('frames.os.close') as close, patch('frames.os.read',return_value=bytes(FRAME_BYTES)) as read:
            with FrameReader('hardware','x',None) as reader:
                reader.read(); reader.read()
            op.assert_called_once(); close.assert_called_once_with(7)
            self.assertEqual(read.call_count,2)
    def test_short_frame_closes(self):
        """读到短帧抛错时，句柄仍必须被关闭（不能泄漏）。"""
        with patch('frames.os.open',return_value=7), patch('frames.os.close') as close, patch('frames.os.read',return_value=b''):
            with self.assertRaises(ValueError):
                with FrameReader('hardware','x',None) as reader: reader.read()
            close.assert_called_once_with(7)

if __name__=='__main__': unittest.main()
