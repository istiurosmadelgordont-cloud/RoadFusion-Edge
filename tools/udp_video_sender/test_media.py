import queue
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch
import cv2
import numpy as np
from app import App, fit_frame
from media import parse_size, source_index, VideoFrames
from protocol import Sender, HEADER, FRAME_BYTES
from test_sender import FakeSocket


class MediaTests(unittest.TestCase):
    def test_rate_conversion(self):
        self.assertEqual([source_index(i,30,30,10) for i in range(4)], [0,3,6,9])
        self.assertEqual([source_index(i,30,10,30) for i in range(6)], [0,0,0,3,3,3])
        self.assertEqual(parse_size('1280×720'), (1280,720))
        self.assertEqual(parse_size('321x181'), (321,181))
        with self.assertRaises(ValueError): parse_size('0x540')

    def test_crop_and_stretch(self):
        frame = np.full((100,200,3), 255, np.uint8)
        for mode in ('居中裁剪','拉伸填满'):
            out = fit_frame(frame, cv2, np, (320,180), mode)
            self.assertEqual(out.shape,(180,320,3))
            self.assertTrue(np.all(out==255))

    def test_image_send_once_custom_output(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'中文图片.png'
            ok, data = cv2.imencode('.png', np.full((100,100,3),(0,0,255),np.uint8))
            self.assertTrue(ok)
            data.tofile(path)
            app = App.__new__(App)
            app.stop = threading.Event()
            app.events, app.preview = queue.Queue(), queue.Queue(maxsize=1)
            sock = FakeSocket()
            sender = Sender('127.0.0.1', sock=sock, wait=lambda _: None, size=(321,181))
            with patch('app.check_network',return_value='test'), patch('app.Sender',return_value=sender):
                app.run(('192.168.1.102',30,120,'图片文件',str(path),False,
                         (640,360),15,(321,181),'拉伸填满'))
            self.assertFalse(any(k in ('error','setup_error') for k,_ in app.events.queue))
            self.assertEqual(len(sock.sent),98)
            self.assertEqual(sock.sent[0][0],bytes.fromhex('f05aa50f014100b5'))
            payload = b''.join(p for p,_ in sock.sent[1:])
            self.assertEqual(len(payload),321*181*2)
            self.assertEqual(len(sock.sent[-1][0]),1002)
            pixels = np.frombuffer(payload,dtype='>u2').reshape(181,321)
            self.assertTrue(np.all(pixels==0xf800))
            self.assertEqual(np.count_nonzero(pixels),321*181)

    def test_video_resampling_and_loop(self):
        with tempfile.TemporaryDirectory() as folder:
            path = str(Path(folder)/'tiny.avi')
            writer = cv2.VideoWriter(path,cv2.VideoWriter_fourcc(*'MJPG'),10,(64,48))
            self.assertTrue(writer.isOpened())
            for v in (20,80,140): writer.write(np.full((48,64,3),v,np.uint8))
            writer.release()
            stop = threading.Event()
            reader = VideoFrames(path,cv2,10,20,False)
            try:
                frames=[reader.read(stop) for _ in range(7)]
                self.assertEqual([round(float(f.mean())) for f in frames[:6]],[20,20,80,80,140,140])
                self.assertIsNone(frames[6])
            finally: reader.close()
            reader=VideoFrames(path,cv2,10,10,True)
            try:
                self.assertEqual([round(float(reader.read(stop).mean())) for _ in range(5)],[20,80,140,20,80])
            finally: reader.close()

    def test_next_video_frame_prepares_during_send(self):
        app = App.__new__(App)
        app.stop = threading.Event()
        app.events, app.preview = queue.Queue(), queue.Queue(maxsize=1)
        second_read = threading.Event()
        frames_sent = []
        class Reader:
            def __init__(self, *_):
                self.count = 0
            def read(self, stop):
                self.count += 1
                if self.count == 2:
                    second_read.set()
                return np.full((48,64,3), self.count * 20, np.uint8)
            def close(self):
                pass
        class CheckingSender:
            def __init__(self, *_ , **__):
                pass
            def send_frame(self, payload):
                if not frames_sent:
                    self_test.assertTrue(second_read.wait(1), '下一帧未在发送本帧时开始准备')
                frames_sent.append(payload)
                if len(frames_sent) == 2:
                    app.stop.set()
            def close(self):
                pass
        self_test = self
        with patch('app.check_network',return_value='test'), \
             patch('app.VideoFrames',Reader), patch('app.Sender',CheckingSender):
            app.run(('192.168.1.102',30,800,'视频文件','test.avi',True,
                     None,30,(64,48),'拉伸填满'))
        self.assertEqual(len(frames_sent),2)
        self.assertNotEqual(frames_sent[0],frames_sent[1])
        self.assertFalse(any(k in ('error','setup_error') for k,_ in app.events.queue))
