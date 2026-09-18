import struct
import unittest
from unittest.mock import patch

import numpy as np
import cv2

from app import App, fit_frame, color_bars, check_network
from protocol import FRAME_BYTES, HEADER, Sender, packets, rgb565


class FakeSocket:
    def __init__(self, fail_at=None):
        self.sent = []
        self.fail_at = fail_at

    def sendto(self, payload, address):
        if len(self.sent) == self.fail_at:
            raise OSError('injected network failure')
        self.sent.append((payload, address))
        return len(payload)

    def close(self):
        pass


class ProtocolTests(unittest.TestCase):
    def test_primary_colors_big_endian(self):
        frame = np.zeros((540, 960, 3), dtype=np.uint8)
        frame[0, :5] = [(0,0,255), (0,255,0), (255,0,0), (255,255,255), (0,0,0)]
        self.assertEqual(rgb565(frame)[:10], bytes.fromhex('f800 07e0 001f ffff 0000'))

    def test_exact_frame_and_packet_boundaries(self):
        data = bytes(range(256)) * 4050
        self.assertEqual(len(data), FRAME_BYTES)
        datagrams = list(packets(data))
        self.assertEqual(datagrams[0], bytes.fromhex('f05aa50f03c0021c'))
        self.assertEqual(len(datagrams), 865)
        self.assertEqual(set(map(len, datagrams[1:])), {1200})
        self.assertEqual(b''.join(datagrams[1:]), data)
        self.assertTrue(all(len(p) % 4 == 0 and len(p) <= 1472 for p in datagrams))

    def test_receiver_word_order_and_two_frames(self):
        # Behavioral protocol model of eth_img_rec: magic, dimensions, then
        # exactly 518400 high-half/low-half pixels. Not an HDL simulation.
        data = rgb565(color_bars(np))
        state, remaining, frames, decoded = 'magic', 0, 0, bytearray()
        for _ in range(2):
            for p in packets(data):
                for (word,) in struct.iter_unpack('>I', p):
                    if state == 'magic':
                        self.assertEqual(word, 0xf05aa50f)
                        state = 'dimensions'
                    elif state == 'dimensions':
                        self.assertEqual(word, (960 << 16) | 540)
                        remaining = 518400
                        state = 'pixels'
                    else:
                        decoded.extend(struct.pack('>HH', word >> 16, word & 65535))
                        remaining -= 2
                        if remaining == 0:
                            frames += 1
                            state = 'magic'
        self.assertEqual(frames, 2)
        self.assertEqual(decoded, data * 2)
        self.assertEqual(state, 'magic')

    def test_short_frame_rejected_before_header(self):
        sock = FakeSocket()
        sender = Sender('127.0.0.1', sock=sock)
        with self.assertRaises(ValueError): sender.send_frame(b'1234')
        self.assertEqual(sock.sent, [])

    def test_pacing_never_catches_up(self):
        sock = FakeSocket()
        now, waits = [0.0], []
        def wait(deadline):
            self.assertGreater(deadline, now[0])
            waits.append(deadline - now[0])
            now[0] = deadline + .010  # simulate a 10 ms scheduling delay
        sender = Sender('127.0.0.1', sock=sock, clock=lambda: now[0], wait=wait)
        sender.send_frame(bytes(FRAME_BYTES))
        self.assertEqual(len(sock.sent), 865)
        self.assertEqual(len(waits), 866)
        self.assertAlmostEqual(waits[0], .002)
        self.assertAlmostEqual(waits[-1], .002)
        self.assertTrue(all(v >= .0000843 for v in waits[1:-1]))

    def test_network_error_does_not_retry(self):
        sock = FakeSocket(fail_at=5)
        sender = Sender('127.0.0.1', sock=sock, wait=lambda _: None)
        with self.assertRaises(OSError): sender.send_frame(bytes(FRAME_BYTES))
        self.assertEqual(len(sock.sent), 5)

    def test_aspect_ratio_black_bars(self):
        frame = np.full((100, 100, 3), 255, dtype=np.uint8)
        out = fit_frame(frame, cv2, np)
        self.assertEqual(out.shape, (540,960,3))
        self.assertTrue(np.all(out[:, :210] == 0))
        self.assertTrue(np.all(out[:, 210:750] == 255))
        self.assertTrue(np.all(out[:, 750:] == 0))

    def test_video_decode_and_convert(self):
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as folder:
            path = str(Path(folder) / 'test.avi')
            writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'MJPG'), 10, (960,540))
            self.assertTrue(writer.isOpened())
            for _ in range(3): writer.write(color_bars(np))
            writer.release()
            capture = cv2.VideoCapture(path)
            count = 0
            while True:
                ok, frame = capture.read()
                if not ok: break
                self.assertEqual(len(rgb565(fit_frame(frame, cv2, np))), FRAME_BYTES)
                count += 1
            capture.release()
            self.assertEqual(count, 3)

    def test_static_arp_preflight(self):
        import json
        from types import SimpleNamespace
        result = {'Prefix':24, 'Status':'Up', 'Speed':'1 Gbps', 'SpeedBps':1000000000, 'Mtu':1500,
                  'Neighbors':[{'LinkLayerAddress':'A0-B1-C2-D3-E4-E4', 'State':'Permanent'}]}
        with patch('app.subprocess.run') as run:
            run.return_value = SimpleNamespace(returncode=0, stdout=json.dumps(result).encode())
            self.assertIn('静态 ARP 正确', check_network('192.168.1.102'))
            result['Neighbors'] = []
            run.return_value.stdout = json.dumps(result).encode()
            with self.assertRaises(ValueError): check_network('192.168.1.102')

    def test_stop_during_header_completes_current_frame(self):
        import queue
        import threading
        app = App.__new__(App)
        app.stop = threading.Event()
        app.events = queue.Queue()
        app.preview = queue.Queue(maxsize=1)
        class StopSocket(FakeSocket):
            def sendto(self, payload, address):
                app.stop.set()  # User presses stop as soon as the header is sent.
                return super().sendto(payload, address)
        sock = StopSocket()
        sender = Sender('127.0.0.1', sock=sock, wait=lambda _: None)
        with patch('app.check_network', return_value='test'), patch('app.Sender', return_value=sender):
            app.run(('192.168.1.102', 10, 120, '彩条测试', '', True))
        self.assertEqual(len(sock.sent), 865)
        self.assertEqual(sum(len(p) for p, _ in sock.sent), FRAME_BYTES + 8)
        self.assertFalse(any(event[0] == 'error' for event in list(app.events.queue)))


if __name__ == '__main__':
    unittest.main()
