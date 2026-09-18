"""Wire format for the supplied udp_rx.v / eth_img_rec.v. No extra metadata."""
import ipaddress
import socket
import sys
import time

WIDTH, HEIGHT = 960, 540
FRAME_BYTES = WIDTH * HEIGHT * 2
CHUNK_BYTES = 1200
HEADER = bytes.fromhex('f0 5a a5 0f 03 c0 02 1c')
BOARD_IP = '192.168.1.10'
BOARD_MAC = 'A0-B1-C2-D3-E4-E4'


def rgb565(frame, size=(WIDTH, HEIGHT)):
    """OpenCV BGR uint8 image -> raster-order, high-byte-first RGB565."""
    import cv2
    import numpy as np
    if frame.shape != (size[1], size[0], 3) or frame.dtype != np.uint8:
        raise ValueError('图像尺寸与发送分辨率不一致，或不是 BGR uint8 图像')
    # OpenCV converts in native little-endian order. Swap the 16-bit words to
    # match the FPGA's high-byte-first UDP stream. This avoids three full-frame
    # uint16 channel arrays and is much faster than per-channel NumPy shifts.
    pixels = cv2.cvtColor(frame, cv2.COLOR_BGR2BGR565)
    return pixels.view('<u2').byteswap().tobytes()


def packets(pixels, size=(WIDTH, HEIGHT)):
    import struct
    w, h = size
    frame_bytes = w * h * 2
    if not 1 <= w <= 65535 or not 1 <= h <= 65535:
        raise ValueError('发送分辨率无效，宽高范围 1–65535')
    if len(pixels) != frame_bytes:
        raise ValueError(f'每帧必须为 {frame_bytes} 字节')
    yield HEADER[:4] + struct.pack('>HH', w, h)
    for offset in range(0, frame_bytes, CHUNK_BYTES):
        yield pixels[offset:offset + CHUNK_BYTES]


def wait_until(deadline):
    # Windows sleep() often rounds a ~0.1 ms packet gap up to ~0.5-1 ms.
    # Spin only for short waits so one 540p frame does not take ~0.5 s.
    # Longer waits still sleep first; late wakeups never trigger catch-up sends.
    while True:
        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            return
        if remaining > 0.005:
            time.sleep(remaining - 0.002)


class Sender:
    def __init__(self, local_ip, port=1234, mbps=120, *,
                 destination=BOARD_IP, sock=None, clock=time.perf_counter,
                 wait=wait_until, size=(WIDTH, HEIGHT)):
        ipaddress.IPv4Address(local_ip)
        if not 1 <= port <= 65535 or not 1 <= mbps <= 950:
            raise ValueError('端口范围 1–65535，限速范围 1–950 Mbps')
        self.address = (destination, port)
        self.size = size
        self.mbps = mbps
        self.clock, self.wait = clock, wait
        self.sock = sock
        if self.sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
                if sys.platform == 'win32':
                    # Windows IP_DONTFRAGMENT. Fail rather than silently fragment.
                    self.sock.setsockopt(socket.IPPROTO_IP, 14, 1)
                self.sock.bind((local_ip, 0))
                self.sock.settimeout(2.0)
            except Exception:
                self.sock.close()
                raise

    def send_frame(self, pixels):
        # Intentionally no stop flag inside a frame: stop only at a boundary.
        for index, payload in enumerate(packets(pixels, self.size)):
            sent = self.sock.sendto(payload, self.address)
            if sent != len(payload):
                raise OSError('UDP 数据报未完整提交；请复位 FPGA 后重试')
            # Include Ethernet/IP/UDP/FCS/preamble/IFG overhead in rate estimate.
            gap = 0.002 if index == 0 else (len(payload) + 66) * 8 / (self.mbps * 1e6)
            self.wait(self.clock() + gap)
        self.wait(self.clock() + 0.002)

    def close(self):
        self.sock.close()
