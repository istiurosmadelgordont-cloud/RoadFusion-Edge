"""Wire format for the supplied udp_rx.v / eth_img_rec.v. No extra metadata."""
import ipaddress
import math
import socket
import sys
import time

WIDTH, HEIGHT = 960, 540
FRAME_BYTES = WIDTH * HEIGHT * 2
# The FPGA has no UDP backpressure. A short datagram bounds each continuous
# wire-speed pixel burst and gives its DDR write FIFO time to drain.
CHUNK_BYTES = 256
MAGIC = bytes.fromhex('f0 5a a5 0f')
HEADER = MAGIC + bytes.fromhex('03 c0 02 1c')
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


def packets(pixels, size=(WIDTH, HEIGHT), chunk_bytes=CHUNK_BYTES):
    import struct
    w, h = size
    frame_bytes = w * h * 2
    if not 1 <= w <= 65535 or not 1 <= h <= 65535:
        raise ValueError('发送分辨率无效，宽高范围 1–65535')
    if len(pixels) != frame_bytes:
        raise ValueError(f'每帧必须为 {frame_bytes} 字节')
    if not 4 <= chunk_bytes <= 1472 or chunk_bytes % 4:
        raise ValueError('UDP 像素负载必须是 4–1472 范围内的 4 字节整数倍')
    # Keep these as two UDP datagrams.  eth_img_rec raises img_data_vs when it
    # receives MAGIC and lowers it when the dimensions word arrives.
    yield MAGIC
    yield struct.pack('>HH', w, h)
    for offset in range(0, frame_bytes, chunk_bytes):
        yield pixels[offset:offset + chunk_bytes]


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
                 wait=wait_until, size=(WIDTH, HEIGHT), vs_blank_ms=2.0,
                 chunk_bytes=CHUNK_BYTES, min_packet_gap_us=20.0):
        ipaddress.IPv4Address(local_ip)
        destination = str(ipaddress.IPv4Address(destination))
        if not 1 <= port <= 65535 or not 1 <= mbps <= 950:
            raise ValueError('端口范围 1–65535，限速范围 1–950 Mbps')
        if not math.isfinite(vs_blank_ms) or not 0.01 <= vs_blank_ms <= 1000:
            raise ValueError('VS 消隐时间范围为 0.01–1000 ms')
        if not 4 <= chunk_bytes <= 1472 or chunk_bytes % 4:
            raise ValueError('UDP 像素负载必须是 4–1472 范围内的 4 字节整数倍')
        if not math.isfinite(min_packet_gap_us) or not 0 <= min_packet_gap_us <= 100000:
            raise ValueError('最小包间隔范围为 0–100000 us')
        self.address = (destination, port)
        self.size = size
        self.mbps = mbps
        self.vs_blank_s = vs_blank_ms / 1000
        self.chunk_bytes = chunk_bytes
        self.min_packet_gap_s = min_packet_gap_us / 1_000_000
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
        for index, payload in enumerate(packets(pixels, self.size, self.chunk_bytes)):
            sent = self.sock.sendto(payload, self.address)
            if sent != len(payload):
                raise OSError('UDP 数据报未完整提交；请复位 FPGA 后重试')
            # Include Ethernet/IP/UDP/FCS/preamble/IFG overhead in rate estimate.
            gap = (self.vs_blank_s if index == 0 else
                   max((len(payload) + 66) * 8 / (self.mbps * 1e6),
                       self.min_packet_gap_s))
            self.wait(self.clock() + gap)
        self.wait(self.clock() + 0.002)

    def close(self):
        self.sock.close()
