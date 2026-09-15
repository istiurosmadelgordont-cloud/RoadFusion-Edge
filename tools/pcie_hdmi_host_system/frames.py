"""PCIE_FIX_20260912 固定格式的帧处理（不依赖 GUI，可单独测试）。

数据格式固定：
  1920×1080、RGB565、小端（"<u2"），单帧 4,147,200 字节。
  每行 8 条彩条、每条 240 像素，颜色顺序见 COLORS。

本模块只做纯计算与文件读写，不碰设备、不起线程，
因此 Windows 上也能跑测试（见 test_frames.py）。
"""
import os
import time
import json
from pathlib import Path
import numpy as np

WIDTH, HEIGHT = 1920, 1080
FRAME_BYTES = WIDTH * HEIGHT * 2

# 标准彩条：白 黄 青 绿 品红 红 蓝 黑，与 FPGA 端 pattern_vg 一致
COLORS = np.array([0xffff, 0xffe0, 0x07ff, 0x07e0, 0xf81f, 0xf800, 0x001f, 0], dtype='<u2')


def demo_frame():
    """生成一帧标准彩条，供 demo 模式使用（不读设备）。

    把 8 个颜色各重复 240 次得到一整行，再纵向铺满 1080 行。
    """
    return np.tile(np.repeat(COLORS, 240), (HEIGHT, 1)).tobytes()


def pixels(raw):
    """把原始字节解释成 (1080, 1920) 的 uint16 像素矩阵。

    长度不合法直接抛错——宁可早失败，也不要拿半帧数据继续算下去。
    """
    if len(raw) != FRAME_BYTES:
        raise ValueError("需要 {} 字节，实际 {} 字节".format(FRAME_BYTES, len(raw)))
    return np.frombuffer(raw, dtype='<u2').reshape(HEIGHT, WIDTH)


def rgb(raw):
    """RGB565 → RGB888（uint8）。

    低位补高位做位扩展（例如 R5 的 5 位扩到 8 位），
    这是最常用的近似方式，不需要浮点运算。
    """
    p = pixels(raw)
    result = np.empty((HEIGHT, WIDTH, 3), dtype=np.uint8)
    r, g, b = (p >> 11) & 31, (p >> 5) & 63, p & 31
    result[:, :, 0] = (r << 3) | (r >> 2)
    result[:, :, 1] = (g << 2) | (g >> 4)
    result[:, :, 2] = (b << 3) | (b >> 2)
    return result


def check(raw):
    """逐像素比对标准彩条，返回校验结果字典。

    返回形如：
      {"passed": bool, "mismatched_pixels": int,
       "first": {"x","y","actual","expected"} 或 None}

    注意：**只有 FPGA 真实采到彩条图案时这个校验才有意义**。
    demo/file 模式下的 PASS 只能说明处理链路没写错，
    不能当作 PCIe 传输成功的证据。
    """
    p = pixels(raw)
    bad = p != np.repeat(COLORS, 240)[None, :]
    count = int(np.count_nonzero(bad))
    first = None
    if count:
        # 定位第一个错误像素，便于看是整段错位还是散点噪声
        y, x = divmod(int(bad.argmax()), WIDTH)
        first = dict(x=int(x), y=int(y), actual=int(p[y, x]), expected=int(COLORS[x // 240]))
    return dict(passed=count == 0, mismatched_pixels=count, first=first)


def acquire(source, device, raw_path):
    """按来源取一帧原始数据，返回 (raw, 耗时秒数)。

    demo     —— 本机合成彩条
    file     —— 从磁盘读一个原始帧文件
    hardware —— 从字符设备读**一次**；驱动保证"一次 read = 一帧完整图像"
    """
    start = time.monotonic()
    if source == "demo":
        raw = demo_frame()
    elif source == "file":
        # 故意多读 1 字节：如果文件比一帧长，pixels() 会因长度不符报错，
        # 避免把"更大的文件"误当成合法帧。
        with open(raw_path, "rb") as f:
            raw = f.read(FRAME_BYTES + 1)
    elif source == "hardware":
        fd = os.open(device, os.O_RDONLY)
        try:
            # One read is one complete capture. Never concatenate independent snapshots.
            raw = os.read(fd, FRAME_BYTES)
        finally:
            os.close(fd)
    else:
        raise ValueError("Unknown source")
    pixels(raw)                     # 长度校验，兼作最基础的格式检查
    return raw, time.monotonic() - start


def save(raw, directory, metadata):
    """把一帧存成三个文件（同名不同后缀），返回不带后缀的路径。

    .rgb565  原始字节，便于事后用别的工具复算
    .png     便于肉眼查看
    .json    元数据（来源、时间、校验结果等），保证可追溯
    """
    from PIL import Image
    folder = Path(directory)
    folder.mkdir(parents=True, exist_ok=True)
    # 时间戳 + 纳秒尾巴，避免同一秒内连拍时重名
    name = time.strftime("frame_%Y%m%d_%H%M%S") + "_{:09d}".format(time.time_ns() % 1000000000)
    base = folder / name
    base.with_suffix(".rgb565").write_bytes(raw)
    Image.fromarray(rgb(raw)).save(str(base.with_suffix(".png")))
    base.with_suffix(".json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    return str(base)


# Convert only preview pixels; full raw data remains available for saving.
_RGB565_LUT = None


def preview_rgb(raw, step=2):
    """只转换预览需要的像素，返回降采样后的 RGB888。

    step=1/2/3 分别对应原尺寸 / 960×540 / 640×360。
    保存路径仍用原始尺寸的 rgb()，这里牺牲的只是显示分辨率。

    RGB565 的取值空间只有 65536，因此首次调用时就建一张完整查找表，
    之后每帧都退化成一次花式索引，比每帧做位运算快得多。
    """
    global _RGB565_LUT
    if step not in (1, 2, 3):
        raise ValueError("preview step must be 1, 2 or 3")

    p = pixels(raw)
    if _RGB565_LUT is None:
        values = np.arange(65536, dtype=np.uint16)
        r, g, b = (values >> 11) & 31, (values >> 5) & 63, values & 31
        _RGB565_LUT = np.stack(((r << 3) | (r >> 2),
                               (g << 2) | (g >> 4),
                               (b << 3) | (b >> 2)), axis=1).astype(np.uint8)
    return _RGB565_LUT[p[::step, ::step]]


class FrameReader:
    """一次采集会话内保持同一个设备描述符打开的读取器。

    连续采集时避免反复 open/close 设备；句柄在退出 with 时统一关闭。
    非 hardware 来源（demo/file）走 acquire()，语义保持一致：
    每次 read 都返回一整帧。
    """
    def __init__(self, source, device, raw_path):
        self.source, self.device, self.raw_path = source, device, raw_path
        self.fd = None

    def __enter__(self):
        if self.source == "hardware":
            self.fd = os.open(self.device, os.O_RDONLY)
        return self

    def read(self):
        if self.source != "hardware":
            return acquire(self.source, self.device, self.raw_path)
        start = time.monotonic()
        raw = os.read(self.fd, FRAME_BYTES)
        pixels(raw)
        return raw, time.monotonic() - start

    def __exit__(self, *exc):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
