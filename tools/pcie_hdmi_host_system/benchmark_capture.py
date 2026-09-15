"""只测量驱动 read 的吞吐：不含显示、转换和写盘。

用来回答"PCIe 这条链路到底每秒能交付多少帧"，所以刻意把其他开销全部排除。

注意：这里测到的是**交付到用户态的读取吞吐**，
并不能证明每一帧都是 FPGA 产生且完整无撕裂的帧。
"""
import argparse
import time
from frames import FrameReader, FRAME_BYTES

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device', default='/dev/pcie_hdmi_host')
    p.add_argument('--seconds', type=float, default=15)
    a = p.parse_args()
    if not 0 < a.seconds < 3600:
        p.error('--seconds must be between 0 and 3600')
    with FrameReader('hardware', a.device, None) as reader:
        # 先空读一帧预热：把"首次下发四地址 + 等第一个完整帧"的耗时排除在统计之外
        reader.read()  # Exclude initial address programming and first-frame wait.
        start = last = time.monotonic()
        total = window = 0
        read_seconds = 0.0
        while time.monotonic() - start < a.seconds:
            raw, elapsed = reader.read()
            total += 1
            window += 1
            read_seconds += elapsed
            now = time.monotonic()
            if now-last >= 1:
                # 每秒输出一次窗口帧率，便于观察抖动而不是只看平均值
                print('read FPS={:.2f}, frames={}'.format(window/(now-last), total), flush=True)
                last, window = now, 0
        elapsed = time.monotonic()-start
        print('TOTAL: {:.2f} FPS, {:.2f} MiB/s, average read {:.1f} ms'.format(
            total/elapsed, total*FRAME_BYTES/elapsed/1048576, read_seconds*1000/total))
        print('Delivered read throughput only; not proof of unique/intact FPGA frames.')

if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError) as exc:
        raise SystemExit('Capture failed: {}'.format(exc))
