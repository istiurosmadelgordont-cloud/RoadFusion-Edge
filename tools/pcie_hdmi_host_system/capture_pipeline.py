"""有界的采集/处理流水线；DMA 的所有权始终留在内核驱动里。

分工：
  采集线程（本模块）—— 只管把帧从驱动读出来，放进有界队列；
  GUI 线程          —— 从队列取帧做转换和显示。

队列容量是有限的（默认 4）：当显示跟不上采集时，**丢最旧的帧**而不是
无限堆积。这样预览始终接近实时，代价只是看不到部分中间帧。
本模块自身不分配也不释放 DMA 内存——那是驱动的职责。
"""
from collections import deque
import threading
import time
from frames import FrameReader


class FramePipeline:
    """在后台线程持续采集，向上层提供一个有界的帧队列。"""

    def __init__(self, source, device, path, stop, continuous, capacity=4):
        if capacity < 1:
            raise ValueError("capacity must be positive")
        self.source, self.device, self.path = source, device, path
        self.stop, self.continuous = stop, continuous
        self.capacity = capacity
        self.frames = deque()               # 有界队列：满了就丢最旧的
        self.condition = threading.Condition()   # 队列空/满时的等待与唤醒
        self.received = self.dropped = 0    # 累计接收数 / 累计丢弃数
        self.done = False                   # 采集线程是否已结束
        self.error = None                   # 采集线程里的异常，交给 take() 抛回
        self.thread = threading.Thread(target=self._capture, daemon=False)

    def _capture(self):
        """采集线程主体：循环读取，直到 stop 被置位（或单帧模式取到一帧）。"""
        try:
            with FrameReader(self.source, self.device, self.path) as reader:
                while not self.stop.is_set():
                    raw, elapsed = reader.read()
                    with self.condition:
                        self.received += 1
                        # 队列已满 → 挤掉最旧的一帧，保证放得下最新的
                        if len(self.frames) == self.capacity:
                            self.frames.popleft()
                            self.dropped += 1
                        self.frames.append((raw, elapsed, time.time(), self.received))
                        self.condition.notify_all()
                    # 单帧模式：取到一帧就收工
                    if not self.continuous:
                        break
                    # 非 hardware 来源没有硬件节流，这里补一点间隔避免空转打满 CPU
                    if self.source != "hardware" and self.stop.wait(0.001):
                        break
        except Exception as exc:
            # 异常不能在采集线程里抛出去，记录下来由 take() 在主线程抛出
            self.error = exc
        finally:
            with self.condition:
                self.done = True
                self.condition.notify_all()

    def take(self):
        """取下一帧；队列空时阻塞等待。

        返回 (raw, elapsed, timestamp, 序号) 或 None。
        上层请求停止、或采集线程已结束且队列已空时返回 None。
        """
        with self.condition:
            while not self.frames and not self.done and not self.stop.is_set():
                # 带超时的等待：即使丢失唤醒也会周期性醒来复查
                self.condition.wait(0.05)
            if self.stop.is_set():
                return None
            if self.frames:
                return self.frames.popleft()
            # 队列空且采集已结束：把采集线程里的异常交给调用方
            if self.error is not None:
                raise self.error
            return None

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *exc):
        # 请求停止并**等待采集线程真正结束**：驱动里的 read 最长约 5 秒返回，
        # 所以退出可能需要等这么久。绝不强杀线程——强杀会让驱动侧状态不干净。
        if self.thread.is_alive():
            self.stop.set()
        self.thread.join()  # The driver bounds each pending read to ~5 seconds.
