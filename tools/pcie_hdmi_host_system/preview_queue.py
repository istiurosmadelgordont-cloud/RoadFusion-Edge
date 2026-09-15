"""只保留最新一帧的预览队列：发布方永不等待 GUI。

与普通队列的关键区别是**覆盖式**：新帧进来直接顶掉还没被取走的旧帧。
视频预览要的是"当前最新"，积压旧帧只会让画面越来越滞后，
所以这里宁可丢帧也不排队。
"""
import threading


class LatestFrame:
    """线程安全的口袋，最多装一帧。put 从不阻塞，take 取走并清空。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._item = None

    def put(self, item):
        """覆盖写入。不阻塞、不排队；消费得慢只会看到更少的中间帧。"""
        with self._lock:
            self._item = item

    def take(self):
        """取走当前帧并清空。没有新帧时返回 None。"""
        with self._lock:
            item = self._item
            self._item = None
            return item
