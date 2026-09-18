"""Media sizing and deterministic frame-rate conversion."""
import math


def parse_size(text):
    if text == '原始尺寸':
        return None
    try:
        w, h = map(int, text.lower().replace('×', 'x').split('x'))
    except ValueError:
        raise ValueError('分辨率格式应为 960x540') from None
    if not 1 <= w <= 65535 or not 1 <= h <= 65535:
        raise ValueError('协议宽高为 16 位正整数，范围 1–65535')
    return w, h


def source_index(output_index, source_fps, input_fps, output_fps):
    # First sample the source at input_fps, then repeat/drop at output_fps.
    tick = math.floor(output_index * input_fps / output_fps + 1e-9)
    return math.floor(tick * source_fps / input_fps + 1e-9)


class VideoFrames:
    def __init__(self, path, cv2, input_fps, output_fps, loop):
        self.cv2, self.path = cv2, path
        self.input_fps, self.output_fps, self.loop = input_fps, output_fps, loop
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            self.cap.release()
            raise ValueError('无法打开视频')
        self.native_fps = self.cap.get(cv2.CAP_PROP_FPS)
        if not math.isfinite(self.native_fps) or self.native_fps <= 0:
            self.native_fps = input_fps
        self.index, self.output_index, self.cached = -1, 0, None

    def read(self, stop):
        target = source_index(self.output_index, self.native_fps, self.input_fps, self.output_fps)
        while self.index < target:
            if stop.is_set():
                return None
            ok, frame = self.cap.read()
            if not ok:
                if not self.loop:
                    return None
                self.cap.release()
                self.cap = self.cv2.VideoCapture(self.path)
                self.index, self.output_index, target = -1, 0, 0
                ok, frame = self.cap.read()
                if not ok:
                    raise ValueError('视频为空或循环重新打开失败')
            self.index += 1
            self.cached = frame
        self.output_index += 1
        return self.cached

    def close(self):
        self.cap.release()
