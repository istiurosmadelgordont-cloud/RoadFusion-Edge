"""Content-aware panorama seams with bounded motion in source-video time.

The cost is disagreement/texture, not an object detector or a depth estimate.
It can avoid many mismatched silhouettes but cannot correct camera parallax.
"""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class SeamConfig:
    search_radius: int = 120
    row_step: int = 4
    speed_pixels_per_second: float = 20.0
    reset_gap_seconds: float = .5
    preference_penalty: float = .006
    temporal_penalty: float = .12
    feather_sigma: float = 1.0


class TemporalSeam:
    def __init__(self, overlap, preferred, config=SeamConfig()):
        self.overlap = np.asarray(overlap, bool)
        self.preferred = np.asarray(preferred, np.int32)
        if self.overlap.ndim != 2 or self.preferred.shape != (self.overlap.shape[0],):
            raise ValueError('Invalid seam geometry')
        self.config = config
        self.path = None
        self.time = None
        self.last_motion = 0.0
        self.last_reset = False

    def update(self, cost, seconds):
        if cost.shape != self.overlap.shape or not np.isfinite(cost).all() or not np.isfinite(seconds):
            raise ValueError('Invalid seam cost or timestamp')
        cfg = self.config
        height, width = cost.shape
        reset = self.time is None or seconds < self.time or seconds-self.time > cfg.reset_gap_seconds
        if self.time is not None and seconds == self.time:
            self.last_motion, self.last_reset = 0., False
            return self.path.copy()
        xx = np.arange(width)[None, :]
        allowed = self.overlap & (abs(xx-self.preferred[:, None]) <= cfg.search_radius)
        objective = cost + abs(xx-self.preferred[:, None])*cfg.preference_penalty
        previous = None if reset else self.path
        if previous is not None:
            # Fractional budget is intentional: tiny time steps cannot move a
            # pixel. Never impose a per-frame limit independent of video FPS.
            budget = cfg.speed_pixels_per_second * (seconds-self.time)
            allowed &= abs(xx-previous[:, None]) <= budget + 1e-8
            objective += abs(xx-previous[:, None])*cfg.temporal_penalty
        fallback = self.preferred.copy() if previous is None else previous.copy()
        path = fallback.copy()
        supported = allowed.any(axis=1)
        # Solve independent contiguous blocks. An unobserved row cannot bridge
        # two regions by creating a seam through unsupported source pixels.
        boundaries = np.flatnonzero(np.diff(np.r_[False, supported, False]))
        for begin, end in zip(boundaries[::2], boundaries[1::2]):
            columns = np.flatnonzero(allowed[begin:end].any(axis=0))
            lo, hi = columns[0], columns[-1]+1
            block = np.where(allowed[begin:end, lo:hi], objective[begin:end, lo:hi], np.inf)
            cumulative = block[0].copy()
            back = np.zeros(block.shape, np.int16)
            segment_start = 0

            def finish_segment(last_row):
                if not np.isfinite(cumulative).any():
                    return
                column = int(cumulative.argmin())
                for local_row in range(last_row, segment_start-1, -1):
                    path[begin+local_row] = lo+column
                    if local_row > segment_start:
                        column += int(back[local_row, column])

            for row in range(1, len(block)):
                candidates = []
                for delta in range(-cfg.row_step, cfg.row_step+1):
                    shifted = np.full_like(cumulative, np.inf)
                    if delta < 0:
                        shifted[-delta:] = cumulative[:delta]
                    elif delta > 0:
                        shifted[:-delta] = cumulative[delta:]
                    else:
                        shifted[:] = cumulative
                    candidates.append(shifted + abs(delta)*.04)
                candidates = np.stack(candidates)
                choice = candidates.argmin(axis=0)
                back[row] = choice-cfg.row_step
                next_cost = block[row] + np.min(candidates, axis=0)
                if not np.isfinite(next_cost).any():
                    # The vehicle mask may shift the overlap by many pixels
                    # between adjacent rows. Finish the reachable prefix rather
                    # than throwing away optimization of the whole image.
                    finish_segment(row-1)
                    segment_start = row
                    cumulative = block[row].copy()
                    back[row] = 0
                else:
                    cumulative = next_cost
            finish_segment(len(block)-1)
        if previous is not None:
            self.last_motion = float(np.max(abs(path-previous)))
        else:
            self.last_motion = 0.
        self.path, self.time, self.last_reset = path, float(seconds), reset
        return path.copy()


def disagreement(first, second):
    color = np.mean(abs(first-second), axis=2) / 24.
    gray_a, gray_b = (cv2.cvtColor(np.clip(x, 0, 255).astype(np.uint8), cv2.COLOR_BGR2GRAY) for x in (first, second))
    edges = sum(abs(cv2.Sobel(gray, cv2.CV_32F, 1, 0)) + abs(cv2.Sobel(gray, cv2.CV_32F, 0, 1)) for gray in (gray_a, gray_b)) / 160.
    return cv2.GaussianBlur(np.minimum(color, 8)+np.minimum(edges, 4), (7, 7), 1.5)


def seam_difference(first, second, path, overlap):
    yy = np.arange(len(path))
    supported = overlap[yy, path]
    if not supported.any():
        return None
    return float(np.mean(abs(first[yy[supported], path[supported]]-second[yy[supported], path[supported]])))
