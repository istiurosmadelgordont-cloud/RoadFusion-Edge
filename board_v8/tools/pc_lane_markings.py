"""Image-based lane paint estimates for PC display; never grants a manoeuvre."""
from dataclasses import dataclass

import cv2
import numpy as np

UNKNOWN, SOLID, DASHED = 'UNKNOWN', 'SOLID', 'DASHED'


@dataclass(frozen=True)
class PaintObservation:
    label: str = UNKNOWN
    coverage: float = 0.
    runs: int = 0
    longest_gap: int = 0
    reason: str = 'insufficient image evidence'


def _runs(mask):
    edges = np.diff(np.r_[False, mask, False].astype(np.int8))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def classify_profile(mask):
    """Classify an ordered, uniformly sampled road-only paint profile."""
    supported = np.asarray(mask, dtype=bool).copy()
    if len(supported) < 36:
        return PaintObservation(reason='short visible span')
    # Remove isolated highlights, then close only sub-marking-sized breaks.
    for first, end in _runs(supported):
        if end-first < 3:
            supported[first:end] = False
    for first, end in _runs(~supported):
        if first > 0 and end < len(supported) and end-first <= 2:
            supported[first:end] = True
    paint = _runs(supported)
    gaps = _runs(~supported)
    coverage = float(supported.mean())
    longest = max((end-first for first, end in gaps), default=0)
    internal = [(first, end) for first, end in gaps
                if first > 0 and end < len(supported) and end-first >= 5]
    label = UNKNOWN
    if coverage >= .83 and longest <= max(4, round(len(supported)*.09)):
        label = SOLID
    elif .10 <= coverage <= .78:
        bounded_patch = (len(paint) == 1 and paint[0][0] >= 5 and
                         len(supported)-paint[0][1] >= 5)
        if (len(paint) >= 2 and internal) or bounded_patch:
            label = DASHED
    return PaintObservation(label, coverage, len(paint), int(longest),
        'road paint continuity' if label != UNKNOWN else 'ambiguous or weak road paint')


class PaintFrame:
    def __init__(self, bgr):
        self.image = cv2.resize(bgr, (640, 360))
        self.gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY).astype(np.float32)
        hls = cv2.cvtColor(self.image, cv2.COLOR_BGR2HLS)
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        # Local contrast handles dim camera exposure; colour rejects foliage.
        self.paint_colour = ((hls[:, :, 2] <= 125) |
            ((hsv[:, :, 0] >= 12) & (hsv[:, :, 0] <= 42) & (hsv[:, :, 1] >= 65)))

    def observe(self, points):
        if points is None or len(points) < 10 or not np.isfinite(points).all():
            return PaintObservation(reason='missing observed boundary')
        ordered = np.asarray(points)[np.argsort(np.asarray(points)[:, 1])]
        top = max(float(ordered[0, 1]), 360*.49)
        bottom = min(float(ordered[-1, 1]), 360*.69)
        if bottom-top < 36:
            return PaintObservation(reason='short visible road span')
        yy = np.arange(int(np.ceil(top)), int(np.floor(bottom))+1)
        xx = np.interp(yy, ordered[:, 1], ordered[:, 0])
        if np.mean((xx >= 11) & (xx < 629)) < .9:
            return PaintObservation(reason='boundary clipped by camera view')
        offsets = np.arange(-24, 25)
        samples_x = np.rint(xx[:, None]+offsets).astype(int)
        valid = (samples_x >= 0) & (samples_x < 640)
        sampled = self.gray[yy[:, None], np.clip(samples_x, 0, 639)]
        colours = self.paint_colour[yy[:, None], np.clip(samples_x, 0, 639)] & valid
        background = np.median(sampled, axis=1)
        centre = (np.abs(offsets) <= 10)[None, :]
        contrast = sampled-background[:, None]
        candidates = colours & valid & centre & (sampled >= 50)
        scores = np.where(candidates, contrast, -1000)
        best = scores.argmax(axis=1)
        peak = scores[np.arange(len(yy)), best]
        supported = peak >= np.maximum(12, background*.12)
        # A coherent narrow ridge is required, rather than broad bright cars.
        ridge_offsets = offsets[best]
        if supported.sum() >= 6:
            fit = np.polyfit(yy[supported], ridge_offsets[supported], 1)
            supported &= np.abs(ridge_offsets-np.polyval(fit, yy)) <= 5
        return classify_profile(supported)


class MarkingTracker:
    """Two fresh confirmations, with immediate UNKNOWN on lost evidence."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.candidates = [UNKNOWN]*4
        self.streaks = [0]*4
        self.last_time = None

    def update(self, observations, lane, seconds, outer_held=(False, False)):
        if (not np.isfinite(seconds) or not lane.valid or lane.partial or
                lane.held or lane.uncertain or lane.reassignment != 'NONE'):
            self.reset()
            return (UNKNOWN,)*4
        if self.last_time is not None and (seconds <= self.last_time or seconds-self.last_time > .5):
            self.reset()
        self.last_time = seconds
        result = []
        for slot, observation in enumerate(observations):
            value = observation.label
            if slot in (0, 3) and outer_held[0 if slot == 0 else 1]:
                value = UNKNOWN
            if value == UNKNOWN:
                self.candidates[slot], self.streaks[slot] = UNKNOWN, 0
            elif self.candidates[slot] != value:
                self.candidates[slot], self.streaks[slot] = value, 1
            else:
                self.streaks[slot] += 1
            result.append(value if self.streaks[slot] >= 2 else UNKNOWN)
        return tuple(result)


def draw_boundary(frame, points, marking, color, thickness):
    """UNKNOWN is neutral dots; dashed always means a paint type, never HOLD."""
    if points is None or len(points) < 2:
        return
    points = np.asarray(points, np.int32)
    if marking == SOLID:
        cv2.polylines(frame, [points], False, color, thickness, cv2.LINE_AA)
        return
    # Resample by arc length so style does not depend on anchor count.
    lengths = np.r_[0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    if lengths[-1] <= 0:
        return
    samples = np.arange(0, lengths[-1]+.01, 1.)
    dense = np.rint(np.column_stack((np.interp(samples, lengths, points[:, 0]),
                                   np.interp(samples, lengths, points[:, 1])))).astype(np.int32)
    if marking == DASHED:
        for first in range(0, len(dense), 26):
            cv2.polylines(frame, [dense[first:first+15]], False, color, thickness, cv2.LINE_AA)
    else:
        for point in dense[::12]:
            cv2.circle(frame, tuple(point), max(1, thickness//2), (175, 180, 185), -1, cv2.LINE_AA)
