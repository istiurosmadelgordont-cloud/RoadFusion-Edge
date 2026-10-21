"""Seam behavior: disagreement avoidance, timestamp bounds and missing support."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from panorama_seams import TemporalSeam, SeamConfig


class SeamRegression(unittest.TestCase):
    def make(self):
        overlap = np.ones((30, 100), bool)
        preferred = np.full(30, 50)
        return TemporalSeam(overlap, preferred)

    def test_avoid_mismatched_silhouette(self):
        seam = self.make()
        cost = np.zeros((30, 100), np.float32)
        cost[8:22, 40:61] = 20
        path = seam.update(cost, 0.)
        self.assertTrue(np.all((path[8:22] < 40) | (path[8:22] > 60)))
        self.assertLessEqual(np.max(abs(np.diff(path))), SeamConfig().row_step)

    def test_speed_uses_timestamp_not_frame_count(self):
        seam = self.make()
        seam.update(np.zeros((30, 100)), 0.)
        for seconds in (.05, .15, .3):
            previous, previous_time = seam.path.copy(), seam.time
            cost = np.full((30, 100), 20.)
            cost[:, :30] = 0
            path = seam.update(cost, seconds)
            self.assertLessEqual(np.max(abs(path-previous)), 20*(seconds-previous_time)+1e-8)

    def test_duplicate_frame_does_not_move(self):
        seam = self.make()
        old = seam.update(np.zeros((30, 100)), 1.)
        cost = np.ones((30, 100))*20
        cost[:, :10] = 0
        np.testing.assert_array_equal(seam.update(cost, 1.), old)

    def test_seek_or_large_gap_reinitializes(self):
        seam = self.make()
        seam.update(np.zeros((30, 100)), 5.)
        cost = np.full((30, 100), 20.)
        cost[:, :10] = 0
        path = seam.update(cost, 0.)
        self.assertTrue(seam.last_reset)
        self.assertTrue(np.all(path < 10))
        seam.update(cost, 2.)
        self.assertTrue(seam.last_reset)

    def test_missing_rows_do_not_invent_overlap(self):
        overlap = np.zeros((30, 100), bool)
        overlap[:10, 30:40] = True
        overlap[20:, 65:75] = True
        preferred = np.r_[np.full(10, 35), np.full(10, 50), np.full(10, 70)]
        seam = TemporalSeam(overlap, preferred)
        path = seam.update(np.zeros((30, 100)), 0.)
        rows = np.flatnonzero(overlap.any(axis=1))
        self.assertTrue(np.all(overlap[rows, path[rows]]))
        np.testing.assert_array_equal(path[10:20], preferred[10:20])

    def test_bad_timestamp_or_cost_rejected(self):
        seam = self.make()
        with self.assertRaises(ValueError):
            seam.update(np.zeros((30, 100)), float('nan'))
        with self.assertRaises(ValueError):
            seam.update(np.full((30, 100), float('inf')), 0.)

    def test_disconnected_columns_preserve_optimized_prefix(self):
        overlap = np.zeros((30, 100), bool)
        overlap[:15, 10:20] = True
        overlap[15:, 70:80] = True
        preferred = np.r_[np.full(15, 15), np.full(15, 75)]
        cost = np.full((30, 100), 20.)
        cost[:15, 17] = 0
        cost[15:, 77] = 0
        seam = TemporalSeam(overlap, preferred)
        path = seam.update(cost, 0.)
        np.testing.assert_array_equal(path[:15], np.full(15, 17))
        np.testing.assert_array_equal(path[15:], np.full(15, 77))


if __name__ == '__main__':
    unittest.main()
