"""Independent paint scenes and loss-of-evidence behaviour for PC display."""
import copy
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from pc_lane_markings import DASHED, SOLID, UNKNOWN, MarkingTracker, PaintFrame, PaintObservation
from pc_ufld_video import Lane


class MarkingTests(unittest.TestCase):
    def setUp(self):
        self.curve = np.column_stack((np.full(32,200),np.linspace(160,280,32)))

    def scene(self, segments, colour=(110,110,110)):
        image = np.full((360,640,3),70,np.uint8)
        for top,bottom in segments:
            cv2.line(image,(200,top),(200,bottom),colour,3)
        return image

    def test_dim_continuous_white_paint(self):
        self.assertEqual(PaintFrame(self.scene([(140,290)])).observe(self.curve).label,SOLID)

    def test_dashed_white_paint(self):
        scene = self.scene([(y,y+11) for y in range(140,290,30)])
        self.assertEqual(PaintFrame(scene).observe(self.curve).label,DASHED)

    def test_continuous_yellow_paint(self):
        scene = self.scene([(140,290)],(35,95,135))
        self.assertEqual(PaintFrame(scene).observe(self.curve).label,SOLID)

    def test_uniform_bright_road_has_no_paint_evidence(self):
        self.assertEqual(PaintFrame(np.full((360,640,3),185,np.uint8)).observe(self.curve).label,UNKNOWN)

    def test_missing_or_clipped_boundary_is_unknown(self):
        frame = PaintFrame(self.scene([(140,290)]))
        self.assertEqual(frame.observe(None).label,UNKNOWN)
        outside = self.curve.copy();outside[:,0] = -4
        self.assertEqual(frame.observe(outside).label,UNKNOWN)

    def test_two_distinct_fresh_confirmations_required(self):
        tracker = MarkingTracker();lane = Lane(valid=True)
        observed = [PaintObservation(SOLID)]*4
        self.assertEqual(tracker.update(observed,lane,0),(UNKNOWN,)*4)
        self.assertEqual(tracker.update(observed,lane,0),(UNKNOWN,)*4)
        self.assertEqual(tracker.update(observed,lane,.033),(SOLID,)*4)

    def test_weak_evidence_resets_confirmation(self):
        tracker = MarkingTracker();lane = Lane(valid=True)
        solid = [PaintObservation(SOLID)]*4
        tracker.update(solid,lane,0);tracker.update(solid,lane,.033)
        self.assertEqual(tracker.update([PaintObservation()]*4,lane,.066),(UNKNOWN,)*4)
        self.assertEqual(tracker.update(solid,lane,.099),(UNKNOWN,)*4)

    def test_held_or_reassigned_lane_cannot_reuse_marking(self):
        for field,value in (('held',True),('uncertain',True),('partial',True),('reassignment','LEFT')):
            tracker = MarkingTracker();lane = Lane(valid=True)
            observed = [PaintObservation(SOLID)]*4
            tracker.update(observed,lane,0);tracker.update(observed,lane,.033)
            changed = copy.deepcopy(lane);setattr(changed,field,value)
            self.assertEqual(tracker.update(observed,changed,.066),(UNKNOWN,)*4)

    def test_outer_hold_loses_type_without_changing_inner_types(self):
        tracker = MarkingTracker();lane = Lane(valid=True)
        observed = [PaintObservation(DASHED)]*4
        tracker.update(observed,lane,0);tracker.update(observed,lane,.033)
        self.assertEqual(tracker.update(observed,lane,.066,(True,False)),
                         (UNKNOWN,DASHED,DASHED,DASHED))


if __name__ == '__main__':
    unittest.main()
