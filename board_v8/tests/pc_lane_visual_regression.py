"""Behavioral checks for stale-display limits and perception/display isolation."""
import copy
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from pc_ufld_video import make_lane,refine
from pc_lane_visual import AdjacentLaneVisualTracker


def scene(right_ratio=.75,center=320.):
    y=np.linspace(165,345,48)
    width=.8*y-40
    left=center-width*.5
    right=center+width*.5
    return [np.column_stack((left-width*.75,y)),np.column_stack((left,y)),
        np.column_stack((right,y)),np.column_stack((right+width*right_ratio,y))]


class VisualTrackingRegression(unittest.TestCase):
    def setUp(self):
        self.tracker=AdjacentLaneVisualTracker(refine)

    def update(self,slots,t,**changes):
        raw=make_lane(slots)
        for name,value in changes.items():setattr(raw,name,value)
        return self.tracker.update(raw,slots,raw,t)

    def confirmed(self,slots=None):
        slots=scene() if slots is None else slots
        self.assertEqual(self.update(slots,0),(None,None))
        result=self.update(slots,1/30)
        self.assertTrue(all(v is not None and not v.held for v in result))
        return slots,result

    def test_confirmation_and_no_mutation_of_decision_geometry(self):
        slots=scene(right_ratio=1.2)
        raw=make_lane(slots)
        self.assertIsNone(raw.polygons[2]) # previous full-span gate rejects it
        snapshot=copy.deepcopy(raw)
        self.tracker.update(raw,slots,raw,0)
        result=self.tracker.update(raw,slots,raw,1/30)
        self.assertIsNotNone(result[1])
        self.assertTrue(result[1].presentation_only)
        self.assertIsNone(raw.polygons[2])
        for old,new in zip(snapshot.boundaries,raw.boundaries):
            np.testing.assert_array_equal(old,new)

    def test_clips_supported_span_instead_of_discarding_visible_curve(self):
        slots,result=self.confirmed(scene(right_ratio=1.2))
        v=result[1]
        self.assertTrue(np.all((v.polygon[:,0]>=0)&(v.polygon[:,0]<=639)))
        self.assertGreaterEqual(v.boundary[:,1].min(),slots[3][:,1].min())
        self.assertLess(v.boundary[:,1].max(),slots[3][:,1].max())
        self.assertGreaterEqual(np.ptp(v.boundary[:,1]),54)

    def test_sides_are_independent_and_short_miss_is_labelled_held(self):
        slots,_=self.confirmed()
        slots[3]=np.empty((0,2))
        left,right=self.update(slots,2/30)
        self.assertIsNotNone(left);self.assertFalse(left.held)
        self.assertIsNotNone(right);self.assertTrue(right.held)
        self.assertLess(right.alpha,.24)
        self.assertAlmostEqual(right.age_ms,1000/30)

    def test_expiry_uses_source_time_with_sparse_samples(self):
        slots,_=self.confirmed()
        slots[3]=np.empty((0,2))
        self.assertIsNotNone(self.update(slots,.20)[1])
        self.assertIsNone(self.update(slots,.24)[1])

    def test_expiry_at_high_frame_rate_and_no_freeze_after_reappearance(self):
        slots,_=self.confirmed()
        missing=copy.deepcopy(slots);missing[3]=np.empty((0,2))
        for t in np.arange(.05,.24,.01):
            v=self.update(missing,float(t))[1]
            if v is not None:self.assertLessEqual(v.age_ms,180+1e-6)
        self.assertIsNone(v)
        self.assertIsNone(self.update(slots,.25)[1])
        self.assertIsNotNone(self.update(slots,.28)[1])

    def test_current_ego_uncertainty_clears_both_sides(self):
        for flag in ('partial','uncertain','held'):
            self.tracker.reset();slots,_=self.confirmed()
            self.assertEqual(self.update(slots,.067,**{flag:True}),(None,None))
        self.tracker.reset();slots,_=self.confirmed()
        self.assertEqual(self.update(slots,.067,valid=False),(None,None))

    def test_observed_edge_crossing_cancels_hold_immediately(self):
        slots,_=self.confirmed()
        slots[3][:,0]=slots[2][:,0]-35
        left,right=self.update(slots,2/30)
        self.assertIsNotNone(left)
        self.assertIsNone(right)

    def test_seek_scene_cut_and_ego_reassignment_reset_history(self):
        slots,_=self.confirmed()
        self.assertEqual(self.update(slots,0),(None,None))
        self.tracker.reset();slots,_=self.confirmed()
        self.assertEqual(self.update(slots,.067,reassignment='RIGHT'),(None,None))
        self.tracker.reset();slots,_=self.confirmed()
        self.assertEqual(self.update(scene(center=385),.067),(None,None))

    def test_jitter_is_reduced_without_retaining_missing_edge_forever(self):
        raw_positions=[];visual_positions=[]
        for i in range(60):
            ratio=.75+(.07 if i%2 else -.07)
            slots=scene(right_ratio=ratio)
            v=self.update(slots,i/30)[1]
            if v is not None:
                raw_positions.append(slots[3][-1,0])
                visual_positions.append(v.boundary[0,0])
        self.assertLess(np.std(visual_positions),np.std(raw_positions)*.7)
        slots[3]=np.empty((0,2))
        self.assertIsNone(self.update(slots,2.3)[1])


if __name__=='__main__':
    unittest.main(verbosity=2)
