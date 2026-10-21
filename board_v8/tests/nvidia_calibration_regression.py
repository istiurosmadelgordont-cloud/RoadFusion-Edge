"""Analytic geometry checks for the PC NVIDIA calibration builder."""
import copy
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'output/pc_surround_review/deps'))
sys.path.insert(0, str(ROOT / 'tools'))
from build_nvidia_avm_calibration import project_ftheta


class ProjectionRegression(unittest.TestCase):
    def setUp(self):
        self.parameters = dict(resolution=[1920, 1080], principal_point=[960., 540.],
            reference_poly='ANGLE_TO_PIXELDIST', angle_to_pixeldist_poly=[0., 100.],
            pixeldist_to_angle_poly=[0., .02], max_angle=1.5,
            linear_cde=[1., 0., 0.], external_distortion_parameters=None)

    def test_exact_forward_reference_ignores_approximate_inverse(self):
        rays = np.array([[np.sin(.4), 0., np.cos(.4)]])
        pixel, valid = project_ftheta(rays, self.parameters)
        np.testing.assert_allclose(pixel, [[1000., 540.]])
        self.assertTrue(valid[0])

    def test_exact_backward_reference_ignores_approximate_forward(self):
        p = copy.deepcopy(self.parameters)
        p['reference_poly'] = 'PIXELDIST_TO_ANGLE'
        pixel, valid = project_ftheta(np.array([[np.sin(.4), 0., np.cos(.4)]]), p)
        np.testing.assert_allclose(pixel, [[980., 540.]], atol=1e-6)
        self.assertTrue(valid[0])

    def test_linear_scale_and_shear(self):
        p = copy.deepcopy(self.parameters)
        p['linear_cde'] = [1.2, .1, -.2]
        ray = np.array([[np.sin(.5)*.6, np.sin(.5)*.8, np.cos(.5)]])
        pixel, valid = project_ftheta(ray, p)
        np.testing.assert_allclose(pixel, [[1000., 574.]], atol=1e-6)
        self.assertTrue(valid[0])

    def test_windshield_forward_vertical_offset(self):
        p = copy.deepcopy(self.parameters)
        p['external_distortion_parameters'] = dict(reference_poly='FORWARD',
            horizontal_poly=[0., 1., 0.], vertical_poly=[.05, 0., 1.])
        pixel, valid = project_ftheta(np.array([[0., 0., 1.]]), p)
        np.testing.assert_allclose(pixel, [[960., 545.]], atol=1e-6)
        self.assertTrue(valid[0])

    def test_windshield_backward_reference_inverted(self):
        p = copy.deepcopy(self.parameters)
        p['external_distortion_parameters'] = dict(reference_poly='BACKWARD',
            horizontal_poly=[0., .9, 0.], vertical_poly=[0., 0., .9],
            horizontal_poly_inverse=[0., 1., 0.], vertical_poly_inverse=[-.05, 0., 1.])
        pixel, valid = project_ftheta(np.array([[0., 0., 1.]]), p)
        np.testing.assert_allclose(pixel, [[960., 545.]], atol=1e-6)
        self.assertTrue(valid[0])

    def test_invalid_domain_and_noninvertible_model(self):
        _, valid = project_ftheta(np.array([[0., 0., -1.]]), self.parameters)
        self.assertFalse(valid[0])
        p = copy.deepcopy(self.parameters)
        p['linear_cde'] = [0., 0., 0.]
        with self.assertRaises(ValueError):
            project_ftheta(np.array([[0., 0., 1.]]), p)


if __name__ == '__main__':
    unittest.main()
