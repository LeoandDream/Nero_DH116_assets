"""Targeted checks for full-survey reporting; no model or state is created."""
import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'examples/right_hand'))
import link5_flange_fullscan as S


class FullscanTests(unittest.TestCase):
    def test_disjoint_failure_intervals_and_unknowns_survive_first_failure(self):
        rows = [{'pose_index_one_based': i, 'anomaly': i in (2, 3, 5, 7)} for i in range(1, 9)]
        self.assertEqual([[r['pose_index_one_based'] for r in group]
                          for group in S.intervals(rows, lambda r: r['anomaly'])], [[2, 3], [5], [7]])

    def test_approximate_negative_dual_alone_does_not_certify_separation(self):
        value = {'strict_convex_solid_interior_overlap': False, 'dual_upper_bound_mm': -1.}
        self.assertEqual(S.status_from_lp(value), 'UNKNOWN_NUMERICAL')
        value['independent_nonnegative_dual_upper_bound_mm'] = .01
        self.assertEqual(S.status_from_lp(value), 'UNKNOWN_NUMERICAL')
        value['independent_nonnegative_dual_upper_bound_mm'] = -1.
        self.assertEqual(S.status_from_lp(value), 'SEPARATED')

    def test_corrected_bound_for_analytically_separated_boxes(self):
        corners = np.array([[x, y, z] for x in (-.001, .001)
                            for y in (-.001, .001) for z in (-.001, .001)])
        result = {'conditioning_origin_body13_mm': [2., 0., 0.], 'certificate': {
            'normals': [[1., 0., 0.], [-1., 0., 0.]],
            'offsets_mm': [1., 1.], 'dual_weights': [.5, .5]}}
        bound = S.dual_bound(result, {'vertices': corners}, {'vertices': corners},
                             np.eye(3), np.array([.004, 0., 0.]))
        self.assertAlmostEqual(bound['independent_nonnegative_dual_upper_bound_mm'], -1.)
        self.assertGreaterEqual(bound['dual_longdouble_roundoff_guard_mm'], 0.)
        self.assertEqual(bound['common_ball_center_absolute_conditioned_AABB_mm'], [3., 1., 1.])


if __name__ == '__main__':
    unittest.main()
