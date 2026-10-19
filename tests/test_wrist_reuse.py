"""Static migration invariants; never step or reset a simulator."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'examples/right_hand'))
import wrist_reuse as W


class ReuseTests(unittest.TestCase):
    def test_candidate_reverses_exactly_to_frozen_robot(self):
        audit = W.xml_audit(W.BASE, W.CANDIDATE)
        self.assertEqual(audit['removed_counts'], [7, 8])
        self.assertEqual(audit['replacement_counts'], [147, 159])
        self.assertEqual(audit['hand_collision_count'], 78)
        self.assertTrue(audit['unauthorized_xml_fields_identical'])

    def test_all_copied_meshes_have_zero_geom_pose_and_original_bytes(self):
        audit = W.resource_audit()
        self.assertEqual(audit['mesh_count'], 306)
        self.assertEqual(audit['hash_mismatches'], [])
        self.assertEqual(audit['nonzero_geom_poses'], [])
        self.assertEqual(audit['external_candidate_paths'], [])

    def test_broadphase_is_conservative(self):
        import numpy as np
        # Only pairs with a strictly positive >1e-10 interval may be skipped.
        a = np.array([[0., 0., 0.], [1., 1., 1.]])
        mins = np.array([[1.+2e-10, 0., 0.], [1.+.5e-10, 0., 0.], [.5, .5, .5]])
        maxs = mins+1
        gaps, axes, directions = W.aabb_gaps(a[0:1], a[1:2], mins, maxs)
        self.assertEqual((gaps[0] > W.P.TOL).tolist(), [True, False, False])

    def test_unnamed_scene_geoms_keep_identity_when_named_geoms_are_added(self):
        def compile_xml(extra):
            return W.mj.MjModel.from_xml_string('<mujoco><worldbody><geom type="plane" size="1 1 .1"/>'
                + extra + '<geom type="sphere" size=".1" pos="0 0 1"/></worldbody></mujoco>')
        a = compile_xml('')
        b = compile_xml('<geom name="added" type="sphere" size=".1" pos="0 0 2"/>')
        self.assertEqual(set(W.geom_names(a)), set(W.geom_names(b))-{'added'})


if __name__ == '__main__':
    unittest.main()
