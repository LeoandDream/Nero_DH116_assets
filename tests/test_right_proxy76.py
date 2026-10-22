"""Small static checks for the isolated left-proxy migration; never integrate."""
import importlib.util
from decimal import Decimal, localcontext
import hashlib
import json
from pathlib import Path
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


class ProxyMigrationTests(unittest.TestCase):
    def builder(self):
        path = ROOT / 'tools/build_right_proxy76.py'
        self.assertTrue(path.is_file(), 'The explicit static migration builder is required')
        spec = importlib.util.spec_from_file_location('right_proxy76_builder_test', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_reflection_preserves_positive_oriented_tetrahedron_volume(self):
        b = self.builder()
        vertices = np.array([[0.,0,0], [1,0,0], [0,1,0], [0,0,1]])
        faces = np.array([[0,2,1], [0,1,3], [0,3,2], [1,2,3]])
        v, f = b.transform_mesh(vertices, faces, np.diag([1.,-1,1]), np.array([2.,3,4]))
        volume = np.einsum('ij,ij->i', v[f[:,0]], np.cross(v[f[:,1]], v[f[:,2]])).sum()/6
        self.assertAlmostEqual(volume, 1/6)
        np.testing.assert_array_equal(v, vertices*[1,-1,1]+[2,3,4])
        np.testing.assert_array_equal(vertices, [[0,0,0], [1,0,0], [0,1,0], [0,0,1]])

    def test_identity_does_not_reorder_faces(self):
        b = self.builder()
        vertices = np.eye(3)
        faces = np.array([[0,1,2]])
        v, f = b.transform_mesh(vertices, faces, np.eye(3), np.zeros(3))
        np.testing.assert_array_equal(v, vertices)
        np.testing.assert_array_equal(f, faces)

    def test_saved_convex_overlap_has_positive_clearance_to_every_plane(self):
        # Independently recheck the recorded geometry witness without MuJoCo,
        # sampling, a constraint solve, or trusting the LP status string.
        proof = json.loads((ROOT/'examples/right_hand/evidence/d3_proxy76/gap_check/convex-common-ball.json').read_text())
        with localcontext() as ctx:
            ctx.prec = 60
            for case in proof['results'].values():
                for hull in case['hulls']:
                    path = ROOT/hull['raw_stl_path']
                    self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),hull['raw_stl_sha256'])
                center = [Decimal.from_float(v) for v in case['common_ball_center_conditioned_mm']]
                normals = case['certificate']['normals']
                offsets = case['certificate']['offsets_mm']
                margins = [-sum(Decimal.from_float(v)*x for v,x in zip(n,center))-Decimal.from_float(b)
                           for n,b in zip(normals,offsets)]
                self.assertGreater(min(margins),0)
                half_radius = Decimal.from_float(case['strict_inner_ball_witness_radius_mm'])
                self.assertGreater(half_radius,0)
                self.assertGreater(min(margins)-half_radius,0)


if __name__ == '__main__':
    unittest.main()
