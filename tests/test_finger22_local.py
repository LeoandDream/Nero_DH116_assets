"""Fixed source partition, without generating or tuning a candidate."""
import sys
from pathlib import Path
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
import build_finger22_local as B


class PartitionTest(unittest.TestCase):
    def test_crossing_and_coplanar_source_faces_have_unique_complete_coverage(self):
        triangles = np.array([[[0., 0., 0.], [.01, 0., .04], [0., .01, .04]],
                              [[0., 0., .0098], [.01, 0., .0098], [0., .01, .0098]]])
        parts, audit = B.partition(triangles)
        self.assertEqual(len(parts), 4)
        self.assertTrue(audit['all_source_triangles_retained'])
        self.assertEqual(sum(1 in p['source_triangle_ids'] for p in parts.values()), 1)
        self.assertTrue(all(0 in p['source_triangle_ids'] for p in parts.values()))
        for i, p in enumerate(parts.values()):
            z = np.asarray(p['triangles'])[:, :, 2]
            if i: self.assertGreaterEqual(z.min()+1e-17, B.CUTS[i-1])
            if i < 3: self.assertLessEqual(z.max()-1e-17, B.CUTS[i])


if __name__ == '__main__':
    unittest.main()
