"""Deterministic polytope witnesses; no simulation or cached-state injection."""
import itertools
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'examples/right_hand'))


class ContactRegionTests(unittest.TestCase):
    def module(self):
        import finger22_contact_regions as C
        return C

    def box(self, center, half):
        return np.array(list(itertools.product((-1,1),repeat=3)))*half+center

    def test_face_edge_corner_keep_all_candidates(self):
        C=self.module();a=C.CertifiedHull(self.box([0,0,0],[1,1,1]))
        for center,expected in [([1,0,0],{0}),([1,1,0],{0,1}),([1,1,1],{0,1,2})]:
            b=C.CertifiedHull(self.box(center,[.1,.1,.1]))
            candidates=C.contact_faces(a,b,0.)
            axes={int(np.argmax(abs(a.normals[c['face']])))for c in candidates}
            self.assertEqual(axes,expected)
            self.assertGreaterEqual(len(candidates),len(expected))
            if len(expected)==1:self.assertEqual(len(candidates),2)

    def test_separation_and_positive_common_ball_certificates(self):
        C=self.module();a=C.CertifiedHull(self.box([0,0,0],[1,1,1]))
        b=C.CertifiedHull(self.box([3,0,0],[.1,.1,.1]))
        self.assertEqual(C.intersection_certificate(a,b,0.)['status'],'SEPARATED')
        b=C.CertifiedHull(self.box([1,0,0],[.1,.1,.1]))
        self.assertEqual(C.intersection_certificate(a,b,0.)['status'],'STRICT_OVERLAP')

    def test_functional_requires_source_domain_not_only_positive_normal(self):
        C=self.module()
        source=np.array([[[1,-.5,-.5],[1,.5,-.5],[1,.5,.5]],
                         [[1,-.5,-.5],[1,.5,.5],[1,-.5,.5]]])
        patch=[[1,-.1,-.1],[1,.1,-.1],[1,.1,.1],[1,-.1,.1]]
        self.assertEqual(C.semantic_patch(patch,1,source,[],0.)['status'],'FUNCTIONAL_SOURCE_PATCH_APPROXIMATION')
        # Identical +X orientation, outside proven source domain: UNKNOWN.
        outside=np.array(patch)+[0,2,0]
        self.assertTrue(C.semantic_patch(outside,1,source,[],0.)['status'].startswith('UNKNOWN'))
        self.assertEqual(C.semantic_patch(patch,0,source,[],0.)['status'],'ROOT_BORE_RISK_ENVELOPE')
        rear=np.array(patch)-[2,0,0]
        self.assertTrue(C.semantic_patch(rear,1,source,[],0.)['status'].startswith('UNKNOWN'))

    def test_risk_or_unknown_never_becomes_continue(self):
        C=self.module()
        self.assertEqual(C.decision(['FUNCTIONAL_SOURCE_PATCH_APPROXIMATION']),'FUNCTIONAL_PROVENANCE_ONLY_NOT_RELEASE_PASS')
        for values in [[],['UNKNOWN_CAVITY_OR_UNREGISTERED_SURFACE'],
                       ['FUNCTIONAL_SOURCE_PATCH_APPROXIMATION','ROOT_BORE_RISK_ENVELOPE']]:
            self.assertTrue(C.decision(values).startswith('STOP_'))

    def test_boundary_touch_not_silently_discarded(self):
        C=self.module();a=C.CertifiedHull(self.box([0,0,0],[1,1,1]))
        b=C.CertifiedHull(self.box([2,2,2],[1,1,1]))
        self.assertTrue(C.contact_faces(a,b,0.))
        self.assertNotEqual(C.intersection_certificate(a,b,0.)['status'],'SEPARATED')

    def test_full_source_envelope_rejects_a_forward_occluder(self):
        C=self.module()
        front=np.array([[1,-1,-1],[1,1,-1],[1,0,1.]])
        back=front-[2,0,0]
        patch=[[1.01,-.1,-.1],[1.01,.1,-.1],[1.01,0,.1]]
        proof=C.source_front_certificate(patch,np.array([front,back]),0,0.)
        self.assertTrue(proof['certified'])
        blocked=C.source_front_certificate(patch,np.array([front,back,front+[.1,0,0]]),0,0.)
        self.assertFalse(blocked['certified'])

    def test_exact_affine_box_keeps_closed_supports(self):
        C=self.module();vertices=self.box([0,0,0],[.01,.02,.03])
        rotation=np.array([[.91,-.37,.02],[.37,.91,.05],[.01,.02,.99]])
        points=C.affine_vertices(vertices,rotation,[.123,.047,.932])
        h=C.CertifiedHull(points,points)
        self.assertTrue(h.certificate['all_vertices_inside_every_exact_support'])


if __name__=='__main__':unittest.main()
