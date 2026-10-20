"""Runner boundary tests against saved data, never simulation."""
import copy
import importlib.util
import itertools
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'examples/right_hand'))


class AdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import finger22_contact_adapter as A
        cls.A = A
        cls.guard = A.no_physics()
        cls.calls = cls.guard.__enter__()
        cls.adapter = A.prepare_saved_model()
        cls.record = next(r for r in A.rows(A.OLD/'run/contacts.jsonl.gz') if r['step'] == 4731)
        cls.event = next(c for c in cls.record['contacts'] if A.NAMES[1] in c['geom'])

    @classmethod
    def tearDownClass(cls):
        assert not cls.calls, cls.calls
        cls.guard.__exit__(None, None, None)

    def classify(self, event=None, **kwargs):
        return self.adapter.classify_event(self.event if event is None else event,
                                          self.record['timing'], 4731, **kwargs)

    def test_4731_actual_cache_local_source_only(self):
        before = copy.deepcopy(self.event)
        r = self.classify()
        self.assertFalse(r['stop'], r)
        self.assertEqual(r['status'], 'REGISTERED_LOCAL_FUNCTIONAL_APPROXIMATION')
        self.assertEqual([c['source_face'] for c in r['candidates']], [6175])
        self.assertEqual(r['native_distance_m'], self.event['distance_m'])
        self.assertAlmostEqual(r['source_complete_box_gap_m'], 4.2765109877300646e-5, places=14)
        self.assertEqual(self.event, before)

    def test_geom_order_and_mpr_midpoint_do_not_select_surface(self):
        event = copy.deepcopy(self.event)
        for key in ('geom', 'body', 'cached_body_pose'):
            event[key].reverse()
        # A proper swapped contact basis: reverse normal and tangent2.
        frame = np.array(event['frame']).reshape(3, 3)
        frame[[0, 2]] *= -1
        event['frame'] = frame.ravel().tolist()
        event['normal_world'] = frame[0].tolist()
        event['position_world_m'] = [100., 200., 300.]
        self.assertEqual(self.classify(event)['status'], 'REGISTERED_LOCAL_FUNCTIONAL_APPROXIMATION')

    def test_schema_cache_binding_and_qualification_fail_closed(self):
        mutations = []
        for key in ('cached_body_pose', 'distance_m', 'force_contact_frame', 'frame', 'filter_reasons'):
            e = copy.deepcopy(self.event); del e[key]; mutations.append(e)
        for key, value in [('body', ['wrong', self.event['body'][1]]),
                           ('filter_reasons', ['COLLISION_MASKS']),
                           ('geom', ['not_registered_box', self.A.NAMES[1]]),
                           ('distance_m', float('nan'))]:
            e = copy.deepcopy(self.event); e[key] = value; mutations.append(e)
        e = copy.deepcopy(self.event); e['cached_body_pose'][0]['rotation_world'] = [0.]*9; mutations.append(e)
        e = copy.deepcopy(self.event); e['normal_world'] = [1., 0., 0.]; mutations.append(e)
        for event in mutations:
            with self.subTest(event=list(event)):
                self.assertTrue(self.classify(event)['status'].startswith('STOP_UNKNOWN'))
        self.assertTrue(self.adapter.classify_event(self.event, 'post-step FK', 4731)['stop'])
        bad = copy.deepcopy(self.record)
        bad['body_poses'][self.A.BODY]['position_world_m'][0] += .001
        self.assertTrue(self.adapter.classify_step(bad)['stop'])

    def test_unrelated_contacts_untouched_and_zero_force_not_skipped(self):
        other = next(c for c in self.record['contacts'] if not any(n in self.A.NAMES for n in c['geom']))
        before = copy.deepcopy(other)
        self.assertEqual(self.classify(other)['status'], 'NOT_APPLICABLE')
        self.assertEqual(other, before)
        e = copy.deepcopy(self.event); e['force_contact_frame'] = [0.]*6
        self.assertEqual(self.classify(e)['status'], 'REGISTERED_LOCAL_FUNCTIONAL_APPROXIMATION')

    def test_runner_can_attach_and_serialize_diagnostic_without_cycles(self):
        event = copy.deepcopy(self.event)
        event['finger22_region'] = self.classify(event)
        json.dumps(event, allow_nan=False)

    def test_initial_cache_contract_never_requires_or_executes_reset(self):
        event = {k: copy.deepcopy(self.event[k]) for k in
                 ('geom', 'body', 'distance_m', 'position_world_m', 'cached_body_pose', 'filter_reasons')}
        r = self.adapter.classify_event(event, self.adapter.rules['runner_boundary']['initial_timing'], 0)
        self.assertEqual(r['status'], 'REGISTERED_LOCAL_FUNCTIONAL_APPROXIMATION')

    def test_native_non_bowl_counterpart_is_not_silently_allowed(self):
        event = copy.deepcopy(self.event)
        model = self.adapter.model
        g = next(i for i in range(model.ngeom) if model.geom(i).name and
                 model.body(int(model.geom_bodyid[i])).name == 'robot0_link3' and
                 model.geom_contype[i])
        event['geom'][0] = model.geom(g).name
        event['body'][0] = model.body(int(model.geom_bodyid[g])).name
        event['filter_reasons'] = self.A.filter_reasons(model, g, self.adapter.ids[self.A.NAMES[1]])
        self.assertEqual(event['filter_reasons'], [])
        self.assertTrue(self.classify(event)['status'].startswith('STOP_UNKNOWN'))

    def test_stop_adapter_retains_physical_reasons_and_unknown_is_fail_closed(self):
        decisions = [{'status': 'REGISTERED_LOCAL_FUNCTIONAL_APPROXIMATION', 'stop': False},
                     {'status': 'STOP_UNKNOWN_SEMANTICS', 'stop': True},
                     {'status': 'STOP_RISK_ENVELOPE', 'stop': True}]
        reasons = self.A.stop_reasons(decisions, ['ARM_TRACKING', 'MIMIC'])
        self.assertEqual(reasons, ['ARM_TRACKING', 'MIMIC', 'FINGER22_NONFUNCTIONAL_REGION_CONTACT',
                                   'FINGER22_RISK_CONTACT_UNKNOWN'])
        self.assertEqual(self.A.stop_reasons([{'status': 'unregistered_continue', 'stop': False}]),
                         ['FINGER22_RISK_CONTACT_UNKNOWN'])

    def test_exceptions_are_unknown_and_all_candidates_aggregate(self):
        with patch.object(self.adapter, 'geometry', side_effect=RuntimeError('deterministic proof failure')):
            r = self.classify()
        self.assertTrue(r['stop']); self.assertIn('RuntimeError', r['reason'])
        for statuses, want in [([], 'STOP_UNKNOWN'),
                               (['FUNCTIONAL_SOURCE_PATCH_APPROXIMATION', 'UNKNOWN_EDGE_OR_POINT'], 'STOP_UNKNOWN'),
                               (['UNKNOWN_EDGE_OR_POINT', 'ROOT_BORE_RISK_ENVELOPE'], 'STOP_RISK')]:
            self.assertTrue(self.A.C.decision(statuses).startswith(want))

    def test_source_envelope_failure_cannot_pass_by_positive_x(self):
        candidate = self.classify()['candidates'][0]
        with patch.object(self.A.C, 'source_front_certificate', return_value={'certified': False}):
            r = self.adapter.classify_patch(candidate, 1, self.adapter.rules['semantic_guard_m'])
        self.assertTrue(r['status'].startswith('UNKNOWN'))

    def test_frozen_geometry_risk_witnesses_and_seam(self):
        for label, part, point, half in self.adapter.fixture_specs():
            vertices = np.array(list(itertools.product((-1, 1), repeat=3)))*half+point
            r = self.adapter.geometry(part, self.A.C.CertifiedHull(vertices), 0.)
            with self.subTest(label=label):
                self.assertTrue(r['status'].startswith('STOP_RISK'), r)
        # A real compiled triangle intersecting a cut: no geometry alteration.
        h = self.adapter.hulls[1]; z = self.adapter.rules['cuts_local_z_m'][0]
        f = next(f for f in h.faces if np.min(h.vertices[f, 2]) <= z+1e-9
                 and np.max(h.vertices[f, 2]) >= z-1e-9)
        c = {'polygon_exact_m': [[str(x) for x in h.points[i]] for i in f]}
        self.assertIn('CUT_SEAM_RISK', self.adapter.classify_patch(c, 1, 3.032172567760128e-9)['status'])

    def test_unregistered_and_edge_contacts_remain_unknown(self):
        for poly in ([[.004, .1, .018], [.004, .101, .018], [.004, .1, .019]],
                     [[.004, .001, .018]], [[.004, .001, .018], [.004, .002, .018]]):
            r = self.adapter.classify_patch({'polygon_exact_m': poly}, 1, 3.032172567760128e-9)
            self.assertTrue(r['status'].startswith('UNKNOWN'), r)


if __name__ == '__main__':
    unittest.main()
