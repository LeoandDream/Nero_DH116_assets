#!/usr/bin/env python3
"""Read-only RK4 event replay plus deterministic negative integration cases."""
import argparse
import copy
from collections import Counter
import gzip
import itertools
import json
import os
from pathlib import Path
import sys
import time
for k in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[k] = '1'
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'examples/right_hand'))
import finger22_contact_adapter as A


def write(path, value):
    raw = (json.dumps(value, indent=2, allow_nan=False)+'\n').encode()
    if path.suffix == '.gz':
        with gzip.open(path, 'wb') as f:
            f.write(raw)
    else:
        path.write_bytes(raw)


def main(output):
    if output.exists():
        raise ValueError('New output only; never overwrite evidence')
    output.mkdir(parents=True)
    start = time.perf_counter()
    outcomes = Counter(); others = 0; actual = []; steps = 0; stops = []
    with A.no_physics() as forbidden:
        adapter = A.prepare_saved_model()
        for cache in A.rows(A.OLD/'run/contacts.jsonl.gz'):
            steps += 1
            if cache['step'] != steps:
                raise ValueError('Noncontiguous saved-cache physical step')
            before = json.dumps(cache, sort_keys=True)
            result = adapter.classify_step(cache)
            if result['stop']:
                stops.append(result)
            for contact, decision in zip(cache['contacts'], result.get('decisions', [])):
                outcomes[decision['status']] += 1
                if decision['status'] == 'NOT_APPLICABLE':
                    others += 1
                else:
                    actual.append(decision)
                    # Same operation the runner will perform before stop checks.
                    attached = copy.deepcopy(contact)
                    attached['finger22_region'] = decision
                    json.dumps(attached, allow_nan=False)
            if before != json.dumps(cache, sort_keys=True):
                raise ValueError('Adapter modified old contact/filter records')
            if cache['step'] == 4731:
                final = cache
        event = next(c for c in final['contacts'] if A.NAMES[1] in c['geom'])
        negative = []
        def add(label, result, expected, kind):
            status = result['status']
            negative.append({'case': label, 'fixture_type': kind, 'expected': expected,
                             'matched': status.startswith(expected), 'result': result})
        # Complete frozen compiled hull against finite synthetic counterpart
        # geometry. These are not native contacts or new physical trajectories.
        for label, part, point, half in adapter.fixture_specs():
            vertices = np.array(list(itertools.product((-1, 1), repeat=3)))*half+point
            result = adapter.geometry(part, A.C.CertifiedHull(vertices), 0.)
            add(label, result, 'STOP_RISK', 'synthetic complete-box / frozen compiled support hull')
        h = adapter.hulls[1]; z = adapter.rules['cuts_local_z_m'][0]
        # The actual local patch passes, but the entire face285 is NOT a
        # registered functional domain. Record its independent disposition.
        whole285 = adapter.classify_patch({'polygon_exact_m':
            [[str(x) for x in h.points[i]] for i in h.faces[285]]}, 1, adapter.rules['semantic_guard_m'])
        face = next(f for f in h.faces if h.vertices[f, 2].min() <= z+1e-9
                    and h.vertices[f, 2].max() >= z-1e-9)
        seam = {'polygon_exact_m': [[str(x) for x in h.points[i]] for i in face]}
        label = adapter.classify_patch(seam, 1, adapter.rules['semantic_guard_m'])
        add('cut_seam', {'status': A.C.decision([label['status']]), 'candidate': label},
            'STOP_RISK', 'frozen compiled cut-plane triangle, no new mesh')
        for name, poly in [('unregistered_positive_x', [[.004, .1, .018], [.004, .101, .018], [.004, .1, .019]]),
                           ('edge', [[.004, .001, .018], [.004, .002, .018]]),
                           ('point', [[.004, .001, .018]])]:
            label = adapter.classify_patch({'polygon_exact_m': poly}, 1, adapter.rules['semantic_guard_m'])
            add(name, {'status': A.C.decision([label['status']]), 'candidate': label},
                'STOP_UNKNOWN', 'deterministic synthetic contact patch')
        # Run full boundary on malformed copies of the actual event. No source
        # or physics input is changed; each copy is a declared negative fixture.
        for key in ('cached_body_pose', 'filter_reasons', 'frame', 'force_contact_frame'):
            e = copy.deepcopy(event); del e[key]
            add('missing_'+key, adapter.classify_event(e, final['timing'], 4731),
                'STOP_UNKNOWN', 'malformed-event injection')
        for key, value in [('geom', ['wrong_box', A.NAMES[1]]),
                           ('body', ['wrong_body', A.BODY]),
                           ('filter_reasons', ['COLLISION_MASKS']),
                           ('distance_m', float('nan'))]:
            e = copy.deepcopy(event); e[key] = value
            add('invalid_'+key, adapter.classify_event(e, final['timing'], 4731),
                'STOP_UNKNOWN', 'malformed-event injection')
        e = copy.deepcopy(event); e['cached_body_pose'][0]['rotation_world'] = [0.]*9
        add('invalid_rotation', adapter.classify_event(e, final['timing'], 4731),
            'STOP_UNKNOWN', 'malformed-event injection')
        add('poststep_FK_not_RK4', adapter.classify_event(event, 'post-step FK', 4731),
            'STOP_UNKNOWN', 'wrong-cache-stage injection')
        # Every candidate remains in the set; favorable candidates cannot hide
        # risk/UNKNOWN. Exact box edge/corner candidate retention is checked in
        # test_finger22_contact_regions as well as the adapter tests.
        for labels, expected in [(['FUNCTIONAL_SOURCE_PATCH_APPROXIMATION', 'UNKNOWN_EDGE_OR_POINT'], 'STOP_UNKNOWN'),
                                  (['FUNCTIONAL_SOURCE_PATCH_APPROXIMATION', 'ROOT_BORE_RISK_ENVELOPE'], 'STOP_RISK'),
                                  ([], 'STOP_UNKNOWN')]:
            add('aggregate_'+str(len(negative)), {'status': A.C.decision(labels), 'all_candidate_labels': labels},
                expected, 'candidate-set aggregation')
        # An engine/proof failure must yield a serializable UNKNOWN stop. The
        # actual geometry classifier and frozen rules are not modified on disk.
        from unittest.mock import patch
        with patch.object(adapter, 'geometry', side_effect=RuntimeError('injected proof failure')):
            add('proof_exception', adapter.classify_event(event, final['timing'], 4731),
                'STOP_UNKNOWN', 'deterministic exception injection')
        with patch.object(A.C, 'source_front_certificate', return_value={'certified': False}):
            add('source_envelope_unproved', adapter.classify_event(event, final['timing'], 4731),
                'STOP_UNKNOWN', 'deterministic unavailable-proof injection')
    protected = A.read(HERE/'protected-before.json.gz')
    changed = [p for p, h in protected['tracked_sha256'].items() if A.sha(ROOT/p) != h]
    readme_ok = A.sha('/data_robot/projects/nero_dh116_assets/README.md') == protected['user_readme_sha256']
    passed = (steps == 4731 and len(actual) == 1 and not stops and all(n['matched'] for n in negative)
              and not forbidden and not changed and readme_ok)
    summary = {'status': 'OFFLINE_INTEGRATION_PASS_READY_TO_APPLY_ONLY' if passed else 'OFFLINE_INTEGRATION_BLOCKED',
               'rules_sha256': A.RULE_SHA256, 'saved_RK4_rows': steps, 'all_contact_event_counts': dict(outcomes),
               'other_contact_records_preserved': others, 'relevant_finger22_events': len(actual),
               'replay_stops': len(stops), 'negative_cases': len(negative),
               'negative_cases_matched': sum(n['matched'] for n in negative),
               'prepare_seconds': adapter.setup_seconds,
               'relevant_event_latency_seconds': [r['latency_seconds'] for r in actual],
               'wall_seconds': time.perf_counter()-start, 'forbidden_API_attempts': forbidden,
               'new_physics_steps': 0, 'new_Reset_FK_collision_calls': 0,
               'protected_old_files': len(protected['tracked_sha256']), 'changed_old_files': changed,
               'user_readme_unchanged': readme_ok, 'historical4731_stop': 'UNKNOWN_UNCHANGED',
               'formal_numeric': 'FAIL_UNCHANGED_1e-10m', 'geometry_material_all_front': 'UNKNOWN',
               'new_dynamics_authorized': False, 'release_approved': False,
               'scope': 'Offline classification/integration only. No native contact solve, no re-integrated motion, no grasp result.'}
    write(output/'replay.json.gz', {'actual_decisions': actual, 'negative_fixtures': negative, 'replay_stops': stops,
                                  'whole_proxy_face285_not_whitelisted': whole285,
                                  'source6175_whole_exterior': adapter.front6175, 'shared6175_6176': adapter.shared})
    write(output/'summary.json', summary)
    print(json.dumps(summary, indent=2))
    if not passed:
        raise SystemExit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    main(parser.parse_args().output.resolve())
