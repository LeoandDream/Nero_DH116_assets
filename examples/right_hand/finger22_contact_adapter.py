"""Frozen experimental runner boundary. No Reset, data allocation or dynamics.

prepare(model) once, then classify_event(record, timing, step) synchronously
BEFORE the runner's extra-stop aggregation. Does not mutate records or filters.
Only REGISTERED_LOCAL_FUNCTIONAL_APPROXIMATION permits this diagnostic to
continue; it is neither a geometry-fidelity waiver nor a grasp/release PASS.
"""
from collections import defaultdict
from contextlib import contextmanager
import gzip
import hashlib
import itertools
import json
import math
from pathlib import Path
import time

import mujoco
import numpy as np

import finger22_contact_regions as C
import finger22_static as F
from white_bowl_geometry import filter_reasons

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
OLD = HERE/'evidence/d3_finger22_dynamic'
EVIDENCE = HERE/'evidence/d3_finger22_runner_offline'
RULES = EVIDENCE/'frozen-rules.json'
RULE_SHA256 = 'f5e837f43332366af3ac964accd6107b4d58772cd185fbc16979fb1acf6f5800'
BODY = F.BODY
NAMES = F.B.NEW


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    with (gzip.open(path, 'rt') if str(path).endswith('.gz') else open(path)) as f:
        return json.load(f)


def rows(path):
    with gzip.open(path, 'rt') as f:
        for line in f:
            yield json.loads(line)


def event_snapshot(value):
    """Detached JSON-safe evidence; attaching a decision must not form cycles."""
    if isinstance(value, dict):
        return {k: event_snapshot(v) for k, v in value.items() if k != 'finger22_region'}
    if isinstance(value, (list, tuple)):
        return [event_snapshot(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return {'invalid_nonfinite': repr(value)}
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return {'unsupported_input_type': type(value).__name__, 'repr': repr(value)}


@contextmanager
def no_physics():
    """Offline verification guard; only model identity lookups are permitted."""
    originals = {n: getattr(mujoco, n) for n in dir(mujoco)
                 if (n.startswith('mj_') and callable(getattr(mujoco, n))
                     and n not in ('mj_id2name', 'mj_name2id')) or n == 'MjData'}
    calls = {}
    def blocked(name):
        def fail(*args, **kwargs):
            calls[name] = calls.get(name, 0)+1
            raise RuntimeError('Offline interface forbids '+name)
        return fail
    try:
        for n in originals:
            setattr(mujoco, n, blocked(n))
        yield calls
    finally:
        for n, function in originals.items():
            setattr(mujoco, n, function)


def finite(value, shape):
    a = np.asarray(value, dtype=float)
    if a.shape != shape or not np.isfinite(a).all():
        raise ValueError('Nonfinite or malformed array: expected '+str(shape))
    return a


def pose(value):
    r = finite(value['rotation_world'], (9,)).reshape(3, 3)
    p = finite(value['position_world_m'], (3,))
    # Structural input validation only, not a geometric acceptance tolerance.
    if np.linalg.norm(r.T@r-np.eye(3), np.inf) > 256*np.finfo(float).eps or np.linalg.det(r) <= 0:
        raise ValueError('Cached rotation is not a proper orthogonal body frame')
    return r, p


def stop_reasons(decisions, existing_reasons=()):
    """Add diagnostic stops, never replace original physical/wrist stops."""
    reasons = list(existing_reasons)
    statuses = [d.get('status', '') for d in decisions]
    if any(s.startswith('STOP_RISK') for s in statuses):
        reasons.append('FINGER22_NONFUNCTIONAL_REGION_CONTACT')
    allowed = {'NOT_APPLICABLE', 'NOT_ELIGIBLE', 'REGISTERED_LOCAL_FUNCTIONAL_APPROXIMATION'}
    if any(s not in allowed and not s.startswith('STOP_RISK') for s in statuses):
        reasons.append('FINGER22_RISK_CONTACT_UNKNOWN')
    if any(d.get('stop', True) and d.get('status') in allowed for d in decisions):
        reasons.append('FINGER22_RISK_CONTACT_UNKNOWN')
    return list(dict.fromkeys(reasons))


class ContactAdapter:
    def __init__(self, model, rules_path=RULES):
        start = time.perf_counter()
        if sha(rules_path) != RULE_SHA256 or mujoco.__version__ != '3.2.3':
            raise ValueError('Frozen rules/compiler identity mismatch')
        self.rules = read(rules_path)
        for path, digest in self.rules['frozen_inputs_sha256'].items():
            if sha(ROOT/path) != digest:
                raise ValueError('Frozen input mismatch: '+path)
        # Full native model, not a replacement robot or a reconstructed filter.
        import sys
        sys.path.insert(0, str(ROOT/'examples/libero_reference'))
        import checks
        snapshot = read(OLD/'run/robot-physical-preflight.json.gz')['actual_snapshot']
        if checks.compiled_snapshot(model) != snapshot:
            raise ValueError('Compiled runtime model differs from saved native model')
        self.model = model
        self.ids = {model.geom(i).name: i for i in range(model.ngeom)}
        self.hulls = []
        old = read(HERE/'evidence/d3_finger22_contact_regions/evidence.json.gz')
        for name in NAMES:
            binding = F.bindings(F.CANDIDATE, [name])[0]
            h = F.P.compiled_hull(model, name, binding)
            digest = hashlib.sha256(h['vertices'].tobytes()).hexdigest()
            if digest != old['hull_certificates'][name]['support_vertices_sha256']:
                raise ValueError('Actual compiled support geometry mismatch: '+name)
            self.hulls.append(C.CertifiedHull(h['vertices']))
        self.source = F.P.geometry.read_stl(F.SOURCE)
        witness = read(HERE/'evidence/d3_finger22_risk_review/witnesses.json')
        self.witness = witness
        t = np.array(witness['buried_source']['compiled_nearest_support_triangle_m'])
        n = np.cross(t[1]-t[0], t[2]-t[0]); n /= np.linalg.norm(n)
        self.cavity = [np.r_[n, -n@t[0]].tolist()]
        edges = defaultdict(list)
        for i, t in enumerate(self.source):
            for a, b in zip(t, np.roll(t, -1, axis=0)):
                edges[tuple(sorted((tuple(a), tuple(b))))].append(i)
        self.shared = [v for v in edges.values() if 6175 in v and 6176 in v]
        self.front6175 = C.source_front_certificate(self.source[6175], self.source, 6175, 0.)
        if self.shared != [[6175, 6176]] or not self.front6175['certified']:
            raise ValueError('Frozen6175 exterior/manifold registration certificate invalid')
        self.setup_seconds = time.perf_counter()-start

    def classify_patch(self, candidate, part, guard):
        p = candidate['polygon_exact_m']
        base = C.semantic_patch(p, part, self.source[[6165, 6176]],
                                self.rules['cuts_local_z_m'], guard, self.cavity)
        if 'RISK_ENVELOPE' in base['status']:
            return base
        # Every possible patch needs BOTH full-domain coverage/outward position
        # and a full-source exterior proof. No normal or facet-ID whitelist.
        for source_id in self.rules['registered_source_faces_zero_based']:
            provenance = C.semantic_patch(p, part, self.source[[source_id]],
                                          self.rules['cuts_local_z_m'], guard, self.cavity)
            if provenance['status'] != 'FUNCTIONAL_SOURCE_PATCH_APPROXIMATION':
                continue
            exterior = C.source_front_certificate(p, self.source, source_id, guard)
            if exterior['certified'] and (source_id != 6175 or
                    (self.front6175['certified'] and self.shared == [[6175, 6176]])):
                return {**provenance, 'source_face': source_id, 'exterior_certificate': exterior,
                        'local_patch_only': True, 'geometry_fidelity_waiver': False}
        return {'status': 'UNKNOWN_NO_COMPLETE_REGISTERED_LOCAL_SOURCE_PROOF', 'initial_semantic': base}

    def geometry(self, part, box, arithmetic_guard):
        certificate = C.intersection_certificate(self.hulls[part], box, arithmetic_guard)
        candidates = C.contact_faces(self.hulls[part], box, arithmetic_guard)
        guard = max(arithmetic_guard, self.rules['semantic_guard_m'])
        result = [{**c, **self.classify_patch(c, part, guard)} for c in candidates]
        decision = C.decision([c['status'] for c in result])
        if decision == 'FUNCTIONAL_PROVENANCE_ONLY_NOT_RELEASE_PASS':
            status = ('REGISTERED_LOCAL_FUNCTIONAL_APPROXIMATION' if certificate['status'] == 'STRICT_OVERLAP'
                      else 'STOP_UNKNOWN_NO_STRICT_INTERSECTION_CERTIFICATE')
        else:
            status = decision
        # No patch (e.g. containment) never silently becomes a functional face.
        return {'status': status, 'stop': status.startswith('STOP_'), 'candidates': result,
                'solid_certificate': certificate}

    def box(self, gid, finger_pose, bowl_pose):
        """Same frozen affine construction/roundoff envelope as cab4305."""
        rf, pf = pose(finger_pose); r, p = pose(bowl_pose)
        lr = F.P.reader.BUILDER.quaternion_matrix(' '.join(map(str, self.model.geom_quat[gid])))
        lp = self.model.geom_pos[gid]; size = self.model.geom_size[gid]
        rr = rf.T@r@lr; tt = rf.T@(r@lp+p-pf)
        v = np.array(list(itertools.product((-1, 1), repeat=3)))*size
        unit = 2.**-53; gamma = 256*unit/(1-256*unit)
        scale = (1+max(np.max(abs(p)), np.max(abs(pf)))+np.max(size)+np.max(abs(lp)))
        scale *= max(1., np.linalg.norm(rf, np.inf))*max(1., np.linalg.norm(r, np.inf))*max(1., np.linalg.norm(lr, np.inf))
        orth = np.linalg.norm(rf.T@rf-np.eye(3), np.inf)
        guard = float(2*(gamma+orth)*scale)
        exact = C.affine_vertices(v, rr, tt)
        return C.CertifiedHull(exact, exact), guard

    def classify_event(self, event, timing, step, body_poses=None):
        start = time.perf_counter()
        try:
            names = event['geom']
            if not isinstance(names, list) or len(names) != 2:
                raise ValueError('Contact geom pair malformed')
            if not any(n in NAMES for n in names):
                return {'status': 'NOT_APPLICABLE', 'stop': False}
            if timing not in (self.rules['runner_boundary']['rk4_timing'],
                              self.rules['runner_boundary']['initial_timing']):
                raise ValueError('No certified RK4/initial cache timing')
            if not isinstance(step, int) or step < 0:
                raise ValueError('Invalid physical step identity')
            ids = [self.ids[n] for n in names]
            bodies = [self.model.body(int(self.model.geom_bodyid[g])).name for g in ids]
            if event['body'] != bodies:
                raise ValueError('Counterpart/geom/body mismatch')
            reasons = filter_reasons(self.model, *ids)
            if event['filter_reasons'] != reasons:
                raise ValueError('Recorded contact eligibility mismatch')
            if reasons:
                return {'status': 'NOT_ELIGIBLE', 'stop': False, 'filter_reasons': reasons}
            i = next(k for k, n in enumerate(names) if n in NAMES); j = 1-i
            if bodies[i] != BODY or bodies[j] != 'white_bowl_1_main':
                raise ValueError('Unsupported counterpart; no semantic registration')
            if int(self.model.geom_type[ids[j]]) != int(mujoco.mjtGeom.mjGEOM_BOX):
                raise ValueError('Counterpart is not the complete native bowl box')
            finite(event['distance_m'], ())
            finite(event['position_world_m'], (3,))  # Retained, never selects a face.
            if timing == self.rules['runner_boundary']['rk4_timing']:
                frame = finite(event['frame'], (9,)).reshape(3, 3)
                pose({'rotation_world': frame.ravel(), 'position_world_m': [0., 0., 0.]})
                if not np.array_equal(finite(event['normal_world'], (3,)), frame[0]):
                    raise ValueError('Normal/frame mismatch')
                finite(event['force_contact_frame'], (6,))
            poses = event['cached_body_pose']
            if len(poses) != 2:
                raise ValueError('Missing per-contact cache poses')
            for body, p in zip(bodies, poses):
                pose(p)
                if body_poses is not None and p != body_poses[body]:
                    raise ValueError('Contact-local and per-step RK4 poses disagree')
            box, guard = self.box(ids[j], poses[i], poses[j])
            result = self.geometry(NAMES.index(names[i]), box, guard)
            result.update(rule_sha256=RULE_SHA256, step=step, native_distance_m=event['distance_m'],
                          geometry_arithmetic_guard_m=guard, native_event=event_snapshot(event),
                          cache_timing=timing, physical_thresholds_unchanged=True)
            if not result['stop']:
                query = F.P.geometry.surface_query(self.source, box.vertices[box.faces])
                result['source_complete_box_gap_m'] = query['surface_min_distance_m']
            return {**result, 'latency_seconds': time.perf_counter()-start}
        except Exception as exc:
            return {'status': 'STOP_UNKNOWN_INPUT_OR_PROOF', 'stop': True,
                    'rule_sha256': RULE_SHA256, 'step': step,
                    'reason': type(exc).__name__+': '+str(exc),
                    'native_event': event_snapshot(event), 'latency_seconds': time.perf_counter()-start}

    def classify_step(self, cache):
        try:
            decisions = [self.classify_event(c, cache['timing'], cache['step'], cache['body_poses'])
                         for c in cache['contacts']]
            reasons = stop_reasons(decisions)
            return {'step': cache['step'], 'stop': bool(reasons),
                    'reasons': reasons, 'decisions': decisions}
        except Exception as exc:
            return {'step': cache.get('step'), 'stop': True, 'status': 'STOP_UNKNOWN_STEP_SCHEMA',
                    'reason': type(exc).__name__+': '+str(exc)}

    def fixture_specs(self):
        return [('root_bore', 0, self.witness['reverse_proxy']['same_witness_compiled_boundary_check']['compiled_nearest_point_m'], 1e-4),
                ('cavity_closure', 1, self.witness['buried_source']['compiled_nearest_surface_point_m'], 1e-4)]


def prepare_saved_model():
    """Compile the unchanged saved native scene; resource paths only relocated.

    No XML is written, no body/geom field changed. Runtime integration instead
    passes its already-compiled model to ContactAdapter(model) before stepping.
    """
    xml = (OLD/'run/model.xml').read_text()
    prefix = '/data_robot/verification/nero_dh116_right/'
    # Dependencies remain at their verified prefix; checkout-owned assets map
    # to this worktree. This is an offline loader, never a runtime model edit.
    import re
    xml = re.sub(re.escape(prefix)+r'(?!\.reference-deps/)', str(ROOT)+'/', xml)
    model = mujoco.MjModel.from_xml_string(xml)
    # Native Reset changed these fixed-scene placements after XML creation.
    # Restore their SAVED model fields in memory, never execute Reset/FK. Every
    # named compiled field is then checked for exact equality by the adapter.
    snapshot = read(OLD/'run/robot-physical-preflight.json.gz')['actual_snapshot']
    placements = {b['name']: b['fields'] for b in snapshot['objects']['BODY']}
    for name in ('flat_stove_1_main', 'wooden_two_layer_shelf_1_main'):
        bid = model.body(name).id
        for field in ('body_pos', 'body_quat'):
            getattr(model, field)[bid] = placements[name][field]
    return ContactAdapter(model)
