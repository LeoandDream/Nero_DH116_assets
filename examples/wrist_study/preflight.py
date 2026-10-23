"""Full saved L task path plus bounded old R wrist witnesses; static only."""
import argparse
import copy
import gzip
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
import compare as C

ROOT = Path(__file__).resolve().parents[2]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--history-root', type=Path, required=True)
    p.add_argument('--left-run', type=Path, required=True)
    p.add_argument('--candidate', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    sys.path.insert(0, str(a.history_root/'examples/right_hand'))
    import wrist_reuse as W
    scene = C.absolute(ET.parse(a.left_run/'bounded_grasp-model.xml').getroot(), a.left_run)
    base = C.absolute(ET.parse(ROOT/'mjcf/nero_dh116.xml').getroot(), ROOT/'mjcf')
    candidate = C.absolute(ET.parse(a.candidate).getroot(), a.candidate.parent)
    C.rebind_robot_assets(scene, base)
    trees = [C.replace_wrist(scene, r) for r in (base, candidate)]
    models = [mj.MjModel.from_xml_string(ET.tostring(x, encoding='unicode')) for x in trees]
    reset = C.read(a.left_run/'full-reset-state.json')
    for m in models:
        for name, pos, quat in zip(reset['body_names'], reset['runtime_body_pos'], reset['runtime_body_quat']):
            m.body_pos[m.body(name).id], m.body_quat[m.body(name).id] = pos, quat
    old_names, new_names = [[g.get('name') for g in C.wrists(x)] for x in trees]
    audit = W.compiled_audit(models[0], models[1], old_names, new_names)
    # Candidate graph/compiled data are compared to the delivered right306
    # diagnostic representation, without copying any delivered OBJ to build it.
    right = mj.MjModel.from_xml_path(str(ROOT/'mjcf/nero_dh116_right_research.xml'))
    pairs = []
    manifest = C.read(a.candidate.parent/'manifest.json')
    for row in manifest['meshes']:
        label = Path(row['file']).stem
        i, j = right.geom('robot0_wrist_reuse_'+label).id, models[1].geom(row['geom']).id
        mi, mjid = int(right.geom_dataid[i]), int(models[1].geom_dataid[j])
        exact = True
        for field, adr, num in [('mesh_vert', 'mesh_vertadr', 'mesh_vertnum'),
                                ('mesh_face', 'mesh_faceadr', 'mesh_facenum')]:
            s, t = int(getattr(right, adr)[mi]), int(getattr(models[1], adr)[mjid])
            n, k = int(getattr(right, num)[mi]), int(getattr(models[1], num)[mjid])
            exact &= n == k and np.array_equal(getattr(right, field)[s:s+n], getattr(models[1], field)[t:t+k])
        for field in ('geom_pos', 'geom_quat'):
            exact &= np.array_equal(getattr(right, field)[i], getattr(models[1], field)[j])
        pairs.append({'candidate': row['geom'], 'compiled_geometry_exact_to_diagnostic306': bool(exact)})
    C.write(a.output/'compiled-audit.json', audit)
    C.write(a.output/'reconstruction-equivalence.json', pairs)
    data = [mj.MjData(m) for m in models]
    wrist_ids = [{m.geom(g.get('name')).id for g in C.wrists(x)} for m, x in zip(models, trees)]
    rows, anomalies = [], []
    frozen = ROOT/'examples/libero_reference/evidence/d2/bounded_grasp.jsonl.gz'
    with gzip.open(frozen, 'rt') as stream:
        saved = [json.loads(line) for line in stream]
    witnesses = []
    for index, row in enumerate(saved, 1):
        witnesses.append(('L_actual_saved_post', index, row['full_qpos']))
    protocol = C.read(Path(__file__).with_name('protocol.json'))
    source = a.history_root/'examples/libero_eval/evidence/functional_v1/raw/state.jsonl.gz'
    with gzip.open(source, 'rt') as stream:
        for line in stream:
            row = json.loads(line)
            if row['step'] in protocol['static_steps']['R']:
                # Same NERO arm angles, L's own open hand and settled scene.
                # This is a synthetic wrist witness, not a historical L state.
                q = saved[249]['full_qpos'].copy()
                q[:7] = row['full_qpos'][:7]
                witnesses.append(('R_arm_witness_with_L_open_hand', row['step'], q))
    for role, step, qpos in witnesses:
        entry = {'role': role, 'step': step, 'contact_counts': []}
        for count, model, d, ids in zip((15, 306), models, data, wrist_ids):
            d.qpos[:] = qpos
            before = d.time
            mj.mj_kinematics(model, d)
            mj.mj_collision(model, d)
            assert d.time == before and np.array_equal(d.qpos, qpos)
            contacts = []
            for c in d.contact:
                if c.geom1 in ids or c.geom2 in ids:
                    contacts.append({'geom': [model.geom(int(g)).name for g in (c.geom1, c.geom2)],
                                     'body': [model.body(int(model.geom_bodyid[g])).name for g in (c.geom1, c.geom2)],
                                     'distance_m': float(c.dist), 'position_m': c.pos.tolist()})
            entry['contact_counts'].append(len(contacts))
            if contacts:
                anomalies.append({'role': role, 'step': step, 'count': count, 'contacts': contacts})
        rows.append(entry)
    blocked = [r for r in anomalies if r['count'] == 306]
    C.write(a.output/'states.json.gz', {'rows': rows, 'anomalies': anomalies})
    result = {'status': 'PASS_FINITE_TASK_PREFLIGHT' if not blocked else 'BLOCKED_STATIC_CONTACT',
              'candidate_306_anomaly_states': len(blocked), 'left_actual_saved_post_states': len(saved),
              'old_R_arm_witnesses': len(witnesses)-len(saved),
              'baseline15_R_witness_contact_states': sum(r['count']==15 and r['role'].startswith('R_') for r in anomalies),
              'compiled_geometry_exact_to_original306': sum(r['compiled_geometry_exact_to_diagnostic306'] for r in pairs),
              'candidate_model_sha256': C.sha(a.candidate), 'mj_kinematics_calls': 2*len(witnesses),
              'mj_collision_calls': 2*len(witnesses), 'explicit_physics_steps': 0, 'mj_forward_calls': 0,
              'limitations': ['Finite post-state geometry, not original RK4 contact caches or continuous trajectory proof.',
                             'Original 0.300mm/2.17mm approximations and high-fidelity FAIL/UNKNOWN remain.']}
    C.write(a.output/'result.json', result)
    print(json.dumps(result), flush=True)
    raise SystemExit(0 if not blocked else 1)


if __name__ == '__main__':
    main()
