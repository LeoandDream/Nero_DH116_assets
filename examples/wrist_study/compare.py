"""Issue #10: bounded same-pose contact queries and real mj_step microbenchmarks.

Existing 15/306 representations are diagnostic controls, not a new candidate.
Frozen ctrl playback isolates step cost; it is not a closed-loop counterfactual.
"""
import argparse
from collections import Counter
import copy
import gzip
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import statistics
import sys
import time
import xml.etree.ElementTree as ET

for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
import mujoco as mj
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
BODIES = ('robot0_link5', 'robot0_gripper_flange')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    with (gzip.open if str(path).endswith('.gz') else open)(path, 'rt') as f:
        return json.load(f)


def write(path, value):
    with (gzip.open if str(path).endswith('.gz') else open)(path, 'xt') as f:
        json.dump(value, f, indent=2, allow_nan=False)
        f.write('\n')


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def wrists(xml):
    return [g for body in BODIES for g in xml.find('.//body[@name="'+body+'"]').findall('geom')
            if int(g.get('contype', '1')) or int(g.get('conaffinity', '1'))]


def absolute(xml, parent):
    for e in xml.iter():
        if e.get('file'):
            e.set('file', str((parent/e.get('file')).resolve()))
    return xml


def rebind_robot_assets(scene, delivered):
    """Resolve archived robot paths to the SHA-indexed delivered resources.

    Only file locations change. If an old path still exists, verify its bytes.
    Missing historical paths are not silently taken from a different geometry.
    """
    mapping = []
    for e in scene.findall('asset/*'):
        if not e.get('file'):
            continue
        source = delivered.find('asset/'+e.tag+'[@name="'+str(e.get('name'))+'"]')
        if source is None or not source.get('file'):
            continue
        old, new = Path(e.get('file')), Path(source.get('file'))
        if old.exists() and sha(old) != sha(new):
            raise ValueError('Archived/delivered bytes differ: '+str(old))
        mapping.append({'name': e.get('name'), 'old': str(old), 'file': str(new), 'sha256': sha(new)})
        e.set('file', str(new))
    return mapping


def replace_wrist(scene, representation):
    """Diagnostic swap only; no transform, hand, physical or filter changes."""
    result = copy.deepcopy(scene)
    old = wrists(result)
    names = {g.get('mesh') for g in old}
    for body in BODIES:
        node = result.find('.//body[@name="'+body+'"]')
        for g in list(node.findall('geom')):
            if g in old:
                node.remove(g)
    for a in list(result.find('asset')):
        if a.tag == 'mesh' and a.get('name') in names:
            result.find('asset').remove(a)
    for body in BODIES:
        source_body = representation.find('.//body[@name="'+body+'"]')
        for g in source_body.findall('geom'):
            if g not in wrists(representation):
                continue
            mesh = representation.find('asset/mesh[@name="'+g.get('mesh')+'"]')
            result.find('asset').append(copy.deepcopy(mesh))
            result.find('.//body[@name="'+body+'"]').append(copy.deepcopy(g))
    return result


def states(path, side, needed):
    result = {}
    with gzip.open(path, 'rt') as f:
        for i, line in enumerate(f, 1):
            r = json.loads(line)
            step = i if side == 'L' else r['step']
            if step in needed:
                result[step] = {'step': step, 'qpos': r['full_qpos'], 'qvel': r['full_qvel'],
                    'ctrl': r['internal_actuator_ctrl_13'] if side == 'L' else r['ctrl'],
                    'phase': r['phase'], 'action': r['robot_action_13'] if side == 'L' else r['action_13'],
                    'time': r['time'] if side == 'L' else r['time_s']}
    if set(result) != needed:
        raise ValueError('Missing frozen states')
    return result


def initialize(model, row):
    data = mj.MjData(model)
    data.qpos[:] = row['qpos']
    data.qvel[:] = row['qvel']
    data.ctrl[:] = row['ctrl']
    data.time = row['time']
    mj.mj_forward(model, data)
    return data


def contacts(model, data):
    rows = []
    jid = sorted((i for i in range(model.njnt) if model.joint(i).name.startswith('robot0_')),
                 key=lambda i: model.jnt_qposadr[i])[5]
    dof = int(model.jnt_dofadr[jid])
    torque = 0.
    for index, c in enumerate(data.contact):
        bodies = [int(model.geom_bodyid[g]) for g in (c.geom1, c.geom2)]
        names = [model.body(b).name for b in bodies]
        if not any(b in BODIES for b in names):
            continue
        force = np.zeros(6)
        mj.mj_contactForce(model, data, index, force)
        rotation = c.frame.reshape(3, 3)
        linear = rotation.T @ force[:3]
        angular = rotation.T @ force[3:]
        jac = []
        for b in bodies:
            jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
            mj.mj_jac(model, data, jp, jr, c.pos, b)
            jac.append((jp[:, dof], jr[:, dof]))
        tau = float((jac[1][0]-jac[0][0]) @ linear + (jac[1][1]-jac[0][1]) @ angular)
        torque += tau
        rows.append({'geom': [model.geom(int(g)).name for g in (c.geom1, c.geom2)],
                     'body': names, 'distance_m': float(c.dist), 'position_m': c.pos.tolist(),
                     'frame': c.frame.tolist(), 'force_contact_frame': force.tolist(),
                     'J6_pair_generalized_torque_Nm': tau})
    return rows, torque, dof


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--history-root', type=Path, required=True)
    p.add_argument('--left-run', type=Path, required=True)
    p.add_argument('--issue9-evidence', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    protocol = read(Path(__file__).with_name('protocol.json'))
    write(a.output/'protocol.json', protocol)
    history = a.history_root.resolve()
    sys.path.insert(0, str(history/'examples/right_hand'))
    import wrist_reuse as existing
    surface = module('source_triangles', history/'examples/right_hand/evidence/d3_geometry/triangle/surface_geometry.py')
    archive = history/'examples/libero_eval/evidence/functional_v1/raw'
    sources = {'L': a.left_run/'bounded_grasp-model.xml', 'R': archive/'model.xml'}
    traces = {'L': ROOT/'examples/libero_reference/evidence/d2/bounded_grasp.jsonl.gz',
              'R': archive/'state.jsonl.gz'}
    paths = [Path(__file__), Path(__file__).with_name('protocol.json'), *sources.values(),
             *traces.values(), a.issue9_evidence, a.left_run/'full-reset-state.json',
             archive/'robot-physical-preflight.json.gz',
             ROOT/'mjcf/nero_dh116.xml', ROOT/'mjcf/nero_dh116_right_research.xml']
    inputs = {str(path): sha(path) for path in paths}
    write(a.output/'inputs-before.json', inputs)
    models, xmls, records, audits, compile_time = {}, {}, {}, {}, {}
    reprs = {count: absolute(ET.parse(ROOT/relative).getroot(), (ROOT/relative).parent)
             for count, relative in [(15, Path('mjcf/nero_dh116.xml')),
                                     (306, Path('mjcf/nero_dh116_right_research.xml'))]}
    old = read(a.issue9_evidence)
    if old['verdict'] != 'BLOCKED_STATIC_INTERFERENCE':
        raise ValueError('Unexpected #9 input')
    for side in ('L', 'R'):
        needed = set(protocol['static_steps'][side])
        for window in protocol['benchmark']['windows']:
            if window['side'] == side:
                start = window['after_step']
                needed.update(range(start, start+protocol['benchmark']['steps_per_window']+1))
        records[side] = states(traces[side], side, needed)
        scene = absolute(ET.parse(sources[side]).getroot(), sources[side].parent)
        mapping = rebind_robot_assets(scene, reprs[15 if side == 'L' else 306])
        write(a.output/('resource-map-'+side+'.json'), mapping)
        models[side] = {}
        for count in (15, 306):
            xml = replace_wrist(scene, reprs[count])
            xmls[side, count] = xml
            for e in xml.iter():
                if e.get('file') and not Path(e.get('file')).is_file():
                    raise FileNotFoundError(e.get('file'))
            start = time.perf_counter()
            model = mj.MjModel.from_xml_string(ET.tostring(xml, encoding='unicode'))
            compile_time[side+str(count)] = time.perf_counter()-start
            if side == 'L':
                reset = read(a.left_run/'full-reset-state.json')
                poses = zip(reset['body_names'], reset['runtime_body_pos'], reset['runtime_body_quat'])
            else:
                snapshot = read(archive/'robot-physical-preflight.json.gz')['actual_snapshot']
                poses = [(b['name'], b['fields']['body_pos'], b['fields']['body_quat'])
                         for b in snapshot['objects']['BODY']]
            for name, position, quat in poses:
                bid = model.body(name).id
                model.body_pos[bid], model.body_quat[bid] = position, quat
            models[side][count] = model
        audits[side] = existing.compiled_audit(models[side][15], models[side][306],
            [g.get('name') for g in wrists(xmls[side, 15])],
            [g.get('name') for g in wrists(xmls[side, 306])])
    write(a.output/'compiled-audit.json', audits)
    static = []
    triangle_sources = [surface.read_stl(ROOT/'meshes/nero/source'/name)
                       for name in ('link5-dc497f5a.stl', 'gripper_flange-186697a7.stl')]
    for side in ('L', 'R'):
        for step in protocol['static_steps'][side]:
            for count in (15, 306):
                model = models[side][count]
                data = initialize(model, records[side][step])
                hits, tau, dof = contacts(model, data)
                row = {'side': side, 'after_step': step, 'count': count,
                       'timing': 'mj_forward at saved post-step qpos/qvel, NOT original RK4 cache',
                       'contacts': hits, 'J6_wrist_generalized_torque_Nm': tau,
                       'J6_total_constraint_Nm': float(data.qfrc_constraint[dof]),
                       'total_scene_contacts': data.ncon}
                if count == 15 and side == 'R' and step in (1401, 2250, 6500):
                    i, j = (model.body(b).id for b in BODIES)
                    ri, rj = data.xmat[i].reshape(3, 3), data.xmat[j].reshape(3, 3)
                    rotation, translation = ri.T@rj, ri.T@(data.xpos[j]-data.xpos[i])
                    row['full_manufacturer_surface_query'] = surface.surface_query(
                        triangle_sources[0], triangle_sources[1]@rotation.T+translation)
                static.append(row)
    write(a.output/'static.json.gz', static)
    # Freeze a separate benchmark ledger before the first new physical step.
    write(a.output/'benchmark-start.json', {'protocol_sha256': sha(Path(__file__).with_name('protocol.json')),
        'script_sha256': sha(__file__), 'purpose': 'registered short ctrl-playback microbenchmark, NOT functional rollout'})
    measurements, events = [], []
    benchmark_steps = 0
    for window in protocol['benchmark']['windows']:
        side, initial = window['side'], window['after_step']
        length = protocol['benchmark']['steps_per_window']
        jobs = [(-1, count, protocol['benchmark']['warmup_steps_per_representation']) for count in (15, 306)]
        jobs += [(rep, count, length) for rep, order in enumerate(protocol['benchmark']['orders']) for count in order]
        for rep, count, steps in jobs:
            model = models[side][count]
            data = initialize(model, records[side][initial])
            elapsed = 0
            observed, stop = [], None
            for offset in range(1, steps+1):
                command = records[side][initial+offset]
                data.ctrl[:] = command['ctrl']
                before = float(data.time)
                start = time.perf_counter_ns()
                mj.mj_step(model, data)
                elapsed += time.perf_counter_ns()-start
                benchmark_steps += 1
                hits, tau, dof = contacts(model, data)
                observed.append({'offset': offset, 'qpos': data.qpos.tolist(), 'qvel': data.qvel.tolist(),
                    'ctrl': data.ctrl.tolist(), 'J6_constraint_Nm': float(data.qfrc_constraint[dof]),
                    'J6_wrist_pair_torque_Nm': tau, 'wrist_contacts': hits, 'ncon': data.ncon,
                    'timing': 'qpos/qvel post-step; contacts/forces/geometry final RK4 internal-stage cache'})
                if any(w.number for w in data.warning) or not all(np.isfinite(x).all() for x in
                    (data.qpos, data.qvel, data.qacc, data.ctrl)) or abs(data.time-before-.002) > 1e-12:
                    stop = 'NUMERICAL_WARNING_NONFINITE_OR_TIME'
                    break
            measurements.append({'side': side, 'after_step': initial, 'count': count, 'rep': rep,
                'physics_steps': len(observed), 'mj_step_total_ns': elapsed,
                'mean_step_us': elapsed/len(observed)/1000, 'warmup': rep < 0,
                'max_ncon': max(r['ncon'] for r in observed),
                'max_abs_J6_wrist_pair_torque_Nm': max(abs(r['J6_wrist_pair_torque_Nm']) for r in observed),
                'stop_reason': stop})
            events.append({'side': side, 'after_step': initial, 'count': count, 'rep': rep, 'states': observed})
            print('benchmark', measurements[-1], flush=True)
            if stop:
                write(a.output/'benchmark-partial.json.gz', events)
                raise RuntimeError(stop)
    summary = []
    for window in protocol['benchmark']['windows']:
        item = dict(window)
        for count in (15, 306):
            runs = [r for r in measurements if r['side'] == window['side'] and
                    r['after_step'] == window['after_step'] and r['count'] == count and not r['warmup']]
            item[str(count)+'_median_step_us'] = statistics.median(r['mean_step_us'] for r in runs)
        item['306_to_15_time_ratio'] = item['306_median_step_us']/item['15_median_step_us']
        summary.append(item)
    write(a.output/'benchmark.json.gz', {'environment': {'python': sys.version, 'mujoco': mj.__version__,
        'numpy': np.__version__, 'machine': platform.platform(), 'cpu': Path('/proc/cpuinfo').read_text().split('model name')[1].split('\n')[0],
        'affinity': sorted(os.sched_getaffinity(0)), 'compile_seconds': compile_time},
        'measurements': measurements, 'summary': summary, 'events': events, 'physics_steps': benchmark_steps})
    if any(sha(path) != digest for path, digest in inputs.items()):
        raise ValueError('A frozen input changed')
    write(a.output/'result.json', {'status': 'COMPARISON_RECORDED', 'static_pose_count': 26,
        'static_calls': len(static), 'benchmark_physics_steps': benchmark_steps, 'functional_rollouts': 0,
        'candidate_created': False, 'summary': summary, 'input_sha256_unchanged': inputs})


if __name__ == '__main__':
    main()
