#!/usr/bin/env python3
"""Generate one right-specific candidate with FK only; never run an episode.

One bounded least-squares solve derives a thumb/index pinch from the delivered
right STL pads and native bowl wall diameter. One continuation solves the arm
along the vertical approach/lift. No collision query, alternate seed, dynamic
feedback, left action or accepted left closure angle is used in planning.
"""
import argparse
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from white_bowl_geometry import geometry_only, load_module, sha, MODEL, ROOT, SURFACE

FORMAL_SHA = '739d3d29bbfd4a6468fb8b75f4de09b719c0f96abb39508959c1dfc33a84b9cc'
LIBERO_COMMIT = 'f78abd68ee283de9f9be3c8f7e2a9ad60246e95c'
ACTIVE = np.array([7, 8, 10, 12, 14, 16])


def hand_state(active):
    """Source URDF linear coupling, in the formal right model's joint order."""
    a, b, c, d, e, f = active
    return np.array([a, b, .004592 + .8808*b, c, .011072 + .9206*c,
                     d, .011072 + .9206*d, e, .011072 + .9206*e,
                     f, .011072 + .9206*f])


def generate(output, libero):
    output, libero = Path(output).resolve(), Path(libero).resolve()
    if output.exists():
        raise ValueError('Refuse to overwrite an earlier candidate')
    if sha(MODEL) != FORMAL_SHA or mujoco.__version__ != '3.2.3':
        raise ValueError('Formal model or MuJoCo version differs')
    commit = subprocess.check_output(['git', '-C', str(libero), 'rev-parse', 'HEAD'], text=True).strip()
    if commit != LIBERO_COMMIT:
        raise ValueError('Pinned LIBERO revision differs')
    assets = libero / 'libero/libero/assets/stable_scanned_objects/white_bowl'
    bowl_path = assets / 'white_bowl.xml'
    task = libero / 'libero/libero/bddl_files/libero_90/KITCHEN_SCENE9_put_the_white_bowl_on_top_of_the_cabinet.bddl'
    # This explicit historical position is a planning reference, never state
    # injection or a new settled-state measurement. A future trial must reset
    # natively, settle under gravity, and record its own baseline.
    left_plan = ROOT / 'examples/libero_reference/fixtures/prepared-white-geometry-plan.json'
    settled = np.array(json.loads(left_plan.read_text())['actual_settled_object_position_m'])
    bowl_xml = ET.parse(bowl_path).getroot()
    walls = [g for g in bowl_xml.findall('.//geom') if g.get('type') == 'box'
             and float(g.get('pos').split()[2]) > .02]
    centers = np.array([np.fromstring(g.get('pos'), sep=' ') for g in walls])
    wall_height = float(np.median(centers[:, 2]))
    # Outer upper-wall radius from its center and normal half thickness. This
    # task uses outer opposing wall contact, not a copied left pad target.
    radius = float(np.median(np.linalg.norm(centers[:, :2], axis=1)
                            + [float(g.get('size').split()[0]) for g in walls]))
    gap = 2*radius - .001  # New candidate's 0.5mm intended compression per side.
    xml = ET.parse(MODEL).getroot()
    surface = load_module('right_bowl_planning_stl', SURFACE)
    features, files = {}, {MODEL, Path(__file__), SURFACE, left_plan, bowl_path, task}
    files.update((MODEL.parent / n.get('file')).resolve()
                 for n in xml.findall('asset/*') if n.get('file'))
    for name, side in [('finger14_link', -1), ('finger23_link', 1)]:
        path = (MODEL.parent / xml.find(f'asset/mesh[@name="gripper0_{name}"]').get('file')).resolve()
        triangles = surface.read_stl(path)
        centers_local = triangles.mean(axis=1)
        lo, hi = triangles.min(axis=(0, 1)), triangles.max(axis=(0, 1))
        desired = (lo + hi)/2
        desired[0] = lo[0] if side < 0 else hi[0]
        index = int(np.argmin(np.linalg.norm(centers_local - desired, axis=1)))
        features[name] = {'face_index_zero_based': index, 'point_body_local_m': centers_local[index].tolist(),
                          'selection': 'Triangle centroid nearest inward-X pad bounding-box face center',
                          'mesh': str(path.relative_to(ROOT)), 'sha256': sha(path)}
    before = {str(path): sha(path) for path in sorted(files)}
    with geometry_only() as (counts, blocked):
        model = mujoco.MjModel.from_xml_path(str(MODEL))
        data = mujoco.MjData(model)
        initial = model.key_qpos[0].copy()
        limits = model.jnt_range[ACTIVE]
        base_id = model.body('gripper0_base_link').id
        bids = [model.body('gripper0_'+name).id for name in features]
        locals_ = [np.array(f['point_body_local_m']) for f in features.values()]

        def pads(active):
            data.qpos[:] = initial
            data.qpos[7:] = hand_state(active)
            mujoco.mj_kinematics(model, data)
            base_r = data.xmat[base_id].reshape(3, 3)
            return np.array([base_r.T @ (data.xmat[b].reshape(3, 3) @ p + data.xpos[b] - data.xpos[base_id])
                             for b, p in zip(bids, locals_)])

        # Three geometric equations / three thumb-index channels. Equal
        # fractional flex travel is a new geometric planning convention, not
        # a controller, calibrated force, historical acceptance bound or sweep.
        def pinch_residual(q):
            a = np.r_[q, 0., 0., 0.]
            thumb, index = pads(a)
            return [(np.linalg.norm(thumb-index)-gap)*100,
                    (thumb[1]-index[1])*100,
                    q[1]/limits[1, 1] - q[2]/limits[2, 1]]

        fit = least_squares(pinch_residual, limits[:3, 1]*.25,
                            bounds=(limits[:3, 0], limits[:3, 1]),
                            xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=1000)
        final_pinch_residual = pinch_residual(fit.x)
        closed = np.r_[fit.x, 0., 0., 0.]
        opened = np.r_[fit.x[0], 0., 0., 0., 0., 0.]
        thumb, index = pads(closed)
        local_midpoint = (thumb+index)/2
        u = (thumb-index)/np.linalg.norm(thumb-index)
        v = np.array([0., 1., 0.])-u*u[1]
        v /= np.linalg.norm(v)
        local_frame = np.column_stack([u, v, np.cross(u, v)])
        data.qpos[:] = initial
        mujoco.mj_kinematics(model, data)
        # Preserve initial horizontal pinch heading; turn the pad line level
        # and the palm-side approach downwards using right-pad geometry.
        world_u = data.xmat[base_id].reshape(3, 3) @ u
        world_u[2] = 0
        world_u /= np.linalg.norm(world_u)
        world_w = np.array([0., 0., -1.])
        target_r = np.column_stack([world_u, np.cross(world_w, world_u), world_w]) @ local_frame.T
        contact_midpoint = settled + [0., 0., wall_height]
        flange_target = contact_midpoint - target_r @ local_midpoint
        arm_limits = model.jnt_range[:7]
        nodes, seed = [], initial[:7].copy()
        for node in range(11):
            target = flange_target + [0., 0., .01*node]
            origin_seed = seed.copy()

            def arm_residual(q):
                data.qpos[:] = np.r_[q, hand_state(closed)]
                mujoco.mj_kinematics(model, data)
                actual_r = data.xmat[base_id].reshape(3, 3)
                return np.r_[100*(data.xpos[base_id]-target),
                             Rotation.from_matrix(target_r @ actual_r.T).as_rotvec(),
                             .00001*(q-origin_seed)]

            ik = least_squares(arm_residual, seed, bounds=(arm_limits[:, 0], arm_limits[:, 1]),
                               xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=1500)
            error = arm_residual(ik.x)
            nodes.append({'height_offset_m': .01*node, 'arm_qpos_rad': ik.x.tolist(),
                          'position_error_m': float(np.linalg.norm(error[:3])/100),
                          'orientation_error_rad': float(np.linalg.norm(error[3:6])),
                          'solver_success': bool(ik.success)})
            seed = ik.x.copy()

        actions = []
        for macro in range(230):
            if macro < 10:
                phase, arm = 'settle_preshape', initial[:7]
                hand = opened*((macro+1)/10)
            elif macro < 50:
                phase, alpha = 'approach_high', (macro-10)/39
                arm = (1-alpha)*initial[:7] + alpha*np.array(nodes[-1]['arm_qpos_rad'])
                hand = opened
            else:
                if macro < 120:
                    phase, coordinate, hand = 'descend', 10*(1-(macro-50)/69), opened
                elif macro < 160:
                    phase, coordinate = 'close', 0.
                    alpha = min(1., (macro-120)/30)
                    hand = (1-alpha)*opened + alpha*closed
                elif macro < 200:
                    phase, coordinate, hand = 'lift', 10*(macro-160)/39, closed
                else:
                    phase, coordinate, hand = 'hold', 10., closed
                i = min(9, int(coordinate))
                alpha = coordinate-i
                arm = (1-alpha)*np.array(nodes[i]['arm_qpos_rad']) + alpha*np.array(nodes[i+1]['arm_qpos_rad'])
            actions.append({'macro': macro, 'phase': phase, 'action_13': np.r_[arm, 2*hand/limits[:, 1]-1].tolist(),
                            'target_qpos_18': np.r_[arm, hand_state(hand)].tolist()})
    samples = [{'sample_id': 'native_formal_reset', 'phase': 'reset', 'time_s': 0., 'qpos': initial.tolist()}]
    prior = initial
    for row in actions:
        target = np.array(row['target_qpos_18'])
        for sub in range(1, 5):
            samples.append({'sample_id': f'm{row["macro"]:03d}_quarter{sub}', 'phase': row['phase'],
                            'time_s': (row['macro']+sub/4)*.05,
                            'qpos': ((1-sub/4)*prior + sub/4*target).tolist()})
        prior = target
    after = {str(path): sha(path) for path in sorted(files)}
    assert before == after
    result = {'schema': 'white-bowl-offline-candidate-v1', 'candidate_id': 'right_white_bowl_001',
              'baseline_commit': '8ffe0333c344669e3e3ad66125ba18f0cf48c0f6',
              'design': 'One right-source-pad thumb/index opposing pinch; remaining three fingers open; no left 13D action reuse or collision-informed optimization',
              'planning_reference_only_settled_bowl_position_m': settled.tolist(),
              'reference_is_runtime_state_injection': False, 'native_seed': 20261007,
              'libero_commit': commit, 'task_sha256': sha(task), 'bowl_xml_sha256': sha(bowl_path),
              'bowl_upper_wall_radius_m': radius, 'bowl_contact_height_above_origin_m': wall_height,
              'planned_pad_gap_m': gap, 'pad_features': features,
              'closed_active_rad': closed.tolist(), 'open_active_rad': opened.tolist(),
              'closed_active_fractions': (closed/limits[:, 1]).tolist(),
              'pinch_solver_success': bool(fit.success), 'pinch_residual': final_pinch_residual,
              'contact_midpoint_world_m': contact_midpoint.tolist(), 'hand_target_rotation': target_r.tolist(),
              'arm_nodes_contact_to_above': nodes, 'commands': actions, 'samples': samples,
              'sampling': '230 command target poses and 3 quarter-interpolants each, plus formal reset (921). Target-space linear interpolation; not predicted tracking or continuous collision certification.',
              'proposed_trial_criteria': {
                  'scope': 'NEW fixed right white-bowl criteria; not left/manufacturer/full-range certification',
                  'lift_from_own_settled_baseline_m_min': .020, 'continuous_hold_s_min': 1.0,
                  'hold_requires': ['height >= settled+20mm', 'no bowl-table or other environmental support contacts',
                                    'positive-force thumb and index contacts on opposing bowl sides'],
                  'steady_active_error_rad_max': .02, 'arm_tracking_error_rad_max': .1,
                  'linear_mimic_error_rad_max': .02, 'joint_limit_overshoot_rad_max': .01,
                  'eligible_internal_and_external_overlap_m_max': .002,
                  'warnings_allowed': 0, 'nonfinite_states_allowed': 0,
                  'native_task_result': 'record native cabinet-placement predicate separately',
                  'strict_grasp_success_claim': False,
                  'stop': 'First numerical/physical bound violation; no automatic retry or edits'},
              'zero_physics_proof': counts, 'blocked_mujoco_apis': blocked,
              'input_sha256_before': before, 'input_sha256_after': after, 'inputs_unchanged': True}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--libero', required=True, type=Path)
    args = parser.parse_args()
    value = generate(args.output, args.libero)
    print(json.dumps({k: value[k] for k in ('candidate_id', 'closed_active_rad', 'pinch_residual',
                                           'arm_nodes_contact_to_above', 'zero_physics_proof')}))
