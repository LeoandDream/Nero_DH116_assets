"""Frozen 81-hand / 306-wrist one-shot policy; no integration or state writes.

Risk classification is a conservative stop, not a source-material certificate.
All original wrist stops and physical criteria remain in force.
"""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import wrist_conditional as W

read, digest, PLAN, WRIST_BODIES = W.read, W.digest, W.PLAN, W.WRIST_BODIES
MODEL = Path('examples/right_hand/candidates/finger22_local/model.xml')
MODEL_SHA = '7f9d84061c3e7d176763c688d2389f8b3d3e38386b8e990a0fa047ff2bc3329c'
AUTHORIZATION = 'https://github.com/LeoandDream/Nero_DH116_assets/pull/4#issuecomment-6273337682'
BODY = 'gripper0_finger22_link'
OLD = 'gripper0_proxy76_finger22_link_00'
NEW = tuple('gripper0_finger22_local_'+str(i).zfill(2) for i in range(4))
RISK_EVIDENCE = Path('examples/right_hand/evidence/d3_finger22_risk_review')
# From the already derived input-scale bound, rounded upward. This only bounds
# face identification arithmetic; it does not relax the original 0.1nm FAIL.
CLASSIFICATION_RESERVE_M = 3.033e-9


def protocol(root):
    p = W.protocol(root)
    p.update(authorization=AUTHORIZATION, model=str(MODEL), model_sha256=MODEL_SHA,
             baseline_commit='685c458c29308bbfb2b4e605766cf7da8e336458',
             previous_dynamic_commit='360e53a023624e96bd88aef549927655b5034445',
             hand_collision_count=81, wrist_collision_count=306,
             classification_reserve_m=CLASSIFICATION_RESERVE_M,
             finger22_region_policy={
                 'root_00': 'Any eligible contact stops conservatively: root/bore risk, not intended bowl pad.',
                 'lower_waist_01': 'Marked cavity-support planes stop. Only unambiguous +X-dominant front planes are normal; side/back/cut/ambiguous faces stop UNKNOWN.',
                 'upper_waist_02_distal_03': 'Unmarked existing functional surfaces; original dynamic stops still apply.',
                 'plane_candidates': 'All support planes within abs(contact.dist)/2 + classification reserve of contact midpoint in actual RK4 body frame.',
                 'positive_X': 'Every possible plane normal has nx > abs(ny) and nx > abs(nz); no angular or physics tolerance is changed.',
                 'no_candidate_or_missing_pose': 'UNKNOWN early stop, never guess contact ownership.'})
    p['additional_stops'] = p['additional_stops'] + [
        'Any eligible finger22 root risk contact, including zero-force contacts.',
        'Any marked lower-waist cavity-support facet contact; any ambiguous/non-front lower-waist contact stops UNKNOWN.']
    return p


def normalized_model(root, path, remove):
    xml = ET.parse(root/path).getroot()
    xml.attrib.pop('model', None)
    for asset in xml.findall('.//*[@file]'):
        resource = (root/path.parent/asset.get('file')).resolve()
        asset.set('file', str(resource.relative_to(root))+':'+digest(resource))
    body = xml.find('.//body[@name="'+BODY+'"]')
    for g in list(body.findall('geom')):
        if g.get('name') in remove:
            body.remove(g)
    for a in list(xml.find('asset')):
        if a.get('name') in NEW:
            xml.find('asset').remove(a)
    def canonical(e):
        return (e.tag, sorted(e.attrib.items()), [canonical(c) for c in e])
    return canonical(xml)


def validate_external_preflight(root, summary_path):
    # Reuse the old wrist provenance, without inheriting a new physics setup.
    old = W.validate_external_preflight(root, summary_path)
    if digest(root/MODEL) != MODEL_SHA:
        raise ValueError('Frozen finger22 model differs')
    if normalized_model(root, W.MODEL, [OLD]) != normalized_model(root, MODEL, NEW):
        raise ValueError('Model changes beyond the approved finger22 replacement')
    manifest = read(root/MODEL.parent/'manifest.json')
    for piece in manifest['pieces']:
        if digest(root/piece['file']) != piece['sha256']:
            raise ValueError('Frozen finger22 mesh differs')
    return {**old, 'protocol': protocol(root), 'finger22_model_sha256': MODEL_SHA,
            'xml_and_resource_equivalence_except_finger22': True,
            'prior_static_result': read(root/'examples/right_hand/evidence/d3_finger22_local/summary.json'),
            'risk_evidence_sha256': {n: digest(root/RISK_EVIDENCE/n) for n in
                                    ('witnesses.json','numeric.json','working-cover.json.gz')}}


def prepare_regions(root, model):
    """Read the actual compiled convex geometry, without a simulation query."""
    import mujoco
    from scipy.spatial import ConvexHull
    witness = read(root/RISK_EVIDENCE/'witnesses.json')['buried_source']
    patch = np.array(witness['compiled_nearest_support_triangle_m'])
    regions = {}
    for name in NEW:
        g = model.geom(name).id
        mid = int(model.geom_dataid[g])
        start, count = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid])
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, model.geom_quat[g])
        vertices = np.asarray(model.mesh_vert[start:start+count],float)@rotation.reshape(3,3).T+model.geom_pos[g]
        eq = ConvexHull(vertices).equations
        # The whole compiled facet(s), not only the witness point, are marked.
        residual = np.max(abs(patch@eq[:,:3].T+eq[:,3]),axis=0)
        marked = np.flatnonzero(residual <= CLASSIFICATION_RESERVE_M).tolist() if name==NEW[1] else []
        regions[name] = {'planes': eq.tolist(), 'marked_planes': marked,
                         'compiled_vertices': len(vertices), 'geom_id': g, 'body': BODY}
    if not regions[NEW[1]]['marked_planes']:
        raise ValueError('Cannot identify recorded cavity-support facet in actual compiled mesh')
    return regions


def classify_contact(contact, regions):
    ids = [i for i,n in enumerate(contact['geom']) if n in NEW]
    if not ids or contact['filter_reasons']:
        return None
    i = ids[0]; name = contact['geom'][i]
    if name == NEW[0]:
        return {'status': 'STOP_ROOT_RISK', 'geom': name}
    if name != NEW[1]:
        return {'status': 'UNMARKED_FUNCTIONAL_PIECE', 'geom': name}
    try:
        pose = contact['cached_body_pose'][i]
        point = np.array(pose['rotation_world']).reshape(3,3).T@(
            np.array(contact['position_world_m'])-pose['position_world_m'])
        eq = np.array(regions[name]['planes'])
        distance = abs(eq[:,:3]@point+eq[:,3])
        uncertainty = abs(contact['distance_m'])/2+CLASSIFICATION_RESERVE_M
        candidates = np.flatnonzero(distance <= uncertainty).tolist()
        if not candidates:
            status = 'UNKNOWN_CONTACT_FACE'
        elif set(candidates)&set(regions[name]['marked_planes']):
            status = 'STOP_MARKED_CAVITY_FACET'
        elif all(eq[j,0] > abs(eq[j,1]) and eq[j,0] > abs(eq[j,2]) for j in candidates):
            status = 'NORMAL_POSITIVE_X_FACE'
        else:
            status = 'UNKNOWN_NONFRONT_OR_EDGE_CONTACT'
        return {'status': status, 'geom': name, 'midpoint_body_m': point.tolist(),
                'possible_plane_ids': candidates, 'midpoint_plane_distances_m': distance.tolist(),
                'identification_uncertainty_m': uncertainty}
    except (KeyError, ValueError, TypeError):
        return {'status': 'UNKNOWN_CONTACT_FACE', 'geom': name}


def extra_failures(contacts, qpos, frozen):
    reasons = W.extra_failures(contacts, qpos, frozen)
    for c in contacts:
        if c['filter_reasons'] or not any(n in NEW for n in c['geom']):
            continue
        status = (c.get('finger22_region') or {}).get('status', 'UNKNOWN_CONTACT_FACE')
        if status.startswith('STOP_'):
            reasons.append('FINGER22_NONFUNCTIONAL_REGION_CONTACT')
        elif status.startswith('UNKNOWN'):
            reasons.append('FINGER22_RISK_CONTACT_UNKNOWN')
    return list(dict.fromkeys(reasons))


def compare_reset_snapshots(old, new, removed, added):
    # The caller supplies the old wrist manifest; only this replacement is now
    # permitted relative to the saved 360e53a native Reset snapshot.
    return W.compare_reset_snapshots(old, new, [OLD], list(NEW))
