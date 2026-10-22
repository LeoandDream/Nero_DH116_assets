#!/usr/bin/env python3
"""One byte-preserving wrist migration and static DH116 regression (no dynamics).

Build needs the explicitly supplied, SHA-locked source ZIP + build inventory.
Audit/scan needs only this checkout and the explicitly supplied LIBERO assets
for the historical full-scene fixture. No controller or environment is created.
"""
import argparse
from collections import Counter
import copy
import gzip
import hashlib
import json
import os
from pathlib import Path
import platform
import sys
import time
import xml.etree.ElementTree as ET
import zipfile

for variable in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[variable] = '1'
import numpy as np
import scipy
from scipy.spatial import ConvexHull
import link5_flange_fullscan as F
P = F.P
mj = P.mujoco
ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
BASE = HERE/'candidates/finger13_local/model.xml'
CANDIDATE = HERE/'candidates/wrist_reuse/model.xml'
BODIES = P.BODIES
COUNTS = (147, 159)
BASE_SHA = 'b27f4f86fbaeacd2bb925463f39f67181ef7db12963c4d9f7fc01797a7924d1b'
ZIP_SHA = '801e5d24cbace320511dcfb25afdce11c8ed2f4afd7572dd1007145d21ea500a'
PREFIX = 'nero-engineering-static/'
RUN = HERE/'evidence/d3_bowl78_slow_001'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    path = Path(path)
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'xt') as f:
        json.dump(value, f, indent=2, allow_nan=False)
        f.write('\n')


def read(path):
    opener = gzip.open if Path(path).suffix == '.gz' else open
    with opener(path, 'rt') as f:
        return json.load(f)


def collision_geoms(xml, body):
    return [g for g in xml.find('.//body[@name="'+body+'"]').findall('geom')
            if int(g.get('contype', '1')) or int(g.get('conaffinity', '1'))]


def signature(g):
    return {k: v for k, v in g.attrib.items() if k not in ('name', 'mesh', 'pos', 'quat')}


def canonical(e):
    return (e.tag, sorted(e.attrib.items()), (e.text or '').strip(),
            [canonical(c) for c in e])


def resolve_files(xml, directory):
    for e in xml.findall('.//*[@file]'):
        e.set('file', str((directory/e.get('file')).resolve()))


def build(archive, inventory):
    if sha(BASE) != BASE_SHA or sha(archive) != ZIP_SHA:
        raise ValueError('Frozen robot or explicit ZIP SHA differs')
    if CANDIDATE.parent.exists():
        raise ValueError('Candidate already exists: never overwrite an old candidate')
    records = read(inventory)['pieces']
    if Counter(r['mesh'] for r in records) != {'link5': 147, 'gripper_flange': 159}:
        raise ValueError('Unexpected source inventory')
    for part, count in zip(('link5', 'gripper_flange'), COUNTS):
        if sorted(r['piece'] for r in records if r['mesh'] == part) != list(range(count)):
            raise ValueError('Duplicate or missing source piece indices')
    xml = ET.parse(BASE).getroot()
    removed, attributes = [], []
    for body, count in zip(BODIES, (7, 8)):
        old = collision_geoms(xml, body)
        if len(old) != count or any(signature(g) != signature(old[0]) for g in old):
            raise ValueError('DH contact attributes differ; explicit mapping is required')
        if any(g.get('pos', '0 0 0') != '0 0 0' or g.get('quat', '1 0 0 0') != '1 0 0 0' for g in old):
            raise ValueError('Unexpected old collision pose')
        attributes.append(signature(old[0]))
        removed.extend({'body': body, 'geom': g.attrib.copy()} for g in old)
    # Verify every raw ZIP member before creating the sole candidate.
    with zipfile.ZipFile(archive) as z:
        payloads = []
        for r in records:
            member = PREFIX+'assets/collision_candidate/'+Path(r['file']).name
            blob = z.read(member)
            if hashlib.sha256(blob).hexdigest() != r['sha256']:
                raise ValueError('Source member hash differs: '+member)
            payloads.append((r, member, blob))
        CANDIDATE.parent.mkdir()
        (CANDIDATE.parent/'meshes').mkdir()
        for name in ('PROVENANCE.md', 'UPSTREAM_README.md'):
            (CANDIDATE.parent/('SOURCE_'+name)).write_bytes(z.read(PREFIX+name))
    resolve_files(xml, BASE.parent)
    for e in xml.findall('.//*[@file]'):
        e.set('file', os.path.relpath(e.get('file'), CANDIDATE.parent))
    old_meshes = {r['geom']['mesh'] for r in removed}
    for body in BODIES:
        b = xml.find('.//body[@name="'+body+'"]')
        for g in collision_geoms(xml, body):
            b.remove(g)
    for m in list(xml.find('asset')):
        if m.tag == 'mesh' and m.get('name') in old_meshes:
            xml.find('asset').remove(m)
    mapping = []
    for r, member, blob in payloads:
        index = 0 if r['mesh'] == 'link5' else 1
        filename = Path(member).name
        target = CANDIDATE.parent/'meshes'/filename
        target.write_bytes(blob)
        name = 'robot0_wrist_reuse_'+filename.removesuffix('.obj')
        ET.SubElement(xml.find('asset'), 'mesh', name=name, file='meshes/'+filename)
        attrs = dict(attributes[index], name=name, mesh=name, pos='0 0 0', quat='1 0 0 0')
        ET.SubElement(xml.find('.//body[@name="'+BODIES[index]+'"]'), 'geom', attrs)
        mapping.append({'zip_member': member, 'file': 'meshes/'+filename, 'sha256': r['sha256'],
                        'geom': name, 'body': BODIES[index], 'source_record': r})
    xml.set('model', 'nero_dh116_right_wrist_reuse_diagnostic')
    ET.indent(xml)
    ET.ElementTree(xml).write(CANDIDATE, encoding='utf-8', xml_declaration=True)
    write(CANDIDATE.parent/'manifest.json', {
        'base_commit': '18497ba9e7b55fc6762afebee35fb740aba43446',
        'base_model': str(BASE.relative_to(ROOT)), 'base_sha256': BASE_SHA,
        'candidate_sha256': sha(CANDIDATE), 'zip_sha256': ZIP_SHA,
        'inventory_sha256': sha(inventory), 'removed': removed, 'meshes': mapping,
        'construction': 'Raw ZIP bytes only; no vertex transform, scaling or hull generation.',
        'precision_exceptions_approved_for_DH116': False})
    return resource_audit()


def xml_audit(base, candidate):
    a, b = (ET.parse(path).getroot() for path in (base, candidate))
    removed_counts, replacement_counts = [], []
    for body in BODIES:
        old, new = collision_geoms(a, body), collision_geoms(b, body)
        removed_counts.append(len(old)); replacement_counts.append(len(new))
        if {g.get('name') for g in old} & {g.get('name') for g in new}:
            raise ValueError('Old wrist geoms were overlaid')
        if any(signature(g) != signature(old[0]) for g in new):
            raise ValueError('Contact attributes changed')
        for xml, geoms in ((a, old), (b, new)):
            names = {g.get('mesh') for g in geoms}
            for g in geoms:
                xml.find('.//body[@name="'+body+'"]').remove(g)
            for m in list(xml.find('asset')):
                if m.tag == 'mesh' and m.get('name') in names:
                    xml.find('asset').remove(m)
    for xml, path in ((a, base), (b, candidate)):
        resolve_files(xml, path.parent)
        xml.set('model', 'ignored_diagnostic_label')
    identical = canonical(a) == canonical(b)
    if not identical:
        raise ValueError('Non-authorized XML fields changed')
    hand = sum(len(collision_geoms(b, body.get('name'))) for body in b.findall('.//body')
               if body.get('name', '').startswith('gripper0_'))
    return {'removed_counts': removed_counts, 'replacement_counts': replacement_counts,
            'hand_collision_count': hand, 'unauthorized_xml_fields_identical': identical}


def resource_audit():
    manifest = read(CANDIDATE.parent/'manifest.json')
    xml = ET.parse(CANDIDATE).getroot()
    rows = manifest['meshes']
    bad = [r['file'] for r in rows if sha(CANDIDATE.parent/r['file']) != r['sha256']]
    nonzero = [r['geom'] for r in rows if any(
        xml.find('.//geom[@name="'+r['geom']+'"]').get(k) != v
        for k, v in (('pos', '0 0 0'), ('quat', '1 0 0 0')))]
    external, assets = [], []
    for e in xml.findall('.//*[@file]'):
        path = (CANDIDATE.parent/e.get('file')).resolve()
        if not path.is_relative_to(ROOT):
            external.append(e.get('file'))
        assets.append({'file': str(path.relative_to(ROOT)), 'sha256': sha(path)})
    if bad or nonzero or external or sha(CANDIDATE) != manifest['candidate_sha256']:
        raise ValueError('Candidate resource or pose integrity failed')
    return {'mesh_count': len(rows), 'hash_mismatches': bad,
            'nonzero_geom_poses': nonzero, 'external_candidate_paths': external,
            'all_assets': assets, 'candidate_sha256': sha(CANDIDATE)}


def geom_names(model):
    """LIBERO has unnamed environment geoms; identify by body and ordinal."""
    ordinal = Counter()
    result = {}
    for i in range(model.ngeom):
        name = P.saved.name(model, mj.mjtObj.mjOBJ_GEOM, i)
        if name is None:
            body = P.saved.name(model, mj.mjtObj.mjOBJ_BODY, model.geom_bodyid[i])
            name = f'@unnamed:{body}:{ordinal[body]}'
            ordinal[body] += 1
        result[name] = i
    return result


def compiled_audit(a, b, removed_names, new_names):
    """Compare all unchanged scalar/array fields, allowing only geometry indices.

    Body masses/inertias, joint/dof/eq/actuator fields, options and sizes remain
    exact. Mesh IDs/name-address tables and BVH topology necessarily change.
    """
    changed, compared = [], []
    ignored_body = {'body_geomadr', 'body_geomnum', 'body_bvhadr', 'body_bvhnum'}
    for key in dir(a):
        if key.startswith(('body_', 'jnt_', 'dof_', 'actuator_', 'eq_', 'exclude_', 'key_')):
            if key in ignored_body or key.endswith('nameadr'):
                continue
            av, bv = getattr(a, key), getattr(b, key)
            if isinstance(av, np.ndarray):
                compared.append(key)
                if not np.array_equal(av, bv):
                    changed.append(key)
    for key in ('nq', 'nv', 'nu', 'na', 'neq', 'nbody', 'njnt', 'nkey', 'nexclude', 'nconmax', 'njmax'):
        compared.append(key)
        if getattr(a, key) != getattr(b, key):
            changed.append(key)
    for obj in ('opt',):
        if P.saved.fields(getattr(a, obj)) != P.saved.fields(getattr(b, obj)):
            changed.append(obj)
    names_a, names_b = geom_names(a), geom_names(b)
    if set(names_a)-set(names_b) != set(removed_names) or set(names_b)-set(names_a) != set(new_names):
        changed.append('geom_name_sets')
    geom_fields = [k for k in dir(a) if k.startswith('geom_') and isinstance(getattr(a, k), np.ndarray)
                   and k not in ('geom_dataid', 'geom_matid')]
    for name in sorted(set(names_a) & set(names_b)):
        ia, ib = names_a[name], names_b[name]
        for k in geom_fields:
            if not np.array_equal(getattr(a, k)[ia], getattr(b, k)[ib]):
                changed.append(name+':'+k)
        if a.geom_type[ia] == mj.mjtGeom.mjGEOM_MESH:
            for prefix in ('vert', 'face'):
                ma, mb = int(a.geom_dataid[ia]), int(b.geom_dataid[ib])
                x, y = int(getattr(a, 'mesh_'+prefix+'adr')[ma]), int(getattr(b, 'mesh_'+prefix+'adr')[mb])
                n, m = int(getattr(a, 'mesh_'+prefix+'num')[ma]), int(getattr(b, 'mesh_'+prefix+'num')[mb])
                if n != m or not np.array_equal(getattr(a, 'mesh_'+prefix)[x:x+n], getattr(b, 'mesh_'+prefix)[y:y+m]):
                    changed.append(name+':compiled_mesh_'+prefix)
        # Material IDs remain stable because the adapter never changes materials.
        if a.geom_matid[ia] != b.geom_matid[ib]:
            changed.append(name+':geom_matid')
    # New geoms inherit each body's actual compiled contact parameters.
    contact_keys = ('geom_contype', 'geom_conaffinity', 'geom_condim', 'geom_priority',
                    'geom_friction', 'geom_solmix', 'geom_solref', 'geom_solimp', 'geom_margin', 'geom_gap')
    for body in BODIES:
        ai = [names_a[n] for n in removed_names if P.saved.name(a, mj.mjtObj.mjOBJ_BODY, a.geom_bodyid[names_a[n]]) == body][0]
        for name in new_names:
            bi = names_b[name]
            if P.saved.name(b, mj.mjtObj.mjOBJ_BODY, b.geom_bodyid[bi]) == body:
                for key in contact_keys:
                    if not np.array_equal(getattr(a, key)[ai], getattr(b, key)[bi]):
                        changed.append(name+':'+key)
    if changed:
        raise ValueError('Unexpected compiled changes: '+str(changed[:30]))
    return {'status': 'PASS', 'compared_fields': compared, 'unchanged_geoms': len(set(names_a)&set(names_b)),
            'unauthorized_differences': changed, 'ngeom': [a.ngeom, b.ngeom],
            'nq_nv_nu_neq': [a.nq, a.nv, a.nu, a.neq],
            'excluded_structural_indices': sorted(ignored_body),
            'new_geom_compiled_contact_fields_checked': list(contact_keys),
            'new_geom_pose_note': 'XML pose zero; compiled mesh principal-axis pose is expected and used in queries.'}


def aabb_gaps(amin, amax, bmin, bmax):
    forward = bmin[None, :, :]-amax[:, None, :]
    backward = amin[:, None, :]-bmax[None, :, :]
    gap = np.maximum(forward, backward)
    axis = gap.argmax(axis=2)
    return gap.max(axis=2), axis, np.take_along_axis(backward > forward, axis[:, :, None], axis=2)[:, :, 0]


def hulls(model):
    xml = ET.parse(CANDIDATE).getroot()
    assets = {e.get('name'): e for e in xml.findall('asset/mesh')}
    groups = []
    numerical = []
    for body in BODIES:
        group = []
        for geom in collision_geoms(xml, body):
            name = geom.get('name')
            binding = P.reader._binding(CANDIDATE, xml, assets, geom)
            h = P.compiled_hull(model, name, binding)
            group.append(h)
            raw, _ = P.reader.BUILDER.read_mesh(Path(binding['file']))
            planes = ConvexHull(h['vertices']).equations
            outside = np.max(raw@planes[:, :3].T+planes[:, 3], axis=1)
            numerical.append({'geom': name, 'maximum_raw_vertex_outside_own_compiled_hull_m': float(outside.max()),
                              'exceeds_unchanged_1e_10m': bool(outside.max() > P.TOL)})
        groups.append(group)
    return groups, numerical


def scan_plan(model, groups, out):
    samples = read(P.PLAN)['samples']
    if sha(P.PLAN) != F.EXPECTED_PLAN or len(samples) != 921:
        raise ValueError('Original 921-pose plan differs')
    old = read(HERE/'evidence/d3_link5_flange_fullscan/scan/poses.json.gz')['poses']
    old_bad = {r['pose_index_one_based'] for r in old if any(p['status'] != 'SEPARATED' for p in r['pairs'])}
    assert len(old_bad) == 435
    amin = np.array([h['vertices'].min(axis=0) for h in groups[0]])
    amax = np.array([h['vertices'].max(axis=0) for h in groups[0]])
    data = mj.MjData(model)
    bids = [mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, b) for b in BODIES]
    rows, certs, counts = [], [], Counter()
    cache = {}
    t0 = time.monotonic()
    for i, sample in enumerate(samples, 1):
        data.qpos[:] = sample['qpos']
        mj.mj_kinematics(model, data)
        transforms = {b: {'rotation': data.xmat[j].reshape(3, 3).tolist(), 'translation_m': data.xpos[j].tolist()}
                      for b, j in zip(BODIES, bids)}
        rotation, translation = P.relative(transforms)
        moved = [P.transformed(h, rotation, translation) for h in groups[1]]
        bmin = np.array([v.min(axis=0) for v in moved]); bmax = np.array([v.max(axis=0) for v in moved])
        gaps, axes, reverse = aabb_gaps(amin, amax, bmin, bmax)
        todo = np.argwhere(gaps <= P.TOL)
        anomalies = []
        minimum = float(np.min(gaps))
        for ia, ib in todo:
            a, b = groups[0][ia], groups[1][ib]
            key = (int(ia), int(ib), rotation.tobytes(), translation.tobytes())
            if key in cache:
                cert = dict(cache[key], reused_exact_transform=True)
            else:
                sep = P.separation(a, b, rotation, translation)
                cert = {'pair': [a['name'], b['name']], 'projection': sep}
                if sep['classification'] == 'CERTIFIED_CONVEX_SOLIDS_SEPARATED':
                    cert['status'] = 'SEPARATED'
                else:
                    lp = P.lp(a, b, transforms)
                    lp.update(F.dual_bound(lp, a, b, rotation, translation))
                    cert.update(status=F.status_from_lp(lp), lp=lp)
                cache[key] = cert
            certs.append(dict(cert, pose_index_one_based=i))
            counts[cert['status']] += 1
            if cert['status'] != 'SEPARATED':
                anomalies.append(cert['pair'])
        counts['AABB_SEPARATED'] += 147*159-len(todo)
        rows.append({'pose_index_one_based': i, 'pose_id': sample['sample_id'], 'phase': sample['phase'],
                     'old_anomaly': i in old_bad, 'anomalies': anomalies,
                     'AABB_separated_pairs': 147*159-len(todo), 'narrowphase_pairs': len(todo),
                     'minimum_AABB_gap_m': minimum, 'relative_rotation': rotation.tolist(),
                     'relative_translation_m': translation.tolist()})
        if i % 100 == 0:
            print('plan', i, 'of 921', dict(counts), 'elapsed_s', round(time.monotonic()-t0, 2), flush=True)
    bad = {r['pose_index_one_based'] for r in rows if r['anomalies']}
    write(out/'plan.json.gz', rows); write(out/'certificates.json.gz', certs)
    summary = {'poses': len(rows), 'potential_pairs_per_pose': 147*159, 'total_pair_cases': 921*147*159,
               'classifications': dict(counts), 'old_anomalous_poses': len(old_bad),
               'old_eliminated': len(old_bad-bad), 'old_remaining': len(old_bad & bad),
               'new_anomalous_poses': len(bad-old_bad), 'anomalous_pose_indices': sorted(bad),
               'status': 'PASS' if not bad else ('FAIL' if counts['STRICT_CONVEX_OVERLAP'] else 'UNKNOWN'),
               'elapsed_s': time.monotonic()-t0, 'projection_threshold_m': P.TOL,
               'old_pair_events_not_subtracted_from_new_pairs': True}
    write(out/'plan-summary.json', summary)
    return summary


def scene_xml(libero_assets, resources):
    """Resolve only known historical asset roots; no heuristic basename search."""
    xml = ET.parse(RUN/'model.xml').getroot()
    compiler = xml.find('compiler')
    compiler.attrib.pop('meshdir', None); compiler.attrib.pop('texturedir', None)
    prefix = '/data_robot/verification/nero_dh116_right/'
    for e in xml.findall('.//*[@file]'):
        source = e.get('file')
        if source.startswith(prefix+'.reference-deps/LIBERO/libero/libero/assets/'):
            rel = source.split('/LIBERO/libero/libero/assets/', 1)[1]
            path = libero_assets/rel
            kind = 'explicit_LIBERO_fixture'
        elif source.startswith(prefix):
            path = ROOT/source.removeprefix(prefix)
            kind = 'checkout'
        else:
            raise ValueError('Unrecognized historical asset root: '+source)
        resources.append({'historical': source, 'resolved': str(path), 'sha256': sha(path), 'kind': kind})
        e.set('file', str(path))
    return xml


def replace_scene_wrist(xml):
    candidate = ET.parse(CANDIDATE).getroot()
    new_names = set()
    old_meshes = set()
    for body in BODIES:
        parent = xml.find('.//body[@name="'+body+'"]')
        old = collision_geoms(xml, body)
        for g in old:
            old_meshes.add(g.get('mesh')); parent.remove(g)
        for g in collision_geoms(candidate, body):
            parent.append(copy.deepcopy(g)); new_names.add(g.get('mesh'))
    for m in list(xml.find('asset')):
        if m.tag == 'mesh' and m.get('name') in old_meshes:
            xml.find('asset').remove(m)
    for m in candidate.findall('asset/mesh'):
        if m.get('name') in new_names:
            e = copy.deepcopy(m)
            e.set('file', str((CANDIDATE.parent/e.get('file')).resolve()))
            xml.find('asset').append(e)
    return xml


def contacts(model, data):
    result = []
    for c in data.contact:
        ids = [int(c.geom1), int(c.geom2)]
        geoms = [P.saved.name(model, mj.mjtObj.mjOBJ_GEOM, g) or '@unnamed' for g in ids]
        bodies = [P.saved.name(model, mj.mjtObj.mjOBJ_BODY, model.geom_bodyid[g]) for g in ids]
        result.append({'geoms': geoms, 'bodies': bodies, 'distance_m': float(c.dist),
                       'position_world_m': c.pos.tolist()})
    return result


def contact_groups(rows, wrist_only=False):
    d = {}
    for row in rows:
        if wrist_only and not any(g.startswith('robot0_wrist_reuse_') or
            g.startswith('robot0_candidate_nero_link5_') or g.startswith('robot0_source_flange_part_')
            for g in row['geoms']):
            continue
        key = ' / '.join(sorted(row['bodies']))
        d[key] = min(d.get(key, float('inf')), row['distance_m'])
    return d


def scan_states(libero_assets, out, removed, added):
    resources = []
    x = scene_xml(libero_assets.resolve(), resources)
    y = replace_scene_wrist(copy.deepcopy(x))
    a, b = (mj.MjModel.from_xml_string(ET.tostring(xml, encoding='unicode')) for xml in (x, y))
    write(out/'scene-resource-map.json', resources)
    write(out/'scene-compiled-audit.json', compiled_audit(a, b, removed, added))
    da, db = mj.MjData(a), mj.MjData(b)
    before = [P.saved.fields(m) for m in (a, b)]
    rows, new_counts, lost_counts = [], Counter(), Counter()
    unchanged_nonwrist_contacts = 0
    added_geometry_counts = Counter()
    t0 = time.monotonic()
    with gzip.open(RUN/'state.jsonl.gz', 'rt') as stream:
        states = [json.loads(line) for line in stream]
    if len(states) != 1401:
        raise ValueError('Saved state count changed')
    cached = {}
    with gzip.open(RUN/'contacts.jsonl.gz', 'rt') as stream:
        for line in stream:
            r = json.loads(line)
            if r['step'] in (1314, 1315, 1321, 1326, 1400, 1401):
                cached[r['step']] = r
    for row in states:
        records = []
        for model, data in ((a, da), (b, db)):
            data.qpos[:] = row['full_qpos']; data.qvel[:] = row['full_qvel']
            data.ctrl[:] = row['ctrl']; data.time = row['time_s']
            protected = {k: getattr(data, k).copy() for k in ('qpos', 'qvel', 'ctrl', 'qacc', 'act', 'qacc_warmstart')}
            mj.mj_kinematics(model, data)
            mj.mj_collision(model, data)
            if any(not np.array_equal(v, getattr(data, k)) for k, v in protected.items()) or data.time != row['time_s']:
                raise ValueError('Static query changed integration state')
            records.append(contacts(model, data))
        ga, gb = map(contact_groups, records)
        nonwrist_a = [c for c in records[0] if set(c['bodies']) != set(BODIES)]
        nonwrist_b = [c for c in records[1] if set(c['bodies']) != set(BODIES)]
        unchanged_nonwrist_contacts += nonwrist_a == nonwrist_b
        wa, wb = (contact_groups(v, True) for v in records)
        added_pairs, lost_pairs = sorted(set(gb)-set(ga)), sorted(set(ga)-set(gb))
        new_counts.update(added_pairs); lost_counts.update(lost_pairs)
        # Include newly occupied contact locations even if another link5/flange
        # geom already contacted the same other body before substitution.
        new_sites = []
        for c in records[1]:
            if not any(g.startswith('robot0_wrist_reuse_') for g in c['geoms']):
                continue
            key = ' / '.join(sorted(c['bodies']))
            matches = [v for v in records[0] if sorted(v['bodies']) == sorted(c['bodies'])]
            distance = min((float(np.linalg.norm(np.asarray(c['position_world_m'])-v['position_world_m'])) for v in matches), default=None)
            new_sites.append(dict(c, nearest_baseline_contact_position_distance_m=distance))
            added_geometry_counts[key] += 1
        rows.append({'step': row['step'], 'phase': row['phase'], 'baseline_FK_contacts': records[0],
                     'candidate_FK_contacts': records[1], 'new_body_pairs': added_pairs, 'lost_body_pairs': lost_pairs,
                     'baseline_wrist_contact_depths_m': wa, 'candidate_wrist_contact_depths_m': wb,
                     'candidate_wrist_contact_locations': new_sites,
                     'saved_RK4_cache': cached.get(row['step'])})
        if row['step'] % 200 == 0:
            print('saved states', row['step'], 'of 1401', 'new body pairs', dict(new_counts), flush=True)
    assert before == [P.saved.fields(m) for m in (a, b)]
    write(out/'saved-states.json.gz', rows)
    wrist = ' / '.join(sorted(BODIES))
    summary = {'states': len(rows), 'baseline_wrist_contact_states': sum(wrist in r['baseline_wrist_contact_depths_m'] for r in rows),
               'candidate_wrist_contact_states': sum(wrist in r['candidate_wrist_contact_depths_m'] for r in rows),
               'new_body_pair_state_counts': dict(new_counts), 'lost_body_pair_state_counts': dict(lost_counts),
               'candidate_wrist_contact_point_counts': dict(added_geometry_counts),
               'states_with_bit_identical_nonwrist_contact_records': unchanged_nonwrist_contacts,
               'selected_steps': [r for r in rows if r['step'] in cached],
               'alignment': 'Fresh static FK + collision at saved post-qpos for BOTH models; original RK4 last-stage caches attached separately, never treated as identical alignment.',
               'elapsed_s': time.monotonic()-t0, 'integration_state_and_model_fields_unchanged': True,
               'external_suitability': 'UNKNOWN: this saved early-approach prefix cannot certify unseen bowl/table/hand configurations or the 2.17mm outward-fill region.'}
    write(out/'saved-summary.json', summary)
    return {k: v for k, v in summary.items() if k != 'selected_steps'}


def scan(output, libero_assets):
    if output.exists():
        raise ValueError('Use a new output directory; never overwrite evidence')
    output.mkdir(parents=True)
    environment = {'python': platform.python_version(), 'mujoco': mj.__version__,
                   'numpy': np.__version__, 'scipy': scipy.__version__}
    if sys.version_info[:2] != (3, 11) or mj.__version__ != '3.2.3':
        raise ValueError('Use frozen Python 3.11 / MuJoCo 3.2.3 for compiled mesh_graph')
    write(output/'environment.json', environment)
    manifest = read(CANDIDATE.parent/'manifest.json')
    write(output/'resources.json', resource_audit())
    write(output/'xml-audit.json', xml_audit(BASE, CANDIDATE))
    removed = [r['geom']['name'] for r in manifest['removed']]
    added = [r['geom'] for r in manifest['meshes']]
    safe = P.saved.SAFE_APIS
    P.saved.SAFE_APIS = {'mj_id2name', 'mj_name2id', 'mj_kinematics', 'mj_collision'}
    try:
        with P.saved.static_only() as audit:
            a, b = (mj.MjModel.from_xml_path(str(path)) for path in (BASE, CANDIDATE))
            write(output/'compiled-audit.json', compiled_audit(a, b, removed, added))
            before = P.saved.fields(b)
            groups, numerical = hulls(b)
            write(output/'compiled-numerics.json', {
                'check': 'Raw OBJ vertices against their own compiled convex halfspaces; records representational inward error, not millimetre-scale physical interference or full-union surface coverage.',
                'threshold_m_unchanged': P.TOL, 'parts': numerical,
                'maximum_m': max(r['maximum_raw_vertex_outside_own_compiled_hull_m'] for r in numerical),
                'parts_exceeding_threshold': sum(r['exceeds_unchanged_1e_10m'] for r in numerical),
                'old_compiled_coverage_FAIL': 'Preserved unchanged; not reclassified by this diagnostic.',
                'continuous_full_source_coverage': 'UNKNOWN: no new full-source coverage acceptance or approximation exception is authorized.'})
            plan = scan_plan(b, groups, output)
            assert before == P.saved.fields(b)
            states = scan_states(libero_assets, output, removed, added)
            audit['collision_calls'] = audit['allowed_api_calls']['mj_collision']
            write(output/'api-audit.json', audit)
    finally:
        P.saved.SAFE_APIS = safe
    write(output/'summary.json', {'migration': 'PASS', 'wrist_plan': plan, 'saved_states': states,
        'external_suitability': 'UNKNOWN', 'dynamics_and_formal_release': 'UNKNOWN_NOT_RUN_NOT_APPROVED',
        'historical_failures_and_original_thresholds_unchanged': True})
    paths = [BASE, CANDIDATE, P.PLAN, RUN/'state.jsonl.gz', RUN/'contacts.jsonl.gz', RUN/'model.xml',
             Path(__file__), Path(F.__file__), Path(P.__file__), Path(P.saved.__file__),
             Path(P.reader.__file__), Path(P.reader.BUILDER.__file__), Path(P.lpcheck.__file__)]
    write(output/'inputs.json', [{'path': str(p.relative_to(ROOT)), 'sha256': sha(p)} for p in paths])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest='command', required=True)
    build_parser = subs.add_parser('build')
    build_parser.add_argument('--zip', type=Path, required=True)
    build_parser.add_argument('--inventory', type=Path, required=True)
    scan_parser = subs.add_parser('scan')
    scan_parser.add_argument('--output', type=Path, required=True)
    scan_parser.add_argument('--libero-assets', type=Path, required=True)
    args = parser.parse_args()
    if args.command == 'build':
        result = build(args.zip, args.inventory)
        print(json.dumps({k: v for k, v in result.items() if k != 'all_assets'}))
    else:
        scan(args.output, args.libero_assets)


if __name__ == '__main__':
    main()
