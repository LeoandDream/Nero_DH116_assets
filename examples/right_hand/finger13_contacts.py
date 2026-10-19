#!/usr/bin/env python3
"""Static saved-pose comparison of one finger13 repair; never integrate.

Reuse the delivered FK/query guard, mesh reader and source certificates. Compare
finger13 against every other hand link, and independently certify the convex
finger13/finger42 parts with halfspaces and common-ball LP primal/dual witnesses.
Numerical LP tolerances are recorded, never used as physical pass thresholds.
"""
import argparse
from decimal import Decimal, localcontext
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

for _variable in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
                  'BLIS_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[_variable] = '1'

import mujoco
import numpy as np
import scipy
from scipy.optimize import linprog
from scipy.spatial import ConvexHull

import proxy76_contacts as saved
import proxy76_validate as mesh_reader


ROOT = saved.ROOT
BASELINE = ROOT/'examples/right_hand/candidates/proxy76/model.xml'
CERTIFICATES = ROOT/'examples/right_hand/evidence/d3_proxy76/existing-source-certificates.json'
PAIR = ('finger13_link', 'finger42_link')
LP_TOLERANCES = (1e-7, 1e-8, 1e-9)


def load_parts(path, links):
    xml = ET.parse(path).getroot()
    assets = {a.get('name'): a for a in xml.findall('asset/mesh')}
    result, inputs = {}, {path}
    for link in links:
        body = xml.find('.//body[@name="gripper0_'+link+'"]')
        result[link] = []
        for geom in body.findall('geom'):
            if geom.get('group', '0') != '0':
                continue
            binding = mesh_reader._binding(path, xml, assets, geom)
            triangles = mesh_reader.mesh_triangles(binding)
            vertices = np.unique(triangles.reshape(-1, 3), axis=0)
            result[link].append({'geom': binding['name'], 'vertices_m': vertices,
                                 'binding': binding})
            inputs.add(Path(binding['file']))
    return result, inputs


def prepare_halfspaces(first, second, transforms, links=PAIR):
    """Both raw convex hulls in conditioned body13 millimetres, no simulator."""
    ta, tb = (transforms[link] for link in links)
    ra, rb = np.array(ta['rotation']), np.array(tb['rotation'])
    pa, pb = np.array(ta['translation_m']), np.array(tb['translation_m'])
    points = [first['vertices_m']*1000,
              (second['vertices_m']@(ra.T@rb).T+ra.T@(pb-pa))*1000]
    low = np.minimum(points[0].min(axis=0), points[1].min(axis=0))
    high = np.maximum(points[0].max(axis=0), points[1].max(axis=0))
    origin = (low+high)/2
    points = [p-origin for p in points]
    normals, offsets, paddings, details = [], [], [], []
    for part, p in zip((first, second), points):
        hull = ConvexHull(p)
        n, b = hull.equations[:, :3], hull.equations[:, 3]
        norm = np.linalg.norm(n, axis=1)
        n, b = n/norm[:, None], b/norm
        # Preserve raw planes for the primary result. A separately reported
        # supplementary expansion encloses every raw vertex + 1e-10 mm roundoff.
        residual = np.full(len(n), -np.inf)
        for start in range(0, len(p), 1024):
            residual = np.maximum(residual, np.max(p[start:start+1024]@n.T+b, axis=0))
        padding = np.maximum(residual, 0)+1e-10
        normals.append(n)
        offsets.append(b)
        paddings.append(padding)
        details.append({'geom': part['geom'], 'raw_vertices': len(p),
                        'halfspaces': len(n),
                        'max_plane_numerical_padding_mm': float(padding.max()),
                        'max_raw_vertex_plane_residual_mm': float(residual.max()),
                        'min_padded_raw_vertex_enclosure_margin_mm': float(np.min(padding-residual))})
    return (np.vstack(normals), np.concatenate(offsets), np.concatenate(paddings),
            origin, (high-low)/2, details)


def solve_halfspaces(n, b, origin, bounds, full_certificate):
    a = np.column_stack([n, np.ones(len(n))])
    runs = []
    full = None
    for tolerance in LP_TOLERANCES:
        lp = linprog([0., 0., 0., -1.], A_ub=a, b_ub=-b,
                     bounds=[(None, None)]*4, method='highs',
                     options={'primal_feasibility_tolerance': tolerance,
                              'dual_feasibility_tolerance': tolerance})
        if not lp.success:
            raise ValueError('Static common-ball LP failed: '+lp.message)
        center, radius = lp.x[:3], float(lp.x[3])
        margins = -(n.astype(np.longdouble)@center.astype(np.longdouble)+b.astype(np.longdouble))
        # A nonnegative normalized dual combination yields an upper bound on r.
        # Residual normal cancellation is bounded over both finite hull AABBs.
        weights = np.maximum(-lp.ineqlin.marginals, 0.)
        weights /= weights.sum()
        normal_residual = weights.astype(np.longdouble)@n.astype(np.longdouble)
        dual = float(-weights.astype(np.longdouble)@b.astype(np.longdouble)+
                     np.abs(normal_residual)@bounds.astype(np.longdouble))
        lower = float(np.min(margins))
        classification = ('STRICT_CONVEX_INTERIOR_OVERLAP' if lower > 0 else
                          'SEPARATED_CONVEX_SOLIDS' if dual < 0 else 'UNKNOWN')
        runs.append({'feasibility_tolerance': tolerance, 'radius_lp_mm': radius,
                     'independent_center_min_margin_mm': lower,
                     'conservative_dual_radius_upper_bound_mm': dual,
                     'classification': classification,
                     'center_conditioned_mm': center.tolist(),
                     'dual_normal_residual': normal_residual.astype(float).tolist(),
                     'dual_weight_sum': float(weights.sum())})
        if full_certificate and tolerance == LP_TOLERANCES[-1]:
            full = {'normals': n.tolist(), 'offsets_mm': b.tolist(),
                    'dual_weights': weights.tolist(), 'center_conditioned_mm': center.tolist(),
                    'conditioned_aabb_abs_bounds_mm': bounds.tolist(),
                    'conditioning_origin_body13_mm': origin.tolist(),
                    'decimal': decimal_check(n, b, center, weights, bounds)}
    classes = {r['classification'] for r in runs}
    row = {'classification': next(iter(classes)) if len(classes) == 1 else 'UNKNOWN_TOLERANCE_DEPENDENT',
           'all_tolerances_agree': len(classes) == 1, 'tolerance_runs': runs}
    if full is not None:
        row['certificate'] = full
    return row


def common_ball(first, second, transforms, full_certificate=False, links=PAIR):
    n,b,padding,origin,bounds,hulls = prepare_halfspaces(first,second,transforms,links)
    raw = solve_halfspaces(n,b,origin,bounds,full_certificate)
    raw.update({'geoms':[first['geom'],second['geom']], 'hulls':hulls,
                'primary_geometry':'UNEXPANDED_ORIGINAL_CONVEX_HALFSPACES',
                'numerical_expansion_check':solve_halfspaces(n,b-padding,origin,bounds,
                                                           full_certificate)})
    agree = raw['classification']==raw['numerical_expansion_check']['classification']
    raw['raw_and_expanded_classifications_agree'] = agree
    raw['robust_classification'] = raw['classification'] if agree else 'UNKNOWN_NUMERICAL_EXPANSION_DEPENDENT'
    return raw


def decimal_check(normals, offsets, center, weights, bounds):
    """Independent 60-digit primal/dual arithmetic over every stored plane."""
    with localcontext() as context:
        context.prec = 60
        d = Decimal.from_float
        c, w, limits = [list(map(d, a)) for a in (center, weights, bounds)]
        ns = [list(map(d, row)) for row in normals]
        bs = list(map(d, offsets))
        margins = [-sum(x*y for x,y in zip(row,c))-offset for row,offset in zip(ns,bs)]
        total = sum(w)
        residual = [sum(weight*row[i] for weight,row in zip(w,ns))/total for i in range(3)]
        upper = -sum(weight*offset for weight,offset in zip(w,bs))/total+sum(abs(x)*y for x,y in zip(residual,limits))
        return {'precision': 60, 'planes_checked': len(bs),
                'minimum_center_plane_margin_mm': str(min(margins)),
                'dual_radius_upper_bound_mm': str(upper),
                'all_dual_weights_nonnegative': min(w) >= 0,
                'strict_overlap_certified': min(margins) > 0,
                'strict_separation_certified': upper < 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    candidate = args.candidate.resolve()
    output = args.output.resolve()
    if output.exists():
        raise ValueError('Use a new output directory; original evidence is read only')
    poses = saved.saved_poses()
    certificates = json.loads(CERTIFICATES.read_text())
    sources = {}
    paths = {Path(__file__).resolve(), CERTIFICATES,
             Path(saved.__file__).resolve(), Path(mesh_reader.__file__).resolve(),
             *[ROOT/p['source_state_path'] for p in poses],
             *[ROOT/'meshes/dh116_right/source'/(link+'.STL') for link in PAIR]}
    for label,path in (('proxy76',BASELINE),('finger13_repair',candidate)):
        links = mesh_reader.BUILDER.hand_collisions(ET.parse(path).getroot())
        sources[label], inputs = load_parts(path, links)
        paths |= inputs
    before = {str(p.relative_to(ROOT)): saved.sha(p) for p in sorted(paths)}
    results = {p['id']: {'pose': p, 'finger13_vs_other_hand_links': {}} for p in poses}
    model_audits = {}
    with saved.static_only() as api_audit:
        for label,path in (('proxy76',BASELINE),('finger13_repair',candidate)):
            model = mujoco.MjModel.from_xml_path(str(path))
            model_before = saved.fields(model)
            by_link = saved.hand_geoms(model)
            if len(by_link) != 17:
                raise ValueError('Expected 17 unchanged hand links')
            for pose in poses:
                row = results[pose['id']]
                data = mujoco.MjData(model)
                data.qpos[:] = pose['full_qpos']
                state_before = {key: np.array(getattr(data,key), copy=True)
                                for key in ('qpos','qvel','qacc','ctrl','act','qacc_warmstart')}
                time_before = data.time
                mujoco.mj_kinematics(model,data)
                transforms = {}
                for link in by_link:
                    bid = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'gripper0_'+link)
                    transforms[link] = {'rotation':data.xmat[bid].reshape(3,3).tolist(),
                                        'translation_m':data.xpos[bid].tolist()}
                if label == 'proxy76':
                    row['body_world_transforms'] = transforms
                elif transforms != row['body_world_transforms']:
                    raise ValueError('Body FK changed between candidates')
                for other in sorted(set(by_link)-{'finger13_link'}):
                    pair = ('finger13_link',other)
                    pair_row = row['finger13_vs_other_hand_links'].setdefault(other,{})
                    pair_row[label] = saved.hull_distances(model,data,by_link,pair)
                source = saved.reuse_certificate(certificates,pose,PAIR,model,data)
                if label == 'proxy76':
                    row['source_finger13_finger42'] = source
                elif source != row['source_finger13_finger42']:
                    raise ValueError('Source certificate no longer matches exact saved state')
                if data.time != time_before or any(not np.array_equal(value,getattr(data,key))
                                                   for key,value in state_before.items()):
                    raise ValueError('Integration state changed in static query')
            if model_before != saved.fields(model):
                raise ValueError('Compiled model mutated in static query')
            model_audits[label] = {'model':str(path.relative_to(ROOT)),
                'sha256':saved.sha(path),'hand_collision_parts':sum(map(len,by_link.values())),
                'protected_state_buffers_unchanged':True,'model_public_fields_unchanged':True,
                'model_fields_checked':len(model_before)}
    # Pure geometry below: no MuJoCo queries, updates, reset, or dynamics.
    for pose in poses:
        row = results[pose['id']]
        row['independent_finger13_finger42'] = {}
        for label,parts in sources.items():
            comparisons = [common_ball(a,b,row['body_world_transforms'],
                                         pose['checkpoint_step']==6451)
                           for a in parts[PAIR[0]] for b in parts[PAIR[1]]]
            row['independent_finger13_finger42'][label] = comparisons
    after = {str(p.relative_to(ROOT)):saved.sha(p) for p in sorted(paths)}
    if before != after:
        raise ValueError('Original input files were modified')
    summary = []
    for row in results.values():
        for other,pair in row['finger13_vs_other_hand_links'].items():
            before_s = pair['proxy76']['query_summaries']
            after_s = pair['finger13_repair']['query_summaries']
            new = [a['cutoff_m'] for b,a in zip(before_s,after_s)
                   if a['eligible_negative_geom_pairs'] and not b['eligible_negative_geom_pairs']]
            if new:
                summary.append({'pose_id':row['pose']['id'],'other_link':other,
                                'new_eligible_negative_cutoffs_m':new,
                                'status':'QUERY_CANDIDATE_NEW_CONTACT_REQUIRES_GEOMETRIC_INTERPRETATION'})
                if other != 'finger42_link':
                    by_model = row.setdefault('new_negative_pair_geometry',{}).setdefault(other,{})
                    for label,parts in sources.items():
                        by_model[label] = [common_ball(a,b,row['body_world_transforms'],
                                                      row['pose']['checkpoint_step']==6451,
                                                      ('finger13_link',other))
                                           for a in parts['finger13_link'] for b in parts[other]]
    report = {'schema':'d3-finger13-local-repair-saved-static-v1','poses':len(poses),
        'body_pair_comparisons':len(poses)*16,'mujoco':mujoco.__version__,
        'numpy':np.__version__,'scipy':scipy.__version__,'model_audits':model_audits,
        'zero_dynamics_audit':api_audit,'input_sha256_before':before,'input_sha256_after':after,
        'all_input_files_unchanged':True,'new_negative_query_pairs':summary,
        'limits':['Saved pre/post poses only; no continuous motion or dynamics acceptance.',
                  'mj_geomDistance signed values depend on cutoff and are not source-triangle penetration depths.',
                  'LP numerical tolerances/padding are arithmetic robustness checks, not physical acceptance thresholds.',
                  'Source surfaces may be nonmanifold; source surface gaps do not classify source solid interiors.',
                  'New query overlap signs require actual part geometry interpretation; unchanged old overlaps may remain.'],
        'results':list(results.values())}
    output.mkdir(parents=True)
    saved.write(output/'contacts.json',report)
    saved.write(output/'poses.json',{'poses':poses})
    key_summary = []
    for row in results.values():
        entry = {'pose_id':row['pose']['id'],
                 'source_surface':row['source_finger13_finger42'], 'models':{}}
        for label in sources:
            independent = row['independent_finger13_finger42'][label]
            classes = [r['robust_classification'] for r in independent]
            classification = ('STRICT_CONVEX_INTERIOR_OVERLAP'
                              if 'STRICT_CONVEX_INTERIOR_OVERLAP' in classes else
                              'SEPARATED_CONVEX_SOLIDS'
                              if set(classes)=={'SEPARATED_CONVEX_SOLIDS'} else 'UNKNOWN')
            entry['models'][label] = {'union_pair_classification':classification,
                'individual_part_pair_classifications':classes,
                'common_ball_radii_mm_in_part_pair_order':[
                    r['tolerance_runs'][-1]['radius_lp_mm'] for r in independent],
                'convex_query':[
                    {'cutoff_m':r['cutoff_m'], 'minimum_distance_m':r['deepest']['distance_m'],
                     'eligible_negative_geom_pairs':r['eligible_negative_geom_pairs']}
                    for r in row['finger13_vs_other_hand_links']['finger42_link'][label]['query_summaries']]}
        key_summary.append(entry)
    saved.write(output/'summary.json',{'physics_steps':0,'poses':key_summary,
                                      'new_negative_query_pairs':summary})
    print(json.dumps({'output':str(output),'poses':len(poses),'pairs':len(poses)*16,
                      'physics_steps':0,'new_negative_query_pairs':summary,
                      'forbidden_api_attempts':api_audit['forbidden_api_attempts']}))


if __name__ == '__main__':
    main()
