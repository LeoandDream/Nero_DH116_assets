#!/usr/bin/env python3
"""Static acceptance of ONE fixed finger22 candidate at saved states only.

Reuse source clipping/coverage, actual mesh_graph hulls, AABB/projection and
verified common-ball LP. No Reset, forward, constraint solve or integration.
Native static collision sets are recorded, but independent solid queries are
used for the changed geometry (native MPR alone is not a source certificate).
"""
import argparse
from collections import Counter
import copy
import gzip
import itertools
import json
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET
for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
import numpy as np
from scipy.spatial import ConvexHull, cKDTree
import finger13_quality as Q
import wrist_external_preflight as X
import wrist_reuse as W
sys.path.insert(0, str(W.ROOT/'tools'))
import build_finger22_local as B

P, mj, G = W.P, W.mj, Q.G
ROOT = W.ROOT
HERE = W.HERE
RUN = HERE/'evidence/d3_wrist_conditional/run'
CANDIDATE = HERE/'candidates/finger22_local/model.xml'
BODY = 'gripper0_finger22_link'
SOURCE = B.SOURCE


def write(path, value):
    if path.exists():
        if W.read(path) != value: raise ValueError('Existing evidence differs: '+str(path))
        return
    W.write(path, value)


def sparse_certificate(lp):
    """Retain every nonzero dual term; omitted planes remain in frozen meshes.

    Classification and independent primal/dual verification use ALL planes
    before serialization. The common-ball center/minimum margin are retained,
    so primal verification is reproducible from the actual compiled hulls.
    """
    cert=lp['certificate']
    weights=np.asarray(cert['dual_weights'])
    ids=np.flatnonzero(weights != 0)
    lp['certificate']={'original_plane_count':len(weights),
        'nonzero_dual_plane_indices':ids.tolist(),
        'normals':np.asarray(cert['normals'])[ids].tolist(),
        'offsets_mm':np.asarray(cert['offsets_mm'])[ids].tolist(),
        'dual_weights':weights[ids].tolist(),
        'omitted_zero_weight_planes':'Reconstruct all primal halfspaces from frozen actual compiled hull vertices; no acceptance result omitted.'}
    return lp


def rows(path):
    with gzip.open(path, 'rt') as f:
        return [json.loads(line) for line in f]


def bindings(path, names):
    xml = ET.parse(path).getroot()
    assets = {a.get('name'): a for a in xml.findall('asset/mesh')}
    return [Q.V._binding(path, xml, assets, xml.find('.//geom[@name="'+n+'"]')) for n in names]


def parts_from_bindings(values):
    return [{'name': v['name'], 'triangles': Q.V.mesh_triangles(v)} for v in values]


def normalized(path, new):
    xml = ET.parse(path).getroot()
    xml.attrib.pop('model', None)
    for e in xml.findall('.//*[@file]'):
        p = (path.parent/e.get('file')).resolve()
        assert p.is_relative_to(ROOT)
        e.set('file', str(p.relative_to(ROOT))+':'+W.sha(p))
    if new:
        for e in list(xml.find('asset')):
            if e.get('name') in B.NEW: xml.find('asset').remove(e)
    body = xml.find('.//body[@name="'+BODY+'"]')
    for e in list(body.findall('geom')):
        if e.get('name') in (*B.NEW, B.OLD): body.remove(e)
    return W.canonical(xml)


def audit(a, b):
    # Reuse exact existing named-geometry compiled-field comparator.
    original = W.BODIES
    W.BODIES = (BODY,)
    try:
        result = W.compiled_audit(a, b, [B.OLD], list(B.NEW))
    finally:
        W.BODIES = original
    return result


def scene(libero, resources):
    original = W.RUN
    W.RUN = RUN
    try: xml = W.scene_xml(libero, resources)
    finally: W.RUN = original
    return xml


def replace(xml):
    candidate = ET.parse(CANDIDATE).getroot()
    body = xml.find('.//body[@name="'+BODY+'"]')
    old = body.find('geom[@name="'+B.OLD+'"]')
    assert old is not None
    body.remove(old)
    for n in B.NEW:
        body.append(copy.deepcopy(candidate.find('.//geom[@name="'+n+'"]')))
        e = copy.deepcopy(candidate.find('asset/mesh[@name="'+n+'"]'))
        e.set('file', str((CANDIDATE.parent/e.get('file')).resolve()))
        xml.find('asset').append(e)
    return xml


def quality(out, model):
    started = time.monotonic()
    src = P.geometry.read_stl(SOURCE)
    raw = bindings(CANDIDATE, B.NEW)
    compiled = [P.compiled_hull(model, n, v) for n,v in zip(B.NEW, raw)]
    old = bindings(B.BASE, [B.OLD])
    source_hull = G.convex_union_surface([{'name':'source_convex_reference_only', 'triangles':src}])
    old_union = G.convex_union_surface(parts_from_bindings(old))
    report = {}
    for label, parts in [('raw', parts_from_bindings(raw)), ('compiled', compiled)]:
        if (out/(label+'-quality.json.gz')).exists():
            value=W.read(out/(label+'-quality.json.gz'))
            cov=value['coverage_summary']; containment=value['source_convex_containment']
            fidelity=value['fidelity']; seams=value['seams']
        else:
            union = G.convex_union_surface(parts)
            # Compiled coverage needs all source faces, not the very expensive
            # optional intersection with the old reference of uncovered strips.
            cov = Q.coverage(src, union, old_union if label=='raw' else None)
            write(out/(label+'-coverage.json.gz'), cov)
            containment = Q.convex_containment(union, source_hull, 'manufacturer_source_convex_hull_not_solid')
            fidelity = G.analyze_link(src, parts, samples_per_direction=4096, seed=220,
                regions={'positive_X_working_face': {'axis':0,'min':.003},
                         'failure_waist': {'axis':2,'min':.0098,'max':.032}})
            normals = Q.normals(src, union)
            seams = []
            for cut in B.CUTS:
                ids = [i for i,h in enumerate(union.hulls) if abs(h['hi'][2]-cut)<1e-7 or abs(h['lo'][2]-cut)<1e-7]
                capfaces = np.max(abs(union.triangles[:,:,2]-cut), axis=1) <= G.CLIP_TOL_M
                seams.append({'z_m':cut, 'adjacent_part_ids':ids,
                    'exposed_coplanar_seam_area_m2':float(G._areas(union.triangles[capfaces]).sum()),
                    'note':'Only exactly coplanar exposed caps at original 1e-10m tolerance; tilted compiled seams also require distance/coverage checks.'})
            value = {'coverage_summary': {k:v for k,v in cov.items() if not k.endswith('_by_source_triangle_m2')},
                     'source_convex_containment':containment, 'fidelity':fidelity,'normals':normals,'seams':seams}
            write(out/(label+'-quality.json.gz'), value)
        report[label] = {'continuous_source_coverage': 'PASS' if cov['all_source_triangles_covered_with_numerical_tolerance'] else 'FAIL',
            'uncovered_area_m2':cov['uncovered_area_m2'], 'uncovered_source_triangles':cov['uncovered_source_triangle_count'],
            'source_convex_containment': 'PASS' if containment['all_parts_contained_with_numerical_tolerance'] else 'FAIL',
            'maximum_halfspace_outward_m':max(p['max_reference_halfspace_violation_m'] for p in containment['parts']),
            'fidelity':fidelity, 'seams':seams}
        print(label,'quality',report[label]['continuous_source_coverage'],flush=True)
    source_surface = G.TriangleSurface(src)
    raw_old_tri = Q.V.mesh_triangles(old[0])
    d1,_,_ = G.TriangleSurface(raw_old_tri).nearest(np.unique(src.reshape(-1,3),axis=0))
    d2,_,_ = source_surface.nearest(np.unique(raw_old_tri.reshape(-1,3),axis=0))
    changes = []
    for v, h in zip(raw, compiled):
        points = np.unique(Q.V.mesh_triangles(v).reshape(-1,3),axis=0)
        hullpoints = points[ConvexHull(points).vertices]
        changes.append({'geom':h['name'], 'raw_support_to_compiled_vertex_max_m':float(cKDTree(h['vertices']).query(hullpoints)[0].max()),
            'compiled_support_to_raw_vertex_max_m':float(cKDTree(hullpoints).query(h['vertices'])[0].max()),
            'actual_mesh_graph':h['audit']})
    report.update({'source_topology':P.geometry.mesh_topology(src), 'source_solid_equivalence':'UNKNOWN_NONMANIFOLD_SOURCE',
        'manufacturer_vs_delivered_raw_surface_vertex_queries':{'manufacturer_to_delivered_max_m':float(d1.max()),
            'delivered_to_manufacturer_max_m':float(d2.max()),'global_surface_maximum':'UNKNOWN_FINITE_VERTEX_QUERIES'},
        'compilation_vertex_changes':changes,'numeric_tolerance_m':G.CLIP_TOL_M,'elapsed_s':time.monotonic()-started})
    write(out/'quality-summary.json.gz', report)
    return report


def working_patch(model):
    """Finite checks on the two source faces identified before construction.

    Query all independent piece faces. For this patch verify the nearest points
    are away from cut interfaces and not strictly buried by any other solid;
    only then are these distances to the exposed union boundary as well.
    """
    src=P.geometry.read_stl(SOURCE)
    points,_=G._sample_surface(src[[6165,6176]],1024,seed=222)
    raw=bindings(CANDIDATE,B.NEW)
    result={'source_face_ids_zero_based':[6165,6176],'samples':1024,'seed':222,
        'limits':'Fixed finite area samples of the previously identified +X failure-facing source patch; not an entire working-face or global Hausdorff bound.'}
    for label,parts in [('raw',parts_from_bindings(raw)),('compiled',[P.compiled_hull(model,n,v) for n,v in zip(B.NEW,raw)])]:
        hulls=[G.convex_union_surface([p]) for p in parts]
        surface=G.TriangleSurface(np.concatenate([u.triangles for u in hulls]))
        distances,nearest,ids=surface.nearest(points)
        not_buried=all(np.all(u.membership_margin(nearest)>=-G.CLIP_TOL_M) for u in hulls)
        seam_gap=float(np.min(abs(nearest[:,2,None]-np.array(B.CUTS)[None,:])))
        result[label]={'observed_max_distance_m':float(distances.max()),'p95_m':float(np.quantile(distances,.95)),
            'minimum_nearest_point_to_cut_plane_m':seam_gap,'nearest_points_not_strictly_inside_any_piece':not_buried,
            'nearest_exposed_boundary_supported':bool(not_buried and seam_gap>G.CLIP_TOL_M)}
    return result


def relative(data, model, g, h):
    return X.relative_pair(model, data, g, h)


def classify(a, b, R, t):
    sep = P.separation(a,b,R,t)
    if sep['separation_lower_bound_m'] > P.TOL:
        return {'status':'SEPARATED','projection':sep}
    tr = {P.BODIES[0]:{'rotation':np.eye(3).tolist(),'translation_m':[0.,0.,0.]},
          P.BODIES[1]:{'rotation':R.tolist(),'translation_m':t.tolist()}}
    lp = P.lp(a,b,tr)
    lp.update(W.F.dual_bound(lp,a,b,R,t))
    label=W.F.status_from_lp(lp)
    return {'status':{'STRICT_CONVEX_OVERLAP':'STRICT_OVERLAP','UNKNOWN_NUMERICAL':'UNKNOWN'}.get(label,label), 'projection':sep, 'lp':sparse_certificate(lp)}


def native_sets(model, data, names):
    mj.mj_kinematics(model,data)
    mj.mj_comPos(model,data)
    mj.mj_collision(model,data)
    pairs = {}
    for c in data.contact[:data.ncon]:
        g,h = map(int,c.geom)
        if not P.saved.eligible(model,g,h): continue
        if not any(model.body(int(model.geom_bodyid[k])).name.startswith('gripper0_') for k in (g,h)): continue
        key = tuple(sorted((names[g],names[h])))
        pairs[key] = min(pairs.get(key,float('inf')),float(c.dist))
    return pairs


def scan(out, oldxml, newxml, oldmodel, model):
    states = [r for r in rows(RUN/'state.jsonl.gz') if r['phase']=='close']
    assert [r['step'] for r in states] == list(range(4001,4743))
    contacts = {r['step']:r for r in rows(RUN/'contacts.jsonl.gz')}
    origdata, data = mj.MjData(oldmodel), mj.MjData(model)
    names = {g:n for n,g in W.geom_names(model).items()}
    oldnames = {g:n for n,g in W.geom_names(oldmodel).items()}
    newids = [model.geom(n).id for n in B.NEW]
    oldid = oldmodel.geom(B.OLD).id
    others = [g for g in range(model.ngeom) if g not in newids and P.saved.eligible(model,newids[0],g)]
    assert model.npair == oldmodel.npair == 0
    oldother = {g:W.geom_names(oldmodel)[names[g]] for g in others}
    polys = X.load_polytopes(model, newxml, newids+others)
    oldpoly = X.load_polytopes(oldmodel, oldxml, [oldid])[oldid]
    src = P.geometry.read_stl(SOURCE)
    source_bvh = P.geometry.BVH(src)
    records, unchanged_errors, counts, changed_native = [], [], Counter(), []
    source_counts=Counter()
    source_queries = {}

    def source_query(g, R, t):
        key = (g,R.tobytes(),t.tobytes())
        if key not in source_queries:
            h=polys[g]
            if h['representation'] not in ('actual_compiled_mesh_graph','exact_box'):
                return {'classification':'UNKNOWN_CONSERVATIVE_PRIMITIVE_OBB'}
            if 'triangles' not in h:
                v=h['vertices']; h['triangles']=v[ConvexHull(v).simplices]; h['bvh']=P.geometry.BVH(h['triangles'])
            moved=h['triangles']@R.T+t
            q=P.geometry.surface_query(src,moved,source_bvh,h['bvh'].refit(moved))
            q['classification'] = ('SOURCE_SURFACE_INTERSECTION' if q['strict_transverse_witness'] else
                'SOURCE_SURFACES_SEPARATED_SOLID_CONTAINMENT_UNKNOWN' if q['surface_min_distance_m']>P.TOL else 'UNKNOWN_NEAR_BOUNDARY')
            q['other_geometry']='actual_compiled_other_collision_surface; source-ground-truth exact for native bowl boxes'
            source_queries[key]=q
        return source_queries[key]

    for i,row in enumerate(states):
        step=row['step']
        data.qpos[:]=origdata.qpos[:]=row['full_qpos']
        a=native_sets(oldmodel,origdata,oldnames); b=native_sets(model,data,names)
        au={k:v for k,v in a.items() if B.OLD not in k}; bu={k:v for k,v in b.items() if not set(B.NEW)&set(k)}
        if au!=bu: unchanged_errors.append({'step':step,'old':[[list(k),v] for k,v in au.items()], 'new':[[list(k),v] for k,v in bu.items()]})
        oldchanged={tuple(sorted('finger22_union' if n==B.OLD else n for n in k)) for k in a if B.OLD in k}
        newchanged={tuple(sorted('finger22_union' if n in B.NEW else n for n in k)) for k in b if set(B.NEW)&set(k)}
        if oldchanged!=newchanged:
            changed_native.append({'step':step,'removed':[list(k) for k in sorted(oldchanged-newchanged)],'added':[list(k) for k in sorted(newchanged-oldchanged)]})
        lo,hi=X.world_aabbs(model,data)
        olo,ohi=X.world_aabbs(oldmodel,origdata)
        fbody=model.body(BODY).id
        rf=data.xmat[fbody].reshape(3,3)
        srcworld=src.reshape(-1,3)@rf.T+data.xpos[fbody]
        guard=64*np.finfo(float).eps
        slo,shi=srcworld.min(0)-guard,srcworld.max(0)+guard
        events=[]
        for g in others:
            # All four new solids plus old solid vs EVERY eligible other geom.
            oldg=oldother[g]
            newgap=np.array([np.maximum(lo[g]-hi[k],lo[k]-hi[g]).max() for k in newids])
            oldgap=float(np.maximum(olo[oldg]-ohi[oldid],olo[oldid]-ohi[oldg]).max())
            sourcegap=float(np.maximum(lo[g]-shi,slo-hi[g]).max())
            counts['eligible_other_geom_states']+=1
            if newgap.min()>P.TOL and oldgap>P.TOL and sourcegap>P.TOL:
                counts['whole_geometry_AABB_separated_old_and_new']+=1;continue
            R,t=relative(data,model,newids[0],g)
            oldq=({'status':'SEPARATED_AABB','lower_bound_m':oldgap} if oldgap>P.TOL else classify(oldpoly,polys[g],R,t))
            qs=[({'status':'SEPARATED_AABB','lower_bound_m':float(gap)} if gap>P.TOL else classify(polys[k],polys[g],R,t))
                for k,gap in zip(newids,newgap)]
            overlap=any(q['status']=='STRICT_OVERLAP' for q in qs)
            separated=all(q['status'].startswith('SEPARATED') for q in qs)
            status='STRICT_OVERLAP' if overlap else 'SEPARATED' if separated else 'UNKNOWN'
            counts['old_'+oldq['status']]+=1;counts['new_'+status]+=1
            event={'other_geom':names[g],'old':oldq,'new_pieces':qs,'new_union_status':status}
            if sourcegap>P.TOL:
                ground={'classification':'SOURCE_SURFACE_SEPARATED_AABB','separation_lower_bound_m':sourcegap}
            else:
                ground=source_query(g,R,t)
            event['source_surface']=ground
            sc=ground['classification']
            if sc=='SOURCE_SURFACE_INTERSECTION':
                category='TRUE_SURFACE_CONTACT_RETAINED' if overlap else 'MISSED_SOURCE_SURFACE_INTERSECTION' if separated else 'UNKNOWN_PROXY_AT_SOURCE_INTERSECTION'
            elif sc.startswith('SOURCE_SURFACE'):
                category=('NEW_PROXY_OVERLAP_WITH_SOURCE_SURFACE_GAP_SOLID_UNKNOWN' if overlap else 'SEPARATED_SURFACES_AND_PROXY' if separated else 'UNKNOWN_PROXY_AT_SOURCE_GAP')
            else: category='UNKNOWN_REFERENCE_GEOMETRY'
            event['contact_assessment']=category
            source_counts[category]+=1
            events.append(event)
        records.append({'step':step,'alignment':'saved post-step full_qpos FK; not RK4 contact cache',
            'old_native_contact_pairs':[[list(k),v] for k,v in sorted(a.items())],
            'new_native_contact_pairs':[[list(k),v] for k,v in sorted(b.items())],
            'changed_geometry_narrowphase':events})
        if i%100==0: print('post-state',step,'/',4742,flush=True)
    write(out/'close-states.json.gz',records)
    write(out/'native-contact-changes.json',changed_native)
    write(out/'unchanged-contact-differences.json',unchanged_errors)
    # Use ONLY exact cached transforms at the two required witness states.
    witnesses=[]
    bowlids=[g for g in others if model.body(int(model.geom_bodyid[g])).name=='white_bowl_1_main']
    for step in (4741,4742):
        cached={body:pose for c in contacts[step]['contacts'] for body,pose in zip(c['body'],c['cached_body_pose'])}
        finger,bowl=cached[BODY],cached['white_bowl_1_main']
        rf,rb=(np.asarray(p['rotation_world']).reshape(3,3) for p in (finger,bowl))
        pf,pb=(np.array(p['position_world_m']) for p in (finger,bowl))
        R,t=rf.T@rb,rf.T@(pb-pf)
        queries=[]
        for g in bowlids:
            qs=[classify(polys[k],polys[g],R,t) for k in newids]
            queries.append({'other_geom':names[g], 'new_pieces':qs,
                'source_surface':source_query(g,R,t) if names[g] in ('white_bowl_1_g22','white_bowl_1_g23') or any(q['status']!='SEPARATED' for q in qs) else None})
        witnesses.append({'step':step,'alignment':contacts[step]['timing'],'cached_finger22_body_pose':finger,
            'cached_bowl_body_pose':bowl,'queries':queries})
    write(out/'rk4-witnesses.json.gz',witnesses)
    at4518={b for c in contacts[4518]['contacts'] for b in c['body']}
    assert BODY not in at4518
    result={'saved_close_post_states':len(states),'state_range':[4001,4742],
        'eligible_other_geoms_per_state':len(others),'all_qualified_hand_pairs_unchanged_outside_finger22':not unchanged_errors,
        'unchanged_native_contact_differences':len(unchanged_errors), 'native_changed_geom_state_count':len(changed_native),
        'counts':dict(counts),'required_cached_states':[4741,4742],
        'source_surface_assessment_counts':dict(source_counts),
        'false_contact_interpretation':'Positive source surface gap + proxy overlap is a surface-based false-contact witness; solid containment remains UNKNOWN for nonmanifold source.',
        'missed_contact_interpretation':'All eligible f22-source-vs-unchanged-other collision geometries checked with whole-source AABB exclusion or full triangle surface query, including when old/new proxies are separated. Other hand proxy approximation is frozen, not revalidated against its manufacturer STL.',
        'cached_all_bowl_boxes_separated':all(q['status']=='SEPARATED' for w in witnesses for p in w['queries'] for q in p['new_pieces']),
        'step4518':'POST_STEP_FK_ONLY; no saved finger22 RK4 cache; original first positive force contact is finger13',
        'source_solid_contacts':'UNKNOWN_NONMANIFOLD_SOURCE; surface crossings/gaps are reported without a solid-containment claim',
        'dynamic_prediction':'UNKNOWN_NOT_RUN; saved poses are not a new trajectory or force counterfactual',
        'physics_steps':0}
    write(out/'scan-summary.json',result)
    return result


def main(out, libero):
    if (out/'summary.json').exists(): raise ValueError('Completed evidence immutable; use a new output')
    out.mkdir(parents=True,exist_ok=True)
    manifest=W.read(CANDIDATE.parent/'manifest.json')
    if mj.__version__!='3.2.3': raise ValueError('Actual mesh_graph reader requires frozen MuJoCo 3.2.3')
    assert W.sha(CANDIDATE)==manifest['candidate_model_sha256']
    assert normalized(B.BASE,False)==normalized(CANDIDATE,True)
    resources=[]
    oldxml=scene(libero,resources); newxml=replace(copy.deepcopy(oldxml))
    P.saved.SAFE_APIS={'mj_id2name','mj_name2id','mj_kinematics','mj_comPos','mj_collision'}
    with P.saved.static_only() as api:
        old=mj.MjModel.from_xml_path(str(B.BASE)); new=mj.MjModel.from_xml_path(str(CANDIDATE))
        standalone=audit(old,new)
        hand=P.saved.hand_geoms(new)
        assert sum(map(len,hand.values()))==81
        oldscene=mj.MjModel.from_xml_string(ET.tostring(oldxml,encoding='unicode'))
        newscene=mj.MjModel.from_xml_string(ET.tostring(newxml,encoding='unicode'))
        sceneaudit=audit(oldscene,newscene)
        write(out/'input-audit.json',{'standalone':standalone,'scene':sceneaudit,'normalized_XML_and_all_old_resource_bytes_exact':True,
            'hand_geoms':{k:len(v) for k,v in hand.items()},'frozen_other_hand_count':77,'frozen_wrist_count':306,
            'source_sha256':W.sha(SOURCE),'baseline_candidate_sha256':W.sha(B.BASE),'candidate_sha256':W.sha(CANDIDATE),
            'saved_state_sha256':W.sha(RUN/'state.jsonl.gz'),'saved_contacts_sha256':W.sha(RUN/'contacts.jsonl.gz')})
        write(out/'scene-resources.json',resources)
        q=W.read(out/'quality-summary.json.gz') if (out/'quality-summary.json.gz').exists() else quality(out,new)
        write(out/'working-face-patch.json',working_patch(new))
        s=scan(out,oldxml,newxml,oldscene,newscene)
        api['collision_calls']=api['allowed_api_calls'].get('mj_collision',0)
        api['kinematics_calls']=api['allowed_api_calls'].get('mj_kinematics',0)
        api['com_position_calls']=api['allowed_api_calls'].get('mj_comPos',0)
        write(out/'api-audit.json',api)
    surface=s['source_surface_assessment_counts']
    contact_ok=(s['all_qualified_hand_pairs_unchanged_outside_finger22'] and not s['counts'].get('new_UNKNOWN',0)
        and not any(v for k,v in surface.items() if k.startswith(('UNKNOWN','MISSED','NEW_PROXY_OVERLAP'))))
    summary={'quality':{k:{n:v for n,v in q[k].items() if n!='fidelity'} for k in ('raw','compiled')},'scan':s,
        'contact_surface_regression':'PASS' if contact_ok else 'FAIL_OR_UNKNOWN_BLOCKED',
        'static_status':'PASS_SURFACE_SCOPE_ONLY' if all(q[k]['continuous_source_coverage']=='PASS' and q[k]['source_convex_containment']=='PASS' for k in ('raw','compiled')) and s['cached_all_bowl_boxes_separated'] and contact_ok else 'FAIL_BLOCKED',
        'dynamics':'UNKNOWN_NOT_RUN','release':'NOT_APPROVED','physics_steps':0}
    write(out/'summary.json',summary)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--libero-assets',type=Path,required=True)
    args=parser.parse_args();main(args.output.resolve(),args.libero_assets.resolve())
