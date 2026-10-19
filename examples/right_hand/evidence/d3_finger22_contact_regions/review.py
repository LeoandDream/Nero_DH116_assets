#!/usr/bin/env python3
"""One offline geometric/semantic review of frozen RK4 caches; never integrate."""
import argparse
from collections import Counter, defaultdict
import gzip
import hashlib
import itertools
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
import numpy as np
import mujoco

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'examples/right_hand'))
import finger22_contact_regions as C
import finger22_static as F

OLD=ROOT/'examples/right_hand/evidence/d3_finger22_dynamic'
RISK=ROOT/'examples/right_hand/evidence/d3_finger22_risk_review'
SOURCE_IDS=(6165,6176)  # Existing source-patch witnesses, NOT a whole-front certificate.


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,obj):
    raw=(json.dumps(obj,indent=2)+'\n').encode()
    if p.suffix=='.gz':
        with gzip.open(p,'wb')as f:f.write(raw)
    else:p.write_bytes(raw)


def rows(path):
    with gzip.open(path,'rt')as f:
        for line in f:yield json.loads(line)


def pose(p):return np.array(p['rotation_world']).reshape(3,3),np.array(p['position_world_m'])


def box_geometry(geom,cache,body):
    r,p=pose(cache['body_poses'][body]);rf,pf=pose(cache['body_poses'][F.BODY])
    local_r=F.P.reader.BUILDER.quaternion_matrix(geom.get('quat','1 0 0 0'))
    local_p=np.fromstring(geom.get('pos','0 0 0'),sep=' ')
    size=np.fromstring(geom.get('size'),sep=' ')
    rr=rf.T@r@local_r;tt=rf.T@(r@local_p+p-pf)
    v=np.array(list(itertools.product((-1,1),repeat=3)))*size
    moved=v@rr.T+tt
    # Arithmetic-only coordinate envelope: >=256 elementary binary64 ops per
    # component, with intermediate absolute scale bounded by input norms.
    # The transpose/inverse discrepancy of the stored body rotation is added.
    unit=2.**-53;gamma=256*unit/(1-256*unit)
    scale=(1+max(np.max(abs(p)),np.max(abs(pf)))+np.max(size)+np.max(abs(local_p)))
    scale*=max(1.,np.linalg.norm(rf,np.inf))*max(1.,np.linalg.norm(r,np.inf))*max(1.,np.linalg.norm(local_r,np.inf))
    orth=np.linalg.norm(rf.T@rf-np.eye(3),np.inf)
    bound=2*(gamma+orth)*scale
    exact=C.affine_vertices(v,rr,tt)
    return moved,{'coordinate_arithmetic_pair_guard_m':float(bound),'operation_bound':256,
                  'unit_roundoff':unit,'intermediate_scale_bound_m':float(scale),
                  'cached_frame_orthogonality_residual_inf':float(orth),
                  'scope':'Conditioned on saved binary64 RK4 body poses and compiled geom fields. Not MPR localization error, quantization waiver, sensor or physical model accuracy.'},exact


def main(output):
    if output.exists():raise ValueError('New output only; old evidence immutable')
    output.mkdir(parents=True)
    files=subprocess.check_output(['git','ls-files','-z'],cwd=ROOT).decode().strip('\0').split('\0')
    protected={p:sha(ROOT/p)for p in files}
    write(output/'protected-before.json.gz',{'head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT).decode().strip(),
        'tracked_sha256':protected,'user_readme_sha256':sha('/data_robot/projects/nero_dh116_assets/README.md')})
    forbidden={}
    for name in ('mj_step','mj_step1','mj_step2','mj_forward','mj_collision','mj_kinematics','mj_comPos','MjData'):
        def blocked(*args,_name=name,**kwargs):
            forbidden[_name]=forbidden.get(_name,0)+1
            raise RuntimeError('Offline review forbids '+_name)
        setattr(mujoco,name,blocked)
    if mujoco.__version__!='3.2.3':raise ValueError('Frozen compiler version required')
    assert sha(F.CANDIDATE)=='7f9d84061c3e7d176763c688d2389f8b3d3e38386b8e990a0fa047ff2bc3329c'
    model=mujoco.MjModel.from_xml_path(str(F.CANDIDATE))
    cache_hulls={};hull_audits={}
    def get_hull(name):
        if name not in cache_hulls:
            bind=F.bindings(F.CANDIDATE,[name])[0];h=F.P.compiled_hull(model,name,bind)
            cache_hulls[name]=C.CertifiedHull(h['vertices'])
            hull_audits[name]={'certificate':cache_hulls[name].certificate,
                'mesh_graph_support_vertices':len(h['vertices']),'mesh_graph_triangles':len(h['triangles']),
                'support_vertices_sha256':hashlib.sha256(h['vertices'].tobytes()).hexdigest(),
                'graph_face_ids_sha256':hashlib.sha256(np.asarray(h['audit']['face_vertex_ids'],dtype=np.int64).tobytes()).hexdigest()}
        return cache_hulls[name]
    pieces=[get_hull(n)for n in F.B.NEW]
    source=F.P.geometry.read_stl(F.SOURCE);working=source[list(SOURCE_IDS)]
    witnesses=F.W.read(RISK/'witnesses.json')
    q=np.array(witnesses['buried_source']['compiled_nearest_support_triangle_m'])
    n=np.cross(q[1]-q[0],q[2]-q[0]);n/=np.linalg.norm(n);cavity=[np.r_[n,-n@q[0]].tolist()]
    quant=max(p['paired_vertex_error_upper_bound_m']for p in F.W.read(RISK/'numeric.json')['pieces'])
    # Audit every support facet; UNKNOWN completes coverage rather than silently
    # considering unmarked pieces or +X normals to be functional.
    semantic_audit={}
    for part,h in enumerate(pieces):
        entries=[]
        for i,face in enumerate(h.faces):
            label=C.semantic_patch(h.vertices[face],part,working,F.B.CUTS,quant,cavity)
            entries.append({'face':i,'status':label['status']})
        semantic_audit[F.B.NEW[part]]={'counts':dict(Counter(e['status']for e in entries)),
            'all_faces_labeled':len(entries)==len(h.faces),'faces':entries}
    selected={};first_pairs={}
    for record in rows(OLD/'run/contacts.jsonl.gz'):
        for contact in record['contacts']:
            if ('white_bowl_1_main'in contact['body']and any(b.startswith('gripper0_')for b in contact['body'])
                    and contact['force_contact_frame'][0]>0):
                key=tuple(contact['geom'])
                if key not in first_pairs:first_pairs[key]=(record,contact)
        if record['step'] in (4518,4663,4730,4731):selected[record['step']]=record
    final=selected[4731]
    scene=ET.parse(OLD/'run/model.xml').getroot()
    bowl_body=scene.find('.//body[@name="white_bowl_1_main"]')
    boxes=[g for g in bowl_body.findall('geom')if g.get('type')=='box'and(int(g.get('contype','1'))or int(g.get('conaffinity','1')))]
    assert len(boxes)==36
    all_pairs=[];actual_candidates=[];native_g22=None
    for box in boxes:
        vertices,arithmetic,exact=box_geometry(box,final,'white_bowl_1_main');b=C.CertifiedHull(vertices,exact)
        for part,a in enumerate(pieces):
            cert=C.intersection_certificate(a,b,arithmetic['coordinate_arithmetic_pair_guard_m'])
            row={'box':box.get('name'),'piece':F.B.NEW[part],'certificate':cert,'arithmetic':arithmetic}
            if cert['status']!='SEPARATED':
                candidates=C.contact_faces(a,b,arithmetic['coordinate_arithmetic_pair_guard_m'])
                for candidate in candidates:
                    candidate['semantic']=C.semantic_patch(candidate['polygon_exact_m'],part,working,F.B.CUTS,
                        max(quant,arithmetic['coordinate_arithmetic_pair_guard_m']),cavity)
                row['all_candidates']=candidates
                row['decision']=C.decision([c['semantic']['status']for c in candidates])
                actual_candidates.append(row)
            if box.get('name')=='white_bowl_1_g22':
                if part==1:native_g22=(b,arithmetic)
            all_pairs.append(row)
    assert native_g22 is not None
    b,arithmetic=native_g22
    source_query=F.P.geometry.surface_query(source,b.vertices[b.faces])
    # Regression of the FIRST actual positive-force cache for every distinct
    # native hand/bowl pair (12 pairs), without repeating the 742-state sweep.
    regress=[]
    for (bowl_geom,hand_geom),(cache,c) in first_pairs.items():
        bg=scene.find('.//geom[@name="'+bowl_geom+'"]')
        vertices,ar,exact=box_geometry(bg,cache,'white_bowl_1_main')
        rf,pf=pose(cache['body_poses'][F.BODY]);rh,ph=pose(cache['body_poses'][c['body'][1]])
        local=get_hull(hand_geom)
        # Transform the other saved hand body into the same cached frame.
        moved=C.affine_vertices(local.points,rf.T@rh,rf.T@(ph-pf))
        transformed=C.CertifiedHull(moved,moved,local.faces)
        other=C.CertifiedHull(vertices,exact);cert=C.intersection_certificate(transformed,other,ar['coordinate_arithmetic_pair_guard_m'])
        candidates=C.contact_faces(transformed,other,ar['coordinate_arithmetic_pair_guard_m'])
        regress.append({'step':cache['step'],'geom':[bowl_geom,hand_geom],
            'native_distance_m':c['distance_m'],'native_normal_force_N':c['force_contact_frame'][0],
            'certificate':cert,'all_face_ids':[c['face']for c in candidates],
            'all_normals':[c['normal']for c in candidates],
            'semantic':'finger22 audited separately; other77 not relabeled'})
    synthetic=[]
    points=[('root_bore',0,witnesses['reverse_proxy']['same_witness_compiled_boundary_check']['compiled_nearest_point_m'],1e-4),
            ('cavity_closure',1,witnesses['buried_source']['compiled_nearest_surface_point_m'],1e-4)]
    working_center=working[0].mean(0)
    local_surface=F.G.TriangleSurface(F.P.compiled_hull(model,F.B.NEW[1],F.bindings(F.CANDIDATE,[F.B.NEW[1]])[0])['triangles'])
    _,near,_=local_surface.nearest(working_center[None])
    points.append(('registered_functional_patch',1,near[0],1e-6))
    for label,part,point,half in points:
        vertices=np.array(list(itertools.product((-1,1),repeat=3)))*half+point
        box=C.CertifiedHull(vertices);cs=C.contact_faces(pieces[part],box,0.)
        for c in cs:c['semantic']=C.semantic_patch(c['polygon_exact_m'],part,working,F.B.CUTS,quant,cavity)
        synthetic.append({'label':label,'synthetic_static_only':True,'half_size_m':half,'center_m':list(map(float,point)),
            'certificate':C.intersection_certificate(pieces[part],box,0.),'all_candidates':cs,
            'decision':C.decision([c['semantic']['status']for c in cs])})
    # Exact source-edge adjacency is evidence of proximity, not permission to
    # promote face6175 into the old two-face functional registry.
    edge_map=defaultdict(list)
    for i,t in enumerate(source):
        for a,b in zip(t,np.roll(t,-1,axis=0)):edge_map[tuple(sorted((tuple(a),tuple(b))))].append(i)
    shared=[list(v)for v in edge_map.values()if 6175 in v and 6176 in v]
    source6175=C.source_front_certificate(source[6175],source,6175,0.)
    refinements=[]
    for row in actual_candidates:
        statuses=[]
        for candidate in row['all_candidates']:
            status=candidate['semantic']['status']
            if row['piece']==F.B.NEW[1] and status.startswith('UNKNOWN'):
                poly=candidate['polygon_exact_m']
                envelope=C.source_front_certificate(poly,source,6175,quant)
                provenance=C.semantic_patch(poly,1,source[[6175]],F.B.CUTS,quant,cavity)
                proved=(source6175['certified'] and envelope['certified'] and shared==[[6175,6176]]
                        and provenance['status']=='FUNCTIONAL_SOURCE_PATCH_APPROXIMATION')
                refinement={'face':candidate['face'],'source_face':6175,'whole_source_face_envelope':source6175,
                    'contact_patch_envelope':envelope,'projected_provenance':provenance,
                    'manifold_source_edge_to_existing_workface6176':shared==[[6175,6176]],
                    'local_front_provenance_confirmed':proved,
                    'scope':'This contact patch only; do not mark entire proxy facet285 or all +X faces as functional. No geometry-fidelity waiver.'}
                refinements.append(refinement)
                if proved:status='FUNCTIONAL_SOURCE_PATCH_APPROXIMATION'
            statuses.append(status)
        row['refined_decision']=C.decision(statuses)
    result={'baseline_commit':'0640c7bc2b16a703ecefda953c94cdb602002659',
        'authorization':'https://github.com/LeoandDream/Nero_DH116_assets/pull/4#issuecomment-6273980014',
        'status':'GEOMETRIC_AND_LOCAL_FRONT_PROVENANCE_CERTIFIED_GLOBAL_SEMANTICS_PARTIAL',
        'frame':'actual saved final RK4 finger22 body frame; no post-step FK',
        'model_sha256':sha(F.CANDIDATE),'registered_source_face_ids':SOURCE_IDS,
        'functional_scope':'Only pre-existing source-patch witnesses, not a whole-front certification or material model.',
        'all36_boxes_four_pieces_pairs':len(all_pairs),'pair_counts':dict(Counter(r['certificate']['status']for r in all_pairs)),
        'all_nonseparated_pairs':actual_candidates,'g22_four_piece_certificates':[r for r in all_pairs if r['box']=='white_bowl_1_g22'],
        'source_complete_g22_surface_query':source_query,'source_topology':F.P.geometry.mesh_topology(source),
        '6175_6176_shared_edges':shared,'all_related_native_pair_regressions':regress,
        'source6175_contact_refinements':refinements,
        'synthetic_frozen_geometry_witnesses':synthetic,'all_facet_semantic_coverage':semantic_audit,
        'hull_certificates':hull_audits,'formal_numeric_status':'FAIL_UNCHANGED_1e-10m',
        'original_4731_early_stop':'UNCHANGED_UNKNOWN','new_dynamic_steps':0,'new_Reset_or_FK_or_collision':0,
        'forbidden_API_calls':forbidden,'protected_original_files_unchanged':all(sha(ROOT/p)==h for p,h in protected.items()),
        'user_readme_unchanged':sha('/data_robot/projects/nero_dh116_assets/README.md')==F.W.read(output/'protected-before.json.gz')['user_readme_sha256'],
        'source_files_sha256':{str(p.relative_to(ROOT)):sha(p)for p in (Path(__file__),ROOT/'examples/right_hand/finger22_contact_regions.py',F.SOURCE,F.CANDIDATE,OLD/'run/contacts.jsonl.gz')},
        'limits':'Exact predicates certify the frozen binary64 support-vertex hulls. Geometric arithmetic guard is independent of MPR and does not amend formal fidelity standards. Complete physical cavity/material semantics remain UNKNOWN.'}
    write(output/'evidence.json.gz',result)
    print(json.dumps({k:result[k]for k in ('status','pair_counts','g22_four_piece_certificates','6175_6176_shared_edges','protected_original_files_unchanged','forbidden_API_calls')},indent=2))
    print('synthetic',[(s['label'],s['decision'])for s in synthetic])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    main(parser.parse_args().output.resolve())
