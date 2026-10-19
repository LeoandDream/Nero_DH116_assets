#!/usr/bin/env python3
"""Read-only witness attribution; never rebuild proxies or repeat 742 poses.

Reuse frozen surface samples, saved narrowphase certificates and contact-cache
transforms. Five selected saved post poses use FK only for assembly context.
"""
import argparse
import gzip
import itertools
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'): os.environ[k]='1'
import numpy as np
from scipy.spatial import ConvexHull
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'examples/right_hand'))
import finger22_static as F
OLD=F.HERE/'evidence/d3_finger22_local'
SELECTED=(4001,4518,4641,4664,4742)


def read(p): return F.W.read(p)
def dump(p,v): F.W.write(p,v)


def half_ulp(x):
    """Componentwise IEEE binary32 round-to-nearest error radius, not a tolerance."""
    a=np.asarray(x,dtype=np.float32)
    up=np.nextafter(a,np.float32(np.inf),dtype=np.float32).astype(float)-a.astype(float)
    down=a.astype(float)-np.nextafter(a,np.float32(-np.inf),dtype=np.float32).astype(float)
    return .5*np.maximum(up,down).max(axis=0)


def numeric(model,raw):
    report=[]
    eps=np.finfo(float).eps/2
    gamma=lambda n:n*eps/(1-n*eps)
    for bind in raw:
        assert bind['scale']==[1.,1.,1.] and bind['pos']==[0.,0.,0.] and bind['quat']==[1.,0.,0.,0.]
        p,_=F.Q.V.BUILDER.read_mesh(bind['file'])
        g=model.geom(bind['name']).id; mid=int(model.geom_dataid[g])
        c=np.array(model.geom_pos[g]);R=F.P.reader.BUILDER.quaternion_matrix(' '.join(map(str,model.geom_quat[g])))
        adr,num=int(model.mesh_vertadr[mid]),int(model.mesh_vertnum[mid])
        actual=np.asarray(model.mesh_vert[adr:adr+num],dtype=float)
        a=p.astype('f').astype(float)
        b=(a-c).astype('f').astype(float)
        z=(b@R).astype('f').astype(float)
        assert p.shape==actual.shape and np.array_equal(z,actual)
        recovered=actual@R.T+c
        d0,d1,d2=(half_ulp(x) for x in (a,b,z))
        orth=float(np.linalg.norm(R@R.T-np.eye(3),'fro'))
        rn=1+orth+64*eps
        sub64=gamma(1)*np.max(np.linalg.norm(abs(a)+abs(c),axis=1))
        rot64=gamma(5)*np.max(np.linalg.norm(abs(b)@abs(R),axis=1))
        back64=gamma(6)*np.max(np.linalg.norm(abs(z)@abs(R.T)+abs(c),axis=1))
        cancellation=orth*np.max(np.linalg.norm(a-c,axis=1))
        arithmetic_guard=128*eps*(1+np.max(abs(p)))
        bound=float(np.linalg.norm(d0)+rn*rn*(np.linalg.norm(d1)+sub64)+rn*(np.linalg.norm(d2)+rot64)+cancellation+back64+arithmetic_guard)
        measured=float(np.max(np.linalg.norm(recovered-p,axis=1)))
        assert measured <= bound
        hull=F.P.compiled_hull(model,bind['name'],bind)
        ch=ConvexHull(hull['vertices'])
        graph_residual=float(np.max(recovered@ch.equations[:,:3].T+ch.equations[:,3]))
        report.append({'geom':bind['name'],'vertices':len(p),'mesh_vert_dtype':str(model.mesh_vert.dtype),
            'all_staged_vertices_bitwise_match':True,'compiled_center_m':c.tolist(),'compiled_rotation':R.tolist(),
            'half_ulp_component_bounds_m':[x.tolist() for x in (d0,d1,d2)],'gamma1_subtraction_m':float(sub64),
            'gamma5_rotation_m':float(rot64),'gamma6_back_transform_m':float(back64),'orthogonality_residual_frobenius':orth,
            'cancellation_error_bound_m':float(cancellation),'explicit_float64_arithmetic_guard_m':float(arithmetic_guard),
            'paired_vertex_error_upper_bound_m':bound,'observed_paired_vertex_max_m':measured,
            'all_compiled_vertices_vs_actual_mesh_graph_halfspace_residual_m':graph_residual})
    # Attribute the frozen maximum plane violation to its original vertex.
    q=read(OLD/'compiled-quality.json.gz')['source_convex_containment']['parts'][-1]['max_violation_witness']
    name=F.B.NEW[-1];bind=raw[-1];p,_=F.Q.V.BUILDER.read_mesh(bind['file']);g=model.geom(name).id
    R=F.P.reader.BUILDER.quaternion_matrix(' '.join(map(str,model.geom_quat[g])));c=model.geom_pos[g]
    a=p.astype('f').astype(float);b=(a-c).astype('f').astype(float);z=(b@R).astype('f').astype(float);back=z@R.T+c
    i=int(np.argmin(np.linalg.norm(back-np.array(q['point_m']),axis=1)))
    assert np.linalg.norm(back[i]-q['point_m'])<1e-16
    normal=np.array(q['reference_plane'][:3]);offset=q['reference_plane'][3]
    terms={'OBJ_float32_parse':a[i]-p[i],'center_float32_writeback':R@(R.T@(b[i]-(a[i]-c))),
           'rotation_float32_writeback':R@(z[i]-b[i]@R),
           'remaining_float64_roundtrip':back[i]-p[i]-(a[i]-p[i])-R@(R.T@(b[i]-(a[i]-c)))-R@(z[i]-b[i]@R)}
    return {'pieces':report,'maximum_plane_violation_attribution':{'geom':name,'OBJ_vertex_zero_based':i,
        'raw_point_m':p[i].tolist(),'compiled_body_point_m':back[i].tolist(),'plane':q['reference_plane'],
        'raw_plane_residual_m':float(normal@p[i]+offset),'compiled_plane_residual_m':float(normal@back[i]+offset),
        'signed_plane_error_contributions_m':{k:float(normal@v) for k,v in terms.items()}},
        'bound_formula':'B=||h0||2 + r^2(||h1||2+gamma1_sub) + r(||h2||2+gamma5_rot) + ||RR^T-I||F max||a-c||2 + gamma6_back + explicit_float64_guard; r>=||R||2. h are input-scale half-ULP radii, not observed error maxima.',
        'assumptions':['IEEE binary32 nearest rounding, binary64 arithmetic; no overflow/underflow in these observed stages.',
            'Default mesh refpos/refquat/scale and identity XML geom pose, checked against bindings.',
            'Conditioned on recorded compiled center/rotation, which are shared by forward and inverse transforms; no claim about inertia/eigenvector physical accuracy.',
            'All three-stage reconstructed arrays exactly match actual 3.2.3 mesh_vert; rotation/centroid uncertainty cancels except measured orthogonality residual.',
            'Vertex pairing gives the same conservative bound for convex-hull Hausdorff distance provided actual mesh_graph encloses all compiled vertices; graph residual is reported separately.'],
        'original_threshold_m':1e-10,'formal_numeric_status':'FAIL_UNCHANGED_NOT_A_NEW_TOLERANCE'}


def witnesses(model,raw,compiled):
    source=F.P.geometry.read_stl(F.SOURCE)
    oldraw=read(OLD/'raw-quality.json.gz');oldcomp=read(OLD/'compiled-quality.json.gz')
    depth=oldraw['fidelity']['source_to_proxy_union']['max_witness']
    reverse=oldraw['fidelity']['proxy_union_to_source']['max_witness']
    samples,ids=F.G._sample_surface(source,4096,220)
    k=int(np.argmin(np.linalg.norm(samples-np.array(depth['query_point_m']),axis=1)))
    assert np.linalg.norm(samples[k]-depth['query_point_m'])<1e-16
    patch,local_ids=F.G._sample_surface(source[[6165,6176]],1024,222)
    surfaces=F.G.TriangleSurface(np.concatenate([p['triangles'] for p in compiled]))
    d,nearest,faces=surfaces.nearest(patch);i=int(np.argmax(d))
    source_face=(6165,6176)[int(local_ids[i])]
    reverse_part=[]
    for bind in raw:
        d0,_,_=F.G.TriangleSurface(F.Q.V.mesh_triangles(bind)).nearest(np.array(reverse['query_point_m'])[None])
        reverse_part.append(float(d0[0]))
    compiled_depth=oldcomp['fidelity']['source_to_proxy_union']['max_witness']
    out={'buried_source':{'source_face_zero_based':int(ids[k]),'source_triangle_m':source[ids[k]].tolist(),
          'piece':depth['nearest_proxy_part'],**depth,
          'compiled_nearest_surface_point_m':compiled_depth['nearest_surface_point_m'],
          'compiled_nearest_support_triangle_m':compiled_depth['nearest_triangle_vertices_m'],
          'compiled_distance_m':compiled_depth['distance_m']},
         'reverse_proxy':{'piece':F.B.NEW[int(np.argmin(reverse_part))],**reverse},
         'working_face':{'source_face_zero_based':source_face,'source_triangle_m':source[source_face].tolist(),
             'query_point_m':patch[i].tolist(),'nearest_surface_point_m':nearest[i].tolist(),'distance_m':float(d[i]),
             'piece':'gripper0_finger22_local_01','sample_id':i,'seed':222,'sample_count':1024}}
    rd,rn,_=F.G.TriangleSurface(compiled[0]['triangles']).nearest(np.array(reverse['query_point_m'])[None])
    sd,sn,si=F.G.TriangleSurface(source).nearest(rn)
    out['reverse_proxy']['same_witness_compiled_boundary_check']={
        'raw_witness_to_compiled_surface_distance_m':float(rd[0]),'compiled_nearest_point_m':rn[0].tolist(),
        'compiled_point_to_source_distance_m':float(sd[0]),'nearest_source_face_zero_based':int(si[0]),
        'old_compiled_finite_sample_max_m':oldcomp['fidelity']['proxy_union_to_source']['max_m'],
        'interpretation':'raw/compiled area samples use different triangulations, so finite sampled maxima 3.079960/3.025393mm differ; the old raw maximum remains a 3.079960mm witness on the compiled boundary. This is not a 0.055mm compiler displacement.'}
    probe=np.array([0.,.003,0.])
    old=F.bindings(F.B.BASE,[F.B.OLD])[0]
    oldplanes=ConvexHull(F.Q.V.mesh_triangles(old).reshape(-1,3)).equations
    rootplanes=ConvexHull(compiled[0]['vertices']).equations
    distance,near,face=F.G.TriangleSurface(source).nearest(probe[None])
    out['root_bore_context']={'source_face10155_vertex_radii_about_joint_Y_m':np.linalg.norm(source[10155][:,[0,2]],axis=1).tolist(),
        'joint_axis':'local Y through body origin','axis_probe_m':probe.tolist(),
        'probe_source_surface_distance_m':float(distance[0]),'probe_nearest_source_face_zero_based':int(face[0]),
        'probe_nearest_source_point_m':near[0].tolist(),
        'probe_old_single_convex_max_halfspace_m':float(np.max(oldplanes[:,:3]@probe+oldplanes[:,3])),
        'probe_current_root_convex_max_halfspace_m':float(np.max(rootplanes[:,:3]@probe+rootplanes[:,3])),
        'interpretation':'The same joint-axis aperture is convex-filled in the old single hull and the frozen root piece; source material interior remains UNKNOWN.'}
    assert abs(out['working_face']['distance_m']-read(OLD/'working-face-patch.json')['compiled']['observed_max_distance_m'])<1e-15
    return source,out


def working_cover(model,compiled):
    """Reuse the exact 1024 working-patch samples; no trajectory queries.

    A forward ray hitting a source23 face establishes a local geometric cover
    along that direction, not material solidity or arbitrary-direction reach.
    """
    src=F.P.geometry.read_stl(F.SOURCE)
    points,_=F.G._sample_surface(src[[6165,6176]],1024,222)
    _,near,_=F.G.TriangleSurface(np.concatenate([p['triangles'] for p in compiled])).nearest(points)
    shell=F.P.geometry.read_stl(ROOT/'meshes/dh116_right/source/finger23_link.STL')
    last=F.rows(F.RUN/'contacts.jsonl.gz')[-1]
    contact=next(c for c in last['contacts'] if c['geom']==['white_bowl_1_g22',F.B.OLD])
    rf=np.array(contact['cached_body_pose'][1]['rotation_world']).reshape(3,3)
    direction=-rf.T@np.array(contact['normal_world'])
    result={}
    for label,direction in [('positive_X',np.array([1.,0.,0.])),('saved4742_toward_bowl',direction)]:
        direction=direction/np.linalg.norm(direction)
        a=shell[:,0];e1=shell[:,1]-a;e2=shell[:,2]-a
        h=np.cross(direction,e2);det=np.einsum('ij,ij->i',e1,h)
        nonparallel=abs(det)>1e-20
        inv=np.divide(1.,det,out=np.zeros_like(det),where=nonparallel)
        hits=[];hit_faces=[]
        for p in near:
            s=p-a;u=np.einsum('ij,ij->i',s,h)*inv;q=np.cross(s,e1)
            v=q@direction*inv;t=np.einsum('ij,ij->i',e2,q)*inv
            eligible=nonparallel&(u>=0)&(v>=0)&(u+v<=1)&(t>F.G.QUERY_TOL_M)
            j=int(np.argmin(np.where(eligible,t,np.inf)))
            hits.append(float(t[j]) if eligible[j] else None);hit_faces.append(j if eligible[j] else None)
        values=[v for v in hits if v is not None]
        result[label]={'direction_in_finger22':direction.tolist(),'ray_count':len(near),'source23_forward_hit_count':len(values),
            'minimum_forward_hit_distance_m':min(values) if values else None,'maximum_first_forward_hit_distance_m':max(values) if values else None,
            'source23_face_ids_zero_based':sorted(set(i for i in hit_faces if i is not None)),
            'distances_m':hits,'face_ids_per_ray':hit_faces}
    # A continuous, conservative +X cover check in the YZ projection. Select
    # only source23 triangles entirely ahead of the WHOLE lower-waist convex
    # piece; subtract their projected triangles from the working-patch box.
    # This makes no source-solid or all-directions accessibility assumption.
    bounds=src[[6165,6176]].reshape(-1,3)
    low=bounds[:,1:].min(0)-.00001; high=bounds[:,1:].max(0)+.00001
    rectangle=np.array([[0,low[0],low[1]],[0,high[0],low[1]],
                        [0,high[0],high[1]],[0,low[0],high[1]]])
    front=float(compiled[1]['vertices'][:,0].max())
    candidates=np.flatnonzero((shell[:,:,0].min(1)>front)&
        np.all(shell[:,:,1:].max(1)>=low,axis=1)&np.all(shell[:,:,1:].min(1)<=high,axis=1))
    def projected_area(i):
        p=shell[i,:,1:];return abs(np.cross(p[1]-p[0],p[2]-p[0]))
    remaining=[rectangle];used=[];after=float(np.prod(high-low))
    for face in sorted(candidates,key=projected_area,reverse=True):
        t=shell[face].copy();t[:,0]=0
        orientation=np.cross(t[1]-t[0],t[2]-t[0])[0]
        if abs(orientation)<=1e-20:continue
        equations=[]
        for a,b in zip(t,np.roll(t,-1,axis=0)):
            e=b-a;n=np.array([0.,e[2],-e[1]])*np.sign(orientation);n/=np.linalg.norm(n)
            equations.append(np.r_[n,-n@a])
        before=sum(F.G._areas(np.array([[p[0],p[j],p[j+1]] for j in range(1,len(p)-1)])).sum() for p in remaining)
        remaining=[piece for polygon in remaining for piece in F.G._subtract_convex(polygon,np.array(equations),F.G.CLIP_TOL_M)]
        after=sum(F.G._areas(np.array([[p[0],p[j],p[j+1]] for j in range(1,len(p)-1)])).sum() for p in remaining)
        if before>after:used.append(int(face))
        if not remaining:break
    cover={'YZ_rectangle_min_max_m':[low.tolist(),high.tolist()],
        'rectangle_padding_m':.00001,'lower_waist_whole_piece_max_X_m':front,
        'continuous_uncovered_projected_area_m2':float(after),'covering_source23_face_ids_zero_based':used,
        'covering_faces_minimum_X_ahead_of_whole_piece_m':float(shell[used,:,0].min()-front) if used else None,
        'clipping_numeric_tolerance_m':F.G.CLIP_TOL_M,
        'scope':'Continuous floating-point projected-triangle subtraction for +X only; rectangle covers the reviewed source patch plus 10um margin. Projection cover is not material equivalence or a certificate for arbitrary approach directions.'}
    return {'fixed_sample_seed':222,'source22_face_ids_zero_based':[6165,6176],'results':result,'positive_X_continuous_cover':cover,
        'source23_topology':F.P.geometry.mesh_topology(shell),
        'limits':'Fixed finite ray witnesses only. Not a continuous coverage proof, no global inside/outside or material claim, no arbitrary-direction or unrun-pose guarantee. The 1e-20 parallel determinant guard is ray arithmetic, not a replacement for any frozen geometry/physics criterion.'}


def slice_segments(tris,axis,value):
    segments=[]
    for t in tris:
        points=[]
        for a,b in zip(t,np.roll(t,-1,axis=0)):
            da,db=a[axis]-value,b[axis]-value
            if da*db<0:points.append(a+(da/(da-db))*(b-a))
            elif abs(da)<1e-12:points.append(a)
        if len(points)>1:
            p=np.unique(np.round(points,14),axis=0)
            if len(p)>1:segments.append(p[:2])
    return np.array(segments)


def bowl_vertices():
    xml=ET.parse(F.RUN/'model.xml').getroot();body=xml.find('.//body[@name="white_bowl_1_main"]')
    result=[]
    for g in body.findall('geom'):
        if g.get('type')!='box' or not (int(g.get('contype','1')) or int(g.get('conaffinity','1'))):continue
        s=np.fromstring(g.get('size'),sep=' ');p=np.fromstring(g.get('pos','0 0 0'),sep=' ')
        R=F.P.reader.BUILDER.quaternion_matrix(g.get('quat','1 0 0 0'))
        v=np.array(list(itertools.product((-1,1),repeat=3)))*s
        result.append((g.get('name'),v@R.T+p))
    assert len(result)==36
    address=0;bowl_address=None
    for j in xml.findall('worldbody//joint'):
        if j.get('name')==body.find('joint').get('name'):bowl_address=address
        address+={'free':7,'ball':4,'hinge':1,'slide':1}[j.get('type','hinge')]
    assert bowl_address is not None and body in list(xml.find('worldbody'))
    return result,bowl_address


def projection_gap(a,b):
    values=np.maximum(b.min(0)-a.max(0),a.min(0)-b.max(0))
    return {'XYZ_projection_gaps_m':values.tolist(),'best_axis_lower_bound_m':float(max(0.,values.max()))}


def context(model,raw,compiled,src,w):
    boxes,badr=bowl_vertices();whole=np.vstack([v for _,v in boxes])
    saved=[r for r in F.rows(F.RUN/'state.jsonl.gz') if r['step'] in SELECTED]
    records=F.rows(F.RUN/'contacts.jsonl.gz');cached=[];cache_bowl4742=None
    regions={'root_whole_piece':compiled[0]['vertices'],
        'buried_source_witness_whole_triangle':np.array(w['buried_source']['source_triangle_m']),
        'buried_witness_whole_raw_proxy_support_triangle':np.array(w['buried_source']['nearest_triangle_vertices_m']),
        'buried_witness_whole_compiled_proxy_support_triangle':np.array(w['buried_source']['compiled_nearest_support_triangle_m'])}
    cache_regions=[]
    for row in records:
        if row['step']<4001:continue
        poses={b:p for c in row['contacts'] for b,p in zip(c['body'],c['cached_body_pose'])}
        if F.BODY not in poses:continue
        f,b=poses[F.BODY],poses['white_bowl_1_main'];rf,rb=(np.array(x['rotation_world']).reshape(3,3) for x in (f,b));pf,pb=(np.array(x['position_world_m']) for x in (f,b))
        moved=whole@(rf.T@rb).T+rf.T@(pb-pf)
        cache_regions.append({'step':row['step'],'alignment':row['timing'],
            'complete_36_box_bounds_in_finger22_m':[moved.min(0).tolist(),moved.max(0).tolist()],
            'whole_region_vs_complete_bowl':{n:projection_gap(v,moved) for n,v in regions.items()}})
        if row['step']==4742:cache_bowl4742={name:v@(rf.T@rb).T+rf.T@(pb-pf) for name,v in boxes}
    parent=F.P.geometry.read_stl(ROOT/'meshes/dh116_right/source/finger21_link.STL')
    welded=F.P.geometry.read_stl(ROOT/'meshes/dh116_right/source/finger23_link.STL')
    data=F.mj.MjData(model);post=[];parent4742=None
    for row in saved:
        data.qpos[:]=row['full_qpos'][:model.nq];before=data.qpos.copy()
        F.mj.mj_kinematics(model,data);assert data.time==0 and np.array_equal(before,data.qpos)
        bid=model.body(F.BODY).id;pid=model.body('gripper0_finger21_link').id
        rf,rp=(data.xmat[k].reshape(3,3) for k in (bid,pid));pf,pp=(data.xpos[k] for k in (bid,pid))
        q=np.array(row['full_qpos']);rb=F.P.reader.BUILDER.quaternion_matrix(' '.join(map(str,q[badr+3:badr+7])))
        moved=whole@(rf.T@rb).T+rf.T@(q[badr:badr+3]-pf)
        parent_in_f=parent@(rf.T@rp).T+rf.T@(pp-pf)
        if row['step']==4742:parent4742=parent_in_f
        pts=np.array([w['buried_source']['query_point_m'],w['reverse_proxy']['query_point_m']])
        d,near,ids=F.G.TriangleSurface(parent_in_f).nearest(pts)
        post.append({'step':row['step'],'alignment':'selected saved post-step FK only, not RK4 cache',
            'finger22_joint_rad':float(q[model.jnt_qposadr[model.joint('gripper0_finger22').id]]),
            'whole_region_vs_complete_bowl':{n:projection_gap(v,moved) for n,v in regions.items()},
            'witness_points_to_parent_source_distances_m':d.tolist(),'parent_source_nearest_face_ids':ids.tolist(),
            'parent_source_nearest_points_in_finger22_m':near.tolist()})
    oldrows=read(OLD/'close-states.json.gz')
    minima=[]
    for p in range(4):
        vals=[]
        for row in oldrows:
            for event in row['changed_geometry_narrowphase']:
                q=event['new_pieces'][p]
                gap=q.get('lower_bound_m',q.get('projection',{}).get('separation_lower_bound_m',0))
                vals.append((gap,row['step'],event['other_geom']))
        value=min(vals)
        minima.append({'piece':F.B.NEW[p],'minimum_explicit_saved_certificate_m':value[0],'step':value[1],'other':value[2],
            'scope':'426 saved narrowphase events only; 375768 whole-AABB exclusions reused, not recomputed. Projection gap is a lower bound, not minimum Euclidean distance.'})
    g=model.geom(F.B.NEW[0]).id
    eligible={n:F.P.saved.eligible(model,g,model.geom(n).id) for n in ('gripper0_proxy76_finger21_link_00','gripper0_proxy76_finger23_link_00')}
    return {'existing_742_summary_reused':read(OLD/'scan-summary.json'),'saved_certificate_minima':minima,
        'cached_pose_count':len(cache_regions),'cached_region_bounds':cache_regions,'selected_post_FK':post,
        'internal_parent_and_weld_eligible':eligible,'finger23_fixed_body_position':model.body_pos[model.body('gripper0_finger23_link').id].tolist(),
        'finger23_fixed_body_quaternion':model.body_quat[model.body('gripper0_finger23_link').id].tolist(),
        'finger23_has_joint':bool(model.body_jntnum[model.body('gripper0_finger23_link').id]),
        'source_topology':F.P.geometry.mesh_topology(src),'witness_source_winding_diagnostic_only':F.P.geometry.winding_number(np.array([w['buried_source']['query_point_m'],w['reverse_proxy']['query_point_m']]),src),
        'limits':'No repeated 742-pose FK or collision sweep. Bounds concern complete witness triangles/root piece vs all 36 bowl boxes at saved caches and five selected post poses. They do not certify all cavity connectivity, new object poses, full closure, lift or hold.'},welded,parent4742,cache_bowl4742


def figure(out,src,raw,w,pad,parent,bowl):
    fig,axes=plt.subplots(1,3,figsize=(14,5))
    settings=[('buried_source',1,(0,2),(-10,10,7,26),'Mid-cavity source floor'),
              ('reverse_proxy',2,(0,1),(-10,10,-11,11),'Root hinge / axial opening'),
              ('working_face',1,(0,2),(2.8,5,16,22),'Actual +X working surface')]
    for ax,(key,cutaxis,view,limits,title) in zip(axes,settings):
        value=w[key]['query_point_m'][cutaxis]
        for ts,color,label,lw in [(src,'black','finger22 source',1.0),(pad,'#777777','welded finger23 source',.8),(parent,'#1f77b4','parent finger21 (post4742)',.7)]:
            seg=slice_segments(ts,cutaxis,value)
            if len(seg):ax.add_collection(LineCollection(seg[:,:,view]*1000,colors=color,linewidths=lw,label=label))
        for i,b in enumerate(raw):
            seg=slice_segments(F.Q.V.mesh_triangles(b),cutaxis,value)
            if len(seg):ax.add_collection(LineCollection(seg[:,:,view]*1000,colors='#d62728',linewidths=.7,label='frozen proxy' if i==0 else None,alpha=.7))
        if key=='working_face':
            for name,v in bowl.items():
                seg=slice_segments(v[ConvexHull(v).simplices],cutaxis,value)
                if len(seg):ax.add_collection(LineCollection(seg[:,:,view]*1000,colors='#2ca02c',linewidths=1.0))
        point=np.array(w[key]['query_point_m'])*1000;near=np.array(w[key]['nearest_surface_point_m'])*1000
        ax.plot(point[view[0]],point[view[1]],'o',color='#ff7f0e',ms=5)
        ax.plot(near[view[0]],near[view[1]],'x',color='#9467bd',ms=6)
        ax.set(xlim=limits[:2],ylim=limits[2:],xlabel='XYZ'[view[0]]+' [mm]',ylabel='XYZ'[view[1]]+' [mm]',
            title=title+'\ncut '+'XYZ'[cutaxis]+f'={value*1000:.3f}mm; 3D error {w[key]["distance_m"]*1000:.4f}mm')
        ax.set_aspect('equal');ax.grid(alpha=.2)
    handles,labels=axes[0].get_legend_handles_labels();fig.legend(handles,labels,loc='lower center',ncol=4)
    fig.suptitle('Frozen finger22 candidate: source sections / proxy sections; dots and crosses are projected 3D witnesses')
    fig.tight_layout(rect=(0,.07,1,.94));fig.savefig(out/'sections.png',dpi=160);plt.close(fig)


def main(out):
    if out.exists():raise ValueError('New output directory required')
    assert F.W.sha(F.CANDIDATE)=='7f9d84061c3e7d176763c688d2389f8b3d3e38386b8e990a0fa047ff2bc3329c'
    assert F.mj.__version__=='3.2.3'
    out.mkdir(parents=True)
    raw=F.bindings(F.CANDIDATE,F.B.NEW)
    F.P.saved.SAFE_APIS={'mj_name2id','mj_id2name','mj_kinematics'}
    with F.P.saved.static_only() as api:
        m=F.mj.MjModel.from_xml_path(str(F.CANDIDATE))
        comp=[F.P.compiled_hull(m,n,b) for n,b in zip(F.B.NEW,raw)]
        dump(out/'numeric.json',numeric(m,raw))
        src,w=witnesses(m,raw,comp);dump(out/'witnesses.json',w)
        ctx,pad,parent,bowl=context(m,raw,comp,src,w);dump(out/'context.json.gz',ctx)
        dump(out/'working-cover.json.gz',working_cover(m,comp))
        figure(out,src,raw,w,pad,parent,bowl)
        dump(out/'api-audit.json',api)
    print(json.dumps({'numeric_bounds_nm':[p['paired_vertex_error_upper_bound_m']*1e9 for p in read(out/'numeric.json')['pieces']],
        'cached_poses_reused':ctx['cached_pose_count'],'post_FK_poses':list(SELECTED),'physics_steps':0},indent=2))


if __name__=='__main__':
    a=argparse.ArgumentParser(description=__doc__);a.add_argument('--output',type=Path,required=True)
    main(a.parse_args().output.resolve())
