import os
"""Attach source-contact locality, independently recomputed witnesses and limits."""
from pathlib import Path
from decimal import Decimal,localcontext
import copy,gzip,json,numpy as np
from scipy.optimize import minimize
from surface_geometry import read_stl
OUT=Path(os.environ['NERO_D3_OUTPUT'])/'triangle';ROOT=next(p for p in Path(__file__).resolve().parents if (p / 'mjcf/nero_dh116_right.xml').is_file())
results=json.loads((OUT/'surface_results.json').read_text());trace=json.loads((OUT.parent/'trace/checkpoint_poses.json').read_text());poses={p['id']:p for p in trace['poses']}
meshes={n:read_stl(ROOT/'meshes/dh116_right/source'/f'{n}.STL') for n in ['base_link','finger12_link','finger42_link']}
contacts={}
with gzip.open(ROOT/'examples/right_hand/evidence/hand-motion-contacts.jsonl.gz','rt') as f:
 for line in f:
  x=json.loads(line)
  if x['step'] in [229,451,6498,6511]:contacts[x['step']]=x

def dot(a,b):return sum(x*y for x,y in zip(a,b))
def sub(a,b):return [x-y for x,y in zip(a,b)]
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def decimal_witness(h):
 with localcontext() as ctx:
  ctx.prec=60
  A=[[Decimal.from_float(x) for x in v] for v in h['triangle_a']];B=[[Decimal.from_float(x) for x in v] for v in h['triangle_b']]
  src,dst=(A,B) if h['edge_from_triangle']==0 else (B,A)
  p,q=[src[j] for j in h['edge_vertices']];a,b,c=dst;ab=sub(b,a);ac=sub(c,a);n=cross(ab,ac);n2=dot(n,n)
  dp=dot(sub(p,a),n);dq=dot(sub(q,a),n);f=dp/(dp-dq);point=[x+f*(y-x) for x,y in zip(p,q)];v=sub(point,a)
  u=dot(cross(v,ac),n)/n2;w=dot(cross(ab,v),n)/n2;bary=[1-u-w,u,w]
  assert dp*dq<0 and 0<f<1 and all(x>0 for x in bary)
  error=max(abs(float(x)-y) for x,y in zip(point,h['point']))
  assert error<1e-13,error
  return {'precision_decimal_digits':60,'strict_crossing_reconfirmed':True,'edge_parameter':str(f),'barycentric':[str(x) for x in bary],
          'signed_plane_distances_m':[str(dp/n2.sqrt()),str(dq/n2.sqrt())],'point_m':[str(x) for x in point],
          'max_point_difference_from_float64_m':error,'scope':'60-digit Decimal arithmetic on stored float64 transformed triangles, not an exact proof about unrecorded solver-stage pose.'}

def normal(t):
 n=np.cross(t[1]-t[0],t[2]-t[0]);return n/np.linalg.norm(n)

for r in results:
 na,nb=r['pair'];pose=poses[r['pose_id']];trs=pose['modes'][r['mode']]['body_transforms'];ta=trs['gripper0_'+na];tb=trs['gripper0_'+nb]
 Ra=np.array(ta['body_world_R']);pa=np.array(ta['body_world_p_m']);Rb=np.array(tb['body_world_R']);pb=np.array(tb['body_world_p_m'])
 A=meshes[na];B=meshes[nb]@(Ra.T@Rb).T+Ra.T@(pb-pa)
 cs=[c for c in contacts[r['checkpoint_step']]['contacts'] if set(c['body_names'])=={'gripper0_'+na,'gripper0_'+nb}]
 r['original_recorded_contacts']=cs;r['recorded_contact_alignment_note']='Original mj_step cached contact is final RK4 internal-stage geometry; pre/post logged qpos are only nearby offline poses, not that exact internal stage.'
 if cs:
  r['closest_or_witness_point_to_recorded_contact_m']=[float(np.linalg.norm(np.array(r['closest_points_world_m'][0])-c['position_world_m'])) for c in cs]
 h=r['strict_transverse_witness'];ia,ib=r['nearest_face_ids_zero_based']
 r['original_face_vertices_body_local_m']=[A[ia].tolist(),meshes[nb][ib].tolist()]
 if h:
  h['decimal_recheck']=decimal_witness(h)
  aa=np.array(h['triangle_a']);bb=np.array(h['triangle_b']);nA=normal(aa);nB=normal(bb)
  h['face_normals_in_A_body_frame']=[nA.tolist(),nB.tolist()];h['face_normals_world']=[(Ra@nA).tolist(),(Ra@nB).tolist()]
  h['face_normal_angle_deg']=float(np.degrees(np.arccos(np.clip(nA@nB,-1,1))))
  world=np.array(h['point_world_m']);h['point_in_original_body_frames_m']={na:h['point'],nb:(Rb.T@(world-pb)).tolist()}
  h['within_15mm_of_original_contact']=bool(cs and min(r['closest_or_witness_point_to_recorded_contact_m'])<.015)
 else:
  alo=A.min(axis=(0,1));ahi=A.max(axis=(0,1));blo=B.min(axis=(0,1));bhi=B.max(axis=(0,1))
  av=A.reshape(-1,3);bv=B.reshape(-1,3)
  outA=np.flatnonzero(np.any((av<blo)|(av>bhi),axis=1));outB=np.flatnonzero(np.any((bv<alo)|(bv>ahi),axis=1))
  r['containment_exclusion_diagnostic']={'A_vertex_outside_B_global_AABB':av[outA[0]].tolist() if len(outA) else None,'B_vertex_outside_A_global_AABB':bv[outB[0]].tolist() if len(outB) else None,
    'A_vertex_connected_components':1 if na!='base_link' else 10,'B_vertex_connected_components':1,
    'meaning':'For the one-component finger meshes, neither complete connected surface is enclosed by the other global AABB. This excludes complete connected-surface containment, but is not a watertight-solid intersection certificate.'}
  # Independent convex QP verification on the returned closest triangle pair.
  a=A[ia]*1000;b=B[ib]*1000;offset=a[0]-b[0];M=np.stack([a[1]-a[0],a[2]-a[0],-(b[1]-b[0]),-(b[2]-b[0])],axis=1)
  fun=lambda x:float(np.dot(M@x+offset,M@x+offset));jac=lambda x:2*M.T@(M@x+offset)
  opt=minimize(fun,np.ones(4)/3,jac=jac,bounds=[(0,1)]*4,constraints=[{'type':'ineq','fun':lambda x:1-x[0]-x[1]},{'type':'ineq','fun':lambda x:1-x[2]-x[3]}],method='SLSQP',options={'ftol':1e-13,'maxiter':500})
  err=abs(np.sqrt(max(opt.fun,0))/1000-r['surface_min_distance_m'])
  assert err<1e-9,(r['pose_id'],r['mode'],err)
  r['closest_pair_independent_QP']={'scipy_SLSQP_success':bool(opt.success),'distance_m':float(np.sqrt(max(opt.fun,0))/1000),'absolute_difference_m':float(err),'note':'Independent barycentric convex QP checks nearest-pair continuous distance; globality follows complete BVH search.'}
# For pair A neither affected joint is a distal mimic; prove transforms identical
# before sharing existing recorded geometry certificates across the two modes.
aliases=[]
for r in results:
 if r['pair'][0]!='base_link' or r['alignment']!='post':continue
 p=poses[r['pose_id']]
 for mode in ['urdf_linear','upstream_quadratic']:
  maxdiff=0.
  for n in r['pair']:
   rec=p['modes']['recorded']['body_transforms']['gripper0_'+n];alt=p['modes'][mode]['body_transforms']['gripper0_'+n]
   for k in ['body_world_R','body_world_p_m']:maxdiff=max(maxdiff,float(np.max(np.abs(np.array(rec[k])-np.array(alt[k])))))
  assert maxdiff==0,(r['pose_id'],mode,maxdiff)
  alias=copy.deepcopy(r);alias['mode']=mode;alias['certificate_reused_from_mode']='recorded';alias['body_transform_max_abs_difference']=maxdiff;aliases.append(alias)
results+=aliases
(OUT/'surface_results.json').write_text(json.dumps(results,indent=2))
summary=[]
for r in results:
 summary.append({k:r[k] for k in ['pose_id','mode','pair','surface_min_distance_m','nearest_face_ids_zero_based']})
 summary[-1]['strict_crossing']=r['strict_transverse_witness'] is not None
 summary[-1]['contact_distance_mm']=[x*1000 for x in r.get('closest_or_witness_point_to_recorded_contact_m',[])]
 summary[-1]['shared_certificate']=r.get('certificate_reused_from_mode')
(OUT/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
