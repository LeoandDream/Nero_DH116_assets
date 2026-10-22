import os
from pathlib import Path
import json,time,sys,hashlib
import numpy as np
from surface_geometry import read_stl,mesh_topology,BVH,surface_query,winding_number
OUT=Path(os.environ['NERO_D3_OUTPUT'])/'triangle'
TRACE=OUT.parent/'trace/checkpoint_poses.json'
ROOT=next(p for p in Path(__file__).resolve().parents if (p / 'mjcf/nero_dh116_right.xml').is_file())
data=json.loads(TRACE.read_text())
names=['base_link','finger12_link','finger42_link']
meshes={n:read_stl(ROOT/'meshes/dh116_right/source'/f'{n}.STL') for n in names}
bvhs={n:BVH(t) for n,t in meshes.items()}
metadata={'trace_path':str(TRACE),'trace_sha256':hashlib.sha256(TRACE.read_bytes()).hexdigest(),'topology':{n:mesh_topology(t) for n,t in meshes.items()},'source_stl_sha256':{n:hashlib.sha256((ROOT/'meshes/dh116_right/source'/f'{n}.STL').read_bytes()).hexdigest() for n in names},
 'method':'Full original binary STL, exact float32 source vertices promoted to float64; original zero-based face IDs. Relative body-frame transform to reduce world-offset roundoff. AABB BVH lower-bound branch-and-bound and continuous triangle distance (6 vertex-face, 9 edge-edge; 6 edge-triangle crossings). No mj_step or dynamics. Initial vertex upper bound only, never used as final answer without complete surface search.',
 'precision':'float64 predicates; strict crossing requires opposite endpoint plane distances beyond 1e-12 m and other-face barycentric coordinates >1e-10. AABB lower-bound pruning retains candidates within 1e-12 m beyond the current upper distance bound.',
 'containment_limit':'Original meshes fail 2-manifold edge watertightness; a positive surface distance does not alone prove disjoint physical solids. Winding numbers are diagnostics only, not an inside/outside certificate for these meshes.'}
(OUT/'metadata.json').write_text(json.dumps(metadata,indent=2))
results=[]
# First all recorded poses, then the two offline mimic hypotheses.
for mode in ['recorded','urdf_linear','upstream_quadratic']:
 for pose in sorted(data['poses'],key=lambda p:(p['checkpoint_step']<6000,p['alignment']=='pre',p['checkpoint_step'])):
  if pose['alignment']=='pre' and mode!='recorded':continue
  for na,nb in [('base_link','finger12_link'),('finger12_link','finger42_link')]:
   if not (mode=='recorded' and pose['alignment']=='post') and ((pose['checkpoint_step']<6000) != (na=='base_link')):continue
   if na=='base_link' and mode!='recorded':continue
   transforms=pose['modes'][mode]['body_transforms'];ta=transforms['gripper0_'+na];tb=transforms['gripper0_'+nb]
   Ra=np.array(ta['body_world_R']);pa=np.array(ta['body_world_p_m']);Rb=np.array(tb['body_world_R']);pb=np.array(tb['body_world_p_m'])
   # row-vectors B local -> world -> A local
   t0=time.time();A=meshes[na];B=meshes[nb]@(Ra.T@Rb).T+Ra.T@(pb-pa)
   bb=bvhs[nb].refit(B)
   answer=surface_query(A,B,bvhs[na],bb)
   answer.update({'pose_id':pose['id'],'checkpoint_step':pose['checkpoint_step'],'alignment':pose['alignment'],'source_state_step':pose['source_state_step'],'mode':mode,'pair':[na,nb],'elapsed_s':time.time()-t0,
                  'coordinate_frame':'body_local_'+na})
   answer['closest_points_world_m']=[(np.array(p)@Ra.T+pa).tolist() for p in answer['closest_points']]
   if answer['strict_transverse_witness']:
    h=answer['strict_transverse_witness'];h['point_world_m']=(np.array(h['point'])@Ra.T+pa).tolist()
   # Winding sample is diagnostic, not a containment certificate for nonmanifold surfaces.
   answer['winding_diagnostic']={'A_first_vertex_in_B':winding_number(A[0,:1],B)[0],'B_first_vertex_in_A':winding_number(B[0,:1],A)[0]}
   results.append(answer)
   (OUT/'surface_results.json').write_text(json.dumps(results,indent=2))
   print(pose['id'],mode,na,nb,'distance_mm=',answer['surface_min_distance_m']*1000,'cross=',answer['strict_transverse_witness'] is not None,'nodes=',answer['visited_node_pairs'],'tri_pairs=',answer['triangle_pairs_evaluated'],'elapsed=',round(answer['elapsed_s'],2),flush=True)
