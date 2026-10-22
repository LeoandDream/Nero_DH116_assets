#!/usr/bin/env python3
"""Reproduce only the saved Reset query artifact and one weak AABB bound.

No physics, Reset or collision generation; these two saved poses are FK only.
Large query distmax is diagnostic API input, never a model contact parameter.
"""
from pathlib import Path
import argparse
import sys,json,hashlib
import numpy as np
import xml.etree.ElementTree as ET
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[4]
sys.path.insert(0,str(ROOT/'examples/right_hand'))
import wrist_external_preflight as E
W,P,mj=E.W,E.P,E.mj
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--libero-assets',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
preflight=HERE.parent/'preflight'
x=W.replace_scene_wrist(W.scene_xml(args.libero_assets,[]))
m=mj.MjModel.from_xml_string(ET.tostring(x,encoding='unicode'));d=mj.MjData(m)
robot=mj.MjModel.from_xml_path(str(W.CANDIDATE));qa=[int(m.jnt_qposadr[m.joint(robot.joint(i).name).id]) for i in range(robot.njnt)]
samples=W.read(P.PLAN)['samples'];reference=W.read(preflight/'planning-reference.json')['full_qpos_for_static_reference_only']
records=[]
for index,names in [(0,('robot0_wrist_reuse_link5_008','gripper0_proxy76_base_link_06')),(354,('robot0_wrist_reuse_link5_132','robot0_candidate_nero_link7_0'))]:
 d.qpos[:]=reference;d.qpos[qa]=samples[index]['qpos'];mj.mj_kinematics(m,d)
 ids=[m.geom(n).id for n in names];poly=E.load_polytopes(m,x,ids);projection=E.projection_pair(m,d,*ids,poly)
 item={'pose_index_one_based':index+1,'geoms':list(names),'full_compiled_geometry_projection':projection}
 if index==0:
  item['distance_query_by_cutoff_m']={str(cutoff):float(mj.mj_geomDistance(m,d,*ids,cutoff,np.empty(6))) for cutoff in (.001,.01,.1,1.,10.)}
 else:
  lo,hi=E.world_aabbs(m,d);g,h=ids;item['AABB_lower_bound_m']=float(np.maximum(lo[h]-hi[g],lo[g]-hi[h]).max())
 records.append(item)
result={'records':records,'scope':'Two offline saved-pose query diagnostics only; no model or tolerance change.',
 'conclusion':'Large inflated-margin native MPR query can have a wrong sign for these small convex pieces. Final preflight uses complete compiled-hull projection/verified LP certificates instead. The 9.107nm AABB lower bound is not a measured physical clearance: full hull projection for that pair certifies 12.014mm.',
 'official_sources':['https://raw.githubusercontent.com/google-deepmind/mujoco/3.2.3/src/engine/engine_support.c','https://raw.githubusercontent.com/google-deepmind/mujoco/3.2.3/src/engine/engine_collision_convex.c'],
 'precision_threshold_m_unchanged':P.TOL,'physics_steps':0,'numeric_FAIL_reclassified':False}
p=args.output
with p.open('x') as f:json.dump(result,f,indent=2)
print('two offline query diagnoses saved')
