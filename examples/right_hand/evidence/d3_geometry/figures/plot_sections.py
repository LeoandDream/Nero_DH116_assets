import os
from pathlib import Path
import json,sys,importlib.util
import numpy as np
from scipy.spatial import ConvexHull
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
BASE=Path(os.environ['NERO_D3_OUTPUT'])
ROOT=next(p for p in Path(__file__).resolve().parents if (p / 'mjcf/nero_dh116_right.xml').is_file())
spec=importlib.util.spec_from_file_location('surface_geometry',Path(__file__).resolve().parents[1]/'triangle/surface_geometry.py')
g=importlib.util.module_from_spec(spec);sys.modules[spec.name]=g;spec.loader.exec_module(g)
poses=json.loads((BASE/'trace/checkpoint_poses.json').read_text())
trace=json.loads((BASE/'trace/selected_trace.json').read_text())
links=('base_link','finger12_link','finger42_link')
raw={name:g.read_stl(ROOT/'meshes/dh116_right/source'/f'{name}.STL') for name in links}
hulls={}
for name,tri in raw.items():
 v=np.unique(tri.reshape(-1,3),axis=0); h=ConvexHull(v);hulls[name]=v[h.simplices]

def cut(tris,origin,u,v):
 n=np.cross(u,v);q=tris-origin;z=q@n
 points=[]
 for i,j in [(0,1),(1,2),(2,0)]:
  den=z[:,i]-z[:,j];safe=np.abs(den)>1e-14
  t=np.divide(z[:,i],den,out=np.full_like(den,np.nan),where=safe)
  valid=safe&(t>=0)&(t<=1)
  p=q[:,i]+t[:,None]*(q[:,j]-q[:,i]);p[~valid]=np.nan
  points.append(np.stack([p@u,p@v],axis=-1))
 x=np.stack(points,axis=1);finite=np.isfinite(x[:,:,0]);lines=[]
 for idx in np.flatnonzero(finite.sum(axis=1)>=2):
  pair=x[idx,finite[idx]][:2]
  if np.linalg.norm(pair[0]-pair[1])>1e-12:lines.append(pair*1000)
 return np.array(lines)

def world(tri,mode,name):
 b=mode['body_transforms']['gripper0_'+name]
 return tri@np.asarray(b['body_world_R']).T+np.asarray(b['body_world_p_m'])

fig,axes=plt.subplots(2,2,figsize=(12,10))
fig2,axes2=plt.subplots(2,2,figsize=(12,10))
for ax,bx,cp in zip(axes.flat,axes2.flat,trace['checkpoints']):
 step=cp['step'];pair=('base_link','finger12_link') if step in [229,451] else ('finger12_link','finger42_link')
 pose=next(p for p in poses['poses'] if p['checkpoint_step']==step and p['alignment']=='post')
 contact=next(c for c in cp['selected_recorded_contacts'] if set(c['body_names'])=={'gripper0_'+a for a in pair})
 origin=np.array(contact['position_world_m']);u=np.array(contact['normal_world']);u/=np.linalg.norm(u)
 rot=np.array(pose['modes']['recorded']['body_transforms']['gripper0_'+pair[0]]['body_world_R'])
 choices=[rot[:,i]-u*np.dot(rot[:,i],u) for i in range(3)]
 v=max(choices,key=np.linalg.norm);v/=np.linalg.norm(v)
 for name,color in zip(pair,['tab:blue','tab:orange']):
  lines=cut(world(raw[name],pose['modes']['recorded'],name),origin,u,v)
  hull=cut(world(hulls[name],pose['modes']['recorded'],name),origin,u,v)
  if hull.size:
   hp=np.unique(hull.reshape(-1,2),axis=0);idx=ConvexHull(hp).vertices
   ax.fill(hp[idx,0],hp[idx,1],color=color,alpha=.15)
   ax.add_collection(LineCollection(hull,colors=color,linewidths=.7,linestyles='--'))
  ax.add_collection(LineCollection(lines,colors=color,linewidths=1.1,label=name+' source'))
  for key,style,alpha in [('urdf_linear','-',1),('upstream_quadratic',':',.8)]:
   lines=cut(world(raw[name],pose['modes'][key],name),origin,u,v)
   bx.add_collection(LineCollection(lines,colors=color,linewidths=1.1,linestyles=style,alpha=alpha,label=name+' '+key))
 for dest in [ax,bx]:
  dest.scatter([0],[0],c='crimson',s=20,zorder=5)
  dest.arrow(0,0,5,0,color='crimson',head_width=.7,length_includes_head=True,zorder=5)
  dest.set_xlim(-24,24);dest.set_ylim(-24,24);dest.set_aspect('equal');dest.grid(alpha=.15)
  dest.set_xlabel('contact normal direction / mm');dest.set_ylabel('section tangent / mm')
  dest.set_title(f'step {step}: {pair[0]} / {pair[1]}\nrecorded post-step FK, t={cp["time_s"]:.3f}s')
  dest.legend(fontsize=7,loc='upper right')
fig.suptitle('Raw STL section (solid) vs whole-link convex hull (fill / dashed)\nSection through recorded contact marker; red arrow = recorded normal. Slice alone is not a global intersection test.')
fig2.suptitle('Offline counterfactual at identical active joint angles\nExact URDF linear (solid) vs source-MJCF quadratic (dotted). No physical stepping or model edits.')
for f,name in [(fig,'contact-sections.png'),(fig2,'mimic-sections.png')]:
 f.tight_layout(rect=[0,0,1,.94]);f.savefig(BASE/'figures'/name,dpi=160);plt.close(f)
