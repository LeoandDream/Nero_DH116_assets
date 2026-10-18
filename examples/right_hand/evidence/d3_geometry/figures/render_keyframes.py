import os
from pathlib import Path
import json,math
import numpy as np
import mujoco
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
BASE=Path(os.environ['NERO_D3_OUTPUT']);ROOT=next(p for p in Path(__file__).resolve().parents if (p / 'mjcf/nero_dh116_right.xml').is_file())
doc=json.loads((BASE/'trace/checkpoint_poses.json').read_text());trace=json.loads((BASE/'trace/selected_trace.json').read_text())
model=mujoco.MjModel.from_xml_path(str(ROOT/'mjcf/nero_dh116_right.xml'));data=mujoco.MjData(model)
opt=mujoco.MjvOption();opt.geomgroup[:]=0;opt.geomgroup[1]=1
fig,axes=plt.subplots(2,2,figsize=(12,10));renderer=mujoco.Renderer(model,height=480,width=640)
frames=[]
for ax,cp in zip(axes.flat,trace['checkpoints']):
 step=cp['step'];data.qpos[:]=cp['full_qpos'];mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data)
 base=data.xmat[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'gripper0_base_link')].reshape(3,3)
 camera=mujoco.MjvCamera();camera.type=mujoco.mjtCamera.mjCAMERA_FREE
 contact=cp['selected_recorded_contacts'][-1];center=np.array(contact['position_world_m'])
 camera.lookat[:]=np.mean([p for name,p in zip(range(model.ngeom),data.geom_xpos) if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,name) or '').startswith('gripper0_')],axis=0);camera.distance=.27
 view=base@np.array([1.3,-1.4,.8]);view/=np.linalg.norm(view)
 camera.azimuth=math.degrees(math.atan2(view[1],view[0]));camera.elevation=-math.degrees(math.asin(view[2]))
 renderer.update_scene(data,camera=camera,scene_option=opt)
 for geom in renderer.scene.geoms[:renderer.scene.ngeom]:
  if geom.objtype==mujoco.mjtObj.mjOBJ_GEOM and geom.objid>=0:
   name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,geom.objid)
   if not name or not name.startswith('gripper0_'):geom.rgba[3]=0
   else:geom.rgba[:]=[.65,.7,.77,1]
 n=renderer.scene.ngeom
 mujoco.mjv_initGeom(renderer.scene.geoms[n],mujoco.mjtGeom.mjGEOM_SPHERE,np.array([.0012]*3),center,np.eye(3).ravel(),np.array([1.,0.,0.,1.]));renderer.scene.ngeom+=1
 n=renderer.scene.ngeom
 mujoco.mjv_initGeom(renderer.scene.geoms[n],mujoco.mjtGeom.mjGEOM_CAPSULE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array([1.,.2,.1,1.]));mujoco.mjv_connector(renderer.scene.geoms[n],mujoco.mjtGeom.mjGEOM_CAPSULE,.0004,center,center+np.array(contact['normal_world'])*.012);renderer.scene.ngeom+=1
 pixels=renderer.render();ax.imshow(pixels);ax.axis('off');ax.set_title(f'Step {step} / t={cp["time_s"]:.3f}s\n{cp["phase"]} (post-step FK only)')
 frames.append({'step':step,'source_qpos_row':step,'physics_steps':0,'constraint_solves':0,'recorded_contact_marker_m':contact['position_world_m'],'recorded_normal':contact['normal_world']})
renderer.close();fig.suptitle('Recorded hand poses: original STL render, no physical stepping\nRed marker/segment: recorded RK4 contact position/normal; not an exact internal-stage reconstruction.')
fig.subplots_adjust(left=.02,right=.98,bottom=.02,top=.85,hspace=.25,wspace=.08);fig.savefig(BASE/'figures/keyframes.png',dpi=140);plt.close(fig)
(BASE/'figures/keyframes.json').write_text(json.dumps(frames,indent=2)+'\n')
