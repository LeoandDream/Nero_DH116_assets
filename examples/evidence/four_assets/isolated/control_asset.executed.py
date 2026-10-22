#!/usr/bin/env python3
"""Load any delivery entry; absolute position actions, no new control law.

Hand-only needs MuJoCo+NumPy. Assemblies reuse stock robosuite 1.4.1 JPC.
"""
import argparse
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]


class AssetRuntime:
    def __init__(self,name,root=ROOT):
        import mujoco
        self.mujoco=mujoco
        self.root=Path(root).resolve()
        manifest=json.loads((self.root/'docs/assets.json').read_text())
        self.spec=manifest['assets'][name]
        self.model=mujoco.MjModel.from_xml_path(str(self.root/self.spec['model']))
        self.sim=None;self.controller=None
        if self.spec['arm']:
            import robosuite
            from robosuite.utils.binding_utils import MjSim
            if robosuite.__version__!='1.4.1':raise ValueError('Assembly example requires frozen stock robosuite 1.4.1')
            self.sim=MjSim(self.model)
            self.data=self.sim.data._data
        else:self.data=mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model,self.data,0)
        mujoco.mj_forward(self.model,self.data)
        self.joint_ids=np.array([self.model.joint(n).id for n in self.spec['state_order']])
        self.qadr=self.model.jnt_qposadr[self.joint_ids]
        self.vadr=self.model.jnt_dofadr[self.joint_ids]
        def actuator_for(joint):
            ids=[i for i in range(self.model.nu) if self.model.actuator_trnid[i,0]==self.model.joint(joint).id]
            if len(ids)!=1:raise ValueError('Missing/ambiguous actuator: '+joint)
            return ids[0]
        self.hand_ids=np.array([actuator_for(n) for n in self.spec['active_order']])
        self.hand_qadr=np.array([self.model.jnt_qposadr[self.model.joint(n).id] for n in self.spec['active_order']])
        self.hand_low,self.hand_high=self.model.actuator_ctrlrange[self.hand_ids].T.copy()
        self.motor_ids=np.array([],dtype=int)
        if self.spec['arm']:
            from robosuite.controllers.joint_pos import JointPositionController
            self.motor_ids=np.array([actuator_for(n) for n in self.spec['state_order'][:7]])
            self.arm_low,self.arm_high=self.model.jnt_range[self.joint_ids[:7]].T.copy()
            self.controller=JointPositionController(sim=self.sim,eef_name='gripper0_grip_site',
                joint_indexes={'joints':self.joint_ids[:7].tolist(),'qpos':self.qadr[:7].tolist(),'qvel':self.vadr[:7].tolist()},
                actuator_range=tuple(self.model.actuator_ctrlrange[self.motor_ids].T),
                kp=50,damping_ratio=1,policy_freq=20,
                qpos_limits=np.stack([self.arm_low,self.arm_high]))
        self.action=None

    def state(self):return self.data.qpos[self.qadr].copy()

    def home_action(self):
        hand=2*(self.data.qpos[self.hand_qadr]-self.hand_low)/(self.hand_high-self.hand_low)-1
        return np.r_[self.state()[:7],hand] if self.spec['arm'] else hand

    def set_action(self,action):
        action=np.asarray(action,dtype=float)
        if action.shape!=(self.spec['action_dim'],) or not np.isfinite(action).all():
            raise ValueError('Wrong action dimension or nonfinite target')
        if np.any(np.abs(action[-6:])>1):raise ValueError('Hand targets must be normalized absolute [-1,1]')
        if self.controller is not None:
            if np.any(action[:7]<self.arm_low) or np.any(action[:7]>self.arm_high):raise ValueError('NERO absolute target outside joint range')
            self.controller.set_goal(np.zeros(7),set_qpos=action[:7])
        self.action=action.copy()
        # Same midpoint/range mapping as stock Manipulator.grip_action.
        self.hand_target=.5*(self.hand_high+self.hand_low)+.5*(self.hand_high-self.hand_low)*action[-6:]

    def step(self,steps=25):
        if self.action is None:raise ValueError('Call set_action before stepping')
        if not isinstance(steps,int) or steps<1:raise ValueError('Positive physical step count required')
        for _ in range(steps):
            if self.controller is not None:
                torques=self.controller.run_controller()
                low,high=self.model.actuator_ctrlrange[self.motor_ids].T
                self.data.ctrl[self.motor_ids]=np.clip(torques,low,high)
            self.data.ctrl[self.hand_ids]=self.hand_target
            self.mujoco.mj_step(self.model,self.data)
            if any(w.number for w in self.data.warning) or not all(np.isfinite(a).all() for a in (self.data.qpos,self.data.qvel,self.data.qacc,self.data.ctrl)):
                raise RuntimeError('Numerical warning/nonfinite state; no retry')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--asset',required=True,choices=['dh116_left','dh116_right','nero_dh116_left','nero_dh116_right'])
    parser.add_argument('--steps',type=int,default=250,help='Physical steps; 25 per 20Hz command at 2ms')
    parser.add_argument('--action',nargs='+',type=float,help='6D hand or 13D assembly absolute action; default holds reference pose')
    args=parser.parse_args()
    runtime=AssetRuntime(args.asset)
    runtime.set_action(runtime.home_action() if args.action is None else args.action)
    runtime.step(args.steps)
    print(json.dumps({'asset':args.asset,'physical_steps':args.steps,'time_s':runtime.data.time,
                      'state':runtime.state().tolist(),'state_dim':runtime.spec['state_dim'],'action_dim':runtime.spec['action_dim']}))


if __name__=='__main__':main()
