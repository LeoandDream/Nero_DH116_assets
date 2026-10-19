"""Explicit formal-right adapter for the frozen native LIBERO bowl diagnostic.

Only the path-parameterized XML splitter is shared with the left reference.
No left fixture, reset state, command, or physics parameter is read here.
"""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET


def register(root, out):
    """Register the delivered right model; never construct or reset a simulator."""
    import numpy as np
    from robosuite.models.robots.manipulators.manipulator_model import ManipulatorModel
    from robosuite.models.grippers.gripper_model import GripperModel
    from robosuite.models.mounts.mount_model import MountModel
    from robosuite.models.grippers import GRIPPER_MAPPING
    from robosuite.models.mounts import MOUNT_MAPPING
    from robosuite.robots import SingleArm, ROBOT_CLASS_MAPPING

    root, out = Path(root).resolve(), Path(out)
    source = root / 'mjcf/nero_dh116_right.xml'
    official = ET.parse(source).getroot()
    spec = importlib.util.spec_from_file_location('frozen_bowl_xml_splitter', root / 'examples/libero_reference/model.py')
    splitter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(splitter)
    paths = splitter.split_official_model(source, out)
    # The shared splitter's hand model label is historical, not an asset path.
    hand_xml = ET.parse(paths['hand'])
    hand_xml.getroot().set('model', 'dh116_urdf_right')
    hand_xml.write(paths['hand'], encoding='utf-8', xml_declaration=True)
    joints = list(official.findall('.//worldbody//joint'))
    limits = np.asarray([np.fromstring(j.get('range'), sep=' ') for j in joints[:7]])
    lower, upper = limits[:, 0], limits[:, 1]
    key = official.find('keyframe/key')
    initial = np.fromstring(key.get('qpos'), sep=' ')
    if len(joints) != 18 or initial.shape != (18,) or any(np.any(np.fromstring(key.get(k, ''), sep=' ')) for k in ('qvel', 'ctrl')):
        raise ValueError('Expected formal right keyframe 0 with 18 positions and zero velocity/control')
    base = official.find('worldbody/body[@name="robot0_base_link"]')
    world_base = np.fromstring(base.get('pos'), sep=' ')
    mount_pos = np.fromstring(base.find('body[@name="mount0_base"]').get('pos'), sep=' ')
    clearance = -.5-world_base[0]  # Native kitchen table's frozen 1 m x dimension.
    thumb = ['finger13_link_collision_0', 'finger14_link_collision_0']
    opposed = [f'finger{finger}{link}_link_collision_0' for finger in range(2, 6) for link in (2, 3)]
    roles = {'left_finger': thumb, 'right_finger': opposed,
             'left_fingerpad': thumb[-1:], 'right_fingerpad': opposed[1::2]}
    names = {g.get('name').removeprefix('gripper0_') for g in official.findall('.//geom')}
    if any(name not in names for values in roles.values() for name in values):
        raise ValueError('Right native contact role names are absent from the formal source')

    class RightDeliveryMount(MountModel):
        def __init__(self, idn=0):
            super().__init__(str(paths['mount']), idn=idn)
        @property
        def top_offset(self):
            return -mount_pos.copy()
        @property
        def bottom_offset(self):
            return np.array([0, 0, -world_base[2]]) + self.top_offset
        @property
        def horizontal_radius(self):
            return 0.25

    class DH116RightURDF(GripperModel):
        def __init__(self, idn=0):
            super().__init__(str(paths['hand']), idn=idn)
        @property
        def init_qpos(self):
            return initial[7:].copy()
        def format_action(self, action):
            action = np.asarray(action, dtype=float)
            if action.shape != (6,) or not np.isfinite(action).all() or np.any(np.abs(action) > 1):
                raise ValueError('DH116 requires six finite normalized absolute commands')
            return action.copy()
        @property
        def _important_geoms(self):
            return roles

    class RightNeroURDF(ManipulatorModel):
        def __init__(self, idn=0):
            super().__init__(str(paths['arm']), idn=idn)
        @property
        def _eef_name(self):
            return 'gripper_flange'
        @property
        def arm_type(self):
            return 'single'
        @property
        def default_mount(self):
            return 'RightDeliveryMount'
        @property
        def default_gripper(self):
            return 'DH116RightURDF'
        @property
        def default_controller_config(self):
            return 'default_joint_position'
        @property
        def init_qpos(self):
            return initial[:7].copy()
        @property
        def base_xpos_offset(self):
            return {key: lambda length: (-length/2-clearance, world_base[1], 0) for key in ('table', 'kitchen_table')}
        @property
        def top_offset(self):
            raise NotImplementedError('Robot envelope is not used by this frozen native scene')
        @property
        def _horizontal_radius(self):
            return clearance

    class AbsoluteJointPosition:
        def __init__(self, backend):
            self.backend = backend
        def __getattr__(self, name):
            return getattr(self.backend, name)
        @property
        def control_limits(self):
            return lower.copy(), upper.copy()
        def set_goal(self, action, set_qpos=None):
            target = np.asarray(action if set_qpos is None else set_qpos, dtype=float)
            if target.shape != (7,) or not np.isfinite(target).all() or np.any(target < lower) or np.any(target > upper):
                raise ValueError('NERO requires legal 7D ABSOLUTE JOINT POSITION targets in rad')
            self.backend.set_goal(np.zeros(7), set_qpos=target)

    class RightNeroURDFArm(SingleArm):
        def _load_controller(self):
            self.controller_config['qpos_limits'] = np.stack([lower, upper])
            super()._load_controller()
            self.controller = AbsoluteJointPosition(self.controller)
        def control(self, action, policy_step=False):
            action = np.asarray(action, dtype=float)
            if action.shape != (13,) or not np.isfinite(action).all() or np.any(action[:7] < lower) or np.any(action[:7] > upper) or np.any(np.abs(action[7:]) > 1):
                raise ValueError('Invalid 13D diagnostic action')
            return super().control(action, policy_step=policy_step)

    class MountedRightNeroURDF(RightNeroURDF):
        pass

    GRIPPER_MAPPING['DH116RightURDF'] = DH116RightURDF
    MOUNT_MAPPING['RightDeliveryMount'] = RightDeliveryMount
    for name in ('RightNeroURDF', 'MountedRightNeroURDF'):
        ROBOT_CLASS_MAPPING[name] = RightNeroURDFArm
    return {'paths': paths, 'lower': lower, 'upper': upper, 'initial': initial, 'roles': roles}


def configure_scene(root):
    """Carry explicit right options into the unchanged native task and predicate."""
    from libero.libero.envs import env_wrapper
    key = 'libero_kitchen_tabletop_manipulation'
    native = env_wrapper.TASK_MAPPING[key]
    official = ET.parse(Path(root) / 'mjcf/nero_dh116_right.xml').getroot()

    def load_model(self):
        native._load_model(self)
        for tag in ('option', 'compiler', 'size'):
            element = self.model.root.find(tag)
            if element is None:
                element = ET.SubElement(self.model.root, tag)
            element.attrib.update(official.find(tag).attrib)

    env_wrapper.TASK_MAPPING[key] = type(native.__name__, (native,), {'_load_model': load_model})
