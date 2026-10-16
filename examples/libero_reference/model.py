"""Adapt the delivered MJCF to native robosuite model interfaces.

There is no second robot template: all bodies, inertials, geoms, joints,
actuators, equalities, sensors and mesh references come from mjcf/nero_dh116.xml.
Only namespace removal and native mounting-frame normalization are performed.
"""
from pathlib import Path
import copy
import xml.etree.ElementTree as ET

PREFIXES = {'arm': 'robot0_', 'hand': 'gripper0_', 'mount': 'mount0_'}
REFERENCES = {'name', 'joint', 'joint1', 'joint2', 'body', 'body1', 'body2',
              'mesh', 'material', 'texture', 'site', 'site1', 'site2', 'camera'}


def split_official_model(path, output):
    path, output = Path(path).resolve(), Path(output)
    source = ET.parse(path).getroot()
    robot = copy.deepcopy(source.find('worldbody/body[@name="robot0_base_link"]'))
    flange = robot.find('.//body[@name="robot0_gripper_flange"]')
    hand = flange.find('body[@name="gripper0_base_link"]')
    if hand is None:
        raise ValueError('Delivered DH116 attachment is missing')
    flange.remove(hand)
    mount = robot.find('body[@name="mount0_base"]')
    robot.remove(mount)
    # Native add_mount / set_base_xpos will restore the delivered world pose.
    # The compiled preflight verifies that pose before any physical integration.
    robot.set('pos', '0 0 0')
    mount.set('pos', '0 0 0')
    result = {}
    for role, body in [('arm', robot), ('hand', hand), ('mount', mount)]:
        prefix = PREFIXES[role]
        root = ET.Element('mujoco', model={'arm': 'nero', 'hand': 'dh116_urdf_left', 'mount': 'delivery_mount'}[role])
        for tag in ('compiler', 'option', 'size', 'default'):
            element = source.find(tag)
            if element is not None:
                root.append(copy.deepcopy(element))
        assets = ET.SubElement(root, 'asset')
        for element in source.find('asset'):
            if element.get('name', '').startswith(prefix):
                element = copy.deepcopy(element)
                if element.get('file'):
                    element.set('file', str((path.parent / element.get('file')).resolve()))
                assets.append(element)
        ET.SubElement(root, 'worldbody').append(body)
        for tag, attribute in [('actuator', 'joint'), ('equality', 'joint1'), ('sensor', 'site'), ('contact', 'body1')]:
            parent = ET.SubElement(root, tag)
            for element in source.findall(tag+'/*'):
                if element.get(attribute, '').startswith(prefix):
                    parent.append(copy.deepcopy(element))
        for element in root.iter():
            for key, value in list(element.attrib.items()):
                if key in REFERENCES and value.startswith(prefix):
                    element.set(key, value[len(prefix):])
        result[role] = output / (role + '.xml')
        ET.ElementTree(root).write(result[role], encoding='utf-8', xml_declaration=True)
    return result


def register_models(asset_root, output, metadata, roles):
    """Register existing absolute-position input semantics, without a new law."""
    import numpy as np
    from robosuite.models.robots.manipulators.manipulator_model import ManipulatorModel
    from robosuite.models.grippers.gripper_model import GripperModel
    from robosuite.models.mounts.mount_model import MountModel
    from robosuite.models.grippers import GRIPPER_MAPPING
    from robosuite.models.mounts import MOUNT_MAPPING
    from robosuite.robots import SingleArm, ROBOT_CLASS_MAPPING

    paths = split_official_model(Path(asset_root) / 'mjcf/nero_dh116.xml', output)
    official = ET.parse(Path(asset_root) / 'mjcf/nero_dh116.xml').getroot()
    joints = list(official.findall('.//worldbody//joint'))
    limits = np.asarray([np.fromstring(j.get('range'), sep=' ') for j in joints[:7]])
    lower, upper = limits[:, 0], limits[:, 1]
    initial = np.fromstring(official.find('keyframe/key').get('qpos'), sep=' ')
    world_base = np.fromstring(official.find('worldbody/body').get('pos'), sep=' ')

    class DeliveryMount(MountModel):
        def __init__(self, idn=0):
            super().__init__(str(paths['mount']), idn=idn)
        @property
        def top_offset(self):
            return np.array([0, 0, -0.01])
        @property
        def bottom_offset(self):
            # The delivered mount root is already attached at +0.01. Native
            # mounting needs its unassembled bottom offset to place the arm.
            return np.array([0, 0, -world_base[2]]) + self.top_offset
        @property
        def horizontal_radius(self):
            return 0.25

    class DH116URDF(GripperModel):
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

    class NeroURDF(ManipulatorModel):
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
            return 'DeliveryMount'
        @property
        def default_gripper(self):
            return 'DH116URDF'
        @property
        def default_controller_config(self):
            return 'default_joint_position'
        @property
        def init_qpos(self):
            return initial[:7].copy()
        @property
        def base_xpos_offset(self):
            clearance = metadata['base_clearance']
            return {key: lambda length: (-length/2-clearance, 0, 0) for key in ('table', 'kitchen_table')}
        @property
        def top_offset(self):
            return np.asarray(metadata['top_offset'])
        @property
        def _horizontal_radius(self):
            return metadata['base_clearance']

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

    class NeroURDFArm(SingleArm):
        def _load_controller(self):
            self.controller_config['qpos_limits'] = np.stack([lower, upper])
            super()._load_controller()
            self.controller = AbsoluteJointPosition(self.controller)
        def control(self, action, policy_step=False):
            action = np.asarray(action, dtype=float)
            if action.shape != (13,) or not np.isfinite(action).all() or np.any(action[:7] < lower) or np.any(action[:7] > upper) or np.any(np.abs(action[7:]) > 1):
                raise ValueError('Invalid 13D diagnostic action')
            return super().control(action, policy_step=policy_step)

    class MountedNeroURDF(NeroURDF):
        pass

    GRIPPER_MAPPING['DH116URDF'] = DH116URDF
    MOUNT_MAPPING['DeliveryMount'] = DeliveryMount
    for name in ('NeroURDF', 'MountedNeroURDF'):
        ROBOT_CLASS_MAPPING[name] = NeroURDFArm
    return {'paths': paths, 'lower': lower, 'upper': upper}


def configure_native_scene(asset_root):
    """Keep native LIBERO construction, explicitly carrying delivered options."""
    from libero.libero.envs import env_wrapper
    key = 'libero_kitchen_tabletop_manipulation'
    native = env_wrapper.TASK_MAPPING[key]
    official = ET.parse(Path(asset_root) / 'mjcf/nero_dh116.xml').getroot()

    def load_model(self):
        native._load_model(self)
        for tag in ('option', 'compiler', 'size'):
            element = self.model.root.find(tag)
            if element is None:
                element = ET.SubElement(self.model.root, tag)
            element.attrib.update(official.find(tag).attrib)

    # LIBERO asserts that the class name identifies the parsed native problem.
    # Keep that identity and its inherited success predicate; only configure XML.
    env_wrapper.TASK_MAPPING[key] = type(native.__name__, (native,), {'_load_model': load_model})
