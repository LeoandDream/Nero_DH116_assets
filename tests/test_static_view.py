import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class StaticViewTests(unittest.TestCase):
    def test_mount_hidden_without_changing_physical_fields(self):
        import mujoco
        import numpy as np
        spec=importlib.util.spec_from_file_location('view_static',ROOT/'examples/view_static.py')
        viewer=importlib.util.module_from_spec(spec);spec.loader.exec_module(viewer)
        self.assertTrue(hasattr(viewer,'setup_view'),'view filtering is not implemented')
        model=mujoco.MjModel.from_xml_path(str(ROOT/'examples/scene.xml'));data=mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model,data,0);mujoco.mj_forward(model,data)
        fields=['body_mass','body_inertia','geom_contype','geom_conaffinity','geom_friction','jnt_range','eq_data']
        before={name:getattr(model,name).copy() for name in fields}
        original_groups=model.geom_group.copy()
        option,camera=viewer.setup_view(model,data)
        mount=[i for i in range(model.ngeom) if mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i).startswith('mount0_')]
        self.assertTrue(mount)
        self.assertTrue(all(not option.geomgroup[model.geom_group[i]] for i in mount))
        for name in fields:np.testing.assert_array_equal(before[name],getattr(model,name))
        self.assertEqual(data.time,0)
        visible=[i for i in range(model.ngeom) if option.geomgroup[model.geom_group[i]]]
        self.assertEqual(len(visible),26)
        self.assertGreater(camera.lookat[2],0.95)
        option,_=viewer.setup_view(model,data,show_mount=True,source_groups=original_groups)
        self.assertTrue(any(option.geomgroup[model.geom_group[i]] for i in mount))


if __name__=='__main__':unittest.main()
