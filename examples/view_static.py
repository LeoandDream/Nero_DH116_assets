"""Display the asset's reference pose; never advance physics or apply actions."""
import argparse
from pathlib import Path
import time


def setup_view(model, data, show_collision=False, show_mount=False, source_groups=None):
    """Filter only drawing groups and frame the visible robot, without stepping."""
    import mujoco
    import numpy as np
    groups = model.geom_group.copy() if source_groups is None else source_groups
    mount = np.array([i for i in range(model.ngeom)
                      if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or '').startswith('mount0_')], dtype=int)
    # Groups 4/5 are drawing-only mount layers. Contact, inertia and poses stay intact.
    model.geom_group[mount] = np.where(groups[mount] == 0, 4, 5)
    option = mujoco.MjvOption()
    option.geomgroup[0] = int(show_collision)
    option.geomgroup[4] = int(show_mount and show_collision)
    option.geomgroup[5] = int(show_mount)
    visible = np.array([i for i in range(model.ngeom) if option.geomgroup[model.geom_group[i]]], dtype=int)
    centers = data.geom_xpos[visible]
    radii = model.geom_rbound[visible, None]
    low, high = (centers - radii).min(axis=0), (centers + radii).max(axis=0)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = (low + high) / 2
    camera.distance = max(0.7, float(np.linalg.norm(high - low)) * 1.35)
    camera.azimuth = 135; camera.elevation = -20
    return option, camera


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, help='save one static image without opening a window')
    parser.add_argument('--collision', action='store_true', help='also show collision proxies')
    parser.add_argument('--mount', action='store_true', help='show the benchmark mounting stand')
    args = parser.parse_args()
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(Path(__file__).parent / 'scene.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    option, camera = setup_view(model, data, args.collision, args.mount)
    if args.image:
        renderer = mujoco.Renderer(model, height=480, width=640)
        renderer.update_scene(data, camera=camera, scene_option=option)
        from PIL import Image
        Image.fromarray(renderer.render()).save(args.image)
        renderer.close()
        print(args.image)
    else:
        import mujoco.viewer
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.lookat[:] = camera.lookat
            viewer.cam.distance = camera.distance
            viewer.cam.azimuth = camera.azimuth; viewer.cam.elevation = camera.elevation
            viewer.opt.geomgroup[:] = option.geomgroup
            while viewer.is_running():
                viewer.sync(); time.sleep(0.05)


if __name__ == '__main__': main()
