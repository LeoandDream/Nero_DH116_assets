"""Display the asset's reference pose; never advance physics or apply actions."""
import argparse
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, help='save one static image without opening a window')
    parser.add_argument('--collision', action='store_true', help='also show collision proxies')
    args = parser.parse_args()
    import mujoco
    import numpy as np
    model = mujoco.MjModel.from_xml_path(str(Path(__file__).parent / 'scene.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    option = mujoco.MjvOption()
    option.geomgroup[0] = int(args.collision)
    camera = mujoco.MjvCamera()
    positions = data.xpos[1:]
    camera.lookat[:] = (positions.min(axis=0) + positions.max(axis=0)) / 2
    camera.distance = max(1.2, float(np.linalg.norm(positions.max(axis=0) - positions.min(axis=0))) * 1.6)
    camera.azimuth = 135; camera.elevation = -20
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
            viewer.opt.geomgroup[0] = int(args.collision)
            while viewer.is_running():
                viewer.sync(); time.sleep(0.05)


if __name__ == '__main__': main()
