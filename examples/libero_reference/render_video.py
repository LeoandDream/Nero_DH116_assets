"""Render the completed recorded diagnostic with mj_forward only; no integration."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def frame_indices(row_count):
    if row_count != 5750:
        raise ValueError('Expected the completed 5750-row recorded diagnostic')
    return list(range(24, row_count, 25))


def render_video(run, output=None):
    run = Path(run).resolve()
    output = Path(output).resolve() if output is not None else run / 'closure-side.mp4'
    provenance = output.with_suffix('.provenance.json')
    for target in (output, provenance):
        if target.exists():
            raise FileExistsError(target)
    paths = [run / name for name in ('bounded_grasp-model.xml', 'full-reset-state.json', 'bounded_grasp.jsonl')]
    source_hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    rows = [json.loads(line) for line in paths[2].read_text().splitlines()]
    ids = frame_indices(len(rows))
    reset = json.loads(paths[1].read_text())
    import mujoco
    def no_physics_steps(*args, **kwargs):
        raise RuntimeError('Video rendering must not integrate physics')
    original = {name: getattr(mujoco, name) for name in ('mj_step', 'mj_step1', 'mj_step2')}
    renderer = encoder = None
    try:
        for name in original:
            setattr(mujoco, name, no_physics_steps)
        model = mujoco.MjModel.from_xml_path(str(paths[0]))
        model.body_pos[:] = reset['runtime_body_pos']
        model.body_quat[:] = reset['runtime_body_quat']
        model.vis.global_.offwidth, model.vis.global_.offheight = 720, 540
        renderer = mujoco.Renderer(model, height=540, width=720)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat[:] = [-.166, .092, 1.02]
        camera.distance, camera.azimuth, camera.elevation = .45, 45, -40
        options = mujoco.MjvOption()
        options.geomgroup[:] = 0
        options.geomgroup[1] = 1
        encoder = subprocess.Popen(['ffmpeg', '-n', '-loglevel', 'error', '-f', 'rawvideo',
            '-pix_fmt', 'rgb24', '-s', '720x540', '-r', '20', '-i', '-', '-an',
            '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output)],
            stdin=subprocess.PIPE, stderr=subprocess.PIPE)
        for index in ids:
            row = rows[index]
            data = mujoco.MjData(model)
            data.qpos[:], data.qvel[:], data.ctrl[:] = row['full_qpos'], row['full_qvel'], row['internal_actuator_ctrl_13']
            data.time = row['time']
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera, scene_option=options)
            encoder.stdin.write(renderer.render().tobytes())
        encoder.stdin.close()
        error = encoder.stderr.read().decode()
        if encoder.wait() != 0:
            raise RuntimeError('ffmpeg failed: ' + error)
    finally:
        if encoder is not None:
            if encoder.poll() is None:
                encoder.terminate()
                encoder.wait()
            encoder.stderr.close()
        if renderer is not None:
            renderer.close()
        for name, function in original.items():
            setattr(mujoco, name, function)
    probe = json.loads(subprocess.check_output(['ffprobe', '-v', 'quiet', '-print_format',
        'json', '-count_frames', '-show_streams', '-show_format', str(output)]))
    metadata = {'new_simulation_steps': 0, 'method': 'Recorded full state/control and reset body transforms; mj_forward only',
        'source_file_sha256': source_hashes, 'renderer_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'video': str(output), 'video_sha256': hashlib.sha256(output.read_bytes()).hexdigest(),
        'frames': len(ids), 'first_source_row': ids[0], 'last_source_row': ids[-1], 'fps': 20,
        'duration_s': len(ids) / 20, 'strict_grasp_success_claim': False, 'ffprobe': probe}
    with provenance.open('x', encoding='utf-8') as stream:
        stream.write(json.dumps(metadata, indent=2) + '\n')
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True, type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    render_video(args.run, args.output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
