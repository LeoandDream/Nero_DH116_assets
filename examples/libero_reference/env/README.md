# Isolated LIBERO reference runtime

From the repository root on Linux x86_64:

```bash
bash examples/libero_reference/env/bootstrap.sh
source examples/libero_reference/env/activate.sh
python examples/libero_reference/env/provenance.py --output .reference-deps/environment.json
```

Install `curl`, `git`, a C compiler, and Linux input headers (Debian/Ubuntu:
`sudo apt-get install curl git build-essential linux-libc-dev libegl1 libgl1 libglib2.0-0`).
Rendering needs an EGL-capable driver (`libegl1` plus the appropriate GPU driver, or Mesa
`libegl-mesa0` and `libgl1-mesa-dri` for software rendering). `MUJOCO_GL=egl` is set
by activation. No display server is required.

The bootstrap downloads SHA256-pinned CPython 3.11.16 into `.reference-deps/python`,
creates a new `.reference-env`, and installs the exact dependencies in
`build.lock` and `requirements.lock`. MuJoCo is 3.2.3 and stock robosuite is 1.4.1.
It clones public LIBERO at `f78abd68ee283de9f9be3c8f7e2a9ad60246e95c`
into `.reference-deps/LIBERO` and checks all 1,116 tracked source and asset files
against `libero-source.sha256`. The editable LIBERO install preserves its assets.
`source-lock.json` records public source URLs and hashes. All download inputs are
fetched from public sources by default; existing matching download files may be
reused. No previous checkout or virtual environment is needed.

Activation clears `PYTHONPATH` and `PYTHONHOME`, disables user site packages, and
keeps LIBERO config and caches inside `.reference-deps`. No policy, dataset,
checkpoint, torch, or JAX is installed. `opencv-python==4.11.0.86` owns `cv2`.

The provenance check rejects incorrect versions, outside import paths, a modified
interpreter or LIBERO source, and changes to any of the 1,019 stock robosuite
package files. It imports `OffScreenRenderEnv` and renders a 32×32 EGL frame.
The generated machine-specific report belongs in ignored
`.reference-deps/environment.json`; it is not a source artifact. Run the focused
rejection tests with `python examples/libero_reference/env/test_provenance.py`.
The upstream robosuite private-macro warning and Gym notice are expected.
