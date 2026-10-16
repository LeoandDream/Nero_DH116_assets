"""Reject version/path contamination and record the LIBERO reference runtime provenance."""
from __future__ import annotations

import argparse
import hashlib
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import zipfile

LIBERO_COMMIT = "f78abd68ee283de9f9be3c8f7e2a9ad60246e95c"
PYTHON_SHA256 = "ec579de7512e84d960bc642a840de69c785c52d6adf89e3a15f51717b29be40f"


def validate_module(name, expected, actual, module_file, allowed_root):
    if actual != expected:
        raise ValueError(f"{name} version {actual!r}, expected {expected!r}")
    if not Path(module_file).resolve().is_relative_to(allowed_root.resolve()):
        raise ValueError(f"{name} outside fresh environment: {module_file}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    worktree = Path(__file__).resolve().parents[3]
    venv = worktree / ".reference-env"
    deps = worktree / ".reference-deps"
    if Path(sys.prefix).resolve() != venv.resolve():
        raise ValueError(f"Use {venv / 'bin/python'}, got prefix {sys.prefix}")
    if platform.python_version() != "3.11.16":
        raise ValueError(f"Expected Python 3.11.16, got {platform.python_version()}")
    if os.environ.get("PYTHONPATH"):
        raise ValueError("PYTHONPATH must be unset for isolated provenance")
    expected_base = deps / "python/bin/python3.11"
    if Path(sys._base_executable).resolve() != expected_base.resolve():
        raise ValueError(f"Interpreter outside fresh dependency directory: {sys._base_executable}")
    interpreter_sha = hashlib.sha256(expected_base.read_bytes()).hexdigest()
    if interpreter_sha != PYTHON_SHA256:
        raise ValueError(f"Unexpected Python executable SHA256: {interpreter_sha}")
    expected_config = deps / "config"
    if Path(os.environ.get("LIBERO_CONFIG_PATH", "")).resolve() != expected_config.resolve():
        raise ValueError(f"LIBERO_CONFIG_PATH must be {expected_config}")
    outside_paths = [entry for entry in sys.path
                     if not Path(entry or os.getcwd()).resolve().is_relative_to(worktree)]
    if outside_paths:
        raise ValueError(f"Import search paths outside fresh checkout: {outside_paths}")
    installed = {d.metadata["Name"]: d.version for d in importlib.metadata.distributions()}
    normalized = {name.lower().replace("_", "-"): version for name, version in installed.items()}
    for lockfile in ("build.lock", "requirements.lock"):
        for line in Path(__file__).with_name(lockfile).read_text().splitlines():
            if not line.strip() or line.startswith("#"):
                continue
            name, expected = line.split()[0].split("==")
            actual = normalized.get(name.lower().replace("_", "-"))
            if actual != expected:
                raise ValueError(f"Locked package {name}: expected {expected}, got {actual}")
    modules = {}
    for name, package, expected in [
        ("numpy", "numpy", "1.26.4"), ("mujoco", "mujoco", "3.2.3"),
        ("robosuite", "robosuite", "1.4.1"), ("cv2", "opencv-python", "4.11.0.86"),
        ("libero.libero", "libero", "0.1.0"),
    ]:
        module = importlib.import_module(name)
        actual = importlib.metadata.version(package)
        allowed = deps / "LIBERO" if package == "libero" else venv
        validate_module(name, expected, actual, module.__file__, allowed)
        modules[name] = {"distribution": package, "version": actual, "file": str(Path(module.__file__).resolve())}
    # Compare installed stock files to the hash-pinned wheel, including full package inventory.
    wheel = deps / "downloads/packages/robosuite-1.4.1-py3-none-any.whl"
    if hashlib.sha256(wheel.read_bytes()).hexdigest() != "62587cd5888d9d3d813f3ffcc7017f6f77efa618952527a0b9bb139ffd0a3741":
        raise ValueError("Stock robosuite wheel hash mismatch")
    stock_count = 0
    stock_names = set()
    with zipfile.ZipFile(wheel) as archive:
        site = Path(modules["robosuite"]["file"]).parents[1]
        for name in archive.namelist():
            if name.startswith("robosuite/") and not name.endswith("/"):
                if (site / name).read_bytes() != archive.read(name):
                    raise ValueError(f"Installed stock robosuite file differs: {name}")
                stock_count += 1
                stock_names.add(name)
    installed_names = {str(path.relative_to(site)) for path in (site / "robosuite").rglob("*")
                       if path.is_file() and "__pycache__" not in path.parts
                       and path.suffix != ".pyc"}
    if installed_names != stock_names:
        raise ValueError(f"Stock robosuite inventory differs: {sorted(installed_names ^ stock_names)}")
    source = deps / "LIBERO"
    if (source / ".git/objects/info/alternates").exists():
        raise ValueError("LIBERO git object store must have no alternate dependency")
    commit = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    if commit != LIBERO_COMMIT:
        raise ValueError(f"Unexpected LIBERO commit: {commit}")
    subprocess.run(["git", "-C", str(source), "diff", "--exit-code", "HEAD"], check=True, stdout=subprocess.DEVNULL)
    manifest = Path(__file__).with_name("libero-source.sha256")
    tracked = set(subprocess.check_output(["git", "-C", str(source), "ls-files"], text=True).splitlines())
    verified = set()
    for line in manifest.read_text().splitlines():
        digest, relative = line.split("  ", 1)
        path = source / relative
        if not path.resolve().is_relative_to(source.resolve()):
            raise ValueError(f"LIBERO source path escapes clone: {relative}")
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError(f"LIBERO source hash mismatch: {relative}")
        verified.add(relative)
    if verified != tracked or len(verified) != 1116:
        raise ValueError("LIBERO source manifest does not match all 1,116 tracked files")
    from libero.libero import get_libero_path
    for key in ("benchmark_root", "bddl_files", "assets", "init_states"):
        path = Path(get_libero_path(key)).resolve()
        if not path.is_relative_to(source.resolve()) or not path.is_dir():
            raise ValueError(f"Invalid LIBERO {key}: {path}")
    from libero.libero.envs import OffScreenRenderEnv
    if any(name.lower().split("-")[0] in {"torch", "jax", "jaxlib"} for name in installed):
        raise ValueError("Training dependencies unexpectedly installed")
    cv2_owners = sorted(name for name in installed if name.lower().startswith("opencv-"))
    if cv2_owners != ["opencv-python"]:
        raise ValueError(f"Unexpected cv2 owners: {cv2_owners}")
    mujoco = importlib.import_module("mujoco")
    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type="sphere" size="0.1"/></worldbody></mujoco>')
    with mujoco.Renderer(model, height=32, width=32) as renderer:
        renderer.update_scene(mujoco.MjData(model))
        rgb = renderer.render()
        egl_smoke = {"backend": os.environ.get("MUJOCO_GL"), "shape": list(rgb.shape), "dtype": str(rgb.dtype)}
    result = {
        "stock_robosuite_files_verified_against_wheel": stock_count,
        "egl_smoke": egl_smoke,
        "python": {"version": platform.python_version(), "executable": sys.executable,
                   "base_executable": sys._base_executable, "sha256": interpreter_sha},
        "libero_source_files_verified": len(verified),
        "libero_source_manifest_sha256": hashlib.sha256(manifest.read_bytes()).hexdigest(),
        "libero_commit": commit, "libero_config_path": str(expected_config),
        "libero_git_alternates": False,
        "libero_origin": subprocess.check_output(["git", "-C", str(source), "remote", "get-url", "origin"], text=True).strip(),
        "modules": modules, "distributions": dict(sorted(installed.items())),
        "offscreen_env_import": f"{OffScreenRenderEnv.__module__}.{OffScreenRenderEnv.__name__}",
        "sys_path": sys.path,
    }
    data = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(data)
    print(data)


if __name__ == "__main__":
    main()
