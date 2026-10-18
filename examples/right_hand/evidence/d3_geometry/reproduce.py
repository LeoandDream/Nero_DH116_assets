#!/usr/bin/env python3
"""Recompute this read-only diagnosis into a new output directory; never step physics."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import runpy
import sys

HERE = Path(__file__).resolve().parent
ROOT = next(p for p in HERE.parents if (p / 'mjcf/nero_dh116_right.xml').is_file())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, help='New directory inside this checkout')
    parser.add_argument('--figures', action='store_true', help='Also generate source-mesh keyframes and sections')
    args = parser.parse_args()
    output = (ROOT / args.output).resolve()
    if output == ROOT or not output.is_relative_to(ROOT):
        parser.error('Output must be a new directory inside this checkout')
    output.mkdir(parents=True, exist_ok=False)
    for folder in ('trace', 'triangle', 'figures'):
        (output / folder).mkdir()
    os.environ['NERO_D3_OUTPUT'] = str(output)
    sys.dont_write_bytecode = True
    import mujoco
    if mujoco.__version__ != '3.2.3':
        raise RuntimeError('Use the fixed MuJoCo 3.2.3 environment')

    def forbidden(*_args, **_kwargs):
        raise RuntimeError('Physical stepping and constraint solving are forbidden in D3 diagnosis')

    for name in ('mj_step', 'mj_step1', 'mj_step2', 'mj_forward', 'mj_forwardSkip',
                 'mj_fwdConstraint', 'mj_RungeKutta', 'mj_Euler'):
        setattr(mujoco, name, forbidden)
    inputs = [ROOT / 'mjcf/nero_dh116_right.xml', ROOT / 'urdf/dh116_right.urdf']
    inputs += sorted((ROOT / 'meshes/dh116_right/source').glob('*.STL'))
    inputs += [ROOT / 'examples/right_hand/evidence' / name for name in
               ('hand-motion-state.jsonl.gz', 'hand-motion-contacts.jsonl.gz', 'frozen-commands.json.gz')]
    before = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    expected = json.loads((HERE / 'read-only-check.json').read_text())['input_sha256']
    if before != expected:
        raise RuntimeError('Model, original meshes, or recorded inputs differ from the fixed D3 baseline')
    stages = ['trace/analyze_trace.py', 'trace/summarize_trace.py', 'triangle/run_queries.py',
              'triangle/enrich_results.py', 'triangle/verify_geometry.py']
    if args.figures:
        stages += ['figures/plot_sections.py', 'figures/render_keyframes.py']
    for stage in stages:
        path = HERE / stage
        sys.path.insert(0, str(path.parent))
        try:
            runpy.run_path(str(path), run_name='__main__')
        finally:
            sys.path.pop(0)
    after = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    if before != after:
        raise RuntimeError('Input model, meshes, or original records changed')
    result = {'physics_steps': 0, 'constraint_solves': 0, 'stepping_and_solve_APIs_blocked': True,
              'input_sha256': before, 'inputs_unchanged': before == after}
    (output / 'read-only-check.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'output': str(output), 'physics_steps': 0, 'inputs_unchanged': True}))


if __name__ == '__main__':
    main()
