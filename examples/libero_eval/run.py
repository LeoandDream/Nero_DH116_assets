#!/usr/bin/env python3
"""Select L/R recorded episodes; integration requires explicit --execute."""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

from episode import aggregate, export_episode, write

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command')
    totals = sub.add_parser('aggregate', help='Aggregate already exported episodes; never integrates')
    totals.add_argument('episodes', nargs='+', type=Path)
    totals.add_argument('--output', type=Path)
    # Preserve the small direct interface requested for the first usable version.
    parser.add_argument('--hand', choices=['L', 'R'])
    parser.add_argument('--mode', choices=['baseline', 'research'], default='baseline')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--recorded-source', type=Path)
    parser.add_argument('--execute', action='store_true', help='Explicitly integrate one selected backend; no retries')
    args = parser.parse_args(argv)
    if args.command == 'aggregate':
        result = aggregate(args.episodes)
        if args.output:
            if args.output.exists():
                parser.error('Aggregate output already exists')
            write(args.output, result)
        print(json.dumps(result, indent=2))
        return 0
    if args.hand is None or args.output is None:
        parser.error('--hand and --output are required')
    if args.output.exists():
        parser.error('Use a NEW output directory')
    source = args.recorded_source
    backend_exit_code = 0
    if args.execute:
        if source is not None:
            parser.error('--execute and --recorded-source are mutually exclusive')
        if args.hand == 'R' and args.mode != 'research':
            parser.error('Right execution is only the explicitly authorized frozen research backend')
        if args.hand == 'L' and args.mode != 'baseline':
            parser.error('Left execution retains its delivered baseline')
        # Execution and export have separate new directories. An interrupted
        # backend keeps its raw evidence; this command never retries it.
        source = args.output.with_name(args.output.name + '-raw')
        if source.exists():
            parser.error('Raw sibling output already exists; refusing another attempt')
        if args.hand == 'L':
            process = subprocess.run([sys.executable, str(ROOT / 'examples/libero_reference/run.py'),
                                      '--output', str(source)])
            backend_exit_code = process.returncode
            if not (source / 'run-result.json').exists():
                return process.returncode or 1
        else:
            right = ROOT / 'examples/right_hand'
            sys.path.insert(0, str(right))
            spec = importlib.util.spec_from_file_location('libero_research', right / 'libero_research.py')
            backend = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = backend
            spec.loader.exec_module(backend)
            backend_exit_code = backend.run_once(source)
            if not isinstance(backend_exit_code, int):
                raise TypeError('Backend must return its actual integer exit code')
    result = export_episode(ROOT, args.hand, args.mode, args.output, source)
    if args.execute:
        manifest_path = args.output / 'manifest.json'
        manifest = json.loads(manifest_path.read_text())
        manifest['source_was_executed_by_this_command'] = True
        write(manifest_path, manifest)
    print(json.dumps(result, indent=2))
    return backend_exit_code


if __name__ == '__main__':
    raise SystemExit(main())
