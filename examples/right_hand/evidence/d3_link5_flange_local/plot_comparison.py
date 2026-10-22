#!/usr/bin/env python3
"""Illustrate two saved poses with raw-mesh sections; no simulator or new poses.

Numbers are continuous 3D results from the saved actual-compiled-hull audit.
The 2D raw-OBJ sections are illustrations, not clearance certificates.
"""
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np
from scipy.spatial import ConvexHull

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'tools'))
import build_right_proxy76 as mesh


def triangles(path, hull=False):
    vertices, faces = mesh.read_mesh(path)
    return vertices[ConvexHull(vertices).simplices if hull else faces]


def sections(tris, z):
    out = []
    for a, b in ((0, 1), (1, 2), (2, 0)):
        p, q = tris[:, a], tris[:, b]
        valid = ((p[:, 2] <= z) & (q[:, 2] > z)) | ((q[:, 2] <= z) & (p[:, 2] > z))
        ids = np.flatnonzero(valid)
        t = (z-p[ids, 2])/(q[ids, 2]-p[ids, 2])
        out.extend((int(i), point) for i, point in zip(ids, p[ids]+t[:, None]*(q[ids]-p[ids])))
    by_face = {}
    for i, point in out:
        by_face.setdefault(i, []).append(point[:2]*1000)
    return np.asarray([v for v in by_face.values() if len(v) == 2])


def main():
    d = json.loads((HERE/'poses/first-failure-classification.json').read_text())
    manifest = json.loads((ROOT/'examples/right_hand/candidates/link5_flange_local/manifest.json').read_text())
    bodies = ('robot0_link5', 'robot0_gripper_flange')
    source = [triangles(ROOT/p) for p in ('meshes/nero/source/link5-dc497f5a.stl',
        'meshes/nero/source/gripper_flange-186697a7.stl')]
    old = [triangles(ROOT/p, True) for p in ('meshes/nero/robot0_candidate_nero_link5_4-384fc2f2.obj',
        'meshes/mount/robot0_source_flange_part_2-c5005972.obj')]
    new = [[triangles(ROOT/r['new_file'], True) for r in manifest['new_geoms'] if r['parent_body'] == b]
           for b in bodies]
    cached = d['cached_and_selected_post_minima']['cached_RK4']['minimum']
    crossing = d['candidate_failed_actual_compiled_graph_pair']['surface_query']['strict_transverse_witness']['point']
    rows = [(cached['body_world_transforms'], float(np.mean(np.asarray(cached['surface_query']['closest_points'])[:, 2])),
             'Saved RK4 1325', 'source surfaces separated; new hull gap 4.417615 mm'),
            (d['body_world_transforms'], crossing[2], 'Plan m040_quarter3 — FAIL',
             'STL gap 3.532863 mm; new hulls still cross')]
    fig, axes = plt.subplots(2, 3, figsize=(12, 8), sharex=True, sharey=True)
    for row, (poses, z, title, detail) in enumerate(rows):
        a, b = (poses[name] for name in bodies)
        ra, rb = np.asarray(a['rotation']), np.asarray(b['rotation'])
        rot = ra.T@rb
        shift = ra.T@(np.asarray(b['translation_m'])-a['translation_m'])
        def moved(tris):
            return tris@rot.T+shift
        groups = [[source[0]], [source[1]]], [[old[0]], [old[1]]], new
        for col, group in enumerate(groups):
            ax = axes[row, col]
            for body, parts in enumerate(group):
                for part in parts:
                    lines = sections(moved(part) if body else part, z)
                    if len(lines):
                        ax.add_collection(LineCollection(lines, colors=['#1766af', '#d46b12'][body], linewidths=1.0))
            if row == 1:
                ax.plot(crossing[0]*1000, crossing[1]*1000, 'rx', markersize=8)
            ax.set(xlim=(-32, 32), ylim=(-49, 9), aspect='equal', xlabel='link5 x (mm)')
            ax.grid(alpha=.2)
            ax.set_title(['Factory STL surfaces', 'Old two raw convex hulls', 'New five raw convex hulls'][col])
        axes[row, 0].set_ylabel(f'{title}\nlink5 y (mm)')
        axes[row, 1].text(.5, -.22, f'{detail}\nSection z={z*1000:.4f} mm', transform=axes[row, 1].transAxes,
                         ha='center', va='top', fontsize=10)
    fig.suptitle('One local candidate: cached contact cleared, planning false contact remains\n'
                 'Blue: link5; orange: flange. Sections illustrate raw geometry; 3D certificates are separate.')
    fig.subplots_adjust(top=.86, bottom=.11, hspace=.52, wspace=.16)
    fig.savefig(HERE/'comparison.png', dpi=180)
    plt.close(fig)


if __name__ == '__main__':
    main()
