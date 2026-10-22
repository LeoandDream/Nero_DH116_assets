"""Static continuous plane sections of the one frozen finger13 repair.

Reads repository assets and saved body transforms only. No simulator is imported,
no kinematics or collision solver is run, and no mesh candidate is generated.
Plane intersections reuse the previously delivered source/proxy plotting tool.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
import numpy as np
from scipy.spatial import ConvexHull

ROOT = next(p for p in Path(__file__).resolve().parents
            if (p/'mjcf/nero_dh116_right.xml').is_file())
sys.path.insert(0, str(ROOT/'examples/right_hand'))
import proxy76_validate as inputs


def _module(path):
    spec = importlib.util.spec_from_file_location('delivered_section_plotter', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


PLOT_PATH = ROOT/'examples/right_hand/evidence/d3_proxy76/visual/plot_static.py'
SECTIONS = _module(PLOT_PATH)
BLUE, PURPLE, OLD, NEW, OTHER = '#176FA8', '#7F3C8D', '#D46330', '#20864B', '#8C6D16'


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def bindings(path, link, group):
    xml = ET.parse(path).getroot()
    assets = {a.get('name'): a for a in xml.findall('asset/mesh')}
    body = next(b for b in xml.findall('.//body')
                if b.get('name') == 'gripper0_'+link)
    return [inputs._binding(path, xml, assets, g) for g in body.findall('geom')
            if g.get('group', '0') == group]


def hull_triangles(triangles):
    vertices = np.unique(triangles.reshape(-1, 3), axis=0)
    return vertices[ConvexHull(vertices).simplices]


def lines(ax, triangles, axis, value, projection, color, *, lw=1.4, style='solid'):
    cut = SECTIONS.intersections(triangles, axis, value)
    if len(cut):
        ax.add_collection(LineCollection(cut[:, :, projection]*1000,
                                        colors=color, linewidths=lw, linestyles=style,
                                        zorder=4))
    return len(cut)


def fill_hull(ax, triangles, axis, value, projection, color, alpha):
    cut = SECTIONS.intersections(triangles, axis, value)
    if not len(cut):
        return []
    points = np.unique(np.round(cut[:, :, projection].reshape(-1, 2), 13), axis=0)
    if len(points) < 3:
        return []
    polygon = points[ConvexHull(points).vertices]*1000
    ax.fill(polygon[:, 0], polygon[:, 1], facecolor=color, alpha=alpha, zorder=1)
    closed = np.vstack([polygon, polygon[0]])
    ax.plot(closed[:, 0], closed[:, 1], color=color, lw=1.25, ls='--', zorder=3)
    return polygon.tolist()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True,
                        help='A new output directory; existing directories are refused')
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise ValueError('Output must be a new directory')
    formal = ROOT/'mjcf/nero_dh116_right.xml'
    old_model = ROOT/'examples/right_hand/candidates/proxy76/model.xml'
    new_model = ROOT/'examples/right_hand/candidates/finger13_local/model.xml'
    cert_path = ROOT/'examples/right_hand/evidence/d3_proxy76/existing-source-certificates.json'
    center_path = ROOT/'examples/right_hand/evidence/d3_proxy76/gap_check/source-center.json'
    record = next(r for r in json.loads(cert_path.read_text())['records']
                  if r['pose_id'] == 'two_exclusion17_step_6451_post')
    center = np.array(json.loads(center_path.read_text())['results']['finger13_link']['center_body_local_m'])
    source13 = bindings(formal, 'finger13_link', '1')[0]
    source42 = bindings(formal, 'finger42_link', '1')[0]
    old13 = bindings(old_model, 'finger13_link', '0')
    new13 = bindings(new_model, 'finger13_link', '0')
    old42 = bindings(old_model, 'finger42_link', '0')
    new42 = bindings(new_model, 'finger42_link', '0')
    if len(old13) != 1 or len(new13) != 3 or old42 != new42:
        raise ValueError('Expected the one fixed 1-to-3 repair with identical finger42')
    paths = {formal, old_model, new_model, cert_path, center_path, PLOT_PATH,
             Path(__file__).resolve(), ROOT/'examples/right_hand/proxy76_validate.py',
             ROOT/'tools/build_right_proxy76.py'}
    paths.update(Path(b['file']) for b in [source13, source42]+old13+new13+old42)
    before = {str(p.relative_to(ROOT)): _sha(p) for p in sorted(paths)}
    t13 = inputs.mesh_triangles(source13)
    t42 = inputs.mesh_triangles(source42)
    old_parts = [hull_triangles(inputs.mesh_triangles(b)) for b in old13]
    new_parts = [hull_triangles(inputs.mesh_triangles(b)) for b in new13]
    other_parts = [hull_triangles(inputs.mesh_triangles(b)) for b in old42]
    transforms = record['body_world_transforms']
    a, b = transforms['finger13_link'], transforms['finger42_link']
    ra, rb = np.array(a['rotation']), np.array(b['rotation'])
    ta, tb = np.array(a['translation_m']), np.array(b['translation_m'])

    def into13(triangles):
        return ((triangles@rb.T+tb)-ta)@ra

    t42 = into13(t42)
    other_parts = [into13(t) for t in other_parts]
    closest = np.asarray(record['certificate']['closest_points_world_m'])
    closest13 = (closest-ta)@ra
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 11.7))
    fig.subplots_adjust(left=.075, right=.985, top=.90, bottom=.155, hspace=.33, wspace=.23)
    panels = []
    for col, (name, color, parts) in enumerate([('Old: one convex hull', OLD, old_parts),
                                               ('Repair: three source-driven convex parts', NEW, new_parts)]):
        ax = axes[0, col]
        polygons = [fill_hull(ax, t, 0, center[0], (1, 2), color, .13) for t in parts]
        count = lines(ax, t13, 0, center[0], (1, 2), BLUE)
        ax.scatter(center[1]*1000, center[2]*1000, marker='x', c='black', s=42, zorder=6)
        ax.set(xlim=(-12, 12), ylim=(-11, 38), xlabel='finger13 local y (mm)',
               ylabel='finger13 local z (mm)',
               title=name+'\nLocal slot section: x = '+f'{center[0]*1000:.6f} mm')
        ax.set_aspect('equal')
        ax.grid(alpha=.15)
        panels.append({'panel': f'top_{col}', 'frame': 'finger13 local',
                       'axis': 'x', 'plane_m': float(center[0]), 'source_section_segments': count,
                       'proxy_section_polygons_mm': polygons,
                       'limits_mm': {'horizontal': [-12, 12], 'vertical': [-11, 38]}})
        ax = axes[1, col]
        other_polygons = [fill_hull(ax, t, 2, center[2], (0, 1), OTHER, .12)
                          for t in other_parts]
        polygons = [fill_hull(ax, t, 2, center[2], (0, 1), color, .15) for t in parts]
        count13 = lines(ax, t13, 2, center[2], (0, 1), BLUE, lw=1.7)
        count42 = lines(ax, t42, 2, center[2], (0, 1), PURPLE, lw=1.7)
        ax.scatter(center[0]*1000, center[1]*1000, marker='x', c='black', s=42, zorder=6)
        ax.set(xlim=(-5.5, .5), ylim=(-9.5, -3.5), xlabel='finger13 local x (mm)',
               ylabel='finger13 local y (mm)',
               title='6451-post: 13 / 42 at the SAME saved pose\n'+name+
                     '; z = '+f'{center[2]*1000:.6f} mm')
        ax.set_aspect('equal')
        ax.grid(alpha=.15)
        panels.append({'panel': f'bottom_{col}', 'frame': 'finger13 local at saved6451post',
                       'axis': 'z', 'plane_m': float(center[2]),
                       'source_section_segments': {'finger13': count13, 'finger42': count42},
                       'finger13_proxy_section_polygons_mm': polygons,
                       'finger42_proxy_section_polygons_mm': other_polygons,
                       'limits_mm': {'horizontal': [-5.5, .5], 'vertical': [-9.5, -3.5]}})
    fig.suptitle('One frozen finger13 slot repair | exact continuous triangle / plane sections\n'
                 'Right-source triangles, independently convexified actual proxy meshes, cached 6451-post transforms',
                 fontsize=13)
    handles = [Line2D([0], [0], color=BLUE, label='Right original finger13 STL'),
               Line2D([0], [0], color=PURPLE, label='Right original finger42 STL'),
               Line2D([0], [0], color=OLD, ls='--', label='Old finger13 hull'),
               Line2D([0], [0], color=NEW, ls='--', label='New finger13 parts'),
               Line2D([0], [0], color=OTHER, ls='--', label='Unchanged finger42 hull'),
               Line2D([0], [0], color='black', marker='x', ls='', label='OLD convex overlap witness')]
    fig.legend(handles=handles, loc='lower center', ncol=3, bbox_to_anchor=(.5, .075), fontsize=9)
    fig.text(.05, .025,
             'Qualitative sections only. Source surfaces at this full pose have a 0.229435 mm minimum gap (saved certificate).\n'
             'The gap is a 3D minimum, not measured in these sections. No global clearance or dynamics PASS is inferred from the figure.\n'
             'No simulator, FK, contact solve, mesh scan, state injection or new dynamics. Other 75 hand proxies are frozen.',
             fontsize=8.8)
    output.mkdir(parents=True)
    fig.savefig(output/'finger13_local_comparison.png', dpi=180)
    fig.savefig(output/'finger13_local_comparison.pdf')
    after = {str(p.relative_to(ROOT)): _sha(p) for p in sorted(paths)}
    if before != after:
        raise AssertionError('An input changed during plotting')
    metadata = {'schema': 'finger13-local-repair-static-sections-v1',
                'method': 'Actual triangle-plane intersection reused from d3_proxy76 visual/plot_static.py. '
                          'Each existing mesh independently ConvexHull-convexified for analysis, not decomposition. '
                          'Only cached body transforms map finger42 to finger13 frame.',
                'input_sha256': before, 'inputs_unchanged': True,
                'pose_id': record['pose_id'], 'body_world_transforms': transforms,
                'full_qpos': record['full_qpos'],
                'old_common_ball_center_finger13_local_m': center.tolist(),
                'saved_source_minimum_distance_m': record['certificate']['surface_min_distance_m'],
                'saved_source_closest_points_finger13_frame_m': closest13.tolist(),
                'panels': panels, 'new_dynamics_steps': 0, 'mujoco_imported': False,
                'kinematics_calls': 0, 'collision_solver_calls': 0,
                'interpretation': 'Sections illustrate a fixed source-driven repair; the figure is not whole-pair clearance evidence.',
                'outputs': {p.name: _sha(p) for p in output.iterdir()}}
    (output/'metadata.json').write_text(json.dumps(metadata, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'output': str(output), 'inputs_unchanged': True,
                      'dynamics_steps': 0, 'png_sha256': metadata['outputs']['finger13_local_comparison.png']}))


if __name__ == '__main__':
    main()
