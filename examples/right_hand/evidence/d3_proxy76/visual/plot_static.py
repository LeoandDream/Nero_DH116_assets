"""One-page qualitative source/actual-proxy sections; no simulator or baking."""
import argparse
from pathlib import Path
import hashlib
import json
import sys
import numpy as np
from scipy.spatial import ConvexHull
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

ROOT = next(p for p in Path(__file__).resolve().parents
            if (p/'mjcf/nero_dh116_right.xml').is_file())
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'examples/right_hand'))
import proxy76_validate as inputs

SOURCE, PROXY = '#176FA8', '#D46330'


def intersections(triangles, axis, value):
    d = triangles[:, :, axis]-value
    mask = (d.min(axis=1) <= 1e-12) & (d.max(axis=1) >= -1e-12)
    segments = []
    for tri, dd in zip(triangles[mask], d[mask]):
        points = []
        for j in range(3):
            k = (j+1)%3
            if abs(dd[j]) <= 1e-12:
                points.append(tri[j])
            if dd[j]*dd[k] < 0:
                points.append(tri[j]+dd[j]/(dd[j]-dd[k])*(tri[k]-tri[j]))
        if len(points) < 2:
            continue
        points = np.unique(np.round(np.asarray(points), 13), axis=0)
        if len(points) == 2:
            segments.append(points)
        elif len(points) == 3:
            segments.extend(points[[[0,1],[1,2],[2,0]]])
    return np.asarray(segments)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True,
                        help='New directory for the static figure and metadata')
    parser.add_argument('--metrics', type=Path, default=HERE.parent/'metrics')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    candidate = ROOT/'examples/right_hand/candidates/proxy76/model.xml'
    alignment_path = candidate.parent/'alignment.json'
    alignment = json.loads(alignment_path.read_text())
    jobs, paths, _, _ = inputs.load_inputs(candidate)
    before = inputs._snapshot(paths)
    jobs = {job['link']:job for job in jobs}
    fig = plt.figure(figsize=(14, 15.6))
    fig.subplots_adjust(left=.085,right=.98,top=.945,bottom=.15,hspace=.58,wspace=.23)
    grid = fig.add_gridspec(3, 2, height_ratios=[1.12,1,1])
    ax = fig.add_subplot(grid[0,:])
    names = list(alignment['links'])
    y = np.arange(len(names))
    identity = [alignment['links'][name]['identity']['pooled']['p95_m']*1000 for name in names]
    chosen = [alignment['links'][name]['chosen']['pooled']['p95_m']*1000 for name in names]
    ax.barh(y-.18, identity, height=.34, color='#97B9CF', label='Identity: left/right source p95')
    ax.barh(y+.18, chosen, height=.34, color='#4B6B55', label='Chosen: left/right source p95')
    ax.set(yticks=y, yticklabels=names, xscale='log', xlabel='Bidirectional pooled distance p95 (mm; log scale)',
           title='Frozen source alignment | 8192 area samples per side per link; continuous target triangles')
    ax.invert_yaxis()
    ax.grid(axis='x', alpha=.2)
    ax.legend(loc='lower right', fontsize=9)
    ax.tick_params(labelsize=8.5)
    specifications = [
        ('base_link', 2, .0338625108279435, (0,1), 'Palm/base: local XY section', None),
        ('finger12_link', 2, 0., (0,1), 'finger12 hinge: local XY section', None),
        ('finger14_link', 2, .022469260266069258, (0,1), 'finger14 tip pad: local XY section', None),
        ('finger13_link', 0, .003967694148982454, (1,2), 'finger13 gap: one convex collision hull', (-11,14)),
    ]
    metadata = {'figure_scope':'Qualitative plane sections; not a quantitative acceptance measurement.',
                'proxy':'Each actual candidate OBJ/STL is independently ConvexHull-convexified for analysis only. No decomposition or asset output.',
                'section_algorithm':'Original source triangle-plane intersection and per-part convex-hull plane intersection; no surface point sampling for section lines.',
                'panels':[]}
    labels = ['x','y','z']
    for i, (name, axis, value, project, title, ylim) in enumerate(specifications):
        ax = fig.add_subplot(grid[1+i//2,i%2])
        job = jobs[name]
        source = inputs.mesh_triangles(job['source'])
        section = intersections(source, axis, value)
        if not len(section):
            raise ValueError('Empty source section: '+name)
        projected = section[:,:,project]*1000
        hull_polygons = []
        for part in job['parts']:
            tri = inputs.mesh_triangles(part)
            vertices = np.unique(tri.reshape(-1,3),axis=0)
            hull = ConvexHull(vertices)
            cut = intersections(vertices[hull.simplices],axis,value)
            if not len(cut):
                continue
            points = np.unique(np.round(cut[:,:,project].reshape(-1,2),12),axis=0)
            if len(points)<3:
                continue
            polygon = points[ConvexHull(points).vertices]*1000
            hull_polygons.append(polygon)
            ax.fill(polygon[:,0],polygon[:,1],facecolor=PROXY,alpha=.14,zorder=1)
            closed = np.vstack([polygon,polygon[0]])
            ax.plot(closed[:,0],closed[:,1],color=PROXY,lw=1.15,ls=(0,(5,2)),alpha=.95,zorder=2)
        ax.add_collection(LineCollection(projected, colors=SOURCE, linewidths=1.35,zorder=3))
        all_points = np.concatenate([projected.reshape(-1,2)]+hull_polygons)
        lo,hi = all_points.min(axis=0),all_points.max(axis=0)
        pad = np.maximum(.7,.04*(hi-lo))
        ax.set(xlim=(lo[0]-pad[0],hi[0]+pad[0]),ylim=(lo[1]-pad[1],hi[1]+pad[1]),
               xlabel=labels[project[0]]+' (mm)',ylabel=labels[project[1]]+' (mm)',
               title=title+'\n'+labels[axis]+f' = {value*1000:.3f} mm; {len(job["parts"])} actual part(s)')
        if ylim is not None:
            ax.set_ylim(*ylim)
        ax.set_aspect('equal', adjustable='box')
        ax.grid(alpha=.15)
        metric_path = args.metrics.resolve()/(name+'.json')
        if not metric_path.is_file():
            raise FileNotFoundError(metric_path)
        record = {'link':name,'plane_axis':axis,'plane_value_m':value,'projection_axes':project,
                  'actual_part_count':len(job['parts']),'section_source_segments':len(section)}
        if metric_path.exists():
            metric = json.loads(metric_path.read_text())
            smax = metric['source_to_proxy_union']['max_m']*1000
            pmax = metric['proxy_union_to_source']['max_m']*1000
            annotation = f'Whole-link 3D sample max: S→U {smax:.3f} mm; U→S {pmax:.3f} mm'
            record.update({'metrics_path':str(metric_path.relative_to(ROOT)),
                           'metrics_sha256':inputs._sha(metric_path),
                           'whole_link_3d_source_to_union_sample_max_mm':smax,
                           'whole_link_3d_union_to_source_sample_max_mm':pmax})
        ax.text(.5,-.19,annotation,ha='center',va='top',transform=ax.transAxes,fontsize=8.5)
        metadata['panels'].append(record)
    fig.suptitle('Right DH116 · delivered proxy76 static comparison', fontsize=16,fontweight='bold',y=.986)
    fig.legend(handles=[Line2D([0],[0],color=SOURCE,lw=1.6,label='S: formal-right original source STL section'),
                        Line2D([0],[0],color=PROXY,lw=1.4,ls='--',label='U: actual per-part convex proxy section'),
                        Patch(facecolor=PROXY,alpha=.14,label='Proxy convex interior (overlaps retained in drawing)')],
               loc='lower center',bbox_to_anchor=(.5,.048),ncol=1,fontsize=9)
    fig.text(.5,.012,'Sections illustrate hinge / pad / gap geometry; individual hull outlines can include internal overlap boundaries.\n'
             'Source STL surfaces are not classified as exterior/interior. 3D area-sample metrics are independent of this drawing.\n'
             'Finite sample maxima are not Hausdorff bounds.',ha='center',va='bottom',fontsize=8.5)
    fig.savefig(args.output/'proxy76_static_comparison.png',dpi=170)
    fig.savefig(args.output/'proxy76_static_comparison.pdf')
    metadata['inputs_unchanged'] = before == inputs._snapshot(paths)
    metadata['input_sha256'] = before
    (args.output/'figure_metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')


if __name__=='__main__':
    main()
