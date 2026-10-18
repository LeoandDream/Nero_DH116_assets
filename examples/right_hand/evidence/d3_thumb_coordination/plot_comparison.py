"""Plot already recorded evidence only; no MuJoCo import or physical stepping."""
import argparse
import gzip
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
ROOT = next(p for p in HERE.parents if (p / 'mjcf/nero_dh116_right.xml').is_file())


def rows(path, count=None):
    with gzip.open(path, 'rt') as stream:
        return [json.loads(line) for _, line in zip(range(count or 10**9), stream)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    out = (ROOT / args.output).resolve()
    if out.exists():
        raise ValueError('Choose a new image path')
    cases = {'lambda0': rows(ROOT / 'examples/right_hand/evidence/hand-motion-state.jsonl.gz', 1000),
             'lambda1': rows(HERE / 'lambda1-state.jsonl.gz')}
    contacts = {'lambda0': rows(ROOT / 'examples/right_hand/evidence/hand-motion-contacts.jsonl.gz', 1000),
                'lambda1': rows(HERE / 'lambda1-contacts.jsonl.gz')}
    geometry = rows(HERE / 'geometry/all_steps.jsonl.gz')
    fig, axes = plt.subplots(3, 2, figsize=(13, 12), sharex=True)
    colors = {'lambda0': 'tab:blue', 'lambda1': 'tab:orange'}
    for case, states in cases.items():
        t = np.array([s['time_s'] for s in states]);color = colors[case]
        for idx, ax in enumerate(axes[0]):
            ax.plot(t, [s['robot_state_18'][7+idx] for s in states], color=color, label=case+' actual')
            ax.plot(t, [s['hand_target_rad'][idx] for s in states], color=color, ls='--', alpha=.65, label=case+' target')
            ax.set_title(('finger11','finger12')[idx]+' position');ax.set_ylabel('rad')
        force, overlap = [], []
        for row in contacts[case]:
            selected = [c for c in row['contacts'] if set(c['body_names']) == {'gripper0_base_link','gripper0_finger12_link'}]
            overlap.append(max([max(0.,-c['distance_m'])*1000 for c in selected] or [0.]))
            force.append(max([c['normal_force_N'] for c in selected] or [0.]))
        axes[1,0].plot(t, overlap, color=color,label=case)
        axes[1,1].plot(t, force, color=color,label=case)
        values = [r for r in geometry if r['case']==case]
        axes[2,0].step(t, [int(r['classification']=='strict_transverse_crossing') for r in values], where='post',color=color,label=case)
        for idx, style in [(0,'-'),(1,':')]:
            errors = [abs(s['active_tracking_error_rad'][idx]) if s['steady_window'] else np.nan for s in states]
            axes[2,1].plot(t, errors,color=color,ls=style,label=case+' '+('finger11','finger12')[idx])
    axes[1,0].axhline(2,color='crimson',ls='--',label='unchanged 2mm screening bound')
    axes[1,0].set(title='Reported RK4 A contact overlap (0 = no contact)',ylabel='mm')
    axes[1,1].set(title='Reported RK4 A normal force',ylabel='N')
    axes[2,0].set(title='Original STL surface crossing at recorded post qpos',ylabel='strict crossing = 1',yticks=[0,1])
    axes[2,1].axhline(.02,color='crimson',ls='--',label='unchanged .02rad steady bound')
    axes[2,1].set(title='Absolute tracking error in frozen steady windows only',ylabel='rad')
    for ax in axes.flat:
        ax.grid(alpha=.2);ax.legend(fontsize=7)
        for x in [.5,1.,1.5]:ax.axvline(x,color='.6',ls=':',alpha=.5)
    for ax in axes[-1]:ax.set_xlabel('recorded simulation time / s')
    fig.suptitle('Diagnostic-only thumb coordination: saved lambda0 vs one real lambda1 run\nA = base_link / finger12_link; post-step FK and RK4 contact cache are different nearby poses. Both candidates FAIL.')
    fig.tight_layout(rect=[0,0,1,.95]);out.parent.mkdir(parents=True,exist_ok=True);fig.savefig(out,dpi=150);plt.close(fig)


if __name__ == '__main__':
    main()
