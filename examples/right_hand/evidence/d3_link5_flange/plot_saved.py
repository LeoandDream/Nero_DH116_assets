"""Plot saved J6 data only; does not import a simulator or advance a state."""
from pathlib import Path
import gzip
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
with gzip.open(HERE / 'torque-timeline.jsonl.gz', 'rt') as stream:
    rows = [json.loads(line) for line in stream]
time = np.array([r['time_s'] for r in rows])
target = np.array([r['target_rad'] for r in rows])
q = np.array([r['q6_rad'] for r in rows])
velocity = np.array([r['q6_velocity_rad_s'] for r in rows])
motor = np.array([r['actuator_force_Nm'] for r in rows])
contact = np.array([r['pair_tau6_estimate_Nm'] for r in rows])

fig, axes = plt.subplots(4, 1, figsize=(9, 9), sharex=True, constrained_layout=True)
axes[0].step(time, target, where='post', label='Recorded target', color='#265caa')
axes[0].plot(time, q, label='Recorded post-step q6', color='#d16a17')
axes[0].set_ylabel('J6 position (rad)')
axes[0].legend(loc='upper right')
axes[1].plot(time, np.abs(q-target), color='#602b8a')
axes[1].axhline(.1, color='black', linestyle=':', label='Frozen 0.1 rad limit')
axes[1].set_ylabel('Tracking error (rad)')
axes[1].legend(loc='upper left')
axes[2].plot(time, velocity, color='#338359')
axes[2].axhline(0, color='black', linewidth=.7)
axes[2].set_ylabel('Post-step velocity\n(rad/s)')
axes[3].plot(time, motor, label='Recorded motor torque', color='#265caa')
axes[3].plot(time, contact, label='Estimated cached pair torque', color='#b43b3b')
axes[3].axhline(0, color='black', linewidth=.7)
axes[3].set_ylabel('J6 torque (N m)')
axes[3].legend(loc='upper right')
for axis in axes:
    axis.axvline(time[1314], color='#b43b3b', linestyle='--', linewidth=1)
    axis.axvline(time[1400], color='black', linestyle='--', linewidth=1)
    axis.grid(alpha=.2)
    axis.set_xlim(2.45, 2.81)
axes[3].set_xlabel('Recorded simulation time (s)')
fig.suptitle('Saved right-bowl trial: first contact at step 1315, stop at step 1401\n'
             'Contact torque uses RK4 cached poses/forces; positions and velocities are post-step', fontsize=11)
output = HERE / 'j6-timeline.png'
if output.exists():
    raise SystemExit('Do not overwrite an existing evidence figure')
fig.savefig(output, dpi=170)
plt.close(fig)
