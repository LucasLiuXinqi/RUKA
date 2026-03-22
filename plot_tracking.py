import numpy as np
import matplotlib.pyplot as plt

data = np.load('/home/lucas/RUKA/Visual/tracking_results_old.npz', allow_pickle=True)

logged_cmd = data['logged_cmd']
logged_dir = data['logged_dir']
dip = data['dip']

# Split into half-sweeps: each contiguous forward or reverse segment
sweeps = []
current = {'cmd': [logged_cmd[0]], 'dip': [dip[0]]}
for i in range(1, len(logged_cmd)):
    if logged_dir[i] != logged_dir[i-1]:
        sweeps.append(current)
        current = {'cmd': [logged_cmd[i]], 'dip': [dip[i]]}
    else:
        current['cmd'].append(logged_cmd[i])
        current['dip'].append(dip[i])
sweeps.append(current)

print(f"Found {len(sweeps)} sweeps")

fig, ax = plt.subplots(figsize=(14, 10), dpi=150)

colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']

for i, sweep in enumerate(sweeps):
    cmd = np.array(sweep['cmd'])
    d = np.array(sweep['dip'])
    ax.plot(cmd, d, '-+', color=colors[i % len(colors)],
            label=f'Sweep {i+1} (DIP)', linewidth=2, markersize=8)

ax.set_xlabel('Commanded Position', fontsize=26)
ax.set_ylabel('Angle (°)', fontsize=26)
ax.set_title('Angle vs Commanded Position (Each Sweep Colored)', fontsize=28)
ax.tick_params(axis='both', labelsize=22)
ax.legend(fontsize=22, loc='lower left', bbox_to_anchor=(0, -0.15), ncol=len(sweeps), frameon=True)
ax.grid(True, alpha=0.3)

plt.tight_layout()
out = '/home/lucas/RUKA/Visual/angle_vs_position_old_new.png'
plt.savefig(out, dpi=150, bbox_inches='tight')
print(f"Saved: {out}")
