import numpy as np
import matplotlib.pyplot as plt

data = np.load('/home/lucas/RUKA/Visual/tracking_results_low_spring.npz', allow_pickle=True)

logged_cmd = data['logged_cmd']
logged_dir = data['logged_dir']
dip = data['dip']

# Split into half-sweeps
sweeps = []
start = 0
for i in range(1, len(logged_dir)):
    if logged_dir[i] != logged_dir[i-1]:
        sweeps.append((start, i))
        start = i
sweeps.append((start, len(logged_dir)))

# Plot first 20 half-sweeps
n_sweeps = 20
fig, ax = plt.subplots(figsize=(14, 7), dpi=150)

cmap = plt.cm.tab20
for i, (s, e) in enumerate(sweeps[:n_sweeps]):
    ax.plot(logged_cmd[s:e], dip[s:e], '-+', color=cmap(i / n_sweeps),
            label=f'Sweep {i+1} (DIP)', linewidth=2, markersize=8)

ax.set_xlabel('Commanded Position', fontsize=26)
ax.set_ylabel('Angle (°)', fontsize=26)
ax.set_title('Angle vs Commanded Position (Each Sweep Colored)', fontsize=28)
ax.tick_params(axis='both', labelsize=22)
ax.grid(True, alpha=0.3)

plt.tight_layout()
out = '/home/lucas/RUKA/Visual/angle_vs_position_low_spring_test.png'
plt.savefig(out, dpi=150, bbox_inches='tight')
print(f"Saved: {out}")
