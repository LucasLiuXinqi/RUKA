import numpy as np
import matplotlib.pyplot as plt
from itertools import cycle

# load data
data = np.load("tracking_results_up_spring.npz")
pos_all = data["logged_cmd"]
dip_all = data["dip"]
pip_all = data["pip"]
dirs = data["logged_dir"]  # 0 = forward, 1 = reverse

# --- helper to add arrows along lines ---
def add_arrows(x, y, color, label=None, n_arrows=3):
    plt.plot(x, y, '-', color=color, label=label)
    step = max(1, len(x)//(n_arrows+1))
    for i in range(step, len(x), step):
        if i < len(x):
            dx, dy = x[i] - x[i-1], y[i] - y[i-1]
            plt.arrow(
                x[i-1], y[i-1], dx*0.8, dy*0.8,
                shape='full', lw=0, length_includes_head=True,
                head_width=3, head_length=5, color=color
            )

# --- split into sweeps ---
# Each time the direction flips, that marks a new sweep
split_points = np.where(np.diff(dirs) != 0)[0] + 1
segments = np.split(np.arange(len(dirs)), split_points)

# --- plot ---
colors = cycle(plt.cm.tab10.colors)  # nice repeating color palette
plt.figure(figsize=(7,5))

for seg_i, seg in enumerate(segments):
    c = next(colors)
    label_dip = f"Sweep {seg_i+1} (DIP)"
    label_pip = f"Sweep {seg_i+1} (PIP)"
    add_arrows(pos_all[seg], dip_all[seg], color=c, label=label_dip)
    add_arrows(pos_all[seg], pip_all[seg], color=c)

plt.xlabel("Commanded Position")
plt.ylabel("Angle (°)")
plt.title("Angle vs Commanded Position (Each Sweep Colored)")
plt.legend(
    loc="upper center",
    bbox_to_anchor=(0.5, -0.15),
    ncol=4,
    fontsize=9
)
plt.grid(True)
plt.tight_layout()
plt.savefig("angle_vs_position_up_spring.png", dpi=300)
print("Saved plot: angle_vs_position_up_spring.png")