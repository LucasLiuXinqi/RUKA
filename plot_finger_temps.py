import pandas as pd
import matplotlib.pyplot as plt
import glob

# Load all chunks 00-09
files = sorted(glob.glob('/home/lucas/RUKA/logs/temps_20251225_171048_chunk0*.csv'))
files = [f for f in files if 'corrected' not in f]
print(f"Loading {len(files)} files")

dfs = []
for f in files:
    dfs.append(pd.read_csv(f))
data = pd.concat(dfs, ignore_index=True)

# Finger groupings: mean of motor temps per finger
groups = {
    'Thumb': ['m12_temp_c', 'm13_temp_c', 'm14_temp_c'],
    'Index': ['m01_temp_c', 'm02_temp_c', 'm03_temp_c'],
    'Middle': ['m10_temp_c', 'm11_temp_c'],
    'Ring': ['m04_temp_c', 'm05_temp_c', 'm06_temp_c'],
    'Pinky': ['m07_temp_c', 'm08_temp_c', 'm09_temp_c'],
}

for name, cols in groups.items():
    data[name] = data[cols].mean(axis=1)

# Smooth with rolling window
window = 50
elapsed_hours = data['elapsed_sec'] / 3600.0

fig, ax = plt.subplots(figsize=(21, 10.5), dpi=120)

for name in groups:
    smoothed = data[name].rolling(window=window, center=True, min_periods=1).mean()
    ax.plot(elapsed_hours, smoothed, label=name, linewidth=1.5)

ax.set_xlabel('Elapsed time (hours)', fontsize=20)
ax.set_ylabel('Motor temperature (C)', fontsize=20)
ax.set_title('Finger-grouped motor temperatures vs time', fontsize=24)
ax.legend(fontsize=18)
ax.tick_params(axis='both', labelsize=16)
ax.grid(True, alpha=0.3)

plt.tight_layout()
out = '/home/lucas/RUKA/logs/temps_20251225_171048_finger_grouped_smoothed_new.png'
plt.savefig(out, dpi=120)
print(f"Saved: {out}")
