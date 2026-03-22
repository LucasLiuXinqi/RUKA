import pandas as pd
import matplotlib.pyplot as plt
import glob

# Load all chunks 00-10
files = sorted(glob.glob('/home/lucas/RUKA/logs/wrist_run_5h_chunk*.csv'))
print(f"Loading {len(files)} files: {[f.split('/')[-1] for f in files]}")

dfs = []
for f in files:
    dfs.append(pd.read_csv(f))
data = pd.concat(dfs, ignore_index=True)

# Smooth with rolling window
window = 50
elapsed_hours = data['elapsed_sec'] / 3600.0

fig, ax = plt.subplots(figsize=(21, 10.5), dpi=120)

flex_ext = data['m15_temp_c'].rolling(window=window, center=True, min_periods=1).mean()
radial_ulnar = data['m16_temp_c'].rolling(window=window, center=True, min_periods=1).mean()

ax.plot(elapsed_hours, flex_ext, label='Flex/Ext', linewidth=1.5, linestyle='-')
ax.plot(elapsed_hours, radial_ulnar, label='Radial/Ulnar', linewidth=1.5, linestyle='--')

ax.set_xlabel('Elapsed time (hours)', fontsize=20)
ax.set_ylabel('Motor temperature (C)', fontsize=20)
ax.set_title('Wrist motor temperatures vs time', fontsize=24)
ax.legend(fontsize=18)
ax.tick_params(axis='both', labelsize=16)
ax.grid(True, alpha=0.3)

plt.tight_layout()
out = '/home/lucas/RUKA/logs/wrist_run_5h_flex_ext_radial_ulnar_smoothed_new.png'
plt.savefig(out, dpi=120)
print(f"Saved: {out}")
