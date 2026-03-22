import pandas as pd
import matplotlib.pyplot as plt
import glob

# --- Load finger data (chunk00-09) ---
finger_files = sorted(glob.glob('/home/lucas/RUKA/logs/temps_20251225_171048_chunk0*.csv'))
finger_files = [f for f in finger_files if 'corrected' not in f]
finger_data = pd.concat([pd.read_csv(f) for f in finger_files], ignore_index=True)

finger_groups = {
    'Thumb': ['m12_temp_c', 'm13_temp_c', 'm14_temp_c'],
    'Index': ['m01_temp_c', 'm02_temp_c', 'm03_temp_c'],
    'Middle': ['m10_temp_c', 'm11_temp_c'],
    'Ring': ['m04_temp_c', 'm05_temp_c', 'm06_temp_c'],
    'Pinky': ['m07_temp_c', 'm08_temp_c', 'm09_temp_c'],
}
for name, cols in finger_groups.items():
    finger_data[name] = finger_data[cols].mean(axis=1)

# --- Load wrist data (chunk00-10) ---
wrist_files = sorted(glob.glob('/home/lucas/RUKA/logs/wrist_run_5h_chunk*.csv'))
wrist_data = pd.concat([pd.read_csv(f) for f in wrist_files], ignore_index=True)

# --- Plot ---
window = 50
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(28, 10), dpi=120)

# Left: finger temps
elapsed_finger = finger_data['elapsed_sec'] / 3600.0
for name in finger_groups:
    smoothed = finger_data[name].rolling(window=window, center=True, min_periods=1).mean()
    ax1.plot(elapsed_finger, smoothed, label=name, linewidth=1.5)
ax1.set_xlabel('Elapsed time (hours)', fontsize=26)
ax1.set_ylabel('Motor temperature (C)', fontsize=26)
ax1.set_title('Finger motor temperatures vs time', fontsize=28)
ax1.legend(fontsize=22)
ax1.tick_params(axis='both', labelsize=22)
ax1.grid(True, alpha=0.3)

# Right: wrist temps
elapsed_wrist = wrist_data['elapsed_sec'] / 3600.0
flex_ext = wrist_data['m15_temp_c'].rolling(window=window, center=True, min_periods=1).mean()
radial_ulnar = wrist_data['m16_temp_c'].rolling(window=window, center=True, min_periods=1).mean()
ax2.plot(elapsed_wrist, flex_ext, label='Flex/Ext', linewidth=1.5, linestyle='-')
ax2.plot(elapsed_wrist, radial_ulnar, label='Radial/Ulnar', linewidth=1.5, linestyle='--')
ax2.set_xlabel('Elapsed time (hours)', fontsize=26)
ax2.set_ylabel('Motor temperature (C)', fontsize=26)
ax2.set_title('Wrist motor temperatures vs time', fontsize=28)
ax2.legend(fontsize=22)
ax2.tick_params(axis='both', labelsize=22)
ax2.grid(True, alpha=0.3)

plt.tight_layout()
out = '/home/lucas/RUKA/logs/combined_finger_wrist_temps.png'
plt.savefig(out, dpi=120)
print(f"Saved: {out}")
