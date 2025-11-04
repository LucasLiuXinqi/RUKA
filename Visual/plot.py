#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

def main():
    ap = argparse.ArgumentParser(description="Plot DIP/PIP angles vs commanded position (back-and-forth sweep).")
    ap.add_argument("npz", type=str, help="Path to tracking_results_pingpong.npz (or tracking_results.npz)")
    ap.add_argument("--out", type=str, default=None, help="Optional output image path (.png/.pdf)")
    args = ap.parse_args()

    npz_path = Path(args.npz)
    data = np.load(npz_path, allow_pickle=True)

    # Determine which data format we have
    if all(k in data for k in ["logged_cmd", "dip", "pip"]):
        x = data["logged_cmd"]  # commanded position per step (includes back and forth)
        dip = data["dip"]
        pip = data["pip"]
    elif all(k in data for k in ["position", "dip", "pip"]):
        x = data["position"]
        dip = data["dip"]
        pip = data["pip"]
    else:
        raise ValueError("Unrecognized file format: expected keys ['logged_cmd','dip','pip'] or ['position','dip','pip'].")

    # --- Plot ---
    plt.figure(figsize=(8, 5))
    plt.plot(x, dip, "-o", linewidth=2, markersize=3, label="DIP")
    plt.plot(x, pip, "-o", linewidth=2, markersize=3, label="PIP")
    plt.xlabel("Commanded Position")
    plt.ylabel("Angle (°)")
    plt.title("Angle vs Commanded Position (Back-and-Forth Sweep)")
    plt.grid(True, alpha=0.3)
    plt.legend()

    # --- Save ---
    out_path = Path(args.out) if args.out else npz_path.with_suffix(".png")
    plt.tight_layout()
    plt.savefig(out_path, dpi=300)
    print(f"✅ Plot saved to: {out_path}")

if __name__ == "__main__":
    main()