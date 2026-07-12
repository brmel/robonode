#!/usr/bin/env python3
"""Plot trapezoid vs S-curve CSV from profile_demo. Usage: plot_profiles.py out.csv [out.png]"""
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

csv = sys.argv[1]
out = sys.argv[2] if len(sys.argv) > 2 else "profiles.png"

df = pd.read_csv(csv).set_index("t")
fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
for ax, what, label in zip(axes, ["pos", "vel", "acc"], ["position [m]", "velocity [m/s]", "acceleration [m/s²]"]):
    ax.plot(df.index, df[f"trap_{what}"], label="trapezoid", lw=1.5)
    ax.plot(df.index, df[f"s_{what}"], label="S-curve (jerk-limited)", lw=1.5)
    ax.set_ylabel(label)
    ax.grid(alpha=0.3)
axes[0].legend()
axes[2].set_xlabel("t [s]")
axes[2].annotate("acceleration steps = infinite jerk", xy=(0.02, 0.8), xycoords="axes fraction", fontsize=9)
fig.suptitle("Trapezoidal vs jerk-limited S-curve — same v/a limits, j=10 m/s³")
fig.tight_layout()
fig.savefig(out, dpi=130)
print(f"wrote {out}")
