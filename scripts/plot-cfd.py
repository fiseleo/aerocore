"""Plot an AeroCore CFD field: python scripts/plot-cfd.py path/to/field.csv.

Requires numpy and matplotlib. This script only plots existing numerical output.
"""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("field", type=Path)
args = parser.parse_args()
data = np.genfromtxt(args.field, delimiter=",", names=True)
x, y = np.unique(data["x_m"]), np.unique(data["y_m"])
shape = (len(y), len(x))
solid = data["solid"].reshape(shape).astype(bool)
u = np.ma.array(data["u_mps"].reshape(shape), mask=solid)
v = np.ma.array(data["v_mps"].reshape(shape), mask=solid)
speed = np.sqrt(u * u + v * v)
pressure = np.ma.array(data["pressure_delta_pa"].reshape(shape), mask=solid)
fig, axes = plt.subplots(2, 1, figsize=(11, 7), layout="constrained")
fig.suptitle(f"AeroCore CFD / {args.field.parent.name}\n2D periodic channel, stationary cylinder; finite-time snapshot", fontsize=14)
for ax, field, label, cmap in zip(axes, (speed, pressure), ("Speed [m/s]", "Pressure perturbation [Pa]"), ("viridis", "RdBu_r")):
    ax.set_facecolor("#283343")
    options = {}
    if field is pressure:
        peak = max(abs(float(pressure.min())), abs(float(pressure.max())), 1e-20)
        options = {"vmin": -peak, "vmax": peak}
    color = ax.pcolormesh(x * 1000, y * 1000, field, cmap=cmap, shading="nearest", **options)
    ax.set(xlabel="x [mm] (periodic)", ylabel="y [mm]", aspect="equal")
    fig.colorbar(color, ax=ax, label=label, shrink=0.85)
axes[0].streamplot(x * 1000, y * 1000, u, v, density=1.0, color="#ffffffaa", linewidth=0.65, arrowsize=0.65)
output = args.field.with_name("flow.png")
fig.savefig(output, dpi=160)
print(output.resolve())
