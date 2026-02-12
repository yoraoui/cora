"""
A file to visualize the different experimental data sets in the examples folder.

NOTE: the odometry visualization/calibration is unrefined and may not work well
for all data sets. It is likely a 80% solution that may need some more work if
you want to really inspect your odometry data/calibration.
"""

from py_factor_graph.io.pyfg_text import read_from_pyfg_text
from py_factor_graph.calibrations.range_measurement_calibration import (
    calibrate_range_measures,
)
from py_factor_graph.calibrations.odom_measurement_calibration import (
    calibrate_odom_measures,
)
import os
import matplotlib.pyplot as plt
import numpy as np




fg = read_from_pyfg_text("/home/raoui/cora-main/examples/data/plaza2.pyfg")


import matplotlib.pyplot as plt

def plot_static_trajectory(fg):

    odom = fg.odometry_trajectories

    xs = [T[0, 2] for T in odom[0]]
    ys = [T[1, 2] for T in odom[0]]

    plt.figure()
    plt.plot(xs, ys, 'b-', label="Odometry")
    plt.axis("equal")
    plt.grid()
    plt.legend()
    plt.title("Odometry Trajectory")
    plt.show()

def plot_static_trajectory(fg, show_gt=True):

    dim = fg.dimension

    if dim != 2:
        raise ValueError("This example handles 2D datasets only")


    # --- Odometry trajectory ---
    odom = fg.odometry_trajectories
    gt = fg.true_trajectories
    

    # If it is a list-of-lists
    traj = odom[0]   # robot 0 trajectory

    xs = [T[0, 2] for T in traj]
    ys = [T[1, 2] for T in traj]

    plt.figure()
    plt.plot(xs, ys, 'b-', linewidth=2)
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Robot 0 Odometry")
    plt.axis("equal")
    plt.grid()
    plt.show()

    # --- Ground truth (if available) ---
    
    """
    if show_gt and fg.true_trajectories is not None:
        gt = fg.true_trajectories
        xs_gt = [p[0] for p in gt]
        ys_gt = [p[1] for p in gt]
        plt.plot(xs_gt, ys_gt, 'r--', label="Ground Truth")

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Plaza2 Trajectory (Static Plot)")
    plt.axis("equal")
    plt.grid()
    plt.legend()"""
    plt.show()





plot_static_trajectory(fg)