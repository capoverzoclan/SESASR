import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


path = Path("results/task12_sim")

odom = pd.read_csv(path / "odom.csv")
ekf = pd.read_csv(path / "ekf.csv")
ekf_ext = pd.read_csv(path / "ekf_ext.csv")
ground_truth = pd.read_csv(path / "ground_truth.csv")

#-----------------
x_odom = odom["x"].to_numpy()
y_odom = odom["y"].to_numpy()

x_ekf = ekf["x"].to_numpy()
y_ekf = ekf["y"].to_numpy()

x_gt = ground_truth["x"].to_numpy()
y_gt = ground_truth["y"].to_numpy()

#------ Trajectory
plt.figure(figsize=(8, 8))

plt.plot(x_odom, y_odom, color="blue",
         label="Odometry", linewidth=1.5)

plt.plot(x_ekf, y_ekf, color="green",
         label="EKF", linewidth=1.5)

plt.plot(x_gt, y_gt, color="red",
         label="Ground Truth", linewidth=1.5,
         linestyle="--")

plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Robot Trajectory Comparison")

plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()

#------- theta vs t
t_odom = odom["bag_elapsed"].to_numpy()
theta_odom = odom["yaw"].to_numpy()

t_ekf = ekf["bag_elapsed"].to_numpy()
theta_ekf = ekf["yaw"].to_numpy()

t_gt = ground_truth["bag_elapsed"].to_numpy()
theta_gt = ground_truth["yaw"].to_numpy()

plt.figure(figsize=(10, 5))

plt.plot(t_odom, theta_odom, color="blue",
         label="Odometry", linewidth=1.5)

plt.plot(t_ekf, theta_ekf, color="green",
         label="EKF", linewidth=1.5)

plt.plot(t_gt, theta_gt, color="red",
         label="Ground Truth", linewidth=1.5,
         linestyle="--")

plt.xlabel("Time [s]")
plt.ylabel(r"$\theta$ [rad]")
plt.title("Robot Orientation Comparison")

plt.grid(True)
plt.legend()
plt.tight_layout()

#------- X vs t
plt.figure(figsize=(10, 5))

plt.plot(t_odom, x_odom, color="blue",
         label="Odometry", linewidth=1.5)

plt.plot(t_ekf, x_ekf, color="green",
         label="EKF", linewidth=1.5)

plt.plot(t_gt, x_gt, color="red",
         label="Ground Truth", linewidth=1.5,
         linestyle="--")

plt.xlabel("Time [s]")
plt.ylabel("X [m]")
plt.title("Robot X Position Comparison")

plt.grid(True)
plt.legend()
plt.tight_layout()

#-------- Y vs t
plt.figure(figsize=(10, 5))

plt.plot(t_odom, y_odom, color="blue",
         label="Odometry", linewidth=1.5)

plt.plot(t_ekf, y_ekf, color="green",
         label="EKF", linewidth=1.5)

plt.plot(t_gt, y_gt, color="red",
         label="Ground Truth", linewidth=1.5,
         linestyle="--")

plt.xlabel("Time [s]")
plt.ylabel("Y [m]")
plt.title("Robot Y Position Comparison")

plt.grid(True)
plt.legend()
plt.tight_layout()

plt.show()
