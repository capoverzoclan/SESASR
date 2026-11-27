import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


BASE_DIR = Path(__file__).resolve().parents[2]
path = BASE_DIR / "lab04_data" / "results" / "rosbag2_2026_11_27-11_57_05"

odom = pd.read_csv(path / "odom.csv")
ekf = pd.read_csv(path / "ekf.csv")
ekf_ext = pd.read_csv(path / "ekf_ext.csv")
ground_truth = pd.read_csv(path / "ground_truth.csv")

landmarks_map = {
    11: (-1.1, -1.1),
    12: (-1.1,  0.0),
    13: (-1.1,  1.1),
    21: ( 0.0, -1.1),
    22: ( 0.0,  0.0),
    23: ( 0.0,  1.1),
    31: ( 1.1, -1.1),
    32: ( 1.1,  0.0),
    33: ( 1.1,  1.1),
}

x_landmarks = [pos[0] for pos in landmarks_map.values()]
y_landmarks = [pos[1] for pos in landmarks_map.values()]

#-----------------
x_odom = odom["x"].to_numpy()
y_odom = odom["y"].to_numpy()

x_ekf = ekf["x"].to_numpy()
y_ekf = ekf["y"].to_numpy()

x_gt = ground_truth["x"].to_numpy()
y_gt = ground_truth["y"].to_numpy()

x_ekf_ext = ekf_ext["x"].to_numpy()
y_ekf_ext = ekf_ext["y"].to_numpy()

#------ Trajectory EKF
plt.figure(figsize=(8, 8))

plt.plot(x_odom, y_odom, color="blue",
         label="Odometry", linewidth=1.5)

plt.plot(x_ekf, y_ekf, color="green",
         label="EKF", linewidth=1.5)

plt.plot(x_gt, y_gt, color="red",
         label="Ground Truth", linewidth=1.5,
         linestyle="--")

plt.scatter(x_landmarks, y_landmarks,
            color="black", marker="s", s=70,
            label="Landmarks", zorder=5)

plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Robot Trajectory Comparison - EKF")

plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()

# #------- theta vs t
# t_odom = odom["bag_elapsed"].to_numpy()
# theta_odom = odom["yaw"].to_numpy()

# t_ekf = ekf["bag_elapsed"].to_numpy()
# theta_ekf = ekf["yaw"].to_numpy()

# t_gt = ground_truth["bag_elapsed"].to_numpy()
# theta_gt = ground_truth["yaw"].to_numpy()

#===========================================

t_odom = odom["time"].to_numpy()
theta_odom = odom["yaw"].to_numpy()

t_ekf = ekf["time"].to_numpy()
theta_ekf = ekf["yaw"].to_numpy()

t_gt = ground_truth["time"].to_numpy()
theta_gt = ground_truth["yaw"].to_numpy()

t_ekf_ext = ekf_ext["time"].to_numpy()
theta_ekf_ext = ekf_ext["yaw"].to_numpy()

t0 = min(t_odom[0], t_ekf[0], t_gt[0], t_ekf_ext[0])

#---------Orientation vs Time EFK

t_odom = t_odom - t0
t_ekf = t_ekf - t0
t_gt = t_gt - t0

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
plt.title("Robot Orientation Comparison EKF")

plt.grid(True)
plt.legend()
plt.tight_layout()

#------- X vs t EFK
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
plt.title("Robot X Position Comparison EKF")

plt.grid(True)
plt.legend()
plt.tight_layout()

#-------- Y vs t EFK
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
plt.title("Robot Y Position Comparison EKF")

plt.grid(True)
plt.legend()
plt.tight_layout()

#------ Trajectory EKF Extended
plt.figure(figsize=(8, 8))

plt.plot(x_odom, y_odom, color="blue",
         label="Odometry", linewidth=1.5)

plt.plot(x_ekf_ext, y_ekf_ext, color="green",
         label="EKF Extended", linewidth=1.5)

plt.plot(x_gt, y_gt, color="red",
         label="Ground Truth", linewidth=1.5,
         linestyle="--")

plt.scatter(x_landmarks, y_landmarks,
            color="black", marker="s", s=70,
            label="Landmarks", zorder=5)

plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Robot Trajectory Comparison - EKF Extended")

plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()

#------- Orientation vs time EKF Extended
t_ekf_ext = t_ekf_ext - t0

plt.figure(figsize=(10, 5))

plt.plot(t_odom, theta_odom, color="blue",
         label="Odometry", linewidth=1.5)

plt.plot(t_ekf_ext, theta_ekf_ext, color="green",
         label="EKF", linewidth=1.5)

plt.plot(t_gt, theta_gt, color="red",
         label="Ground Truth", linewidth=1.5,
         linestyle="--")

plt.xlabel("Time [s]")
plt.ylabel(r"$\theta$ [rad]")
plt.title("Robot Orientation Comparison EFK Extended")

plt.grid(True)
plt.legend()
plt.tight_layout()

#------- X vs t EFK Extended
plt.figure(figsize=(10, 5))

plt.plot(t_odom, x_odom, color="blue",
         label="Odometry", linewidth=1.5)

plt.plot(t_ekf_ext, x_ekf_ext, color="green",
         label="EKF", linewidth=1.5)

plt.plot(t_gt, x_gt, color="red",
         label="Ground Truth", linewidth=1.5,
         linestyle="--")

plt.xlabel("Time [s]")
plt.ylabel("X [m]")
plt.title("Robot X Position Comparison EKF Extended")

plt.grid(True)
plt.legend()
plt.tight_layout()

#-------- Y vs t EFK Extended
plt.figure(figsize=(10, 5))

plt.plot(t_odom, y_odom, color="blue",
         label="Odometry", linewidth=1.5)

plt.plot(t_ekf_ext, y_ekf_ext, color="green",
         label="EKF", linewidth=1.5)

plt.plot(t_gt, y_gt, color="red",
         label="Ground Truth", linewidth=1.5,
         linestyle="--")

plt.xlabel("Time [s]")
plt.ylabel("Y [m]")
plt.title("Robot Y Position Comparison EKF Extended")

plt.grid(True)
plt.legend()
plt.tight_layout()

plt.show()
