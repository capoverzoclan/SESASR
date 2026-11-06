#!/usr/bin/env python3
"""
Plot robot trajectory from /odom and accumulated LiDAR points from /scan
recorded in ROS 2 rosbag (.db3).
SESASR 2026/2027 - Laboratory 03
"""

import sqlite3
import struct
import numpy as np
import matplotlib.pyplot as plt

def main():
    bag_path = '/home/fvelilla/sesasr_course/lab_repo/SESASR/lab03_data/lab03_final_result_0.db3'
    output_png = '/home/fvelilla/sesasr_course/lab_notes/report1/figures/fig_trajectory_comparison.png'
    output_png2 = '/home/fvelilla/sesasr_course/lab_notes/report1/figures/fig_odometry_laser_map.png'

    print(f"Reading rosbag from: {bag_path}")
    con = sqlite3.connect(bag_path)
    cur = con.cursor()

    # 1. Extract Odometry
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=2 ORDER BY timestamp ASC")
    odom_rows = cur.fetchall()

    odom_t = []
    odom_x = []
    odom_y = []
    odom_yaw = []

    for ts, raw in odom_rows:
        x, y, z = struct.unpack_from('<ddd', raw, 44)
        qx, qy, qz, qw = struct.unpack_from('<dddd', raw, 44 + 24)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        odom_t.append(ts)
        odom_x.append(x)
        odom_y.append(y)
        odom_yaw.append(yaw)

    odom_t = np.array(odom_t)
    odom_x = np.array(odom_x)
    odom_y = np.array(odom_y)
    odom_yaw = np.unwrap(np.array(odom_yaw))

    # 2. Extract Laser Scans
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=1 ORDER BY timestamp ASC")
    scan_rows = cur.fetchall()

    all_pts_x = []
    all_pts_y = []

    for idx in range(0, len(scan_rows), 1):
        ts, raw = scan_rows[idx]
        rx = np.interp(ts, odom_t, odom_x)
        ry = np.interp(ts, odom_t, odom_y)
        ryaw = np.interp(ts, odom_t, odom_yaw)

        angle_min, angle_max, angle_inc, time_inc, scan_time, r_min, r_max = struct.unpack_from('<7f', raw, 28)
        n_ranges = struct.unpack_from('<I', raw, 56)[0]
        ranges = np.frombuffer(raw, dtype=np.float32, count=n_ranges, offset=60)

        angles = angle_min + np.arange(n_ranges) * angle_inc
        valid = np.isfinite(ranges) & (ranges > 0.12) & (ranges < 3.5)

        r_v = ranges[valid]
        a_v = angles[valid]

        lidar_x_base = r_v * np.cos(a_v) - 0.032
        lidar_y_base = r_v * np.sin(a_v)

        gx = rx + (lidar_x_base * np.cos(ryaw) - lidar_y_base * np.sin(ryaw))
        gy = ry + (lidar_x_base * np.sin(ryaw) + lidar_y_base * np.cos(ryaw))

        all_pts_x.extend(gx)
        all_pts_y.extend(gy)

    all_pts_x = np.array(all_pts_x)
    all_pts_y = np.array(all_pts_y)

    print(f"Total points projected: {len(all_pts_x)}")

    # 3. Plotting
    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
    fig, ax = plt.subplots(figsize=(9, 7), dpi=300)

    # Plot obstacle points
    ax.scatter(all_pts_x, all_pts_y, s=2.0, c='#2b5c8f', alpha=0.22, edgecolors='none',
               label='LiDAR scan points (walls/obstacles)')

    # Plot odometry trajectory
    ax.plot(odom_x, odom_y, color='#d9381e', linewidth=2.8, label='Robot trajectory (Odometry)', zorder=5)

    # Orientation arrows along path
    step_arrow = max(1, len(odom_x) // 10)
    for i in range(0, len(odom_x), step_arrow):
        dx = 0.22 * np.cos(odom_yaw[i])
        dy = 0.22 * np.sin(odom_yaw[i])
        ax.arrow(odom_x[i], odom_y[i], dx, dy, head_width=0.08, head_length=0.10,
                 fc='#8e24aa', ec='#8e24aa', linewidth=1.4, zorder=6)

    # Start and End markers
    ax.plot(odom_x[0], odom_y[0], marker='o', markersize=10, color='#2e7d32',
            markeredgecolor='white', markeredgewidth=1.8, label=f'Start ({odom_x[0]:.2f}, {odom_y[0]:.2f}) m', zorder=7)
    ax.plot(odom_x[-1], odom_y[-1], marker='s', markersize=10, color='#c2185b',
            markeredgecolor='white', markeredgewidth=1.8, label=f'End ({odom_x[-1]:.2f}, {odom_y[-1]:.2f}) m', zorder=7)

    # --- AXIS LABELS CONFIGURATION ---
    ax.set_xlabel('Global X Position [m]', fontsize=13, fontweight='bold', labelpad=8)
    ax.set_ylabel('Global Y Position [m]', fontsize=13, fontweight='bold', labelpad=8)
    ax.tick_params(axis='both', which='major', labelsize=11)

    # Title
    ax.set_title('Reconstructed Odometry Path & Perceived 2D Environment\n(PIC4SeR Arena — Laboratory 03)',
                 fontsize=14, fontweight='bold', pad=12)

    # --- LEGEND CONFIGURATION ---
    # fontsize: controls text size
    # markerscale: scales marker dots/symbols in the legend so small points are clearly visible
    # framealpha: background transparency
    ax.legend(loc='upper right', fontsize=11, markerscale=3.0, framealpha=0.92, edgecolor='#cccccc')

    ax.grid(True, linestyle='--', alpha=0.6)
    ax.set_aspect('equal', 'box')

    plt.tight_layout()
    plt.savefig(output_png, dpi=300)
    plt.savefig(output_png2, dpi=300)
    print(f"Figures saved to:\n  - {output_png}\n  - {output_png2}")

if __name__ == '__main__':
    main()
