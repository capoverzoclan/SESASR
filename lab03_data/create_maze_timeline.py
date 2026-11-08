#!/usr/bin/env python3
"""
Extract key milestone frames from bumpngo_final.mp4 and generate an 8-frame
experimental timeline showing the TurtleBot3 state progression through the maze.
SESASR 2026/2027 — Laboratory 03
"""

import os
import subprocess
import textwrap
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

def main():
    video_path = "/home/fvelilla/sesasr_course/lab_repo/SESASR/lab03_data/img/bumpngo_final.mp4"
    frames_dir = "/tmp/frames"
    os.makedirs(frames_dir, exist_ok=True)

    # 1. Extract frames at 1 fps if not already extracted
    if not os.path.exists(os.path.join(frames_dir, "frame_001.png")):
        print(f"Extracting frames from {video_path}...")
        cmd = [
            "ffmpeg", "-i", video_path, "-vf", "fps=1",
            os.path.join(frames_dir, "frame_%03d.png"), "-y"
        ]
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # 2. Key milestone frames, states, and descriptions
    frames_info = [
        {
            "file": os.path.join(frames_dir, "frame_002.png"),
            "t": "t = 2.0 s",
            "state": "FORWARD",
            "title": "(a) Initial Cruising",
            "desc": "Robot drives straight along the boundary cabinet."
        },
        {
            "file": os.path.join(frames_dir, "frame_008.png"),
            "t": "t = 8.0 s",
            "state": "TURNING",
            "title": "(b) Obstacle Detection",
            "desc": "Front obstacle detected (d < 0.30 m); initiates turn."
        },
        {
            "file": os.path.join(frames_dir, "frame_013.png"),
            "t": "t = 13.0 s",
            "state": "FORWARD",
            "title": "(c) Maze Entry",
            "desc": "Heading aligned; enters cardboard corridor."
        },
        {
            "file": os.path.join(frames_dir, "frame_017.png"),
            "t": "t = 17.0 s",
            "state": "TURNING",
            "title": "(d) 90° Corner Turn",
            "desc": "Detects Yellow-X box; turns to clear inner corner."
        },
        {
            "file": os.path.join(frames_dir, "frame_021.png"),
            "t": "t = 21.0 s",
            "state": "FORWARD",
            "title": "(e) Corridor Navigation",
            "desc": "Cruising between Blue-O and Blue-X boxes."
        },
        {
            "file": os.path.join(frames_dir, "frame_026.png"),
            "t": "t = 26.0 s",
            "state": "TURNING",
            "title": "(f) Maze Exit Turn",
            "desc": "Reaches end of corridor; searches clear escape angle."
        },
        {
            "file": os.path.join(frames_dir, "frame_032.png"),
            "t": "t = 32.0 s",
            "state": "TURNING",
            "title": "(g) Boundary Avoidance",
            "desc": "Avoids outer obstacle and re-orients toward open area."
        },
        {
            "file": os.path.join(frames_dir, "frame_038.png"),
            "t": "t = 38.0 s",
            "state": "FORWARD",
            "title": "(h) Open Arena Cruise",
            "desc": "Successfully clears maze; resumes forward cruise."
        }
    ]

    # 3. Create 2x4 composite figure
    fig, axes = plt.subplots(2, 4, figsize=(15, 11), dpi=300)
    axes = axes.flatten()

    for i, (ax, info) in enumerate(zip(axes, frames_info)):
        img = mpimg.imread(info["file"])
        ax.imshow(img)
        ax.set_xticks([])
        ax.set_yticks([])

        for spine in ax.spines.values():
            spine.set_edgecolor('#222222')
            spine.set_linewidth(1.5)

        # State badge overlay on top-left of image
        is_fwd = (info["state"] == "FORWARD")
        badge_bg = '#1b5e20' if is_fwd else '#b71c1c'
        badge_text = f" {info['t']} | {info['state']} "

        ax.text(18, 48, badge_text, color='white', fontsize=10, fontweight='bold',
                bbox=dict(boxstyle='square,pad=0.35', facecolor=badge_bg, edgecolor='none', alpha=0.90))

        # Wrapped caption below image
        wrapped_desc = textwrap.fill(info["desc"], width=23)
        caption = f"{info['title']}\n{wrapped_desc}"
        ax.set_xlabel(caption, fontsize=10, fontweight='bold', labelpad=9, linespacing=1.2)
        ax.xaxis.label.set_color('#111111')

    plt.suptitle("Experimental Obstacle Avoidance Sequence in Arena Maze (TurtleBot3 — SESASR Lab 03)",
                 fontsize=14, fontweight='bold', y=0.98)

    plt.tight_layout(rect=[0.01, 0.02, 0.99, 0.96], h_pad=2.5, w_pad=2.0)

    out1 = "/home/fvelilla/sesasr_course/lab_repo/SESASR/lab03_data/img/maze_timeline.png"
    out2 = "/home/fvelilla/sesasr_course/lab_notes/report1/figures/fig_maze_timeline.png"
    plt.savefig(out1, dpi=300)
    plt.savefig(out2, dpi=300)
    print(f"Generated timeline saved to:\n  - {out1}\n  - {out2}")

if __name__ == "__main__":
    main()
