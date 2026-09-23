"""The two views that are not camera views: what the robot saw, and what it planned.

play_in_cell.py renders the three that a camera could take -- the room, the
head, the wrist. Neither the point cloud the grasp was predicted from nor the
path the arm was told to follow exists in the cell as a thing to photograph, so
they are drawn instead, from the same two files the pick itself ran on:

  results/<capture>/          depth, segmentation and the camera pose, which is
                              all the grasp model ever saw of the object
  <run>/trajectory.json       GraspGenX's 80 grasp candidates, the one cuRobo
                              committed to, and every link's pose on every one
                              of the trajectory's frames

The grasps and the link poses are in the planner's own world, so the capture's
cloud is carried across the same way e2e_grasp_demo.py carries it: the
plan_from_cell the capture recorded, then the robot YAML's robot_base_pose
(see --base-z).

The frame counts line up with play_in_cell.py's video on purpose: pass --lead
with the number of walk frames and the first --lead frames hold still, so all
five videos can be watched side by side on the same clock.

    python grasp/render_plan_views.py CAPTURE_DIR TRAJECTORY_JSON OUT_PREFIX \\
        [--lead N] [--object LABEL] [--base-z Z]
"""

import json
import os
import sys

import imageio.v2 as imageio
import matplotlib
import numpy as np
from PIL import Image

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

CAP, TRAJ, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
LEAD = int(sys.argv[sys.argv.index("--lead") + 1]) if "--lead" in sys.argv else 0
LABEL = (sys.argv[sys.argv.index("--object") + 1]
         if "--object" in sys.argv else "obj_plan")
# e2e_grasp_demo.py moves a capture into the planner's world with
# `robot_base_T @ plan_from_cell`, so the grasps and the link poses in
# trajectory.json are in THAT world. plan_from_cell alone leaves the cloud a
# robot's chest-height below them. This is the z of the robot YAML's
# robot_base_pose -- 0.98 for g1_right_arm.yaml, where torso_link stands.
BASE_Z = (float(sys.argv[sys.argv.index("--base-z") + 1])
          if "--base-z" in sys.argv else 0.98)
FPS = 30
PALM = "right_hand_palm_link"


def observed_cloud():
    """Unproject the segmented object back out of the depth image.

    This is the cloud GraspGenX was conditioned on -- not the mesh, not the
    simulator's ground truth. If it looks thin, that IS what the model had.
    """
    meta = json.load(open(os.path.join(CAP, "meta_data.json")))
    K = np.asarray(meta["intrinsics"], dtype=np.float64)
    cam = np.asarray(meta["camera_pose"], dtype=np.float64)
    pfc = np.asarray(meta.get("plan_from_cell") or np.eye(4), dtype=np.float64)
    depth = np.load(os.path.join(CAP, "depth.npy"))
    seg = np.array(Image.open(os.path.join(CAP, "seg.png")))
    lid = meta["label_map"][LABEL]

    m = (seg == lid) & np.isfinite(depth) & (depth > 0)
    ys, xs = np.nonzero(m)
    z = depth[ys, xs]
    pts_cam = np.stack([(xs - K[0, 2]) * z / K[0, 0],
                        (ys - K[1, 2]) * z / K[1, 1], z], axis=1)
    pts_cell = pts_cam @ cam[:3, :3].T + cam[:3, 3]
    # into the planner's world, where the grasps and the trajectory already are
    pts_plan = pts_cell @ pfc[:3, :3].T + pfc[:3, 3]
    return pts_plan + np.array([0.0, 0.0, BASE_Z])


def axes_at(T, length):
    """Three short segments along a pose's own x, y, z."""
    o = T[:3, 3]
    return [(o, o + T[:3, i] * length) for i in range(3)]


def setup(ax, cloud, title):
    c = cloud.mean(axis=0)
    r = max(0.16, float(np.abs(cloud - c).max()) * 3.2)
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1))
    ax.set_title(title, color="0.9", fontsize=11, pad=2)
    ax.set_facecolor("0.10")
    for a in (ax.xaxis, ax.yaxis, ax.zaxis):
        a.set_pane_color((0.10, 0.10, 0.11, 1.0))
        a.line.set_color("0.3")
        a.set_tick_params(colors="0.45", labelsize=6)
    ax.grid(color="0.22")


def grab(fig):
    fig.canvas.draw()
    w, h = fig.canvas.get_width_height()
    buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
    return buf.reshape(h, w, 4)[..., :3].copy()


def main():
    cloud = observed_cloud()
    d = json.load(open(TRAJ))
    ann = d["annotations"]
    grasps = np.asarray(ann["all_grasps"], dtype=np.float64)
    chosen = np.asarray(ann["target_grasp_transform"], dtype=np.float64)
    frames = d["frames"]
    palm = np.array([[p["transform"] for p in f["parts"] if p["name"] == PALM][0]
                     for f in frames], dtype=np.float64)
    print(f"[views] cloud {cloud.shape[0]} pts, {len(grasps)} grasps, "
          f"{len(frames)} trajectory frames, {LEAD} lead frames")

    n = LEAD + len(frames)

    # --- view 4: what the grasp was predicted from -------------------------
    fig = plt.figure(figsize=(9.6, 5.4), dpi=100, facecolor="0.10")
    ax = fig.add_subplot(111, projection="3d")
    cloud_out, traj_out = f"{OUT}_cloud.mp4", f"{OUT}_traj.mp4"
    w = imageio.get_writer(cloud_out, fps=FPS, quality=8)
    for i in range(n):
        ax.clear()
        setup(ax, cloud, f"observed cloud + {len(grasps)} grasp candidates")
        ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2], s=3,
                   c="#7fb3ff", alpha=0.85, linewidths=0)
        # The candidates sit almost on top of each other -- they are 80
        # top-down grasps on one small box face, and that IS the answer the
        # model gave. Drawn long enough to see the spread rather than hidden
        # by it: only the approach axis, so the picture stays readable.
        for g in grasps:
            a, b = axes_at(g, 0.035)[1]          # grasp-frame y = approach
            ax.plot(*zip(a, b), color="#b0b0b0", lw=0.8, alpha=0.75)
        for (a, b), col in zip(axes_at(chosen, 0.05), ("#ff5a5a", "#5aff8f", "#5a9dff")):
            ax.plot(*zip(a, b), color=col, lw=2.4)
        # turn slowly so the shape of the cloud reads in a flat image
        ax.view_init(elev=22, azim=-60 + 360.0 * i / max(n, 1))
        w.append_data(grab(fig))
    w.close()
    print(f"[views] wrote {cloud_out}: {n} frames")

    # --- view 5: the path the arm was told to take -------------------------
    w = imageio.get_writer(traj_out, fps=FPS, quality=8)
    for i in range(n):
        k = max(0, i - LEAD)
        ax.clear()
        setup(ax, cloud, "planned palm path")
        ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2], s=3,
                   c="#7fb3ff", alpha=0.35, linewidths=0)
        for (a, b), col in zip(axes_at(chosen, 0.04), ("#ff5a5a", "#5aff8f", "#5a9dff")):
            ax.plot(*zip(a, b), color=col, lw=1.6, alpha=0.7)
        ax.plot(palm[:, 0, 3], palm[:, 1, 3], palm[:, 2, 3],
                color="0.4", lw=1.0, alpha=0.6)
        if k > 0:
            ax.plot(palm[:k, 0, 3], palm[:k, 1, 3], palm[:k, 2, 3],
                    color="#ffd24a", lw=2.2)
        for (a, b), col in zip(axes_at(palm[k], 0.045), ("#ff5a5a", "#5aff8f", "#5a9dff")):
            ax.plot(*zip(a, b), color=col, lw=2.4)
        ax.view_init(elev=22, azim=-60)
        w.append_data(grab(fig))
    w.close()
    print(f"[views] wrote {traj_out}: {n} frames")


if __name__ == "__main__":
    main()
