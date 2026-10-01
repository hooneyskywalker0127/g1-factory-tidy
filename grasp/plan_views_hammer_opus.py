"""The two views no camera can take, for the floor-object chain: what the grasp model saw, and what cuRobo planned.

render_plan_views.py draws them from e2e_grasp_demo's trajectory.json; the floor chain has no such file -- the grasp
candidates are grasps_all.json (GraspGen-X, Inspire hand, in the cell frame) and the reach is cuRobo's whole-body
retargeting solution (reach_*_pick.npz: joint q per frame). So:
    compute (graspgenx env, cuRobo FK):  python plan_views_hammer_opus.py compute CAP GRASPS.json PICK.npz OUT.npz
    render  (any env with matplotlib):   python plan_views_hammer_opus.py render  OUT.npz OUT_PREFIX [--frames 300]
Same look as render_plan_views.py (dark 3D axes, cloud + approach axes, the chosen grasp in RGB).
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("HAND", "inspire")


def observed_cloud(cap, label="obj_lang"):
    from PIL import Image
    meta = json.load(open(os.path.join(cap, "meta_data.json")))
    K = np.asarray(meta["intrinsics"], float); cam = np.asarray(meta["camera_pose"], float)
    depth = np.load(os.path.join(cap, "depth.npy")); seg = np.array(Image.open(os.path.join(cap, "seg.png")))
    m = (seg == meta["label_map"][label]) & np.isfinite(depth) & (depth > 0)
    ys, xs = np.nonzero(m); z = depth[ys, xs]
    pts_cam = np.stack([(xs - K[0, 2]) * z / K[0, 0], (ys - K[1, 2]) * z / K[1, 1], z], axis=1)
    return pts_cam @ cam[:3, :3].T + cam[:3, 3]          # cell frame, where the grasps and the reach live


def compute(cap, grasps_json, pick_npz, out):
    import torch
    import reach_from_pose_opus as RP
    cloud = observed_cloud(cap)
    wrists, conf = RP.grasps_in_cell(grasps_json, cap)        # (N, 4, 4) wrist poses in the cell
    d = np.load(pick_npz); q = np.asarray(d["q"], np.float32); jn = [str(n) for n in d["joint_names"]]
    r, cfg = RP.build()
    frames = list(r.kinematics.tool_frames if hasattr(r.kinematics, "tool_frames") else cfg.tool_frames)
    wi = frames.index("right_wrist_yaw_link")
    path = []
    for i in range(len(q)):
        qt = torch.as_tensor(q[i:i + 1], dtype=torch.float32, device="cuda")
        ks = r.kinematics.compute_kinematics(RP.JointState.from_position(qt, joint_names=jn))
        pos = ks.tool_poses.position
        path.append(pos.reshape(-1, len(frames), 3)[0, wi].detach().cpu().numpy())
    path = np.asarray(path)
    np.savez(out, cloud=cloud, wrists=wrists, conf=conf, chosen=np.asarray(d["grasp"]), path=path,
             close_from=int(d["close_from"]), lift_from=int(d["lift_from"]), best=int(d["best"]))
    print(f"[views] cloud {len(cloud)} pts, {len(wrists)} grasps, path {len(path)} frames "
          f"(close at {int(d['close_from'])}, lift at {int(d['lift_from'])}), chosen #{int(d['best'])} -> {out}")


def axes_at(T, length):
    o = T[:3, 3]
    return [(o, o + T[:3, i] * length) for i in range(3)]


def setup(ax, cloud, title, r_min=0.16, zoom=1.6):
    c = cloud.mean(axis=0); r = max(r_min, float(np.abs(cloud - c).max()) * zoom)
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r * 0.4, c[2] + r * 1.6)
    ax.set_box_aspect((1, 1, 1)); ax.set_title(title, color="0.9", fontsize=11, pad=2); ax.set_facecolor("0.10")
    for a in (ax.xaxis, ax.yaxis, ax.zaxis):
        a.set_pane_color((0.10, 0.10, 0.11, 1.0)); a.line.set_color("0.3"); a.set_tick_params(colors="0.45", labelsize=6)
    ax.grid(color="0.22")


def grab(fig):
    fig.canvas.draw(); w, h = fig.canvas.get_width_height()
    return np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8).reshape(h, w, 4)[..., :3].copy()


def render(npz, prefix, n_frames=300, fps=30):
    import imageio.v2 as imageio
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    d = np.load(npz); cloud, wrists, chosen, path = d["cloud"], d["wrists"], d["chosen"], d["path"]
    close_from = int(d["close_from"]); T = len(path)
    fig = plt.figure(figsize=(9.6, 5.4), dpi=100, facecolor="0.10"); ax = fig.add_subplot(111, projection="3d")
    w = imageio.get_writer(f"{prefix}_cloud.mp4", fps=fps, quality=8)
    for i in range(n_frames):
        ax.clear(); setup(ax, cloud, f"what the grasp model saw: {len(cloud)} pts, {len(wrists)} grasp candidates")
        ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2], s=3, c="#7fb3ff", alpha=0.85, linewidths=0)
        for g in wrists:
            a, b = axes_at(g, 0.035)[1]
            ax.plot(*zip(a, b), color="#b0b0b0", lw=0.7, alpha=0.6)
        for (a, b), col in zip(axes_at(chosen, 0.06), ("#ff5a5a", "#5aff8f", "#5a9dff")):
            ax.plot(*zip(a, b), color=col, lw=2.4)
        ax.view_init(elev=28, azim=-60 + 40 * i / n_frames)
        w.append_data(grab(fig))
    w.close()
    w = imageio.get_writer(f"{prefix}_traj.mp4", fps=fps, quality=8)
    for i in range(n_frames):
        k = max(2, int((i + 1) / n_frames * T))
        ax.clear(); setup(ax, np.concatenate([cloud, path]), "planned wrist path: cuRobo whole-body reach from the measured kneel", zoom=0.75)
        ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2], s=2, c="#7fb3ff", alpha=0.5, linewidths=0)
        ax.plot(path[:k, 0], path[:k, 1], path[:k, 2], color="#ffd27f", lw=2.6)
        ax.scatter(path[k - 1:k, 0], path[k - 1:k, 1], path[k - 1:k, 2], s=40, c="#ffd27f")
        for (a, b), col in zip(axes_at(chosen, 0.06), ("#ff5a5a", "#5aff8f", "#5a9dff")):
            ax.plot(*zip(a, b), color=col, lw=2.0, alpha=0.9)
        if k >= close_from:
            ax.text2D(0.02, 0.92, "fingers close", transform=ax.transAxes, color="#ffd27f", fontsize=10)
        ax.view_init(elev=22, azim=-60 + 40 * i / n_frames)
        w.append_data(grab(fig))
    w.close()
    print(f"[views] wrote {prefix}_cloud.mp4 and {prefix}_traj.mp4 ({n_frames} frames @ {fps})")


if __name__ == "__main__":
    if sys.argv[1] == "compute":
        compute(*sys.argv[2:6])
    else:
        n = int(sys.argv[sys.argv.index("--frames") + 1]) if "--frames" in sys.argv else 300
        render(sys.argv[2], sys.argv[3], n)
