"""Order grasp candidates the way a person picks a spot on a handle.

    python grasp/rank_handle.py CAPTURE_DIR REACH_ALL.npz [--label obj_part] [--out order.txt]

The handle is the labelled part (find_part.py). Its long axis and the head
end come from the pixels: the head is the end nearer the whole object's
centroid. A tool lying on its side rests on its bulk, so the handle is
lifted near the head and slopes to the floor at the free end; the grip a
hand can make on the floor is on the raised part, a quarter to a half of
the way from the head. Candidates are ordered: pinch point on the axis
(< 3 cm off), then nearest 35 % from the head, then reach error.
"""
import json
import os
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, "/home/sehoon/Projects/GraspGenX")
from graspgenx.utils.scene_loaders import depth_to_camera_xyz, transform_xyz  # noqa: E402

cap, npz = sys.argv[1], sys.argv[2]
label = sys.argv[sys.argv.index("--label") + 1] if "--label" in sys.argv else "obj_part"
out = sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv else os.path.join(os.path.dirname(npz), "order.txt")
target = float(os.environ.get("HANDLE_AT", "0.35"))
meta = json.load(open(os.path.join(cap, "meta_data.json")))
seg = np.asarray(Image.open(os.path.join(cap, "seg.png")), dtype=np.int32)
depth = np.load(os.path.join(cap, "depth.npy"))
xyz = transform_xyz(depth_to_camera_xyz(depth, np.asarray(meta["intrinsics"])), np.asarray(meta["camera_pose"]))
part = xyz[seg == meta["label_map"][label]]
whole = xyz[seg == meta["label_map"].get("obj_lang", label)]
c = part.mean(0); ax = np.linalg.svd(part[:, :2] - c[:2], full_matrices=False)[2][0]
t = (part[:, :2] - c[:2]) @ ax; t0, t1 = t.min(), t.max()
if (whole.mean(0)[:2] - c[:2]) @ ax > 0:
    ax, t0, t1 = -ax, -t1, -t0                      # +ax runs from the head to the free end
d = np.load(npz); W = d["grasps"]; n = int(d["n_go"]); err = d["err"][:, n - 1]
P2 = np.eye(4); P2[:3, 3] = [0.0415, -0.003, 0.0]
# The hand comes straight down onto the grasp with its fingers open. Any part of the open hand whose
# footprint lands on the object's bulk (the head, the jaw, the body -- what is labelled but is not the
# handle) sweeps it during the descent: measured on the hammer, the Inspire's open thumb landed on the
# head 9 cm beside the handle and tilted the hammer 16 deg before the fingers closed (5지/hammer/v6).
# Open-hand sample points in the palm frame (results/inspire_links_in_palm.json; Dex3 from its URDF).
if os.environ.get("HAND") == "inspire":
    OPEN_PTS = [[0.20, 0.0, 0.032], [0.20, 0.0, -0.03], [0.17, 0.0, 0.032], [0.17, 0.0, -0.03],
                [0.105, 0.093, 0.013], [0.095, 0.072, 0.012], [0.06, 0.05, 0.01]]
else:
    OPEN_PTS = [[0.12, 0.0, 0.03], [0.12, 0.0, -0.03], [0.09, 0.043, 0.0], [0.06, 0.03, 0.0]]
head = whole[np.linalg.norm(whole[:, None, :2] - part[None, :, :2], axis=2).min(1) > 0.01] if len(whole) > len(part) else np.zeros((0, 3))
CLEAR = float(os.environ.get("HEAD_CLEAR", "0.03"))
rows = []
for k in range(len(W)):
    pc = [0.15, 0.06, 0] if os.environ.get('HAND') == 'inspire' else [0.07, 0.043, 0]
    G = W[k] @ P2
    pinch = (G @ np.array(pc + [1]))[:3]
    tp = (pinch[:2] - c[:2]) @ ax; frac = (tp - t0) / max(t1 - t0, 1e-6)
    off = np.linalg.norm((pinch[:2] - c[:2]) - tp * ax)
    hit = 0.0
    if len(head):
        pts = np.array([(G @ np.array(p + [1]))[:3] for p in OPEN_PTS])
        dxy = np.linalg.norm(pts[:, None, :2] - head[None, :, :2], axis=2).min(1)
        below = pts[:, 2] < head[:, 2].max() + 0.01          # the point ends up level with the bulk
        hit = float(np.sum((dxy < CLEAR) & below))
    rows.append((k, off, frac, err[k], hit))
order = sorted(rows, key=lambda r: (r[1] > 0.03, r[4] > 0, abs(r[2] - target) if r[1] <= 0.03 else 9, r[3]))
print(f"[rank] open-hand footprint on the bulk ({len(head)} pts): {sum(r[4] > 0 for r in rows)} of {len(rows)} candidates flagged, ordered last")
with open(out, "w") as f:
    f.write(",".join(str(r[0]) for r in order))
print(f"[rank] handle {t1 - t0:.2f} m; {sum(r[1] <= 0.03 for r in rows)} of {len(rows)} candidates on the axis; first: "
      + ", ".join(f"#{r[0]}({r[2]*100:.0f}% from head, {r[3]*1000:.0f} mm)" for r in order[:6]))
print(f"[rank] wrote {out}")
