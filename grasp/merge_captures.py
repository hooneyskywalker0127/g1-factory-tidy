# Two cameras, one observation.
#
# The head camera finds the object but barely sees its top face -- on the v15
# table 48 of 1222 object points were within 5 mm of the top, so GraspGen,
# which conditions on the observed cloud, never proposes a top-down grasp. The
# wrist camera sees the top but only once the hand is already over the work.
# Using both is what the eye-in-hand literature says to do, and what every
# system that ships does (ACT/ALOHA, DROID, Diffusion Policy).
#
# GraspGenX already reads a scene given as raw point clouds --
# scene_loaders.load_graspgenx_json_scene, the format scripts/demo_scene_pc.py
# calls "json". That is where the merged cloud goes; nothing is re-projected.
#
#   python grasp/merge_captures.py OUT_DIR CAP_A [CAP_B ...] [--object obj_plan]
import json
import os
import sys

import numpy as np
from PIL import Image

OUT_DIR = sys.argv[1]
_OBJ_AT = sys.argv.index("--object") if "--object" in sys.argv else None
OBJ = sys.argv[_OBJ_AT + 1] if _OBJ_AT else "obj_plan"
# --object's VALUE does not start with "--" either, so it has to be skipped by
# position, not by shape.
_SKIP = {_OBJ_AT, _OBJ_AT + 1} if _OBJ_AT else set()
CAPS = [a for i, a in enumerate(sys.argv)
        if i >= 2 and i not in _SKIP and not a.startswith("--")]


def unproject(cap):
    """World-frame object points and scene points from one M2T2 capture."""
    md = json.load(open(os.path.join(cap, "meta_data.json")))
    depth = np.load(os.path.join(cap, "depth.npy")).astype(np.float64)
    rgb = np.asarray(Image.open(os.path.join(cap, "rgb.png")))[..., :3]
    seg = np.asarray(Image.open(os.path.join(cap, "seg.png")), dtype=np.int32)
    K = np.asarray(md["intrinsics"], dtype=np.float64)
    T = np.asarray(md["camera_pose"], dtype=np.float64)
    if OBJ not in md["label_map"]:
        return None, None, None, None, md
    sid = int(md["label_map"][OBJ])

    v, u = np.nonzero(depth > 0)
    z = depth[v, u]
    cam = np.stack([(u - K[0, 2]) * z / K[0, 0],
                    (v - K[1, 2]) * z / K[1, 1], z], axis=1)
    world = (T[:3, :3] @ cam.T).T + T[:3, 3]
    col = rgb[v, u]
    is_obj = seg[v, u] == sid
    return world[is_obj], col[is_obj], world[~is_obj], col[~is_obj], md


obj_xyz, obj_rgb, scn_xyz, scn_rgb, meta = [], [], [], [], None
for cap in CAPS:
    ox, oc, sx, sc, md = unproject(cap)
    meta = meta or md
    name = os.path.basename(cap.rstrip("/"))
    if ox is None or len(ox) == 0:
        print(f"[merge] {name}: object not visible, skipped")
        continue
    print(f"[merge] {name}: {len(ox)} object points, {len(sx)} scene points, "
          f"object z {ox[:, 2].min():.3f}..{ox[:, 2].max():.3f}")
    obj_xyz.append(ox)
    obj_rgb.append(oc)
    scn_xyz.append(sx)
    scn_rgb.append(sc)

if not obj_xyz:
    print("[merge] no camera saw the object")
    sys.exit(2)

obj_xyz = np.concatenate(obj_xyz)
obj_rgb = np.concatenate(obj_rgb)
scn_xyz = np.concatenate(scn_xyz)
scn_rgb = np.concatenate(scn_rgb)
top = obj_xyz[:, 2].max()
print(f"[merge] merged object {len(obj_xyz)} points, "
      f"{100 * (obj_xyz[:, 2] > top - 0.015).mean():.1f}% within 15 mm of the top")

# Into the plan frame, the same transform capture_rgbd.py writes.
P = np.asarray(meta["plan_from_cell"], dtype=np.float64)
obj_plan = (P[:3, :3] @ obj_xyz.T).T + P[:3, 3]
scn_plan = (P[:3, :3] @ scn_xyz.T).T + P[:3, 3]
print(f"[merge] object centre in the plan frame "
      f"{np.round((obj_plan.min(0) + obj_plan.max(0)) / 2, 4)}")

os.makedirs(OUT_DIR, exist_ok=True)
dst = os.path.join(OUT_DIR, "merged.json")
json.dump({
    "object_info": {"pc": obj_plan.tolist(), "pc_color": obj_rgb.tolist()},
    "scene_info": {"full_pc": [scn_plan.tolist()],
                   "img_color": scn_rgb.tolist()},
}, open(dst, "w"))
print(f"[merge] wrote {dst}")
