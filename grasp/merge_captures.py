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
# A flag's value is not a capture directory: walk the list and skip the token
# after any --flag, or "--object obj_plan" is read as a directory called
# obj_plan.
CAPS = []
_skip = False
for _a in sys.argv[2:]:
    if _skip:
        _skip = False
        continue
    if _a.startswith("--"):
        _skip = True
        continue
    CAPS.append(_a)
OBJ = (sys.argv[sys.argv.index("--object") + 1] if "--object" in sys.argv
       else "obj_plan")


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

if "--balance" in sys.argv and len(obj_xyz) > 1:
    # Concatenating raw views weights them by sampling density, not by how
    # much of the object each one shows. The wrist camera sits 0.13 m from the
    # box and the head 0.95 m, so it contributes twelve points for every one
    # of the head's: measured, 17263 against 1423, and the merged cloud came
    # out 92.7% top face. GraspGen conditions on that cloud, and a cloud that
    # is almost all one flat plane is a plate -- it proposed 28 grasps and not
    # one of them came from above, against 22 of 55 from the head view alone.
    #
    # Taking the same number from each view makes the merge about geometry
    # instead. No constant: the count is the smallest view's own.
    n = min(len(a) for a in obj_xyz)
    rng = np.random.default_rng(0)
    picks = [rng.choice(len(a), n, replace=False) for a in obj_xyz]
    obj_xyz = [a[i] for a, i in zip(obj_xyz, picks)]
    obj_rgb = [c[i] for c, i in zip(obj_rgb, picks)]
    print(f"[merge] balanced: {n} object points taken from each of "
          f"{len(obj_xyz)} views")

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
