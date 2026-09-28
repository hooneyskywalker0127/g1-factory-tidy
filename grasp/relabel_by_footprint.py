"""Give a close-range capture the object label from a far look's footprint.

    python grasp/relabel_by_footprint.py NEAR_DIR CRATE.json [--label obj_lang]

The first look found the crate by language 1.4 m away (crate_target.py: centre, axes, half sizes, top).
A capture from the kneel is dense but the language mask on it is unreliable at that range (it grabbed a
floor strip, then only the far wall). The crate has not moved, so the far look's footprint labels the
near capture's points: every pixel whose 3D point lies inside the footprint (2 cm margin) and below
top + 2 cm becomes `label`. Nothing here is typed in by hand -- the footprint came from the words.
"""
import json, os, sys
import numpy as np
from PIL import Image
sys.path.insert(0, "/home/sehoon/Projects/GraspGenX")
from graspgenx.utils.scene_loaders import depth_to_camera_xyz, transform_xyz  # noqa: E402

near, cj_path = sys.argv[1], sys.argv[2]
label = sys.argv[sys.argv.index("--label") + 1] if "--label" in sys.argv else "obj_lang"
cj = json.load(open(cj_path)); c = np.array(cj["centre"][:2]); ax = np.array(cj["long_axis"]); side = np.array([-ax[1], ax[0]])
meta = json.load(open(os.path.join(near, "meta_data.json"))); depth = np.load(os.path.join(near, "depth.npy"))
xyz = transform_xyz(depth_to_camera_xyz(depth, np.asarray(meta["intrinsics"])), np.asarray(meta["camera_pose"]))
H, W = depth.shape; P = xyz.reshape(-1, 3); rel = P[:, :2] - c
inside = ((np.abs(rel @ ax) < cj["half_long"] + 0.02) & (np.abs(rel @ side) < cj["half_short"] + 0.02)
          & (P[:, 2] > 0.015) & (P[:, 2] < cj["top"] + 0.02) & np.isfinite(P).all(1)).reshape(H, W)
seg_path = os.path.join(near, "seg.png")
seg = np.asarray(Image.open(seg_path), dtype=np.int32).copy()
if not os.path.isfile(os.path.join(near, "seg_orig.png")):
    Image.fromarray(seg.astype(np.uint8)).save(os.path.join(near, "seg_orig.png"))
lid = meta["label_map"].get(label, max(meta["label_map"].values()) + 1); meta["label_map"][label] = lid
seg[seg == lid] = 0; seg[inside] = lid
Image.fromarray(seg.astype(np.uint8)).save(seg_path); json.dump(meta, open(os.path.join(near, "meta_data.json"), "w"), indent=1)
rgb = np.asarray(Image.open(os.path.join(near, "rgb.png")).convert("RGB")).copy(); rgb[inside] = (0.5 * rgb[inside] + 0.5 * np.array([0, 255, 200])).astype(np.uint8)
Image.fromarray(rgb).save(os.path.join(near, f"{label}_overlay.png"))
print(f"[footprint] {label}: {int(inside.sum())} px in the near capture inside the far look's crate footprint (top {cj['top']:.3f} m)")
