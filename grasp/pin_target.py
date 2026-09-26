"""Where the other hand pins the tool: the thick part, seen.

    python grasp/pin_target.py CAPTURE_DIR OUT.json [--object obj_lang --part obj_part]

The object minus its handle (find_part.py) is the head. Its top and its
centre are read off the pixels; the left palm presses there, flat, while the
right hand takes the handle -- so the tool cannot be kicked or slid by the
first touch, which is how every floor grasp so far was lost.
"""
import json
import os
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, "/home/sehoon/Projects/GraspGenX")
from graspgenx.utils.scene_loaders import depth_to_camera_xyz, transform_xyz  # noqa: E402

cap, out = sys.argv[1], sys.argv[2]
obj = sys.argv[sys.argv.index("--object") + 1] if "--object" in sys.argv else "obj_lang"
part = sys.argv[sys.argv.index("--part") + 1] if "--part" in sys.argv else "obj_part"
meta = json.load(open(os.path.join(cap, "meta_data.json")))
seg = np.asarray(Image.open(os.path.join(cap, "seg.png")), dtype=np.int32)
depth = np.load(os.path.join(cap, "depth.npy"))
xyz = transform_xyz(depth_to_camera_xyz(depth, np.asarray(meta["intrinsics"])), np.asarray(meta["camera_pose"]))
head = xyz[seg == meta["label_map"][obj]]
handle = xyz[seg == meta["label_map"][part]]
# the head is the object's points farther than 3 cm from every handle point
from scipy.spatial import cKDTree
d, _ = cKDTree(handle[:, :2]).query(head[:, :2])
head = head[d > 0.03]
c = head.mean(axis=0); top = float(np.percentile(head[:, 2], 95))
axis = np.linalg.svd(handle[:, :2] - handle[:, :2].mean(0), full_matrices=False)[2][0]
res = {"point": [float(c[0]), float(c[1]), top], "handle_axis": [float(axis[0]), float(axis[1])], "pixels": int(len(head))}
json.dump(res, open(out, "w"), indent=1)
print(f"[pin] head: {len(head)} px, centre {np.round(c[:2], 3)}, top z {top:.3f}; handle axis {np.round(axis, 2)}")
