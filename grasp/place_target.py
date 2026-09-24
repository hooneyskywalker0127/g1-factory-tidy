"""Where to put the thing, and where to stand to do it -- from what was seen.

    python grasp/place_target.py CAPTURE_DIR --label obj_crate --from X Y OUT.json

The crate is what find_by_text.py labelled in that capture. Its rim is the
top of its observed points, its centre the middle of them; the drop point is
above the centre by a hand's height. The stand is in front of the crate on
the side the robot comes from, at the reach the desk pick used (0.35 m,
where_to_stand.py's own measured reach), facing it. No number here is a place
in the cell; every one is read off the picture.
"""
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, "/home/sehoon/Projects/GraspGenX")
from graspgenx.utils.scene_loaders import depth_to_camera_xyz, transform_xyz  # noqa: E402
from PIL import Image  # noqa: E402

cap = sys.argv[1]
label = sys.argv[sys.argv.index("--label") + 1]
fx, fy = (float(sys.argv[sys.argv.index("--from") + k]) for k in (1, 2))
out = sys.argv[-1]
meta = json.load(open(os.path.join(cap, "meta_data.json")))
seg = np.asarray(Image.open(os.path.join(cap, "seg.png")), dtype=np.int32)
depth = np.load(os.path.join(cap, "depth.npy"))
xyz = transform_xyz(depth_to_camera_xyz(depth, np.asarray(meta["intrinsics"])), np.asarray(meta["camera_pose"]))
m = (seg == meta["label_map"][label]) & (depth > 0)
p = xyz[m]
lo, hi = np.percentile(p, 3, axis=0), np.percentile(p, 97, axis=0)
centre = (lo + hi) / 2.0
rim = float(hi[2])
drop = np.array([centre[0], centre[1], rim + 0.12])
f = np.array([centre[0] - fx, centre[1] - fy])
f /= np.linalg.norm(f)
stand = centre[:2] - 0.35 * f
yaw = math.degrees(math.atan2(f[1], f[0]))
d = {"label": label, "pixels": int(m.sum()), "centre": [float(v) for v in centre],
     "rim_z": rim, "extent": [float(v) for v in (hi - lo)],
     "drop": [float(v) for v in drop],
     "stand": {"x": float(stand[0]), "y": float(stand[1]), "yaw_deg": float(yaw)},
     "base_path": None, "grasp_index": -1, "confidence": 0.0, "reachable": 0, "total": 0}
json.dump(d, open(out, "w"), indent=1)
print(f"[place] '{label}' {int(m.sum())} px: centre {np.round(centre, 3)}, rim z {rim:.3f}, "
      f"extent {np.round(hi - lo, 2)}; drop {np.round(drop, 3)}; stand {np.round(stand, 3)} yaw {yaw:.1f}")
