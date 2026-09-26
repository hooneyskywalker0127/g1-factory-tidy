"""A crate on the floor, seen: where it is, which way it lies, where to stand to lift it.

    python grasp/crate_target.py CAPTURE_DIR --label obj_lang OUT.json

From the labelled pixels: centre, top, the long axis in the floor plane (PCA)
and the two half-extents. The lift takes the crate between both palms on its
two LONG faces (the narrow width, 0.39 m, between the hands), so the robot
stands at one short end, facing along the long axis, close enough that the
crate's middle is a forearm ahead. Nothing here is typed in; it is all read
off the picture.
"""
import json
import math
import os
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, "/home/sehoon/Projects/GraspGenX")
from graspgenx.utils.scene_loaders import depth_to_camera_xyz, transform_xyz  # noqa: E402

cap = sys.argv[1]
label = sys.argv[sys.argv.index("--label") + 1] if "--label" in sys.argv else "obj_lang"
out = sys.argv[-1]
meta = json.load(open(os.path.join(cap, "meta_data.json")))
seg = np.asarray(Image.open(os.path.join(cap, "seg.png")), dtype=np.int32)
depth = np.load(os.path.join(cap, "depth.npy"))
xyz = transform_xyz(depth_to_camera_xyz(depth, np.asarray(meta["intrinsics"])), np.asarray(meta["camera_pose"]))
P = xyz[seg == meta["label_map"][label]]
top = float(np.percentile(P[:, 2], 97))
# the rim: the highest band of points outlines the crate's footprint from above
rim = P[P[:, 2] > top - 0.03]
c = rim[:, :2].mean(axis=0)
ax = np.linalg.svd(rim[:, :2] - c, full_matrices=False)[2][0]
side = np.array([-ax[1], ax[0]])
t = (rim[:, :2] - c) @ ax; u = (rim[:, :2] - c) @ side
lo_t, hi_t = np.percentile(t, 2), np.percentile(t, 98); lo_u, hi_u = np.percentile(u, 2), np.percentile(u, 98)
centre = c + ax * (lo_t + hi_t) / 2 + side * (lo_u + hi_u) / 2
half_long, half_short = (hi_t - lo_t) / 2, (hi_u - lo_u) / 2
# the robot was here (camera pose) -- stand at the short end nearer to it
cam = np.asarray(meta["camera_pose"])[:2, 3]
sign = 1.0 if (cam - centre) @ ax > 0 else -1.0
face_ax = -sign * ax                                  # the robot faces along +face_ax, toward the crate
stand_back = float(os.environ.get("CRATE_STAND_BACK", "0.30"))   # feet this far behind the near short face
stand = centre - face_ax * (half_long + stand_back)
yaw = math.degrees(math.atan2(face_ax[1], face_ax[0]))
# where along the long faces the palms go: CRATE_GRIP_AT=0.5 is the middle, smaller is
# nearer the robot -- a squatting G1 reaches 0.4 m ahead with both hands, not 0.6
grip_at = float(os.environ.get("CRATE_GRIP_AT", "0.5"))
grip_c = centre + face_ax * (grip_at - 0.5) * 2 * half_long
res = {"label": label, "pixels": int(len(P)), "centre": [float(centre[0]), float(centre[1]), float(top / 2)],
       "top": top, "long_axis": [float(v) for v in face_ax], "half_long": float(half_long), "half_short": float(half_short),
       "stand": {"x": float(stand[0]), "y": float(stand[1]), "yaw_deg": float(yaw)},
       "grasp_faces": {"left": [float(v) for v in (grip_c + side * half_short * (1 if np.cross(face_ax, side) > 0 else -1))],
                       "right": [float(v) for v in (grip_c - side * half_short * (1 if np.cross(face_ax, side) > 0 else -1))]},
       "base_path": None, "grasp_index": -1, "confidence": 0.0, "reachable": 0, "total": 0}
json.dump(res, open(out, "w"), indent=1)
print(f"[crate] {len(P)} px: centre {np.round(centre, 3)}, top {top:.3f}, long {2*half_long:.2f} x short {2*half_short:.2f} m, "
      f"faces left {np.round(res['grasp_faces']['left'], 2)} right {np.round(res['grasp_faces']['right'], 2)}; "
      f"stand ({stand[0]:.3f}, {stand[1]:.3f}, {yaw:.1f} deg)")
