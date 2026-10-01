"""Kneel clip + hold, ending in the pose the robot actually reached (DUMP_STATE json).
    python kneel_from_state_wb.py KNEEL.pkl STATE.json HOLD_FRAMES OUT_NAME
The walk/kneel frames are kept verbatim (the second run must repeat the first), then
HOLD_FRAMES of the last frame, and the very last frame is the measured root pose and
joints, so reach_from_pose plans from where the robot really is (it reads the last
frame: reach_from_pose_wb.py:126-128). The deployment perceives after arriving; the
chain's step 5 ("look again from the kneel") is the offline form of the same thing.
"""
import json
import os
import sys

import joblib
import numpy as np
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_reach_reference import mujoco_names, qpos_to_motion_lib  # noqa: E402
from build_place_reference import clip_qpos  # noqa: E402

kneel_pkl, state_json, hold_n, name = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]
q = clip_qpos(kneel_pkl)                                   # (N, 36): xyz, quat wxyz, 29 dof
st = json.load(open(state_json))
names = mujoco_names()
last = q[-1].copy()
last[:3] = st["root_pos"]
last[3:7] = st["root_quat_wxyz"]
last[7:] = [st["dof"][n] for n in names]
yaw_ref = R.from_quat(q[-1][[4, 5, 6, 3]]).as_euler("xyz", degrees=True)[2]
yaw_m = R.from_quat(last[[4, 5, 6, 3]]).as_euler("xyz", degrees=True)[2]
print(f"[kneel] planned end xy {np.round(q[-1][:2], 3)} yaw {yaw_ref:.1f}; measured xy {np.round(last[:2], 3)} "
      f"yaw {yaw_m:.1f}; xy error {np.linalg.norm(q[-1][:2] - last[:2]) * 1000:.0f} mm, "
      f"joint error max {np.abs(q[-1][7:] - last[7:]).max():.3f} rad")
out = np.concatenate([q, np.repeat(q[-1:], hold_n - 1, axis=0), last[None]]).astype(np.float32)
path = os.path.join("results", "motion", f"{name}.pkl")
joblib.dump({name: qpos_to_motion_lib(out, 30)}, path, compress=True)
print(f"[kneel] {len(q)} + {hold_n} frames -> {path}")
