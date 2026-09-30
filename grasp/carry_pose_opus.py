"""Move the right arm and waist to a carrying pose while standing, holding what the hand holds.
    python carry_pose_opus.py BASE.pkl BASE_hands.npy OUT_NAME [N=60] [HOLD=30]
The pose is the planner's own objectCarrying (mode 21) arm, averaged over its walking frames
(GR00T planner_onnx.md:145 "Walking with hands reaching out"; measured on a mode-21 clip:
[-0.33,-0.43,0.27,0.50,-0.06,-0.05,-0.25]), waist 0.1. v9/v10 carried with the pick's last arm
(shoulder_roll -1.03, elbow -0.18, waist 0.40) and SONIC walked <30% of the reference, backwards or
sideways. Linear joint interpolation over N frames, then HOLD frames so the end is static (for a
DUMP_STATE that the next plan starts from). Fingers keep the base's last hand row (the wrap).
"""
import os
import sys

import joblib
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_reach_reference import mujoco_names, qpos_to_motion_lib  # noqa: E402
from build_place_reference import clip_qpos, RIGHT_ARM  # noqa: E402

CARRY_ARM = [float(v) for v in os.environ.get("CARRY_ARM", "-0.33,-0.43,0.27,0.50,-0.06,-0.05,-0.25").split(",")]
CARRY_WAIST = [float(v) for v in os.environ.get("CARRY_WAIST", "0.0,0.0,0.10").split(",")]   # yaw, roll, pitch
base_pkl, base_hands, name = sys.argv[1:4]
N = int(sys.argv[4]) if len(sys.argv) > 4 else 60
HOLD = int(sys.argv[5]) if len(sys.argv) > 5 else 30
names = mujoco_names()
arm = [7 + names.index(n) for n in RIGHT_ARM]
waist = [7 + names.index(f"waist_{a}_joint") for a in ("yaw", "roll", "pitch")]
base = clip_qpos(base_pkl); hands = np.load(base_hands)
last = base[-1]
seg = np.repeat(last[None], N + HOLD, axis=0)
for k in range(N):
    a = (k + 1) / N
    seg[k, arm] = (1 - a) * last[arm] + a * np.array(CARRY_ARM)
    seg[k, waist] = (1 - a) * last[waist] + a * np.array(CARRY_WAIST)
seg[N:, arm] = CARRY_ARM; seg[N:, waist] = CARRY_WAIST
q = np.concatenate([base, seg]).astype(np.float32)
h = np.concatenate([hands, np.tile(hands[-1], (N + HOLD, 1))]).astype(np.float32)
joblib.dump({name: qpos_to_motion_lib(q, 30)}, f"results/motion/{name}.pkl", compress=True)
np.save(f"results/motion/{name}_hands.npy", h)
print(f"[carry-pose] {len(base)} base + {N} move + {HOLD} hold = {len(q)} frames; arm {np.round(last[arm], 2)} -> {CARRY_ARM}, "
      f"waist {np.round(last[waist], 2)} -> {CARRY_WAIST}")
