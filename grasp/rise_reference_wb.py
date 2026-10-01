"""PICK reference + the planner's own stand-up, so the stand-up is IN the video.

    python rise_reference.py PICK.pkl PICK_hands.npy CARRY.pkl OUT_NAME

Same construction as the original build_place_reference.py:54-79 (read, not guessed):
the carry clip's right arm is replaced by the pick's last arm pose so the hand
keeps what it holds, the frames in which the pelvis rises are stretched
RISE_SLOW times, and the segment is blended into the pick's last frame. The
difference from build_place_reference: no PLACE leg, the clip is cut just after
the pelvis stops rising, and the fingers hold the pick's LAST hand pose (the
wrap) instead of HAND_CLOSED -- the wrap is not HAND_CLOSED.
"""
import os, sys
import joblib, numpy as np
sys.path.insert(0, "grasp")
from build_reach_reference import mujoco_names, qpos_to_motion_lib
from build_place_reference import clip_qpos, blend_in, RIGHT_ARM

FPS = 30
pick_pkl, pick_hands, carry_pkl, name = sys.argv[1:5]
names = mujoco_names()
arm = [7 + names.index(n) for n in RIGHT_ARM]
pick = clip_qpos(pick_pkl)
hands_pick = np.load(pick_hands)
rise = clip_qpos(carry_pkl)
rise[:, arm] = pick[-1, arm]

rs = float(os.environ.get("RISE_SLOW", "2"))
z = rise[:, 2]; rising = np.nonzero(np.abs(np.diff(z)) > 0.002)[0]
a, b = int(rising[0]), int(rising[-1]) + 2
rise = rise[:b + int(os.environ.get("RISE_TAIL", "10"))]          # cut the walk that follows the stand-up
if rs > 1:
    n = int(round((b - a) * rs)); idx = np.linspace(a, b - 1, n)
    seg = np.stack([np.interp(idx, np.arange(a, b), rise[a:b, j]) for j in range(rise.shape[1])], axis=1)
    seg[:, 3:7] = rise[np.clip(np.round(idx).astype(int), a, b - 1), 3:7]
    rise = np.concatenate([rise[:a], seg, rise[b:]]).astype(np.float32)
    print(f"[rise] stand-up frames {a}-{b} stretched x{rs:.1f} ({b - a} -> {n} frames)")
rise = blend_in(pick[-1], rise)
hold = int(os.environ.get("RISE_HOLD", "30"))
qpos = np.concatenate([pick, rise, np.repeat(rise[-1:], hold, axis=0)])
hands = np.concatenate([hands_pick, np.tile(hands_pick[-1], (len(rise) + hold, 1))]).astype(np.float32)
out = os.path.join("results", "motion", f"{name}.pkl")
joblib.dump({name: qpos_to_motion_lib(qpos, FPS)}, out, compress=True)
np.save(os.path.join("results", "motion", f"{name}_hands.npy"), hands)
print(f"[rise] {len(pick)} pick + {len(rise)} rise + {hold} hold = {len(qpos)} frames -> {out}")
print(f"[rise] pelvis z {qpos[len(pick)-1,2]:.3f} -> {qpos[-1,2]:.3f}; fingers hold the wrap for {len(rise)+hold} frames")
