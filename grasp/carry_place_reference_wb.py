"""Reference for carrying the held object to the crate and releasing it, appended to a
pick+rise reference.
    python carry_place_reference_wb.py RISE.pkl RISE_hands.npy CARRY.pkl PLACE.npz OUT_NAME
Same construction as build_place_reference.py (read, not guessed): the carry clip's right arm
is replaced by the last arm pose of the reference it follows (the hand keeps what it holds),
the seams are blended, then the place reach (reach_from_pose_wb --place) and a release.
Differences from build_place_reference: the fingers keep the reference's LAST hand row (the
actual wrap, rise_reference_wb does the same) instead of HAND_CLOSED; the release follows
GraspGenX's PickAndDropInBinTask (end2end/tasks.py:608-625): the arm holds still above the
drop for hold_frames (60 f @60 = 1 s -> 30 f @30) before the fingers ramp closed->open over
close_frames (20 f @60 -> 10 f @30), then hold so the object falls; and PLACE.npz's own
open_from is honoured only as the start of that settle.
"""
import os
import sys

import joblib
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_reach_reference import mujoco_names, qpos_to_motion_lib, HAND_OPEN, HAND_NAMES  # noqa: E402
from build_place_reference import clip_qpos, blend_in, npz_qpos, RIGHT_ARM  # noqa: E402

FPS = 30
rise_pkl, rise_hands, carry_pkl, place_npz, name = sys.argv[1:6]
names = mujoco_names()
arm = [7 + names.index(n) for n in RIGHT_ARM]
base = clip_qpos(rise_pkl)
hands_base = np.load(rise_hands)
wrap = hands_base[-1]

carry = clip_qpos(carry_pkl)
if os.environ.get("FREEZE_ARM", "1") == "1":
    carry[:, arm] = base[-1, arm]                  # the arm stays where the pick left it (PLANNER_FROZEN_UPPER)
else:
    print("[carry] FREEZE_ARM=0: the right arm follows the planner's own walking swing; only the fingers keep the wrap")
carry = blend_in(base[-1], carry)

place, open_from = npz_qpos(place_npz, names)
# PLACE_SLOW: stretch the place motion (GraspGenX moves to the bin over 6 s, tasks.py:325-335; ours was 3 s).
# PLACE_BLEND: frames over which the carry pose fades into the place plan (default 45 as build_place_reference).
# v13 lost the hammer here: the wrist rose 0.65 -> 1.05 m at 0.8 m/s in the 45-frame blend, contact spiked 270 -> 408 N.
_slow = float(os.environ.get("PLACE_SLOW", "1"))
if _slow > 1:
    idx = np.linspace(0, len(place) - 1, int(round(len(place) * _slow)))
    place = np.stack([np.interp(idx, np.arange(len(place)), place[:, j]) for j in range(place.shape[1])], axis=1).astype(np.float32)
    place[:, 3:7] /= np.linalg.norm(place[:, 3:7], axis=1, keepdims=True)
place = blend_in(carry[-1], place, n=int(os.environ.get("PLACE_BLEND", "45")))
settle = int(os.environ.get("RELEASE_SETTLE", "30"))   # GraspGenX hold_frames 60 @60 fps
ramp = int(os.environ.get("RELEASE_RAMP", "10"))       # GraspGenX close_frames 20 @60 fps
after = int(os.environ.get("RELEASE_HOLD", "45"))
n_place = len(place)
tail = np.repeat(place[-1:], settle + ramp + after, axis=0)
qpos = np.concatenate([base, carry, place, tail]).astype(np.float32)

hands = np.tile(wrap, (len(qpos), 1)).astype(np.float32)
hands[:len(hands_base)] = hands_base
t0 = len(base) + len(carry) + n_place + settle            # release starts after the settle
right = np.array([n.startswith(("right", "R_")) for n in HAND_NAMES], bool)
for k in range(ramp):
    a = (k + 1) / ramp
    h = wrap.copy(); h[right] = (1 - a) * wrap[right] + a * np.array(HAND_OPEN, np.float32)[right]
    hands[t0 + k] = h
h = wrap.copy(); h[right] = np.array(HAND_OPEN, np.float32)[right]
hands[t0 + ramp:] = h

out = os.path.join("results", "motion", f"{name}.pkl")
joblib.dump({name: qpos_to_motion_lib(qpos, FPS)}, out, compress=True)
np.save(os.path.join("results", "motion", f"{name}_hands.npy"), hands)
print(f"[carry] {len(base)} pick+rise + {len(carry)} carry + {n_place} place + {settle} settle + {ramp} open + {after} hold "
      f"= {len(qpos)} frames -> {out}")
print(f"[carry] carry starts at frame {len(base)}, place at {len(base) + len(carry)}, fingers open at {t0} ({t0 / FPS:.1f} s)")
