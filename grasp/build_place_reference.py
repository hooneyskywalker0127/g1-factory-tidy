"""Pick reference + walk to the desk carrying the object + place it in the crate.

    python grasp/build_place_reference.py PICK.pkl PICK_hands.npy WALK2.pkl PLACE.npz OUT_NAME

PICK is the walk+kneel+reach+lift reference (build_reach_reference.py) and its
finger schedule. WALK2 is the planner clip from the kneel-and-lift pose to the
desk (walk_clip.py --from-clip); its right arm is replaced by the lift's last
arm pose so the hand keeps what it holds, fingers stay closed. PLACE is the
whole-body reach above the crate (reach_from_pose.py --place); the fingers
open there. Writes results/motion/OUT_NAME.pkl and OUT_NAME_hands.npy.
"""
import os
import sys

import joblib
import numpy as np
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_reach_reference import HAND_OPEN, HAND_CLOSED, mujoco_names, qpos_to_motion_lib  # noqa: E402

FPS = 30
RIGHT_ARM = ["right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
             "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"]


def clip_qpos(path):
    m = list(joblib.load(path).values())[0]
    return np.concatenate([np.asarray(m["root_trans_offset"]), np.asarray(m["root_rot"])[:, [3, 0, 1, 2]],
                           np.asarray(m["dof"])], axis=1).astype(np.float32)


def npz_qpos(path, names):
    d = np.load(path)
    q, jn = d["q"], [str(n) for n in d["joint_names"]]
    dof = q[:, [jn.index(n) for n in names]]
    base = q[:, [jn.index(f"base_j_{a}") for a in ("x", "y", "z")]]
    rpy = q[:, [jn.index(f"base_j_{a}") for a in ("xtheta", "ytheta", "ztheta")]]
    quat = R.from_euler("XYZ", rpy).as_quat()[:, [3, 0, 1, 2]]
    return np.concatenate([base, quat, dof], axis=1).astype(np.float32), int(d["open_from"])


def blend_in(prev, seq, n=45):
    for i in range(min(n, len(seq))):
        a = (i + 1) / (n + 1)
        seq[i, 0:3] = (1 - a) * prev[0:3] + a * seq[i, 0:3]
        seq[i, 7:] = (1 - a) * prev[7:] + a * seq[i, 7:]
        qa, qb = prev[3:7], seq[i, 3:7]
        if np.dot(qa, qb) < 0:
            qb = -qb
        q = (1 - a) * qa + a * qb
        seq[i, 3:7] = q / np.linalg.norm(q)
    return seq


def main():
    pick_pkl, pick_hands, walk2_pkl, place_npz, name = sys.argv[1:6]
    names = mujoco_names()
    arm = [7 + names.index(n) for n in RIGHT_ARM]
    if os.environ.get("BOTH_ARMS") == "1":               # a two-hand carry (the crate): the left arm stays too
        arm += [7 + names.index(n.replace("right_", "left_")) for n in RIGHT_ARM]
    pick = clip_qpos(pick_pkl)
    hands_pick = np.load(pick_hands)
    walk2 = clip_qpos(walk2_pkl)
    walk2[:, arm] = pick[-1, arm]                       # carry: the arm stays where the lift left it
    # RISE_SLOW (default 2): the planner stands up in ~1.4 s (pelvis 0.42 -> 0.79 m); the hammer held by finger
    # friction slid out during exactly that (260928/5지/hammer/v4). Stretch the frames in which the pelvis rises
    # by this factor -- a person stands up slowly with something in hand.
    rs = float(os.environ.get("RISE_SLOW", "2"))
    z = walk2[:, 2]; rising = np.nonzero(np.abs(np.diff(z)) > 0.002)[0]
    if rs > 1 and len(rising):
        a, b = int(rising[0]), int(rising[-1]) + 2
        n = int(round((b - a) * rs)); idx = np.linspace(a, b - 1, n)
        seg = np.stack([np.interp(idx, np.arange(a, b), walk2[a:b, j]) for j in range(walk2.shape[1])], axis=1)
        q0, q1 = walk2[a:b, 3:7], None                  # quaternions: nearest-frame instead of lerp (they barely turn while rising)
        seg[:, 3:7] = walk2[np.clip(np.round(idx).astype(int), a, b - 1), 3:7]
        walk2 = np.concatenate([walk2[:a], seg, walk2[b:]]).astype(np.float32)
        print(f"[place] stand-up frames {a}-{b} stretched x{rs:.1f} ({b - a} -> {n} frames)")
    walk2 = blend_in(pick[-1], walk2)
    place, open_from = npz_qpos(place_npz, names)
    place = blend_in(walk2[-1], place)
    hold = 30
    qpos = np.concatenate([pick, walk2, place, np.repeat(place[-1:], hold, axis=0)])
    n_closed = len(pick) + len(walk2) + open_from
    hands = np.concatenate([hands_pick, np.tile(HAND_CLOSED, (len(walk2) + len(place) + hold, 1))]).astype(np.float32)
    ramp = 20
    for k in range(ramp):
        a = (k + 1) / ramp
        hands[n_closed + k] = (1 - a) * np.array(HAND_CLOSED) + a * np.array(HAND_OPEN)
    hands[n_closed + ramp:] = HAND_OPEN
    out = os.path.join("results", "motion", f"{name}.pkl")
    joblib.dump({name: qpos_to_motion_lib(qpos, FPS)}, out, compress=True)
    np.save(os.path.join("results", "motion", f"{name}_hands.npy"), hands)
    print(f"[place] {len(pick)} pick + {len(walk2)} walk + {len(place)} place + {hold} hold = {len(qpos)} frames -> {out}")
    print(f"[place] fingers open at frame {n_closed} ({n_closed/FPS:.1f} s)")


if __name__ == "__main__":
    main()
