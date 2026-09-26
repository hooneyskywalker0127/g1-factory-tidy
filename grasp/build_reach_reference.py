"""Walk + kneel clip, then the whole-body reach cuRobo solved, as one SONIC reference.

    python grasp/build_reach_reference.py WALK.pkl REACH.npz OUT_NAME [--hold N]

Writes results/motion/OUT_NAME.pkl (motion_lib layout, 30 fps) and
results/motion/OUT_NAME_hands.npy: (frames, 14) finger targets in GR00T's
G1_HAND_JOINTS order, open until the reach's close frame, closed after --
the same schedule shape the evaluator's SONIC_HAND_SCHEDULE hook reads.
"""
import os
import sys

import joblib
import numpy as np
from scipy.spatial.transform import Rotation as R

SWARM = "/home/sehoon/Documents/GitHub/humanoid-swarm-sim"
sys.path.insert(0, os.path.join(SWARM, "common"))
from gen_planner_motion import qpos_to_motion_lib  # noqa: E402

FPS = 30
# Dex3-1 closed, as the grasp plan ramps them (end2end/tasks.py close_vals);
# the same values grasp/run_wbc_pick.sh read off the plan's last frame.
HAND_OPEN = [0, 0, 0, 0, 0, 0, 0, 0.4, 0.0, 0.4, 0.0, 0.0, 0.7243, 0.0]
HAND_CLOSED = [0, 0, 0, 0, 0, 0, 0, 1.5708, 1.7453, 1.5708, 1.7453, 0.0, -1.0472, -1.5]
# THUMB0: the right thumb's abduction, 0 in GraspGenX's description (copied from
# the unitree_g1 family). With 0 the thumb hangs 5.4 cm (+4 cm of flesh) under
# the palm and every palm-down pinch on a floor-level handle jammed it; swung
# sideways it can oppose the index from beside the object instead of from
# underneath (a person's key pinch on a stick on the floor).
if os.environ.get("THUMB0"):
    HAND_OPEN[11] = HAND_CLOSED[11] = float(os.environ["THUMB0"])


def mujoco_names():
    cwd = os.getcwd()
    os.chdir("/home/sehoon/Projects/GR00T-WholeBodyControl")
    try:
        from foot_height import load_urdf
        return load_urdf()[1]
    finally:
        os.chdir(cwd)


def main():
    walk_pkl, reach_npz, name = sys.argv[1:4]
    hold = int(sys.argv[sys.argv.index("--hold") + 1]) if "--hold" in sys.argv else 30
    w = list(joblib.load(walk_pkl).values())[0]
    dof_w = np.asarray(w["dof"], np.float32)
    root_w = np.asarray(w["root_trans_offset"], np.float32)
    quat_w = np.asarray(w["root_rot"], np.float32)                  # xyzw
    qpos_w = np.concatenate([root_w, quat_w[:, [3, 0, 1, 2]], dof_w], axis=1)

    d = np.load(reach_npz)
    q, jn = d["q"], [str(n) for n in d["joint_names"]]
    names = mujoco_names()
    col = [jn.index(n) for n in names]
    dof_r = q[:, col].astype(np.float32)
    base = q[:, [jn.index(f"base_j_{a}") for a in ("x", "y", "z")]]
    rpy = q[:, [jn.index(f"base_j_{a}") for a in ("xtheta", "ytheta", "ztheta")]]
    quat_r = R.from_euler("XYZ", rpy).as_quat()      # xyzw; cuRobo's base chain is Rx*Ry*Rz
    qpos_r = np.concatenate([base, quat_r[:, [3, 0, 1, 2]], dof_r], axis=1).astype(np.float32)

    # The solved reach starts from the clip's last pose, so the two meet; the
    # base still differs by what the solver moved it, which the tracker sees
    # as one step -- print it so it is a number, not a surprise.
    jump = np.abs(qpos_r[0] - qpos_w[-1])
    print(f"[ref] seam: base moves {jump[:3].max()*1000:.1f} mm, joints max "
          f"{np.degrees(jump[7:].max()):.1f} deg at {names[int(jump[7:].argmax())]}")
    # Cross-fade into the reach over its first BLEND frames, root by lerp,
    # rotation by slerp, joints by lerp -- the way walk_clip joins two plans
    # (planner_onnx.md, "Animation Blending"). The reach's own first frames
    # are a hold at the wrist's current pose, so nothing is lost to the fade;
    # without it the base stepped 37 mm and the shoulder 31 deg in one frame,
    # which is the jolt at 40 s that kicked the box.
    BLEND = int(os.environ.get("SEAM_BLEND", "0"))   # the solver is velocity-limited now; a joint-space fade swept the hand through the object
    for i in range(BLEND):
        a = (i + 1) / (BLEND + 1)
        qpos_r[i, 0:3] = (1 - a) * qpos_w[-1, 0:3] + a * qpos_r[i, 0:3]
        qpos_r[i, 7:] = (1 - a) * qpos_w[-1, 7:] + a * qpos_r[i, 7:]
        qa, qb = qpos_w[-1, 3:7], qpos_r[i, 3:7]
        if np.dot(qa, qb) < 0:
            qb = -qb
        q = (1 - a) * qa + a * qb
        qpos_r[i, 3:7] = q / np.linalg.norm(q)
    qpos = np.concatenate([qpos_w, qpos_r, np.repeat(qpos_r[-1:], hold, axis=0)], axis=0)
    out = os.path.join("results", "motion", f"{name}.pkl")
    joblib.dump({name: qpos_to_motion_lib(qpos, FPS)}, out, compress=True)
    print(f"[ref] {len(qpos_w)} walk+kneel + {len(qpos_r)} reach + {hold} hold "
          f"= {len(qpos)} frames at {FPS} fps -> {out}")
    print(f"[ref] pelvis z: kneel {qpos_w[-1, 2]:.3f}, reach {qpos_r[:, 2].min():.3f}"
          f"..{qpos_r[:, 2].max():.3f}")

    close_at = len(qpos_w) + int(d["close_from"])
    hands = np.tile(np.array(HAND_OPEN, np.float32), (len(qpos), 1))
    ramp = 20
    for k in range(ramp):
        a = (k + 1) / ramp
        if os.environ.get("CLOSE_ORDER", "together") == "thumb_first":
            # the close that held the hammer's handle (results/fable6/handle
            # #54): the thumb first over the first half of the ramp, the
            # fingers over the second -- the thumb closing after the fingers
            # jammed on every palm-down grasp (DIAGNOSIS.md, 02:40)
            th = np.array([0] * 11 + [1, 1, 1], bool)
            a_t = min(1.0, (k + 1) / (ramp // 2)); a_f = min(1.0, max(0.0, (k + 1 - ramp // 2) / (ramp // 2)))
            h = np.array(HAND_OPEN, np.float32)
            h[th] = (1 - a_t) * np.array(HAND_OPEN)[th] + a_t * np.array(HAND_CLOSED)[th]
            h[~th] = (1 - a_f) * np.array(HAND_OPEN)[~th] + a_f * np.array(HAND_CLOSED)[~th]
            hands[close_at + k] = h
        else:
            hands[close_at + k] = (1 - a) * np.array(HAND_OPEN) + a * np.array(HAND_CLOSED)
    hands[close_at + ramp:] = HAND_CLOSED
    if os.environ.get("NO_CLOSE"):               # a crate is carried between open palms
        hands[:] = HAND_OPEN
    hp = os.path.join("results", "motion", f"{name}_hands.npy")
    np.save(hp, hands)
    print(f"[ref] hands close at frame {close_at} ({close_at/FPS:.1f} s), lift from "
          f"{len(qpos_w) + int(d['lift_from'])} -> {hp}")


if __name__ == "__main__":
    main()
