# Bend the right arm of a planner clip onto our grasp point.
#
# Everything here is humanoid-swarm-sim's own code:
#   gen_planner_motion.py         makes the stance (planner mode 0, idle)
#   coop_arm_reach.apply_reach    bends the arms onto a world target, ramped
#   make_squat_demo_motion        ARMS / hand_xyz for the chains and palm FK
#
# The only thing added is where to aim: the grasp the vision pipeline picked,
# carried from the plan frame (origin = torso_link at robot_base_pose z 0.98)
# into the clip's world.
#
#   python grasp/reach_clip.py IN.pkl OUT_DIR NAME PALM_X PALM_Y PALM_Z
import os
import sys

import joblib
import numpy as np
from scipy.spatial.transform import Rotation as R

SWARM = "/home/sehoon/Documents/GitHub/humanoid-swarm-sim"
SONIC = "/home/sehoon/Projects/GR00T-WholeBodyControl"
sys.path[:0] = [os.path.join(SWARM, "common"), os.path.join(SWARM, "demos", "squat"),
                os.path.join(SONIC, "gear_sonic", "data_process")]

from foot_height import load_urdf                      # noqa: E402
from coop_arm_reach import apply_reach                 # noqa: E402
from make_squat_demo_motion import ARMS, hand_xyz      # noqa: E402

PLAN_BASE_Z = 0.98          # robots/g1_right_arm.yaml: robot_base_pose
IN, OUT_DIR, NAME = sys.argv[1], sys.argv[2], sys.argv[3]
PALM = np.array([float(v) for v in sys.argv[4:7]])     # in the plan frame


def torso_world(joints, order, dof, rt, rq, f):
    idx = {nm: i for i, nm in enumerate(order)}
    T = np.eye(4)
    T[:3, :3] = R.from_quat(rq[f]).as_matrix()
    T[:3, 3] = rt[f]
    for jn in [j for j in ARMS["R"] if "waist" in j]:
        j = joints[jn]
        L = np.eye(4)
        L[:3, :3] = R.from_euler("xyz", j["rpy"]).as_matrix()
        L[:3, 3] = j["xyz"]
        T = T @ L
        if j["axis"] is not None:
            Rr = np.eye(4)
            Rr[:3, :3] = R.from_rotvec(j["axis"] * dof[f, idx[jn]]).as_matrix()
            T = T @ Rr
    return T


urdf = load_urdf()
joints, order, _ = urdf
src = joblib.load(IN)
key = list(src)[0]
m = dict(src[key])
dof = np.asarray(m["dof"], dtype=np.float32)
rt = np.asarray(m["root_trans_offset"], dtype=np.float32)
rq = np.asarray(m["root_rot"], dtype=np.float32)       # xyzw
n = len(dof)
bottom, ramp_start = n - 1, n // 4

T = torso_world(joints, order, dof, rt, rq, bottom)
palms = hand_xyz(dof, rt, rq, urdf)
tgt_R = T[:3, :3] @ (PALM - np.array([0.0, 0.0, PLAN_BASE_Z])) + T[:3, 3]
targets = {"R": tgt_R, "L": palms["L"][bottom]}        # left keeps its pose

print(f"[reach] {key}: {n} frames, ramp {ramp_start} -> {bottom}")
print(f"[reach] torso  {np.round(T[:3, 3], 3)}")
print(f"[reach] palm R {np.round(palms['R'][bottom], 3)} -> {np.round(tgt_R, 3)}")
rep = apply_reach(urdf, m, rt, rq, bottom, ramp_start, targets)
for side, r in rep.items():
    print(f"[reach] {side} residual {r['residual'] * 1000:.1f} mm")
    for jn, (a, b) in r["moved"].items():
        print(f"[reach]   {jn}: {a:+.1f} -> {b:+.1f} deg")

os.makedirs(OUT_DIR, exist_ok=True)
dst = os.path.join(OUT_DIR, f"{NAME}.pkl")
joblib.dump({NAME: m}, dst)
print(f"[reach] wrote {dst}")
# Where the table and the box have to stand, in the clip's world, so the box
# is the one the plan grasped: same torso-relative offsets the Isaac cell uses.
box_torso = np.array([0.3502, -0.2299, 0.032])
box_world = T[:3, :3] @ box_torso + T[:3, 3]
print(f"[reach] BOX GOES AT {np.round(box_world, 4)}")
