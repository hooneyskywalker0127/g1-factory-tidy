# How wide apart are G1's hands, and how high off the floor?
#
# The box it has to pick up gets built to these numbers, not the other way
# round: a two-handed press grip only works if the box is as wide as the hands
# are apart, and only if the grip point clears the floor by enough that the
# backs of the hands do not hit it first.
#
# Measured in the default pose with GRAVITY OFF. With it on, G1 has no balance
# controller here and is face down within a second -- the first run of this
# measured a robot lying on the floor (pelvis at 0.169 m, palms below zero).
# This is the robot's own geometry, not a reach study.
#
#   python map/measure_g1_hands.py
import os
import sys

from isaaclab.app import AppLauncher

app = AppLauncher(headless=True).app

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab_assets.robots.unitree import G1_MINIMAL_CFG  # noqa: E402

sim = SimulationContext(sim_utils.SimulationCfg(dt=1.0 / 120.0, device="cpu",
                                                gravity=(0.0, 0.0, 0.0)))
robot = Articulation(G1_MINIMAL_CFG.replace(prim_path="/World/G1"))
sim.reset()
for _ in range(20):
    sim.step()
robot.update(sim.get_physics_dt())

names = robot.data.body_names
pos = robot.data.body_pos_w[0].cpu().numpy()
print(f"[g1] bodies: {len(names)}")
want = ["pelvis", "torso_link", "head_link",
        "left_shoulder_roll_link", "right_shoulder_roll_link",
        "left_elbow_roll_link", "right_elbow_roll_link",
        "left_palm_link", "right_palm_link"]
p = {}
for n in want:
    if n in names:
        v = pos[names.index(n)]
        p[n] = v
        print(f"[g1] {n:<26} x {v[0]:+.3f}  y {v[1]:+.3f}  z {v[2]:.3f}")

if "left_palm_link" in p and "right_palm_link" in p:
    lp, rp = p["left_palm_link"], p["right_palm_link"]
    print(f"[g1] palm separation (y) {abs(lp[1] - rp[1]):.3f} m, "
          f"height {(lp[2] + rp[2]) / 2.0:.3f} m")
if "left_shoulder_roll_link" in p and "right_shoulder_roll_link" in p:
    ls, rs = p["left_shoulder_roll_link"], p["right_shoulder_roll_link"]
    print(f"[g1] shoulder separation (y) {abs(ls[1] - rs[1]):.3f} m")
print(f"[g1] pelvis height {p['pelvis'][2]:.3f} m")
sys.stdout.flush()
os._exit(0)
