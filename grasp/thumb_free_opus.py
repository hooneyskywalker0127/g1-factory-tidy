"""Why the Inspire thumb never closes, with nothing to touch.

Measured in hammer/v23 and v24 (render log, f450-f500): with the contact force still
0.00 N and the four finger proximals still at 0.000, R_thumb_intermediate is already at
-0.160 rad -- exactly the USD lower limit (-9.167 deg) -- and its master
R_thumb_proximal_pitch then sits pinned at 0.050 rad against a 0.500 target for the whole
hold.  No object is involved at f450, so the cause is inside the hand.

This reproduces just that: the render's articulation, gains, stiffen_mimic() and the
per-substep soft_mimic() loop, with the root fixed, the body joints held, and no object.
One config a run, chosen by the same env vars the render reads.  ~20 s instead of 20 min.

    MIMIC_FREQ=200 MIMIC_DAMPING=1.0 MIMIC_URDF_RATIO=1 SOFT_MIMIC=1 \
    HAND=inspire HAND_KP=40 HAND_KD=4.0 HAND_EFFORT=30 HAND_ARMATURE=0.001 \
    python grasp/thumb_free_opus.py
"""
import os
import sys

import numpy as np
from isaaclab.app import AppLauncher

app = AppLauncher(headless=True, enable_cameras=False).app

import torch  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab_assets.robots.unitree import G1_29DOF_CFG  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import build_reach_reference as brr  # noqa: E402

FPS = 30
SUB = int(os.environ.get("SUB", "33"))
WATCH = ["R_thumb_proximal_pitch_joint", "R_thumb_intermediate_joint", "R_thumb_distal_joint",
         "R_thumb_proximal_yaw_joint", "R_index_proximal_joint", "R_index_intermediate_joint"]

sim = SimulationContext(sim_utils.SimulationCfg(dt=1.0 / 1000.0, device="cpu"))
cfg = brr.robot_cfg(G1_29DOF_CFG.replace(prim_path="/World/G1"), sim_utils, ImplicitActuatorCfg)
if os.environ.get("HAND_KD"):                       # play_in_cell_opus.py:173-182, verbatim
    _hz = cfg.actuators["hands"]
    _rep = dict(damping=float(os.environ["HAND_KD"]),
                effort_limit=float(os.environ.get("HAND_EFFORT", "200")))
    if os.environ.get("HAND_ARMATURE"):
        _rep["armature"] = float(os.environ["HAND_ARMATURE"])
    cfg.actuators["hands"] = _hz.replace(**_rep)
    print(f"[free] hand drive: kp {_hz.stiffness} kd {_rep['damping']} effort {_rep['effort_limit']}", flush=True)

# --- the hand's simulated velocity limit: the asset says 0.5 rad/s -------------
# Measured in free air (grasp/thumb_free_opus.py): robot.data.joint_velocity_limits prints
# 0.500 rad/s on every Inspire joint and joint_vel saturates exactly there -- +0.50 on the
# six masters, +0.67/+0.33 on thumb_intermediate/distal (= 0.5 x the URDF's 1.334/0.667
# through the mimic gearing). SUB=1 saturates at the same 0.50, so it is a real limit, not
# a substep artifact. A full proximal close (1.47 rad) therefore needs >= 2.94 s and the
# thumb's 0.5 rad >= 1.0 s at best; hammer v24 took f460->f840 = 12.7 s against contact
# while the grasp window is only f460->f760 = 10 s.
# robot_cfg() already asks for 10.0 rad/s -- but through `velocity_limit`, which Isaac Lab
# discards for implicit actuators. actuators/actuator_pd.py:81-91, ImplicitActuator.__init__:
#   "Previously, although this value was specified, it was not getting used by implicit
#    actuators. ... we continue to not use it. ... please use 'velocity_limit_sim' instead."
#   -> it sets cfg.velocity_limit = None.
# articulation.py:1773 writes only actuator.velocity_limit_sim to PhysX, and with
# cfg.velocity_limit_sim None that resolves to the USD prim's value (actuator_base_cfg.py:91-97,
# actuator_base.py:183). So the hand has closed at 0.5 rad/s in every attempt so far.
# HAND_VEL sets velocity_limit_sim. The manufacturer's own number is 5.0 rad/s: all 12 Inspire
# joints in GraspGenX ext/gripper_descriptions/.../inspire_hand/gripper_spherical_dof.urdf
# carry <limit ... velocity="5.0"/> (lines 131-422).
if os.environ.get("HAND_VEL"):
    _hv = cfg.actuators["hands"]
    cfg.actuators["hands"] = _hv.replace(velocity_limit=None,
                                         velocity_limit_sim=float(os.environ["HAND_VEL"]))
    print(f"[hand] velocity_limit_sim {os.environ['HAND_VEL']} rad/s "
          f"(asset 0.5, manufacturer URDF 5.0; cfg velocity_limit {_hv.velocity_limit} is discarded)")
cfg.spawn = cfg.spawn.replace(articulation_props=sim_utils.ArticulationRootPropertiesCfg(
    enabled_self_collisions=False, solver_position_iteration_count=int(os.environ.get("SOLVER_IT", "100")),
    solver_velocity_iteration_count=int(os.environ.get("SOLVER_VIT", "50")), fix_root_link=True))
robot = Articulation(cfg)
stage = sim_utils.SimulationContext.instance().stage
brr.stiffen_mimic(stage)

if os.environ.get("MIMIC_URDF_RATIO") == "1":        # play_in_cell_opus.py:435-472, verbatim
    _URDF_MULT = {"thumb_intermediate_joint": 1.334, "thumb_distal_joint": 0.667}
    brr._MIMIC[:] = [(a, b, _URDF_MULT.get(a.split("_", 1)[1], r)) for a, b, r in brr._MIMIC]
    from pxr import Usd as _Usd
    for _prim in _Usd.PrimRange(stage.GetPrimAtPath("/World/G1")):
        _k = next((k for k in _URDF_MULT if _prim.GetName().endswith(k)), None)
        if _k is None:
            continue
        for _a in _prim.GetAttributes():
            if _a.GetName().startswith("physxMimicJoint:") and _a.GetName().endswith(":gearing"):
                _a.Set(-_URDF_MULT[_k])
    print(f"[free] MIMIC_URDF_RATIO on: thumb gearings -1.334 / -0.667", flush=True)

sim.reset()
robot.update(0.0)
hand_ids = [robot.find_joints([n])[0][0] for n in brr.HAND_NAMES]
watch_ids = [robot.find_joints([n])[0][0] for n in WATCH]
body_ids = [j for j in range(robot.num_joints) if j not in hand_ids]
lim = robot.data.joint_limits[0]
print(f"[free] SUB={SUB} substeps/frame, dt={sim.get_physics_dt()*1000:.3f} ms  "
      f"vel_limit {float(robot.data.joint_velocity_limits[0, watch_ids[0]]):.3f} rad/s", flush=True)
print("[free] watched joint limits (rad): " + ", ".join(
    f"{n.replace('R_thumb_','T.').replace('R_index_','I.').replace('_joint','')} "
    f"{float(lim[i,0]):+.3f}..{float(lim[i,1]):+.3f}" for n, i in zip(WATCH, watch_ids)), flush=True)

tgt = robot.data.default_joint_pos.clone()
zero = torch.zeros_like(tgt)
OPEN = torch.tensor([brr.HAND_OPEN], dtype=torch.float32)
CLOSED = torch.tensor([brr.HAND_CLOSED], dtype=torch.float32)
for f in range(90):
    tgt[0, hand_ids] = (OPEN if f < 15 else CLOSED)[0]
    for _ in range(SUB):
        robot.update(sim.get_physics_dt())
        brr.soft_mimic(robot, tgt)
        robot.set_joint_position_target(tgt)
        robot.write_joint_state_to_sim(tgt[:, body_ids], zero[:, body_ids], joint_ids=body_ids)
        robot.write_data_to_sim()
        sim.step()
    robot.update(sim.get_physics_dt())
    if f % 5 == 0 or f in (15, 16, 17, 18):
        q = robot.data.joint_pos[0, watch_ids].numpy()
        t = tgt[0, watch_ids].numpy()
        v = robot.data.joint_vel[0, watch_ids].numpy()
        print(f"[free] f{f:3d} " + "  ".join(
            f"{n.replace('R_thumb_proximal_','T.').replace('R_thumb_','T.').replace('R_index_','I.').replace('_joint','')}"
            f" {q[i]:+.3f}/{t[i]:+.3f}v{v[i]:+.2f}" for i, n in enumerate(WATCH)), flush=True)
print("[free] DONE", flush=True)
os._exit(0)
