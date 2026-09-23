# Can G1 stand in our cell on GR00T-WholeBodyControl's lower-body policy?
#
# Everything else in this repo pins the pelvis (fix_root_link=True) because
# nothing was holding the legs up. That pin is also why the arm alone has to
# reach everything: 36 vision grasp candidates, 5 reachable. Freeing the root
# needs a balance controller, and GR00T-WholeBodyControl ships one -- see
# grasp/wbc.py for the interface.
#
# This is the first step and checks only one thing: with the arm held still,
# does the robot stay up? Pelvis height and tilt are printed throughout.
#
#   python grasp/stand_wbc.py [seconds] [--video OUT.mp4]
import math
import os
import sys

from isaaclab.app import AppLauncher

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "map"))
sys.path.insert(0, os.path.join(REPO, "grasp"))

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else 10.0
# --height H commands a different base height. g1_gear_wbc.yaml stands at 0.74;
# humanoid-swarm-sim's planner modes squat to 0.55 / 0.35 / 0.20. Getting the
# pelvis down is half of why the arm cannot reach: at the default stance the
# palm bottoms out 0.106 m below the pelvis (map/measure_reach.py).
HEIGHT = (float(sys.argv[sys.argv.index("--height") + 1])
          if "--height" in sys.argv else None)
OUT = (sys.argv[sys.argv.index("--video") + 1] if "--video" in sys.argv else None)
STAND = (-1.30, -0.60, -90.0)

app = AppLauncher(headless=True, enable_cameras=OUT is not None).app

import numpy as np  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
import omni.usd  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sensors import Camera, CameraCfg  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab_assets.robots.unitree import G1_29DOF_CFG  # noqa: E402
from pxr import Sdf, UsdGeom  # noqa: E402
import torch  # noqa: E402

from wbc import ARM_JOINTS, ARM_KD, ARM_KP, LowerBodyWBC, LOWER_JOINTS, POLICY_JOINTS  # noqa: E402

wbc = LowerBodyWBC()
# The policy was deployed at simulation_dt 0.005 with control_decimation 4.
sim = SimulationContext(sim_utils.SimulationCfg(dt=wbc.dt, device="cpu"))
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
stage.DefinePrim("/World/Cell", "Xform").GetReferences().AddReference(
    Sdf.Reference(os.path.join(REPO, "map", "cell.usd")))
light = sim_utils.DomeLightCfg(intensity=900.0)
light.func("/World/light", light)
for _ in range(3):
    app.update()

yaw = math.radians(STAND[2])
cfg = G1_29DOF_CFG.replace(prim_path="/World/G1")
# The whole point: the root is free and the policy holds it up.
cfg.spawn = cfg.spawn.replace(
    articulation_props=sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=False, solver_position_iteration_count=12,
        solver_velocity_iteration_count=4, fix_root_link=False))
# PD gains straight from g1_gear_wbc.yaml for the 15 the policy drives, and
# the reference script's own arm gains for the rest.
# IsaacLab splits these 15 across three actuator groups and each group only
# accepts names it owns: "legs" is hips+knees, "feet" is the ankles, "waist"
# is the three waist joints.
KP = {n: float(v) for n, v in zip(LOWER_JOINTS, wbc.kps)}
KD = {n: float(v) for n, v in zip(LOWER_JOINTS, wbc.kds)}
GROUPS = {"legs": lambda n: "hip" in n or "knee" in n,
          "feet": lambda n: "ankle" in n,
          "waist": lambda n: "waist" in n}
for grp, belongs in GROUPS.items():
    names = [n for n in LOWER_JOINTS if belongs(n)]
    cfg.actuators[grp] = cfg.actuators[grp].replace(
        stiffness={n: KP[n] for n in names},
        damping={n: KD[n] for n in names})
cfg.actuators["arms"] = cfg.actuators["arms"].replace(
    stiffness=ARM_KP, damping=ARM_KD)
cfg.init_state = cfg.init_state.replace(
    pos=(STAND[0], STAND[1], 0.78),
    rot=(math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)),
    joint_pos={n: float(v) for n, v in zip(LOWER_JOINTS, wbc.default_angles)})
robot = Articulation(cfg)

cam = None
if OUT:
    import imageio.v2 as imageio
    UsdGeom.Xform.Define(stage, "/Render")
    cam = Camera(CameraCfg(
        prim_path="/Render/Cam", update_period=0.0, width=960, height=540,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(focal_length=22.0,
                                         clipping_range=(0.05, 40.0))))

sim.reset()

if cam is not None:
    tgt = np.array([STAND[0], STAND[1], 0.85])
    eye = tgt + np.array([-1.65, -2.05, 1.05])
    cam.set_world_poses_from_view(
        eyes=torch.tensor([eye], dtype=torch.float32, device=cam.device),
        targets=torch.tensor([tgt], dtype=torch.float32, device=cam.device))

palm_id = robot.find_bodies(["right_hand_palm_link"])[0][0]
pol_ids = [robot.find_joints([n])[0][0] for n in POLICY_JOINTS]
low_ids = pol_ids[:len(LOWER_JOINTS)]
arm_ids = pol_ids[len(LOWER_JOINTS):]

tgt_q = robot.data.default_joint_pos.clone()
for k, jid in enumerate(low_ids):
    tgt_q[0, jid] = float(wbc.default_angles[k])
for jid in arm_ids:
    tgt_q[0, jid] = 0.0
robot.write_joint_state_to_sim(tgt_q, torch.zeros_like(tgt_q))
robot.set_joint_position_target(tgt_q)
robot.write_data_to_sim()

steps = int(SECONDS / wbc.dt)
print(f"[wbc ] {SECONDS:.0f} s = {steps} physics steps, policy every "
      f"{wbc.decimation} ({1.0 / (wbc.dt * wbc.decimation):.0f} Hz)")
cmd, loco_n = wbc.command(height=HEIGHT)
print(f"[wbc ] height command {cmd[3]:.3f}")
frames = []
for i in range(steps):
    if i % wbc.decimation == 0:
        robot.update(wbc.dt)
        qj = np.array([robot.data.joint_pos[0, j].item() for j in pol_ids])
        dqj = np.array([robot.data.joint_vel[0, j].item() for j in pol_ids])
        quat = robot.data.root_quat_w[0].cpu().numpy()          # wxyz
        omega = robot.data.root_ang_vel_b[0].cpu().numpy()
        targets = wbc.step(qj, dqj, quat, omega, cmd, loco_n)
        for k, jid in enumerate(low_ids):
            tgt_q[0, jid] = float(targets[k])
        robot.set_joint_position_target(tgt_q)
    robot.write_data_to_sim()
    sim.step()

    if i % 200 == 0:
        robot.update(wbc.dt)
        p = robot.data.root_pos_w[0].cpu().numpy()
        q = robot.data.root_quat_w[0].cpu().numpy()
        up = 1.0 - 2.0 * (q[1] ** 2 + q[2] ** 2)   # world z of the body z axis
        palm = robot.data.body_pos_w[0, palm_id].cpu().numpy()
        print(f"[wbc ] t {i * wbc.dt:5.2f}s  pelvis z {p[2]:.3f}  "
              f"tilt {math.degrees(math.acos(max(-1.0, min(1.0, up)))):5.1f}deg"
              f"  palm z {palm[2]:.3f}")
    if cam is not None:
        app.update()
        cam.update(0.0)
        if i % 4 == 0:
            frames.append(
                cam.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8))

robot.update(wbc.dt)
p = robot.data.root_pos_w[0].cpu().numpy()
q = robot.data.root_quat_w[0].cpu().numpy()
up = 1.0 - 2.0 * (q[1] ** 2 + q[2] ** 2)
tilt = math.degrees(math.acos(max(-1.0, min(1.0, up))))
print(f"[wbc ] end pelvis z {p[2]:.3f}  tilt {tilt:.1f}deg  "
      f"drift {np.linalg.norm(p[:2] - np.array(STAND[:2])):.3f} m  "
      f"-> {'STANDING' if p[2] > 0.55 and tilt < 25 else 'FELL'}")
if cam is not None:
    imageio.mimsave(OUT, frames, fps=50, quality=8)
    print(f"[wbc ] wrote {OUT}: {len(frames)} frames")
sys.stdout.flush()
os._exit(0)
