# Play a cuRobo-planned arm trajectory on G1 inside the cell, in Isaac.
#
# The planning happens outside this process (cuRobo + GraspGenX, in their own
# environment) and arrives as joint angles over time. Here the same joints are
# driven on Isaac's 29-dof G1 -- the joint names match exactly, so the
# trajectory transfers without a mapping table.
#
# The root is fixed: the plan moves the waist and right arm only, and a robot
# that falls over mid-reach says nothing about whether the plan was good.
#
#   python grasp/play_in_cell.py [traj.npy] [--video out.mp4]
import json
import os
import sys

from isaaclab.app import AppLauncher

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "map"))
sys.path.insert(0, os.path.join(REPO, "grasp"))
TRAJ = sys.argv[1] if len(sys.argv) > 1 else os.path.join(REPO, "results", "g1_traj.npy")
META = os.path.splitext(TRAJ)[0] + ".json"
# Head camera geometry: the D435i's 86 deg horizontal depth FoV, pitched the
# same 15 deg grasp/capture_rgbd.py uses.
HEAD_FOV = 86.0
HEAD_APERTURE = 20.955
HEAD_FOCAL = HEAD_APERTURE / (2.0 * __import__("math").tan(
    __import__("math").radians(HEAD_FOV) / 2.0))
HEAD_PITCH_DEG = 15.0

OUT = sys.argv[sys.argv.index("--video") + 1] if "--video" in sys.argv \
    else os.path.join(REPO, "results", "g1_cell_pick.mp4")

app = AppLauncher(headless=True, enable_cameras=True).app

import math  # noqa: E402

import imageio.v2 as imageio  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402
import torch  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
import omni.usd  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sensors import Camera, CameraCfg  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab_assets.robots.unitree import G1_29DOF_CFG  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics  # noqa: E402

import cell_layout as L  # noqa: E402
from plan_scene import (  # noqa: E402
    build as build_plan_scene, torso_pose)
from props import spawn_props  # noqa: E402

traj = np.load(TRAJ)
if traj.ndim == 3:
    traj = np.concatenate(list(traj), axis=0)
meta = json.load(open(META))
names = meta["joint_names"]
FPS = int(meta.get("fps", 30))
print(f"[play] {traj.shape[0]} frames, {len(names)} joints, {FPS} fps")

sim = SimulationContext(sim_utils.SimulationCfg(dt=1.0 / 120.0, device="cpu"))
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
stage.DefinePrim("/World/Cell", "Xform").GetReferences().AddReference(
    Sdf.Reference(os.path.join(REPO, "map", "cell.usd")))
light = sim_utils.DomeLightCfg(intensity=900.0)
light.func("/World/light", light)
for _ in range(3):
    app.update()
spawn_props(stage, app)

# stand the robot where the plan was made: facing the object it reaches for
cfg = G1_29DOF_CFG.replace(prim_path="/World/G1")
cfg.spawn = cfg.spawn.replace(
    articulation_props=sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=False, solver_position_iteration_count=12,
        solver_velocity_iteration_count=4, fix_root_link=True))
yaw = math.radians(-90.0)
cfg.init_state = cfg.init_state.replace(
    pos=(-1.30, -0.60, cfg.init_state.pos[2]),
    rot=(math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)))
robot = Articulation(cfg)

# The plan's scene, placed relative to where the robot's torso will stand.
# This has to happen BEFORE sim.reset(): PhysX builds its scene there, and a
# rigid body added afterwards is never simulated -- it just hangs frozen in
# mid-air. Same ordering map/props.py uses. torso_link sits at a fixed offset
# from the pelvis (the waist joints are at 0 in the default pose and the plan
# never moves them), measured off the G1 URDF.
T_torso = torso_pose(cfg.init_state.pos, yaw)

UsdGeom.Xform.Define(stage, "/Render")
cam = Camera(CameraCfg(
    prim_path="/Render/Cam", update_period=0.0, width=960, height=540,
    data_types=["rgb"],
    spawn=sim_utils.PinholeCameraCfg(focal_length=22.0, clipping_range=(0.05, 40.0))))

# The robot's own view, same mount grasp/capture_rgbd.py shoots from, so the
# second video shows what the head camera sees while the arm works.
_hp = math.radians(HEAD_PITCH_DEG) / 2.0
head_cam = Camera(CameraCfg(
    prim_path="/World/G1/head_link/head_cam", update_period=0.0,
    width=960, height=540, data_types=["rgb"],
    offset=CameraCfg.OffsetCfg(pos=(0.08, 0.0, 0.05),
                               rot=(math.cos(_hp), 0.0, math.sin(_hp), 0.0),
                               convention="world"),
    spawn=sim_utils.PinholeCameraCfg(focal_length=HEAD_FOCAL,
                                     horizontal_aperture=HEAD_APERTURE,
                                     clipping_range=(0.01, 20.0))))

build_plan_scene(stage, app, meta, T_torso)

sim.reset()

ids = [robot.find_joints([n])[0][0] for n in names]
missing = [n for n in names if not robot.find_joints([n])[0]]
if missing:
    raise SystemExit(f"joints missing on the Isaac G1: {missing}")

# Frame the reach: far enough back to see the robot, the object it goes for,
# and the rack behind them. Aim at the object when the plan carries one.
tgt = np.array([-1.30, -0.75, 0.95])
if "object" in meta:
    tgt = 0.5 * (tgt + np.array(T_torso @ np.array(
        meta["object"]["transform_in_torso"]))[:3, 3])
eye = tgt + np.array([-1.65, -2.05, 1.05])
cam.set_world_poses_from_view(
    eyes=torch.tensor([eye], dtype=torch.float32, device=cam.device),
    targets=torch.tensor([tgt], dtype=torch.float32, device=cam.device))

# Start the robot AT the plan's first waypoint. Left at G1's default pose the
# arm snaps across the whole reach in the first frame, and that swing throws
# the object off the table before the plan has even started.
tgt_q = robot.data.default_joint_pos.clone()
for k, jid in enumerate(ids):
    tgt_q[0, jid] = float(traj[0, k])
robot.write_joint_state_to_sim(tgt_q, torch.zeros_like(tgt_q))
robot.set_joint_position_target(tgt_q)
robot.write_data_to_sim()
# Let the scene settle before recording: the object is placed from the plan's
# numbers, not dropped, so it starts a hair above its support and needs a
# moment of gravity to actually rest on it.
for _ in range(120):
    robot.write_data_to_sim()
    sim.step()
robot.update(sim.get_physics_dt())

frames, head_frames = [], []
substeps = max(1, round((1.0 / FPS) / sim.get_physics_dt()))
print(f"[play] {substeps} physics steps per trajectory frame")
for i in range(traj.shape[0]):
    for k, jid in enumerate(ids):
        tgt_q[0, jid] = float(traj[i, k])
    robot.set_joint_position_target(tgt_q)
    for _ in range(substeps):
        robot.write_data_to_sim()
        sim.step()
    robot.update(sim.get_physics_dt())
    if i % 200 == 0:
        got = np.array([robot.data.joint_pos[0, j].item() for j in ids])
        want = np.array([traj[i, k] for k in range(len(ids))])
        print(f"[play] frame {i:5d} 목표 {np.round(want,2)}")
        print(f"[play]              실제 {np.round(got,2)}  최대오차 "
              f"{np.abs(got-want).max():.3f} rad")
    app.update()
    cam.update(0.0)
    head_cam.update(0.0)
    frames.append(cam.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8))
    head_frames.append(
        head_cam.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8))

os.makedirs(os.path.dirname(OUT), exist_ok=True)
imageio.mimsave(OUT, frames, fps=FPS, quality=8)
HEAD_OUT = OUT.replace(".mp4", "_head.mp4")
imageio.mimsave(HEAD_OUT, head_frames, fps=FPS, quality=8)
print(f"[play] wrote {OUT}: {len(frames)} frames")
print(f"[play] wrote {HEAD_OUT}: {len(head_frames)} frames")
sys.stdout.flush()
os._exit(0)
