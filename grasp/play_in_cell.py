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
# The second view rides the wrist, where Unitree's own G1 datasets put a
# camera. Numbers are Zulkhuu/g1-wrist-camera-tools' right-side mount, with the
# D405 optical +Z along the mount body's +Y as that repo states. The head is
# not used for this view: G1 has no neck joint, so a head camera loses the work
# as soon as the arm goes sideways.
WRIST_PARENT = "right_wrist_yaw_link"
WRIST_MOUNT_XYZ = (0.07, 0.0, 0.0)
WRIST_MOUNT_RPY = (1.5707963267948966, 0.7853981633974483, 1.5707963267948966)
WRIST_CAM_XYZ = (0.008, 0.091, 0.002)
WRIST_CAM_RPY = (1.9792033717615698, 0.3490658503988659, 0.0)

OUT = sys.argv[sys.argv.index("--video") + 1] if "--video" in sys.argv \
    else os.path.join(REPO, "results", "g1_cell_pick.mp4")
# --no-video runs the same physics and prints the same object track, but skips
# the cameras. Rendering 830 frames through two cameras is most of the wall
# clock here, and judging whether a grasp held needs none of it -- this is how
# a batch of grasp candidates gets evaluated in reasonable time.
NO_VIDEO = "--no-video" in sys.argv

app = AppLauncher(headless=True, enable_cameras=not NO_VIDEO).app

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
from plan_scene import FINGER_MU, OBJECT_MU  # noqa: E402
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

# 1 kHz, the physics step GraspGenX's own dynamic_playback uses (sim_dt=0.001
# with sim_fps=60, so ~17 solver steps per trajectory waypoint). At 1/120 there
# are only 2 steps per waypoint and the hand passes through a contact in one
# step, which reads as the arm swatting the object.
sim = SimulationContext(sim_utils.SimulationCfg(dt=1.0 / 1000.0, device="cpu"))
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
# Dex3-1 finger torque, from Unitree's own URDF: every hand joint in
# g1_29dof_with_hand_rev_1_0.urdf is <limit effort="1.4" velocity="12">.
# IsaacLab's G1_29DOF_CFG leaves the hands at effort_limit=300, 214x the real
# actuator, so the closing fingers drive straight through the object and PhysX
# ejects it instead of stalling them on contact. The trajectory commands the
# fingers all the way to their joint limits (end2end/tasks.py ramps to
# close_vals), so what stops them has to be the actuator, not the command.
cfg.actuators["hands"] = cfg.actuators["hands"].replace(
    effort_limit=1.4, velocity_limit=12.0)
yaw = math.radians(-90.0)
# Where the plan was made, and so where the scene is placed and where the arm
# has to be standing when it reaches. Nothing below moves it.
STAND = (-1.30, -0.60, cfg.init_state.pos[2])
# A walk, if one was given: the robot starts wherever the clip starts and
# walks in. The pelvis cannot be pinned for that, and the pick below needs it
# pinned, so the clip is replayed first and then the robot is put back on
# STAND exactly -- after which everything is the no-walk run, unchanged.
WALK = (sys.argv[sys.argv.index("--walk") + 1]
        if "--walk" in sys.argv else None)
walk = None
if WALK:
    import joblib
    sys.path.insert(0, "/home/sehoon/Documents/GitHub/humanoid-swarm-sim/common")
    from foot_height import load_urdf
    _w = list(joblib.load(WALK).values())[0]
    walk = {"dof": np.asarray(_w["dof"], dtype=np.float32),
            "pos": np.asarray(_w["root_trans_offset"], dtype=np.float32),
            "quat": np.asarray(_w["root_rot"], dtype=np.float32)}   # xyzw
    # The clip carries no joint names, and its order is not IsaacLab's --
    # cuRobo's conventions page is explicit that the two traverse the tree
    # differently. The names come from the same URDF the clip was written
    # against. load_urdf() opens it by a relative path, so it only resolves
    # from the SONIC repo root; borrow that cwd for the one call.
    _cwd = os.getcwd()
    os.chdir("/home/sehoon/Projects/GR00T-WholeBodyControl")
    try:
        walk["names"] = load_urdf()[1]
    finally:
        os.chdir(_cwd)
    print(f"[walk] {len(walk['dof'])} frames, "
          f"{np.round(walk['pos'][0][:2], 2)} -> {np.round(walk['pos'][-1][:2], 2)}")
    cfg.spawn = cfg.spawn.replace(
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=12,
            solver_velocity_iteration_count=4, fix_root_link=False))
    _q0 = walk["quat"][0]
    cfg.init_state = cfg.init_state.replace(
        pos=tuple(float(v) for v in walk["pos"][0]),
        rot=(float(_q0[3]), float(_q0[0]), float(_q0[1]), float(_q0[2])))
else:
    cfg.init_state = cfg.init_state.replace(
        pos=STAND,
        rot=(math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)))
robot = Articulation(cfg)

# Finger pad friction, from GraspGenX's own --finger_mu default of 3.0. Left
# alone the stage runs on PhysX's 0.5, and a grasp generated under mu 3 on the
# pads slips the moment the fingers touch the box.
_fm = sim_utils.RigidBodyMaterialCfg(static_friction=FINGER_MU,
                                     dynamic_friction=FINGER_MU,
                                     restitution=0.0)
_fm.func("/World/G1/FingerMaterial", _fm)
# Bound at the articulation root: the link prim paths inside the G1 USD are
# not /World/G1/<link>, and a subtree binding reaches every collider anyway.
# The pelvis is pinned for this replay, so the feet never use it.
sim_utils.bind_physics_material("/World/G1", "/World/G1/FingerMaterial")
print(f"[play] finger pads mu {FINGER_MU}, object mu {OBJECT_MU}")

# The plan's scene, placed relative to where the robot's torso will stand.
# This has to happen BEFORE sim.reset(): PhysX builds its scene there, and a
# rigid body added afterwards is never simulated -- it just hangs frozen in
# mid-air. Same ordering map/props.py uses. torso_link sits at a fixed offset
# from the pelvis (the waist joints are at 0 in the default pose and the plan
# never moves them), measured off the G1 URDF.
T_torso = torso_pose(STAND, yaw)

UsdGeom.Xform.Define(stage, "/Render")
cam = None if NO_VIDEO else Camera(CameraCfg(
    prim_path="/Render/Cam", update_period=0.0, width=960, height=540,
    data_types=["rgb"],
    spawn=sim_utils.PinholeCameraCfg(focal_length=22.0, clipping_range=(0.05, 40.0))))

# The robot's own view, the same mount grasp/capture_rgbd.py shoots from, so
# the second video is what the hand sees while it works.
def _rpy_matrix(r, p_, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p_), math.sin(p_)
    cy, sy = math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
            @ np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
            @ np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]))


def _link(xyz, rpy):
    M = np.eye(4)
    M[:3, :3] = _rpy_matrix(*rpy)
    M[:3, 3] = xyz
    return M


_T_wrist_cam = (_link(WRIST_MOUNT_XYZ, WRIST_MOUNT_RPY)
                @ _link(WRIST_CAM_XYZ, WRIST_CAM_RPY)
                @ _link((0.0, 0.0, 0.0), (-math.pi / 2.0, 0.0, 0.0)))
_wq = Gf.Matrix4d(_T_wrist_cam.T.tolist()).ExtractRotationQuat()
head_cam = None if NO_VIDEO else Camera(CameraCfg(
    prim_path=f"/World/G1/{WRIST_PARENT}/wrist_cam", update_period=0.0,
    width=960, height=540, data_types=["rgb"],
    offset=CameraCfg.OffsetCfg(
        pos=tuple(float(v) for v in _T_wrist_cam[:3, 3]),
        rot=(float(_wq.GetReal()), *[float(v) for v in _wq.GetImaginary()]),
        convention="ros"),
    spawn=sim_utils.PinholeCameraCfg(clipping_range=(0.01, 20.0))))

build_plan_scene(stage, app, meta, T_torso)

# Track the object in Isaac too: the Newton replay is a different engine, and
# what matters is what this one does. Wrapped before reset -- an asset made
# after it is never initialised.
target_body = None
if "object" in meta:
    from isaaclab.assets import RigidObject, RigidObjectCfg  # noqa: E402
    target_body = RigidObject(RigidObjectCfg(prim_path="/World/GraspTarget",
                                             spawn=None))

sim.reset()

obj_start = None
if target_body is not None:
    target_body.update(0.0)
    obj_start = target_body.data.root_pos_w[0].cpu().numpy()
    print(f"[obj ] start {np.round(obj_start,4)}")

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
if cam is not None:
    cam.set_world_poses_from_view(
        eyes=torch.tensor([eye], dtype=torch.float32, device=cam.device),
        targets=torch.tensor([tgt], dtype=torch.float32, device=cam.device))

frames, head_frames = [], []
tgt_q = robot.data.default_joint_pos.clone()
zero = torch.zeros_like(tgt_q)


def _shoot():
    """One recorded frame, the same pair the trajectory loop collects."""
    if NO_VIDEO:
        return
    app.update()
    cam.update(0.0)
    head_cam.update(0.0)
    frames.append(cam.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8))
    head_frames.append(
        head_cam.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8))


if walk is not None:
    # Walk in, then put the robot back on STAND exactly. The clip is played
    # kinematically -- root and joints written every step -- because what it
    # has to deliver is a body in the right place, not a balance problem.
    #
    # The last stretch is a forced idle in the clip itself, the way the
    # reference loop ends every run (interactive_demo_g1.py: force_idle for
    # the last 100 steps), so the robot arrives standing rather than
    # mid-stride. Then the root is eased onto STAND over 15 frames: the
    # planner commits 8 frames at a time and stops within a step of the goal,
    # and the arm's joint angles only reach the box from STAND.
    walk_ids = [robot.find_joints([n])[0][0] for n in walk["names"]]
    # Bring the arm to the plan's first waypoint while the robot is still
    # walking, out over nothing.
    #
    # The no-walk run writes that waypoint in a single step too -- it has to,
    # because the plan starts with the hand already above the object and
    # leaving the arm at G1's default pose swings it across the table on
    # frame one. There it is invisible: it happens before the settle, and the
    # settle happens before recording starts. Put a walk in front and the
    # same one line lands in the middle of the video, and the hand teleports.
    #
    # Easing it at the table is worse than the teleport -- measured, the arm
    # sweeps the box 1.17 m onto the floor. So it is done on approach, which
    # is also what a person does: you raise your hand as you walk up.
    _lift_n = min(45, len(walk["dof"]))
    _lift_from = len(walk["dof"]) - _lift_n
    _plan_q = [float(traj[0, k]) for k in range(len(ids))]
    print(f"[walk] replaying {len(walk['dof'])} frames, arm onto the plan's "
          f"first waypoint over the last {_lift_n}")
    for i in range(len(walk["dof"])):
        for k, jid in enumerate(walk_ids):
            tgt_q[0, jid] = float(walk["dof"][i, k])
        if i >= _lift_from:
            a = (i - _lift_from + 1) / _lift_n
            for k, jid in enumerate(ids):
                tgt_q[0, jid] = (1.0 - a) * float(tgt_q[0, jid]) + a * _plan_q[k]
        q = walk["quat"][i]
        robot.write_root_state_to_sim(torch.tensor(
            [[float(walk["pos"][i][0]), float(walk["pos"][i][1]),
              float(walk["pos"][i][2]), float(q[3]), float(q[0]),
              float(q[1]), float(q[2]), 0, 0, 0, 0, 0, 0]],
            dtype=torch.float32, device=sim.device))
        robot.write_joint_state_to_sim(tgt_q, zero)
        robot.set_joint_position_target(tgt_q)
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim.get_physics_dt())
        _shoot()
    stand_root = torch.tensor(
        [[STAND[0], STAND[1], STAND[2],
          math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0),
          0, 0, 0, 0, 0, 0]], dtype=torch.float32, device=sim.device)
    last = robot.data.root_state_w[:1].clone()
    print(f"[walk] settling onto the stand {STAND[:2]} over 15 frames")
    for i in range(15):
        a = (i + 1) / 15
        robot.write_root_state_to_sim(last * (1.0 - a) + stand_root * a)
        robot.write_joint_state_to_sim(tgt_q, zero)
        robot.set_joint_position_target(tgt_q)
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim.get_physics_dt())
        _shoot()

frozen_ids = frozen_q = frozen_v = None
if walk is not None:
    # Only the legs. The arm and the hand have to keep doing physics -- they
    # are the things touching the box.
    frozen_ids = [robot.find_joints([n])[0][0] for n in robot.joint_names
                  if ("hip" in n or "knee" in n or "ankle" in n)]
    frozen_q = robot.data.joint_pos[0, frozen_ids].clone().unsqueeze(0)
    frozen_v = torch.zeros_like(frozen_q)
    print(f"[walk] holding {len(frozen_ids)} leg joints still for the reach")

# Start the robot AT the plan's first waypoint. Left at G1's default pose the
# arm snaps across the whole reach in the first frame, and that swing throws
# the object off the table before the plan has even started.
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

substeps = max(1, round((1.0 / FPS) / sim.get_physics_dt()))
print(f"[play] {substeps} physics steps per trajectory frame")
for i in range(traj.shape[0]):
    for k, jid in enumerate(ids):
        tgt_q[0, jid] = float(traj[i, k])
    robot.set_joint_position_target(tgt_q)
    for _ in range(substeps):
        if walk is not None:
            # With a clip there is no fix_root_link to hold the pelvis, so it
            # is written every step -- and the legs with it. Writing the root
            # alone is not the same thing: the legs keep pushing on the floor,
            # the write cancels the reaction, and the two fight at the physics
            # rate, which is the buzzing the legs do once the robot stops.
            robot.write_root_state_to_sim(stand_root)
            robot.write_joint_state_to_sim(frozen_q, frozen_v,
                                           joint_ids=frozen_ids)
        robot.write_data_to_sim()
        sim.step()
    robot.update(sim.get_physics_dt())
    if i % 20 == 0 and target_body is not None:
        target_body.update(sim.get_physics_dt())
        op = target_body.data.root_pos_w[0].cpu().numpy()
        print(f"[obj ] frame {i:5d} pos {np.round(op,4)}  moved "
              f"{np.linalg.norm(op - obj_start):.4f} m")
    if i % 200 == 0:
        got = np.array([robot.data.joint_pos[0, j].item() for j in ids])
        want = np.array([traj[i, k] for k in range(len(ids))])
        print(f"[play] frame {i:5d} 목표 {np.round(want,2)}")
        print(f"[play]              실제 {np.round(got,2)}  최대오차 "
              f"{np.abs(got-want).max():.3f} rad")
    if not NO_VIDEO:
        app.update()
        cam.update(0.0)
        head_cam.update(0.0)
        frames.append(cam.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8))
        head_frames.append(
            head_cam.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8))

if target_body is not None:
    target_body.update(sim.get_physics_dt())
    op = target_body.data.root_pos_w[0].cpu().numpy()
    d = op - obj_start
    # Held = the object came up with the hand. The lift is the plan's own
    # last cartesian segment, so a grasp that worked ends the run higher than
    # it started; one that was knocked away ends at or below the table.
    print(f"[eval] end pos {np.round(op,4)}  dxy {np.linalg.norm(d[:2]):.4f} m"
          f"  dz {d[2]:+.4f} m  -> {'HELD' if d[2] > 0.05 else 'LOST'}")

if NO_VIDEO:
    sys.stdout.flush()
    os._exit(0)

os.makedirs(os.path.dirname(OUT), exist_ok=True)
imageio.mimsave(OUT, frames, fps=FPS, quality=8)
HEAD_OUT = OUT.replace(".mp4", "_wrist.mp4")
imageio.mimsave(HEAD_OUT, head_frames, fps=FPS, quality=8)
print(f"[play] wrote {OUT}: {len(frames)} frames")
print(f"[play] wrote {HEAD_OUT}: {len(head_frames)} frames")
sys.stdout.flush()
os._exit(0)
