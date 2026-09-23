# Play a cuRobo-planned arm trajectory on G1 inside the cell, in Isaac.
#
# The planning happens outside this process (cuRobo + GraspGenX, in their own
# environment) and arrives as joint angles over time. Here the same joints are
# driven on Isaac's 29-dof G1 -- the joint names match exactly, so the
# trajectory transfers without a mapping table.
#
# The root is driven, not simulated: the plan moves the waist and right arm
# only, and a robot that falls over mid-reach says nothing about whether the
# plan was good. So the pelvis is written where it belongs every frame and the
# arm does real physics against the object.
#
# --walk CLIP.pkl plays a walk in front of the reach, from the same motion_lib
# clip grasp/walk_clip.py writes. The robot starts wherever that clip starts,
# walks to where it ends -- which is the stand the plan was made from -- and
# only then does the trajectory run. Both halves render through the same three
# cameras, so the walk and the pick are one video, not two.
#
#   python grasp/play_in_cell.py [traj.npy] [--video out.mp4] [--walk clip.pkl]
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
WALK = (sys.argv[sys.argv.index("--walk") + 1]
        if "--walk" in sys.argv else None)

app = AppLauncher(headless=True, enable_cameras=not NO_VIDEO).app

import math  # noqa: E402

import imageio.v2 as imageio  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402
import trimesh.transformations as tf  # noqa: E402
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
# The walk arrives as a motion_lib clip: 29 joints in the URDF's own order,
# plus a root path. Loaded before the robot is built because it decides where
# the robot starts.
walk = None
if WALK:
    import joblib  # noqa: E402
    sys.path.insert(0, "/home/sehoon/Documents/GitHub/humanoid-swarm-sim/common")
    from foot_height import load_urdf  # noqa: E402
    _w = list(joblib.load(WALK).values())[0]
    walk = {"dof": np.asarray(_w["dof"], dtype=np.float32),
            "pos": np.asarray(_w["root_trans_offset"], dtype=np.float32),
            "quat": np.asarray(_w["root_rot"], dtype=np.float32)}   # xyzw
    # load_urdf() reads gear_sonic/data/robots/g1/g1_29dof.urdf by a relative
    # path, so it only resolves from the SONIC repo root. Borrow that cwd for
    # the one call rather than leaving this process there.
    _cwd = os.getcwd()
    os.chdir("/home/sehoon/Projects/GR00T-WholeBodyControl")
    try:
        walk["names"] = load_urdf()[1]
    finally:
        os.chdir(_cwd)
    print(f"[walk] {len(walk['dof'])} frames, "
          f"{np.round(walk['pos'][0][:2], 2)} -> {np.round(walk['pos'][-1][:2], 2)}")

cfg = G1_29DOF_CFG.replace(prim_path="/World/G1")
cfg.spawn = cfg.spawn.replace(
    articulation_props=sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=False, solver_position_iteration_count=12,
        solver_velocity_iteration_count=4, fix_root_link=walk is None))
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
_start = (-1.30, -0.60, cfg.init_state.pos[2])
_rot = (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))
if walk is not None:
    _q = walk["quat"][0]
    _start = (float(walk["pos"][0][0]), float(walk["pos"][0][1]),
              float(walk["pos"][0][2]))
    _rot = (float(_q[3]), float(_q[0]), float(_q[1]), float(_q[2]))  # -> wxyz
cfg.init_state = cfg.init_state.replace(pos=_start, rot=_rot)
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
# ALWAYS the stand the plan was made from, never wherever a walk happens to
# start: the trajectory's joint angles only reach the object if the object is
# where the planner thought it was.
T_torso = torso_pose((-1.30, -0.60, G1_29DOF_CFG.init_state.pos[2]), yaw)

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

# Third view: the head camera, the same mount grasp/capture_rgbd.py shoots the
# capture from -- 86 deg horizontal FoV, pitched 15 deg down because G1 has no
# neck joint. This is the view the grasp was predicted from, so a render
# should show it next to what the hand is doing.
HEAD_FOV, HEAD_APERTURE, HEAD_PITCH_DEG = 86.0, 20.955, 15.0
_hf = HEAD_APERTURE / (2.0 * math.tan(math.radians(HEAD_FOV) / 2.0))
_hp = math.radians(HEAD_PITCH_DEG) / 2.0
eye_cam = None if NO_VIDEO else Camera(CameraCfg(
    prim_path="/World/G1/head_link/head_cam", update_period=0.0,
    width=960, height=540, data_types=["rgb"],
    offset=CameraCfg.OffsetCfg(pos=(0.08, 0.0, 0.05),
                               rot=(math.cos(_hp), 0.0, math.sin(_hp), 0.0),
                               convention="world"),
    spawn=sim_utils.PinholeCameraCfg(focal_length=_hf,
                                     horizontal_aperture=HEAD_APERTURE,
                                     clipping_range=(0.01, 20.0))))

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
if walk is not None:
    # Frame both ends of the walk plus the table between them, from the cell's
    # open south-east corner (map/cell_layout.py: only north and west have
    # walls, so this is the one corner with nothing in the way).
    mid = 0.5 * (walk["pos"][0][:2] + np.array(tgt[:2]))
    tgt = np.array([mid[0], mid[1], 0.80])
    eye = np.array([3.4, -3.2, 2.3])
if cam is not None:
    cam.set_world_poses_from_view(
        eyes=torch.tensor([eye], dtype=torch.float32, device=cam.device),
        targets=torch.tensor([tgt], dtype=torch.float32, device=cam.device))

# Start the robot AT the plan's first waypoint. Left at G1's default pose the
# arm snaps across the whole reach in the first frame, and that swing throws
# the object off the table before the plan has even started.
tgt_q = robot.data.default_joint_pos.clone()
walk_ids = None
if walk is not None:
    walk_ids = [robot.find_joints([n])[0][0] for n in walk["names"]]
    for k, jid in enumerate(walk_ids):
        tgt_q[0, jid] = float(walk["dof"][0, k])
else:
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

frames, head_frames, eye_frames = [], [], []


# head_link is not in the articulation's body list: IsaacLab's G1 USD is
# imported with fixed joints merged, so the head is part of torso_link. (The
# swarm repo's demos import with merge_fixed_joints=False for exactly this
# reason, but that is a different asset than the one this file spawns.) The
# mount is still exact -- head_joint is fixed, straight out of main.urdf --
# so the camera hangs off the torso with that offset folded in.
_HEAD_FROM_TORSO = (0.0039635, 0.0, -0.044)
_head_id = robot.find_bodies(["torso_link"])[0][0]
_wrist_id = robot.find_bodies([WRIST_PARENT])[0][0]
_T_head_cam = (_link(_HEAD_FROM_TORSO, (0.0, 0.0, 0.0))
               @ _link((0.08, 0.0, 0.05), (0.0, math.radians(HEAD_PITCH_DEG), 0.0)))


def _place(camera, body_id, T_mount, convention):
    """Put a camera where its link is now.

    The cameras are prims under the robot, which is enough while the pelvis is
    welded in place. It is not enough once the robot walks: the root is moved
    by writing it, and with Fabric off those writes do not reach the USD
    transforms the camera prim is parented to, so the view stays behind at the
    start of the walk. Reading the link pose out of the articulation and
    setting the camera from it does not care how the robot got there.
    """
    p = robot.data.body_pos_w[0, body_id].cpu().numpy()
    q = robot.data.body_quat_w[0, body_id].cpu().numpy()      # wxyz
    T = np.eye(4)
    T[:3, :3] = tf.quaternion_matrix([q[0], q[1], q[2], q[3]])[:3, :3]
    T[:3, 3] = p
    W = T @ T_mount
    wq = Gf.Matrix4d(W.T.tolist()).ExtractRotationQuat()
    camera.set_world_poses(
        torch.tensor([W[:3, 3]], dtype=torch.float32, device=camera.device),
        torch.tensor([[wq.GetReal(), *wq.GetImaginary()]],
                     dtype=torch.float32, device=camera.device),
        convention=convention)


def shoot():
    """One frame from each of the three cameras."""
    if NO_VIDEO:
        return
    _place(head_cam, _wrist_id, _T_wrist_cam, "ros")
    _place(eye_cam, _head_id, _T_head_cam, "world")
    app.update()
    cam.update(0.0)
    head_cam.update(0.0)
    eye_cam.update(0.0)
    frames.append(cam.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8))
    head_frames.append(
        head_cam.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8))
    eye_frames.append(
        eye_cam.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8))


hold_root = None
if walk is not None:
    # A replay, not a controller: the clip already says where every joint and
    # the pelvis are on every frame, so they are written rather than tracked.
    # Nothing here has to balance -- SONIC's tracker is what proves the walk is
    # walkable (results in the 260923 videos), and this is the same clip.
    print(f"[walk] replaying {len(walk['dof'])} frames")
    zero = torch.zeros((1, robot.num_joints), device=sim.device)
    for i in range(len(walk["dof"])):
        for k, jid in enumerate(walk_ids):
            tgt_q[0, jid] = float(walk["dof"][i, k])
        q = walk["quat"][i]
        root = torch.tensor(
            [[float(walk["pos"][i][0]), float(walk["pos"][i][1]),
              float(walk["pos"][i][2]), float(q[3]), float(q[0]), float(q[1]),
              float(q[2]), 0, 0, 0, 0, 0, 0]],
            dtype=torch.float32, device=sim.device)
        robot.write_root_state_to_sim(root)
        robot.write_joint_state_to_sim(tgt_q, zero)
        # Without this the actuators keep pulling toward whatever target was
        # set last -- the default stance -- and fight the pose being written
        # every step, which reads as the legs shaking.
        robot.set_joint_position_target(tgt_q)
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim.get_physics_dt())
        shoot()
    # The planner commits eight frames at a time, so the walk stops within a
    # step of the goal rather than on it -- and the trajectory's joint angles
    # only reach the object from the stand it was planned at. Close that gap
    # before the arm starts instead of reaching from 17 cm off.
    _sx, _sy, _sz = -1.30, -0.60, float(walk["pos"][-1][2])
    _syaw = math.radians(-90.0)
    _sq = (math.cos(_syaw / 2.0), 0.0, 0.0, math.sin(_syaw / 2.0))
    stand_root = torch.tensor(
        [[_sx, _sy, _sz, *_sq, 0, 0, 0, 0, 0, 0]],
        dtype=torch.float32, device=sim.device)
    glide = 15
    print(f"[walk] arrived at {np.round(walk['pos'][-1][:2], 3)}; "
          f"settling onto the stand ({_sx}, {_sy}) over {glide} frames")
    for i in range(glide):
        a = (i + 1) / glide
        robot.write_root_state_to_sim(root * (1.0 - a) + stand_root * a)
        robot.write_joint_state_to_sim(tgt_q, zero)
        robot.set_joint_position_target(tgt_q)
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim.get_physics_dt())
        shoot()
    hold_root = stand_root
    # Freezing the pelvis by writing it every step is not the same thing as
    # the fix_root_link constraint the no-walk run uses: the legs are still
    # PD-driven and still pushing on the floor, the write cancels the reaction
    # and its velocity, and the two fight at the physics rate -- which is the
    # buzzing the legs do once the robot stops. Writing the leg state too
    # makes the whole lower body kinematic, which is what the welded run
    # effectively had, and leaves the arm and hand as the only things doing
    # physics against the object.
    frozen_ids = [robot.find_joints([n])[0][0] for n in walk["names"]
                  if ("hip" in n or "knee" in n or "ankle" in n)]
    frozen_q = robot.data.joint_pos[0, frozen_ids].clone().unsqueeze(0)
    frozen_v = torch.zeros_like(frozen_q)
    print(f"[walk] holding {len(frozen_ids)} leg joints still for the reach")
    # Hand over to the plan: the arm goes to its first waypoint before the
    # trajectory starts, for the same reason it does without a walk -- left at
    # the walk's pose it would swing across the table on frame one.
    for k, jid in enumerate(ids):
        tgt_q[0, jid] = float(traj[0, k])
    robot.write_joint_state_to_sim(tgt_q, zero)
    robot.set_joint_position_target(tgt_q)
    robot.write_data_to_sim()

substeps = max(1, round((1.0 / FPS) / sim.get_physics_dt()))
print(f"[play] {substeps} physics steps per trajectory frame")
for i in range(traj.shape[0]):
    for k, jid in enumerate(ids):
        tgt_q[0, jid] = float(traj[i, k])
    robot.set_joint_position_target(tgt_q)
    for _ in range(substeps):
        if hold_root is not None:
            # The root is unwelded so the walk could happen; pin it here by
            # writing it instead, so the reach is judged the same way it is
            # without a walk. The legs go with it -- see above.
            robot.write_root_state_to_sim(hold_root)
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
    shoot()

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
EYE_OUT = OUT.replace(".mp4", "_head.mp4")
imageio.mimsave(EYE_OUT, eye_frames, fps=FPS, quality=8)
print(f"[play] wrote {EYE_OUT}: {len(eye_frames)} frames")
print(f"[play] wrote {OUT}: {len(frames)} frames")
print(f"[play] wrote {HEAD_OUT}: {len(head_frames)} frames")
sys.stdout.flush()
os._exit(0)
