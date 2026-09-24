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
# Writing the leg joint states every physics step is what keeps the lower body
# from buzzing once the robot stops, but it also disturbs the articulation
# solver mid-step: measured, the fingers reached only thumb -0.38 / index 0.38
# by the time the no-walk run had them at -1.46 / 1.63, and the box was pushed
# away before they closed on it. This turns that write off so the two can be
# compared.
FREEZE_LEGS = "--no-freeze-legs" not in sys.argv
# Every leg write during the trajectory costs the grasp: the box was shoved
# 119.9 mm with the legs written each substep, 77.6 mm once a frame, 33.9 mm
# with only the pelvis per substep. This drops the leg writes entirely once
# the reach starts -- the settle still squares the lower body first.
LEGS_SETTLE_ONLY = "--legs-settle-only" in sys.argv
# How often the legs are written during the reach. The box is shoved 119.9 mm
# when that happens every physics substep, 33.9 mm once a frame, and 1046 mm
# -- off the table -- when it never happens at all, so the disturbance and the
# hold trade against each other and the best setting is somewhere between.
LEG_HOLD_EVERY = int(sys.argv[sys.argv.index("--leg-hold-every") + 1]) \
    if "--leg-hold-every" in sys.argv else 1
STIFF_LEGS = "--stiff-legs" in sys.argv
# Render the walk and stop, leaving the pick to a second run.
#
# The two phases are two controllers -- GR00T moves the body, cuRobo moves the
# arm -- and forcing them into one simulation means holding the pelvis and the
# legs by hand while the arm works. Every way of doing that costs the grasp:
# writing the legs each substep shoves the box 119.9 mm, once a frame 33.9 mm,
# every fourth frame blows the sim up, not at all lets the robot sag and the
# box travels 1046 mm. The no-walk run, whose pelvis is a real fixed joint,
# lifts the box by 121.5 mm every time. So the walk is rendered with a free
# root and the pick with a fixed one, and the two runs are joined on one
# clock afterwards.
WALK_ONLY = "--walk-only" in sys.argv

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
WBC = "--wbc" in sys.argv
# --drive STAND.json: one loop for the whole thing.
#
# decoupled_wbc takes navigate_cmd and target_upper_body_pose in the SAME
# call (g1_decoupled_whole_body_policy.get_action), and picks the walking or
# the standing policy off the command's magnitude with no reset between them:
#
#     if np.linalg.norm(self.cmd) < 0.05: policy = self.policy_1   # standing
#     else:                               policy = self.policy_2   # walking
#
# So walking to the object and reaching for it are not two phases to join.
# They are one loop whose command decays to zero on arrival. Both poses come
# out of where_to_stand's own file: base_path[0] is where the robot is,
# "stand" is where cuRobo decided it has to be.
DRIVE = None
if "--drive" in sys.argv:
    import json as _json
    _d = _json.load(open(sys.argv[sys.argv.index("--drive") + 1]))
    _g = _d["stand"]
    _b = _d["base_path"][0]
    DRIVE = {"goal": (float(_g["x"]), float(_g["y"]),
                      math.radians(float(_g["yaw_deg"]))),
             "start": (float(_b[0]), float(_b[1]), float(_b[2]))}
NO_SETTLE = "--no-settle" in sys.argv or WBC
SONIC = "--sonic" in sys.argv   # track the motion with SONIC itself
# --wbc: stand on GR00T's own balance policy instead of welding the pelvis.
# Welding it is what a real G1 cannot do, and it is what makes the join
# between the walk and the pick read as a teleport -- the walk has a body and
# the pick has a fixture. decoupled_wbc's lower-body policy is trained to hold
# a stance while the arms move; grasp/wbc_balance.py drives it.
cfg = G1_29DOF_CFG.replace(prim_path="/World/G1")
cfg.spawn = cfg.spawn.replace(
    articulation_props=sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=False, solver_position_iteration_count=12,
        solver_velocity_iteration_count=4,
        fix_root_link=not (WBC or SONIC)))
# Dex3-1 finger torque, from Unitree's own URDF: every hand joint in
# g1_29dof_with_hand_rev_1_0.urdf is <limit effort="1.4" velocity="12">.
# IsaacLab's G1_29DOF_CFG leaves the hands at effort_limit=300, 214x the real
# actuator, so the closing fingers drive straight through the object and PhysX
# ejects it instead of stalling them on contact. The trajectory commands the
# fingers all the way to their joint limits (end2end/tasks.py ramps to
# close_vals), so what stops them has to be the actuator, not the command.
cfg.actuators["hands"] = cfg.actuators["hands"].replace(
    effort_limit=1.4, velocity_limit=12.0)
if SONIC:
    # SONIC's own gains, computed the way policy_parameters.hpp computes them:
    # stiffness = armature * (2*pi*10)^2, damping = 2 * 2 * armature * (2*pi*10),
    # with the ankles at twice that. Driving its actions through anything else
    # is driving a different robot than the one it was trained on.
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from sonic_control import JOINTS as _SJ, STIFFNESS as _SK, DAMPING as _SD
    _kp = dict(zip(_SJ, _SK))
    _kd = dict(zip(_SJ, _SD))
    for _g, _act in list(cfg.actuators.items()):
        _pk, _pd = {}, {}
        for _n, _v in _kp.items():
            if any(__import__("re").fullmatch(e.replace(".*", "[a-z_]*"), _n)
                   for e in _act.joint_names_expr):
                _pk[_n] = float(_v)
                _pd[_n] = float(_kd[_n])
        if _pk:
            cfg.actuators[_g] = _act.replace(stiffness=_pk, damping=_pd)
            print(f"[sonic] {_g}: {len(_pk)} joints on SONIC's gains")
elif WBC:
    # The gains the balance policy was trained under, from decoupled_wbc's own
    # g1_gear_wbc.yaml: hips 150, knees 200, ankles 40, waist 250, with
    # damping 2 / 4 / 2 / 5. Isaac's defaults are close on the hips and knees
    # and half the policy's on the ankles, which is the joint that actually
    # keeps a standing robot standing.
    #
    # The keys are taken from each group's own joint_names_expr, because a
    # stiffness dict whose pattern matches nothing in its group is rejected
    # outright, and which joints sit in which group differs between G1 configs.
    _KP = (("knee", 200.0), ("hip", 150.0), ("waist", 250.0),
           ("torso", 250.0), ("ankle", 40.0))
    _KD = (("knee", 4.0), ("hip", 2.0), ("waist", 5.0),
           ("torso", 5.0), ("ankle", 2.0))
    for _g in ("legs", "feet"):
        _act = cfg.actuators.get(_g)
        if _act is None:
            continue
        _kp, _kd = {}, {}
        for _e in _act.joint_names_expr:
            for _key, _v in _KP:
                if _key in _e:
                    _kp[_e] = _v
                    break
            for _key, _v in _KD:
                if _key in _e:
                    _kd[_e] = _v
                    break
        if _kp:
            cfg.actuators[_g] = _act.replace(stiffness=_kp, damping=_kd)
            print(f"[wbc] {_g}: {len(_kp)} gain patterns set from the policy's yaml")
yaw = math.radians(-90.0)
# Where the plan was made, and so where the scene is placed and where the arm
# has to be standing when it reaches. Nothing below moves it.
# Where the scene is placed: the desk's own spot in the cell, unchanged.
SCENE_STAND = (-1.30, -0.60, cfg.init_state.pos[2])
# Where the robot stands for the pick. Vision chooses it; --pick-stand carries
# its answer in. Left out, it is the scene's own stand, which is what every
# run before this did.
if "--pick-stand" in sys.argv:
    _i = sys.argv.index("--pick-stand")
    STAND = (float(sys.argv[_i + 1]), float(sys.argv[_i + 2]),
             cfg.init_state.pos[2])
    PICK_YAW = math.radians(float(sys.argv[_i + 3]))
    print(f"[play] picking from the stand vision chose: "
          f"{np.round(STAND[:2], 3)} yaw {math.degrees(PICK_YAW):.1f} deg")
else:
    STAND = SCENE_STAND
    PICK_YAW = None
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
elif DRIVE is not None:
    cfg = cfg.replace(spawn=cfg.spawn.replace(
        articulation_props=cfg.spawn.articulation_props.replace(
            fix_root_link=False)))
    _sx, _sy, _syaw = DRIVE["start"]
    cfg.init_state = cfg.init_state.replace(
        pos=(_sx, _sy, STAND[2]),
        rot=(math.cos(_syaw / 2.0), 0.0, 0.0, math.sin(_syaw / 2.0)))
    print(f"[drive] starting where the robot is: ({_sx:.3f}, {_sy:.3f}, "
          f"{math.degrees(_syaw):.1f} deg)")
else:
    _y = yaw if PICK_YAW is None else PICK_YAW
    cfg.init_state = cfg.init_state.replace(
        pos=STAND,
        rot=(math.cos(_y / 2.0), 0.0, 0.0, math.sin(_y / 2.0)))
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

# Third view: the head camera, the same mount grasp/capture_rgbd.py shoots the
# capture from -- 86 deg horizontal FoV, pitched 15 deg down because G1 has no
# neck joint. This is the view the grasp was predicted from, so a render
# should show it next to what the hand is doing. (Taken from commit 2bb0fde.)
HEAD_FOV, HEAD_APERTURE, HEAD_PITCH_DEG = 86.0, 20.955, 15.0
_hf = HEAD_APERTURE / (2.0 * math.tan(math.radians(HEAD_FOV) / 2.0))
_hp = math.radians(HEAD_PITCH_DEG) / 2.0
# Not parented under head_link. IsaacLab's G1 USD merges fixed joints and
# absorbs head_link into the torso, so that prim path is not a body the
# articulation moves -- a camera hung there costs a great deal per frame and
# does not follow a root that is written rather than simulated. This is a free
# prim whose world pose is set from torso_link each frame, the same way the
# room camera is placed.
eye_cam = None if NO_VIDEO else Camera(CameraCfg(
    prim_path="/Render/HeadCam", update_period=0.0,
    width=960, height=540, data_types=["rgb"],
    spawn=sim_utils.PinholeCameraCfg(focal_length=_hf,
                                     horizontal_aperture=HEAD_APERTURE,
                                     clipping_range=(0.01, 20.0))))

# The plan's scene, placed relative to where the robot's torso will stand.
# This has to happen BEFORE sim.reset(): PhysX builds its scene there, and a
# rigid body added afterwards is never simulated -- it just hangs frozen in
# mid-air. Same ordering map/props.py uses. torso_link sits at a fixed offset
# from the pelvis (the waist joints are at 0 in the default pose and the plan
# never moves them), measured off the G1 URDF.
T_torso = torso_pose(SCENE_STAND, yaw)   # the desk does not move

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

frames, head_frames, eye_frames = [], [], []
tgt_q = robot.data.default_joint_pos.clone()
if "--legs-from" in sys.argv:
    # Start the pick standing the way the walk left the robot standing.
    #
    # The two phases are rendered separately, so the pick spawns at the
    # config's default pose while the walk ends squatted -- knee 0.65 rad, hip
    # -0.49 -- and joined on one clock that reads as the robot springing
    # upright in a single frame. Measured on the delivered clip: the root moves
    # 2.5 mm at the seam, the knee 37 degrees.
    #
    # The root is pinned for the pick, so leg angles move the feet and not the
    # torso: nothing the arm does is affected, only what the seam looks like.
    # The angles come from the clip's own last frame, which walk_clip.py
    # already writes out beside it.
    _ej = os.path.splitext(sys.argv[sys.argv.index("--legs-from") + 1])[0] + "_end.json"
    _end = json.load(open(_ej))
    _n = 0
    for _nm, _v in _end.items():
        if "hip" in _nm or "knee" in _nm or "ankle" in _nm:
            _ids = robot.find_joints([_nm])[0]
            if _ids:
                tgt_q[0, _ids[0]] = float(_v)
                _n += 1
    print(f"[play] legs taken from the walk's last frame: {_n} joints")
zero = torch.zeros_like(tgt_q)


_TORSO_ID = (robot.find_bodies(["torso_link"])[0] or [None])[0]


def _shoot():
    """One recorded frame, the same set the trajectory loop collects."""
    if NO_VIDEO:
        return
    # The head camera is a prim under head_link, and a prim does not follow a
    # root that is moved by a WRITE rather than by the simulation (Fabric is
    # off). During the walk it would sit at the start of the room looking at
    # nothing, so its world pose is set from torso_link every frame. head_link
    # itself is not in the articulation -- IsaacLab's G1 USD merges fixed
    # joints and absorbs it into the torso -- so the offset is main.urdf's
    # head_joint transform, folded onto the torso's pose.
    if _TORSO_ID is not None:
        _p = robot.data.body_pos_w[0, _TORSO_ID].cpu().numpy()
        _q = robot.data.body_quat_w[0, _TORSO_ID].cpu().numpy()
        _w, _x, _y, _z = _q
        _R = np.array([
            [1 - 2 * (_y * _y + _z * _z), 2 * (_x * _y - _z * _w), 2 * (_x * _z + _y * _w)],
            [2 * (_x * _y + _z * _w), 1 - 2 * (_x * _x + _z * _z), 2 * (_y * _z - _x * _w)],
            [2 * (_x * _z - _y * _w), 2 * (_y * _z + _x * _w), 1 - 2 * (_x * _x + _y * _y)]])
        _eye = _p + _R @ np.array([0.0039635 + 0.08, 0.0, -0.044 + 0.05])
        _fwd = _R @ np.array([math.cos(math.radians(HEAD_PITCH_DEG)), 0.0,
                              -math.sin(math.radians(HEAD_PITCH_DEG))])
        eye_cam.set_world_poses_from_view(
            eyes=torch.tensor([_eye], dtype=torch.float32, device=sim.device),
            targets=torch.tensor([_eye + _fwd], dtype=torch.float32,
                                 device=sim.device))
    app.update()
    cam.update(0.0)
    head_cam.update(0.0)
    eye_cam.update(0.0)
    frames.append(cam.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8))
    head_frames.append(
        head_cam.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8))
    eye_frames.append(
        eye_cam.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8))


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
    # Legs from the clip, arm from the plan -- for the whole walk.
    #
    # This is how the stack is built, not a trick to dodge a seam:
    # decoupled_wbc.md toggles the lower-body and upper-body policies
    # separately ("menu + left trigger: toggle lower-body policy / menu +
    # right trigger: toggle upper-body policy"), so while GR00T walks the
    # legs the arm is commanded by the manipulation side. Holding the plan's
    # own start pose there means there is no gap between where the arm is and
    # where the plan begins, so nothing has to cross it -- which is what the
    # teleport was.
    #
    # Measured, this is also the only pose that CAN be planned from. The
    # table's collision shape is a solid block floor-to-top, and with the arm
    # hanging at the robot's side 30 of its 183 collision spheres are inside
    # it, 22 mm deep; from there cuRobo finds nothing. The plan's start pose
    # clears the same block by 51 mm, and its hand rides 47 cm above the
    # tabletop, so walking in with it held passes over the desk rather than
    # through it.
    _plan_q = [float(traj[0, k]) for k in range(len(ids))]
    # The waist comes back to the pose the plan assumes over the last stretch
    # of the walk. Snapping it at the end instead put 7 degrees of torso tilt
    # into one physics step and threw the box 51 m; easing it in while the
    # robot is still walking costs nothing and arrives upright.
    _waist = [j for j, n in enumerate(robot.joint_names) if "waist" in n]
    _waist_n = min(45, len(walk["dof"]))
    _waist_from = len(walk["dof"]) - _waist_n
    print(f"[walk] replaying {len(walk['dof'])} frames: legs from the clip, "
          f"arm held at the plan's start pose, waist eased upright over the "
          f"last {_waist_n}")
    for i in range(len(walk["dof"])):
        for k, jid in enumerate(walk_ids):
            tgt_q[0, jid] = float(walk["dof"][i, k])
        for k, jid in enumerate(ids):
            tgt_q[0, jid] = _plan_q[k]
        if i >= _waist_from:
            a = (i - _waist_from + 1) / _waist_n
            for j in _waist:
                tgt_q[0, j] = ((1.0 - a) * float(tgt_q[0, j])
                               + a * float(robot.data.default_joint_pos[0, j]))
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
    # The checkpoint's way: slide the robot onto STAND over 15 frames. It is
    # what the welded pick needs, because the plan is built for STAND and the
    # walk does not land exactly there. It is also the leap -- measured on the
    # delivered video, frames 710-712 carried 9.8x the median frame difference,
    # the largest in the whole clip. Position was down to 2 mm but the heading
    # was not: the walk ends 4.66 deg off, and a body that turns 4.66 deg in
    # half a second with its feet planted is what you see.
    #
    # Kept, because it is the path that reaches HELD. --no-settle leaves the
    # robot where the locomotion policy actually stopped it, for the runs that
    # plan the pick at that pose instead.
    _rp = robot.data.root_pos_w[0].cpu().numpy()
    _rq = robot.data.root_quat_w[0].cpu().numpy()
    _ry = math.degrees(2.0 * math.atan2(float(_rq[3]), float(_rq[0])))
    print(f"[walk] stopped at ({_rp[0]:.4f}, {_rp[1]:.4f}, {_ry:.2f} deg)")
    if NO_SETTLE:
        print("[walk] not settling -- the pick runs from here, nothing is slid")
    else:
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
    # Everything the trajectory does not drive, held where the plan assumes it.
    #
    # The legs keep the pose the walk left them in -- that is what standing
    # looks like. The WAIST does not: the plan's joint angles are measured
    # from torso_link, and the scene is placed against a torso the checkpoint
    # builds from the pelvis assuming "the waist joints are at 0 in the
    # default pose and the plan never moves them". A walk does not end with
    # the waist at 0, and leaving it where the clip put it tilts the torso --
    # measured, the torso quaternion came out [0.7185, 0.0621, 0.0637,
    # -0.6899] against the no-walk run's [-0.7070, -0.0001, -0.0002, 0.7072],
    # about 7 degrees of roll and pitch. Same stand, same object, same arm
    # angles to a millirad, and the hand still lands somewhere else.
    _driven = set(ids)
    # Legs only. The waist is brought upright over the walk and then left to
    # its PD target -- tgt_q already holds the plan's value for it. Writing
    # its joint state every physics step instead, the way the legs are held,
    # made the articulation fight the solver hard enough to throw the box
    # 51 m and 1457 m down. Holding a joint kinematically is for the parts
    # that are doing nothing; the waist carries the arm.
    frozen_ids = ([] if WBC else
                  [robot.find_joints([n])[0][0] for n in robot.joint_names
                   if ("hip" in n or "knee" in n or "ankle" in n)])
    frozen_ids = [j for j in frozen_ids if j not in _driven]
    frozen_q = robot.data.joint_pos[0, frozen_ids].clone().unsqueeze(0)
    if STIFF_LEGS:
        # Let the legs hold themselves instead of being written.
        #
        # Writing their joint state is what disturbs the articulation solver,
        # and the box feels every bit of it: 119.9 mm shoved when the write
        # happens each substep, 33.9 mm once a frame, 1046 mm when it never
        # happens and the robot sags, and an outright blow-up when the writes
        # are spaced out to every fourth frame. Stiffness is the other way to
        # keep a joint still, and it costs the solver nothing extra --
        # IsaacLab exposes it at runtime for exactly this.
        _sf = 50.0
        _st = robot.data.joint_stiffness[:, frozen_ids] * _sf
        _dp = robot.data.joint_damping[:, frozen_ids] * _sf
        robot.write_joint_stiffness_to_sim(_st, joint_ids=frozen_ids)
        robot.write_joint_damping_to_sim(_dp, joint_ids=frozen_ids)
        print(f"[walk] leg gains x{_sf:.0f} so the lower body holds itself")
    frozen_v = torch.zeros_like(frozen_q)
    print(f"[walk] holding {len(frozen_ids)} leg joints still for the reach")

if WALK_ONLY:
    if not NO_VIDEO:
        import imageio.v2 as _iio
        for _fr, _sfx in ((frames, ""), (head_frames, "_wrist"),
                          (eye_frames, "_head")):
            if not _fr:
                continue
            _out = OUT.replace(".mp4", f"{_sfx}.mp4")
            _w = _iio.get_writer(_out, fps=FPS, quality=8)
            for _f in _fr:
                _w.append_data(_f)
            _w.close()
            print(f"[play] wrote {_out}: {len(_fr)} frames")
    _got = np.array([robot.data.joint_pos[0, j].item() for j in ids])
    _want = np.array([float(traj[0, k]) for k in range(len(ids))])
    _rp = robot.data.root_pos_w[0].cpu().numpy()
    print(f"[walk] end-of-walk arm error: max {np.abs(_got-_want).max()*1000:.1f} mrad, "
          f"per joint {np.round((_got-_want)*1000, 1)}")
    print(f"[walk] end-of-walk root {np.round(_rp, 4)} vs STAND "
          f"{np.round(np.array(STAND), 4)}  gap "
          f"{np.linalg.norm(_rp[:2]-np.array(STAND[:2]))*1000:.1f} mm")
    print(f"[walk] walk-only: {len(frames)} frames, stopping before the pick")
    sys.stdout.flush()
    os._exit(0)

sonic = None
if SONIC:
    # The reference SONIC tracks: the pose the robot is actually standing in,
    # with the right arm walking through cuRobo's plan. SONIC is a tracking
    # model -- give it a motion and it produces the whole body that executes
    # it, legs included, which is the part this pipeline has been faking.
    from sonic_control import (SonicTracker, JOINTS as SJ, CONTROL_DT,
                               DEFAULT as SONIC_DEFAULT)
    sonic_ids = [robot.find_joints([n])[0][0] for n in SJ]
    # Stand the robot in the pose the walk actually left it in.
    #
    # SONIC's default_angles is a bent-knee stance (hips -0.312, knees 0.669)
    # and it has its own pelvis height. Putting the pelvis at the height the
    # plan was made for and the joints at that stance is two different robots
    # at once: measured, the torso settled at 0.7195 instead of 0.79 and
    # toppled. The walk's last frame is a leg pose that genuinely stands at
    # this height, which is the whole point of having walked there.
    _pose = SONIC_DEFAULT.copy()
    _ej = (os.path.splitext(sys.argv[sys.argv.index("--legs-from") + 1])[0]
           + "_end.json") if "--legs-from" in sys.argv else None
    if _ej and os.path.exists(_ej):
        _end = json.load(open(_ej))
        for _c, _n in enumerate(SJ):
            if _n in _end:
                _pose[_c] = float(_end[_n])
        print(f"[sonic] standing in the walk's last pose from {os.path.basename(_ej)}")
    else:
        print("[sonic] standing in SONIC's own default stance")
    for _c, _j in enumerate(sonic_ids):
        tgt_q[0, _j] = float(_pose[_c])
    robot.write_joint_state_to_sim(tgt_q, torch.zeros_like(tgt_q))
    robot.set_joint_position_target(tgt_q)
    robot.write_data_to_sim()
    sim.step()
    robot.update(sim.get_physics_dt())
    _now = robot.data.joint_pos[0, sonic_ids].cpu().numpy().astype(np.float64)
    _ref = np.tile(_now, (traj.shape[0], 1))
    # ids[] are the joints the plan drives, in the plan's own order; map the
    # ones SONIC knows about onto its columns.
    _plan_to_sonic = {}
    for _k, _jid in enumerate(ids):
        _nm = robot.joint_names[_jid]
        if _nm in SJ:
            _plan_to_sonic[_k] = SJ.index(_nm)
    for _k, _c in _plan_to_sonic.items():
        _ref[:, _c] = traj[:, _k]
    # SONIC's clock is 50 Hz and the plan's is FPS (30). motion_reference.md is
    # explicit -- "each row is one timestep at 50 Hz" -- and the encoder's
    # lookahead is ten frames at twenty milliseconds, so handing it rows at 30
    # fps stretches the future it is shown by a factor of 1.67. Resample.
    _n50 = max(2, int(round(traj.shape[0] / FPS / CONTROL_DT)))
    _src = np.arange(traj.shape[0]) / FPS
    _dst = np.arange(_n50) * CONTROL_DT
    _ref = np.stack([np.interp(_dst, _src, _ref[:, c]) for c in range(_ref.shape[1])],
                    axis=1)
    print(f"[sonic] reference resampled {traj.shape[0]} frames at {FPS} fps "
          f"-> {_n50} at {1/CONTROL_DT:.0f} Hz")
    # The reference's own root pose, not a placeholder. The deployment reads
    # both off the motion itself (BodyPositions(frame)[0] for the height, the
    # anchor quaternion for motion_anchor_orientation), and telling SONIC the
    # reference faces +x while the robot faces -90 degrees is a mismatch it
    # would have to fight for the whole clip.
    _rq = robot.data.root_quat_w[0].cpu().numpy().astype(np.float64)
    _rz = float(robot.data.root_pos_w[0, 2].item())
    sonic = SonicTracker(_ref,
                         ref_quat=np.tile(_rq, (len(_ref), 1)),
                         ref_root_z=np.full(len(_ref), _rz))
    print(f"[sonic] reference anchored at root z {_rz:.4f}, "
          f"quat {np.round(_rq, 4)}")
    sonic.prime(robot.data.joint_pos[0, sonic_ids].cpu().numpy().astype(np.float64),
                robot.data.joint_vel[0, sonic_ids].cpu().numpy().astype(np.float64),
                _rq, robot.data.root_ang_vel_b[0].cpu().numpy().astype(np.float64))
    sonic_dec = max(1, round(CONTROL_DT / sim.get_physics_dt()))
    sonic_t = 0
    print(f"[sonic] tracking {traj.shape[0]} reference frames, "
          f"{len(_plan_to_sonic)} of them driven by the plan, "
          f"every {sonic_dec} physics steps")
    # The hand joints are not SONIC's; the plan still drives them.
    sonic_hand_k = [k for k in range(len(ids)) if k not in _plan_to_sonic]


def _sonic_step(_unused=None):
    """One 50 Hz tick of the tracker; writes all 29 targets."""
    global sonic_t
    q = robot.data.joint_pos[0, sonic_ids].cpu().numpy().astype(np.float64)
    dq = robot.data.joint_vel[0, sonic_ids].cpu().numpy().astype(np.float64)
    quat = robot.data.root_quat_w[0].cpu().numpy().astype(np.float64)
    om = robot.data.root_ang_vel_b[0].cpu().numpy().astype(np.float64)
    out = sonic.step(min(sonic_t, sonic.T - 1), q, dq, quat, om)
    sonic_t += 1
    for _c, _j in enumerate(sonic_ids):
        tgt_q[0, _j] = float(out[_c])


wbc = None
if WBC:
    # GR00T's balance policy, stepped at its own rate. sim2mujoco runs 5 ms
    # physics with a decimation of 4, so the policy sees 50 Hz; Isaac's dt is
    # whatever plan_scene set, so the decimation is computed rather than
    # copied.
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from wbc_balance import BalanceWBC, LEG_WAIST, POLICY_JOINTS
    wbc = BalanceWBC()
    wbc_ids = [robot.find_joints([n])[0][0] for n in LEG_WAIST]
    # Exactly the 29 joints the policy was trained on, in its own order. This
    # robot has 43 -- the Dex3-1 fingers are not among them.
    obs_ids = [robot.find_joints([n])[0][0] for n in POLICY_JOINTS]
    # Everything the trajectory does not command and the policy does not
    # drive: the left arm and its hand.
    _driven = set(wbc_ids) | set(ids)
    hold_ids = [j for j in range(robot.num_joints) if j not in _driven]
    wbc_dec = max(1, round(0.02 / sim.get_physics_dt()))
    print(f"[wbc] holding {len(hold_ids)} uncommanded joints at their measured pose")
    _hold_pending = True
    print(f"[wbc] balance policy on {len(wbc_ids)} joints, "
          f"observing {len(obs_ids)}, every {wbc_dec} physics steps")


def _wbc_hold_uncommanded():
    """Hold every upper-body joint the plan does not command.

    GR00T's own rule, from decoupled_wbc's run_g1_control_loop.py: when there
    is no command for the upper body it does not fall back to a configured
    pose, it targets what the robot is currently holding --

        upper_body_cmd = {"target_upper_body_pose":
                          obs["q"][robot_model.get_joint_group_indices("upper_body")]}

    The plan drives the right arm and says nothing about the left, so the left
    arm is exactly that case. Left at Isaac's default it swings forward through
    the desk; held the way GR00T holds an uncommanded joint, it stays where the
    walk left it.
    """
    for _j in hold_ids:
        tgt_q[0, _j] = robot.data.joint_pos[0, _j]


def _wbc_step(nav=None):
    """One policy tick: read the robot, write leg and waist targets."""
    global _hold_pending
    if _hold_pending:
        _wbc_hold_uncommanded()
        _hold_pending = False
    q = robot.data.joint_pos[0, obs_ids].cpu().numpy().astype(np.float32)
    dq = robot.data.joint_vel[0, obs_ids].cpu().numpy().astype(np.float32)
    quat = robot.data.root_quat_w[0].cpu().numpy().astype(np.float32)
    omega = robot.data.root_ang_vel_b[0].cpu().numpy().astype(np.float32)
    tgt = wbc.step(q, dq, quat, omega, nav)
    for _k, _j in enumerate(wbc_ids):
        tgt_q[0, _j] = float(tgt[_k])


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
for _si in range(120):
    if SONIC:
        if _si % sonic_dec == 0:
            _sonic_step()
            sonic_t = 0          # settling holds frame 0, it does not advance
        for _k in sonic_hand_k:
            tgt_q[0, ids[_k]] = float(traj[0, _k])
        robot.set_joint_position_target(tgt_q)
    elif WBC:
        if _si % wbc_dec == 0:
            _wbc_step()
        robot.set_joint_position_target(tgt_q)
    elif walk is not None:
        # There is no fix_root_link in a walk run, so an unheld robot spends
        # these 120 steps sagging on its own legs and the arm then reaches
        # from a body that is no longer on the stand. The trajectory loop
        # below already holds it; the settle has to hold it too.
        robot.write_root_state_to_sim(stand_root)
        if FREEZE_LEGS:
            robot.write_joint_state_to_sim(frozen_q, frozen_v,
                                           joint_ids=frozen_ids)
    robot.write_data_to_sim()
    sim.step()
robot.update(sim.get_physics_dt())

if DRIVE is not None and wbc is not None:
    # Walk there. Same loop, same policy stack, same observation -- the only
    # thing that changes on arrival is that the command goes to zero and
    # g1_gear_wbc_policy picks the standing network instead of the walking
    # one. Nothing is welded, nothing is frozen, nothing is slid.
    #
    # The command itself is a velocity toward the goal in the body frame.
    # decoupled_wbc's own sources for it are a keyboard and a teleop stream,
    # both of which hand the policy [vx, vy, wz] directly, so turning a goal
    # into that is ours; the goal is not.
    _gx, _gy, _gyaw = DRIVE["goal"]
    _ARRIVE, _YAWTOL = 0.06, math.radians(4.0)
    _nav = np.zeros(3, np.float32)
    _arrived_for = 0
    _drove = 0
    _every = max(1, round((1.0 / FPS) / sim.get_physics_dt()))
    print(f"[drive] walking to ({_gx:.3f}, {_gy:.3f}, "
          f"{math.degrees(_gyaw):.1f} deg) under the policy")
    for _step in range(int(60.0 / sim.get_physics_dt())):
        if _step % wbc_dec == 0:
            _p = robot.data.root_pos_w[0].cpu().numpy()
            _q = robot.data.root_quat_w[0].cpu().numpy()
            _y = 2.0 * math.atan2(float(_q[3]), float(_q[0]))
            _dx, _dy = _gx - float(_p[0]), _gy - float(_p[1])
            _c, _sn = math.cos(_y), math.sin(_y)
            _ex, _ey = _c * _dx + _sn * _dy, -_sn * _dx + _c * _dy
            _dist = math.hypot(_ex, _ey)
            _dyaw = math.atan2(math.sin(_gyaw - _y), math.cos(_gyaw - _y))
            if _dist > _ARRIVE:
                _nav[0] = float(np.clip(1.2 * _ex, -0.5, 0.5))
                _nav[1] = float(np.clip(1.2 * _ey, -0.3, 0.3))
                _nav[2] = float(np.clip(1.5 * math.atan2(_ey, _ex), -0.6, 0.6))
            elif abs(_dyaw) > _YAWTOL:
                _nav[:] = (0.0, 0.0, float(np.clip(1.2 * _dyaw, -0.6, 0.6)))
            else:
                _nav[:] = 0.0
            _wbc_step(_nav)
            if _dist <= _ARRIVE and abs(_dyaw) <= _YAWTOL:
                _arrived_for += 1
            else:
                _arrived_for = 0
        for _k in range(len(ids)):
            tgt_q[0, ids[_k]] = float(traj[0, _k])
        robot.set_joint_position_target(tgt_q)
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim.get_physics_dt())
        _drove += 1
        if _drove % _every == 0:
            _shoot()
        # Half a second standing still at the goal, then the plan starts from
        # exactly the pose the policy left the robot in.
        if _arrived_for >= 25:
            break
    _p = robot.data.root_pos_w[0].cpu().numpy()
    _q = robot.data.root_quat_w[0].cpu().numpy()
    _y = math.degrees(2.0 * math.atan2(float(_q[3]), float(_q[0])))
    print(f"[drive] {_drove} steps; arrived at ({_p[0]:.4f}, {_p[1]:.4f}, "
          f"{_y:.2f} deg), {math.hypot(_p[0]-_gx, _p[1]-_gy)*1000:.1f} mm and "
          f"{abs(_y - math.degrees(_gyaw)):.2f} deg from the goal")

# What the arm is actually standing on when the plan starts. The same
# trajectory picks the box up without a walk and tips it with one, so the two
# runs have to differ somewhere by the time this line is reached -- print it
# rather than reason about it.
_ti = robot.find_bodies(["torso_link"])[0]
if _ti:
    _t = robot.data.body_pos_w[0, _ti[0]].cpu().numpy()
    _tq = robot.data.body_quat_w[0, _ti[0]].cpu().numpy()
    print(f"[start] torso {np.round(_t, 4)} quat {np.round(_tq, 4)}")
_got = np.array([robot.data.joint_pos[0, j].item() for j in ids])
_want = np.array([float(traj[0, k]) for k in range(len(ids))])
print(f"[start] arm error max {np.abs(_got - _want).max()*1000:.2f} mrad, "
      f"per joint {np.round((_got - _want) * 1000, 1)}")
if target_body is not None:
    target_body.update(0.0)
    print(f"[start] object {np.round(target_body.data.root_pos_w[0].cpu().numpy(), 4)}")

substeps = max(1, round((1.0 / FPS) / sim.get_physics_dt()))
print(f"[play] {substeps} physics steps per trajectory frame")
for i in range(traj.shape[0]):
    for k, jid in enumerate(ids):
        tgt_q[0, jid] = float(traj[i, k])
    robot.set_joint_position_target(tgt_q)
    if (not WBC and walk is not None and FREEZE_LEGS
            and not LEGS_SETTLE_ONLY
            and i % LEG_HOLD_EVERY == 0):
        # Once per rendered frame, not once per physics substep.
        #
        # With a clip there is no fix_root_link, so the pelvis has to be
        # written -- and the legs with it, or they keep pushing on the floor,
        # the write cancels the reaction, and the two fight at the physics
        # rate (the buzz the legs do once the robot stops). But doing that
        # write inside every substep also disturbs the articulation solver
        # mid-step, and the hand pays for it: measured, the fingers had only
        # reached thumb -0.38 / index 0.38 where the no-walk run had -1.46 /
        # 1.63, so the box was shoved away before they closed on it. Dropping
        # the write entirely is not an option either -- the unheld robot then
        # blows up (box thrown 3025 m). Once per frame holds the lower body
        # and leaves the substeps to the solver.
        robot.write_joint_state_to_sim(frozen_q, frozen_v,
                                       joint_ids=frozen_ids)
    for _ss in range(substeps):
        if SONIC:
            if (i * substeps + _ss) % sonic_dec == 0:
                _sonic_step()
                for _k in sonic_hand_k:
                    tgt_q[0, ids[_k]] = float(traj[i, _k])
                robot.set_joint_position_target(tgt_q)
        elif WBC:
            if (i * substeps + _ss) % wbc_dec == 0:
                _wbc_step()
                robot.set_joint_position_target(tgt_q)
        elif walk is not None:
            # The pelvis every substep, the legs only once a frame. Holding
            # the pelvis is cheap -- it is one body, not a joint the solver is
            # iterating on -- and letting it drift between substeps lets the
            # arm's reaction push the whole robot and then snap it back. The
            # leg joint writes are the ones that disturb the solver, and the
            # hand is what pays for that.
            robot.write_root_state_to_sim(stand_root)
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
    _shoot()

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
EYE_OUT = OUT.replace(".mp4", "_head.mp4")
imageio.mimsave(EYE_OUT, eye_frames, fps=FPS, quality=8)
print(f"[play] wrote {EYE_OUT}: {len(eye_frames)} frames")
sys.stdout.flush()
os._exit(0)
