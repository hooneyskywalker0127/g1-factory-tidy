# Which of GraspGenX's grasps does the hand actually hold, from the body that
# reaches them? GraspGenX validates its grasps by replaying them under
# physics; this does the same in the cell, with the whole-body reach cuRobo
# solved for each candidate: body kinematic (as the walk replay is), fingers
# simulated, box simulated. Each candidate: kneel -> grasp, close, lift 15 cm,
# read the box.
#
#   python grasp/test_grasps_in_isaac.py SCENE.npy CLIP.pkl REACH_ALL.npz [--top N]
import json
import os
import sys

from isaaclab.app import AppLauncher

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "map"))
sys.path.insert(0, os.path.join(REPO, "grasp"))
SCENE, CLIP, ALL = sys.argv[1:4]
TOP = int(sys.argv[sys.argv.index("--top") + 1]) if "--top" in sys.argv else 30
SNAP = os.environ.get("SNAP")            # SNAP=1: PNGs of the hand at grasp / close / lift, two views
app = AppLauncher(headless=True, enable_cameras=bool(SNAP)).app

import math  # noqa: E402
import joblib  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
import omni.usd  # noqa: E402
from isaaclab.assets import Articulation, RigidObject, RigidObjectCfg  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab_assets.robots.unitree import G1_29DOF_CFG  # noqa: E402
from pxr import Sdf, UsdGeom  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402

from plan_scene import FINGER_MU, build as build_plan_scene, torso_pose, keep_only_hand_collisions, floor_slab  # noqa: E402
from props import spawn_props  # noqa: E402
from build_reach_reference import HAND_OPEN, HAND_CLOSED, HAND_NAMES, PALM_LINK, HAND_KEEP, mujoco_names, robot_cfg, stiffen_mimic  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402

meta = json.load(open(os.path.splitext(SCENE)[0] + ".json"))
FPS = 30
sim = SimulationContext(sim_utils.SimulationCfg(dt=1.0 / 1000.0, device="cpu",
                                                physx=sim_utils.PhysxCfg(enable_ccd=True)))
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
stage.DefinePrim("/World/Cell", "Xform").GetReferences().AddReference(
    Sdf.Reference(os.path.join(REPO, "map", "cell.usd")))
sim_utils.DomeLightCfg(intensity=900.0).func("/World/light", sim_utils.DomeLightCfg(intensity=900.0))
for _ in range(3):
    app.update()
spawn_props(stage, app)
cfg = robot_cfg(G1_29DOF_CFG.replace(prim_path="/World/G1"), sim_utils, ImplicitActuatorCfg)
cfg.spawn = cfg.spawn.replace(collision_props=sim_utils.CollisionPropertiesCfg(
    contact_offset=0.002, rest_offset=0.0))   # fingers as thin as they are: no phantom floor contact
cfg.spawn = cfg.spawn.replace(articulation_props=sim_utils.ArticulationRootPropertiesCfg(
    enabled_self_collisions=False, solver_position_iteration_count=12,
    solver_velocity_iteration_count=4, fix_root_link=(os.environ.get("FIX_ROOT", "0") == "1")))
# ARM_KP_SCALE: with the body on its PD drives (PD_BODY) the stock arm gains let an
# outstretched arm sag -- measured on the crate approach, the left palm 10 cm under
# its reference while the IK was within 24 mm -- and the sagging hand dragged the crate.
if os.environ.get("ARM_KP_SCALE") and "arms" in cfg.actuators:
    _k = float(os.environ["ARM_KP_SCALE"])
    cfg.actuators["arms"] = cfg.actuators["arms"].replace(
        stiffness={n: v * _k for n, v in cfg.actuators["arms"].stiffness.items()} if isinstance(cfg.actuators["arms"].stiffness, dict) else cfg.actuators["arms"].stiffness * _k,
        damping={n: v * _k ** 0.5 for n, v in cfg.actuators["arms"].damping.items()} if isinstance(cfg.actuators["arms"].damping, dict) else cfg.actuators["arms"].damping * _k ** 0.5)
if os.environ.get("HAND") != "inspire": cfg.actuators["hands"] = cfg.actuators["hands"].replace(effort_limit=1.4, velocity_limit=12.0,
    # HAND_DAMPING: the body is written every substep, and PhysX then
    # reports garbage joint velocities on the simulated fingers (-2.5 to
    # -7.7 rad/s on joints that are not moving). The stock damping 2 turns
    # that into +-15 Nm, far past the 1.4 Nm cap, so the finger torque was
    # saturated by noise; a damping-only (velocity-mode) close is hopeless
    # here for the same reason. NVIDIA's own IsaacLab G1 hand runs 0.2.
    damping=float(os.environ.get("HAND_DAMPING", "0.05")))
SPAWN_Z = cfg.init_state.pos[2]          # the scene is placed against this, not the clip's start
clip = list(joblib.load(CLIP).values())[0]
p0, q0 = np.asarray(clip["root_trans_offset"])[0], np.asarray(clip["root_rot"])[0]
cfg.init_state = cfg.init_state.replace(pos=tuple(float(v) for v in p0),
                                        rot=(float(q0[3]), float(q0[0]), float(q0[1]), float(q0[2])))
robot = Articulation(cfg)
stiffen_mimic(stage)   # HAND=inspire: rigid four-bar fingertips (build_reach_reference.py)
if os.environ.get("BODY_COLLISION", "0") == "0":
    _kept = keep_only_hand_collisions(stage, keep=HAND_KEEP)
    print(f"[test] body does not collide with the cell; hand links kept: {len(_kept)} ({_kept[:2]}...)")
_fm = sim_utils.RigidBodyMaterialCfg(static_friction=FINGER_MU, friction_combine_mode="max", dynamic_friction=FINGER_MU, restitution=0.0)
_fm.func("/World/G1/FingerMaterial", _fm)
sim_utils.bind_physics_material("/World/G1", "/World/G1/FingerMaterial")
build_plan_scene(stage, app, meta, torso_pose((-1.30, -0.60, SPAWN_Z), math.radians(-90.0)))
floor_slab(stage)
box = RigidObject(RigidObjectCfg(prim_path=os.environ.get("TARGET_PRIM", "/World/GraspTarget"), spawn=None))
# The rigid body's origin is the mesh origin, which for the lying tools sits
# 16 cm above the bottom of the mesh -- a can knocked over by the fingers
# moves that origin 10-15 cm down without going anywhere. What is measured
# below is the mesh centroid, carried with the body's pose.
import trimesh as _tm  # noqa: E402
_c_off = (torch.zeros(3, device=sim.device) if os.environ.get("TARGET_PRIM")
          else torch.tensor(_tm.load(meta["object"]["mesh"], force="mesh").centroid, dtype=torch.float32, device=sim.device))


def obj_centre():
    box.update(sim.get_physics_dt())
    q = box.data.root_quat_w[0]                       # w x y z
    w, x, y, z = q[0], q[1], q[2], q[3]
    Rm = torch.stack([torch.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)]),
                      torch.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)]),
                      torch.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)])])
    return (box.data.root_pos_w[0] + Rm @ _c_off).cpu().numpy()
cams = []
if SNAP:
    from isaaclab.sensors import Camera, CameraCfg
    UsdGeom.Xform.Define(stage, "/Render")
    for nm in ("A", "B"):
        cams.append(Camera(CameraCfg(prim_path=f"/Render/Snap{nm}", update_period=0.0, width=960, height=720,
                                     data_types=["rgb"], spawn=sim_utils.PinholeCameraCfg(focal_length=24.0, clipping_range=(0.05, 20.0)))))
sim.reset()
# 1.5 s, not 0.12: a hammer lying on the side of its head was still rolling
# to rest after 120 ms, and every candidate then read "box at close -0.016"
# -- the settle, not the fingers.
for _ in range(1500):
    sim.step()
box.update(0.0)
box0 = box.data.root_state_w[:1].clone()
print(f"[test] box at {np.round(box0[0, :3].cpu().numpy(), 3)}")
# How far the object moved from where it was placed (and seen by the camera)
# to where physics lets it rest: the capture is made of the placed pose.
_T = torso_pose((-1.30, -0.60, SPAWN_Z), math.radians(-90.0)) @ np.asarray(meta["object"]["transform_in_torso"])
_q0 = R.from_matrix(_T[:3, :3]); _qs = box0[0, 3:7].cpu().numpy()
_q1 = R.from_quat([_qs[1], _qs[2], _qs[3], _qs[0]])
print(f"[test] settle: moved {np.round(box0[0, :3].cpu().numpy() - _T[:3, 3], 3)} m, "
      f"rotated {np.degrees((_q0.inv() * _q1).magnitude()):.1f} deg from the placed pose")

names = mujoco_names()
body_ids = [robot.find_joints([n])[0][0] for n in names]
# ARM_PD=1: the right arm is a PD-driven articulation like everything the
# desk pick that held was replayed with, not a fixture written every substep.
# A hand written into place cannot give when the fingers press the object
# against it; the contact solver then throws the object instead.
RIGHT_ARM = ["right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
             "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"]
if os.environ.get("ARM_PD"):
    _arm = {robot.find_joints([n])[0][0] for n in RIGHT_ARM}
    write_ids = [j for j in body_ids if j not in _arm]
    print("[test] right arm PD-driven, not written")
else:
    write_ids = body_ids
hand_names = list(HAND_NAMES)
hand_ids = [robot.find_joints([n])[0][0] for n in hand_names]
d = np.load(ALL)
jn = [str(n) for n in d["joint_names"]]
col = [jn.index(n) for n in names]
bcol = [jn.index(f"base_j_{a}") for a in ("x", "y", "z", "xtheta", "ytheta", "ztheta")]
n_go, n_lift = int(d["n_go"]), int(d["n_lift"])
order = np.argsort(d["err"][:, n_go - 1])[:TOP]
if "--only" in sys.argv:
    order = [int(v) for v in sys.argv[sys.argv.index("--only") + 1].split(",")]
if "--order" in sys.argv:                              # rank_handle.py's list, first TOP of it
    order = [int(v) for v in open(sys.argv[sys.argv.index("--order") + 1]).read().strip().split(",")][:TOP]
substeps = round((1.0 / FPS) / sim.get_physics_dt())
tgt = robot.data.default_joint_pos.clone()
zero = torch.zeros_like(tgt)
kneel_dof = np.asarray(clip["dof"])[-1]
kneel_root = np.concatenate([np.asarray(clip["root_trans_offset"])[-1],
                             np.asarray(clip["root_rot"])[-1][[3, 0, 1, 2]]])


SQUEEZING = [False, None, None]


def snap(tag):
    """Two close views of the hand and the object, saved as PNG."""
    if not cams:
        return
    from PIL import Image
    box.update(sim.get_physics_dt())
    o = box.data.root_pos_w[0].cpu().numpy().copy()
    _b = robot.find_bodies([PALM_LINK[os.environ.get("SNAP_HAND", "right")]])[0]
    palm = robot.data.body_pos_w[0, _b[0]].cpu().numpy()
    at = palm.copy() if os.environ.get("SNAP_HAND") else 0.5 * (o + palm); at[2] = (palm[2] - 0.05) if os.environ.get("SNAP_HAND") else min(o[2], palm[2]) - 0.02
    for c, eye in zip(cams, (at + np.array([-0.45, 0.35, 0.25]), at + np.array([0.45, -0.35, 0.25]))):
        c.set_world_poses_from_view(torch.tensor([eye], dtype=torch.float32, device=sim.device),
                                    torch.tensor([at], dtype=torch.float32, device=sim.device))
    for _ in range(3):
        sim.render()
    for i, c in enumerate(cams):
        c.update(sim.get_physics_dt())
        img = c.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8)
        Image.fromarray(img).save(os.path.join(os.path.dirname(ALL), f"snap_{tag}_{'AB'[i]}.png"))
    print(f"[test]    snapshot {tag}")


PD_BODY = os.environ.get("PD_BODY", "1") == "1"


def put(root7, dof29, hands14):
    for k, j in enumerate(body_ids):
        tgt[0, j] = float(dof29[k])
    for k, j in enumerate(hand_ids):
        tgt[0, j] = float(hands14[k])
    rs = torch.tensor([[*root7, 0, 0, 0, 0, 0, 0]], dtype=torch.float32, device=sim.device)
    if PD_BODY:
        # The way the one grasp that held was replayed (the desk box, pelvis
        # welded, every joint on its PD drive): no joint is written, the root
        # is placed once per frame, and PhysX sees one consistent
        # articulation. Written every substep, the body gave the simulated
        # fingers garbage velocities and contact kicks (see DIAGNOSIS.md).
        robot.write_root_state_to_sim(rs)
        for _ in range(substeps):
            robot.update(sim.get_physics_dt())
            robot.set_joint_position_target(tgt)
            if SQUEEZING[0]:
                robot.set_joint_velocity_target(SQUEEZING[1], joint_ids=SQUEEZING[2])
            robot.write_data_to_sim()
            sim.step()
        robot.update(sim.get_physics_dt())
        return
    # Targets every physics substep, not once per frame: a joint target set
    # once and followed by 33 write_data_to_sim/step calls acts on the first
    # step only (measured: velocity-mode fingers moved 0.0075 rad/s for a
    # 0.25 rad/s target, 1/33). Every close this evening ran like that.
    for _ in range(substeps):
        # write_joint_state_to_sim with a joint subset still pushes the WHOLE
        # joint_pos/joint_vel buffer to PhysX (articulation.py:616, 646), so
        # the simulated fingers are reset every substep to the values the
        # buffer held at the last update() -- once per frame, they moved at
        # 1/33 of the drive (0.0075 rad/s for 0.25). Refresh the buffer first.
        robot.update(sim.get_physics_dt())
        robot.set_joint_position_target(tgt)
        if SQUEEZING[0]:
            robot.set_joint_velocity_target(SQUEEZING[1], joint_ids=SQUEEZING[2])
        robot.write_root_state_to_sim(rs)
        robot.write_joint_state_to_sim(tgt[:, write_ids], zero[:, write_ids], joint_ids=write_ids)
        robot.write_data_to_sim()
        sim.step()
    robot.update(sim.get_physics_dt())


results = []
for k in order:
    q = d["q"][k]
    roots = np.concatenate([q[:, bcol[:3]], R.from_euler("XYZ", q[:, bcol[3:]]).as_quat()[:, [3, 0, 1, 2]]], axis=1)
    dofs = q[:, col]
    # reset: box back, body back to the kneel
    box.write_root_state_to_sim(box0)
    for _ in range(15):
        put(kneel_root, kneel_dof, HAND_OPEN)
    z_ref = float(obj_centre()[2])                 # where it rests before this candidate's reach
    if os.environ.get("TEST_VERBOSE"):
        print(f"[test]    object after the reset to the kneel: {np.round(obj_centre(), 3)} (placed at {np.round(box0[0, :3].cpu().numpy(), 3)})")
    for i in range(n_go):
        put(roots[i], dofs[i], HAND_OPEN)
        if os.environ.get("TEST_VERBOSE") and i % 15 == 14:
            _o = obj_centre(); _b = robot.find_bodies([PALM_LINK["right"]])[0][0]; _l = robot.find_bodies([PALM_LINK["left"]])[0][0]
            print(f"[test]    approach frame {i:3d}: object {np.round(_o, 3)}  right palm {np.round(robot.data.body_pos_w[0, _b].cpu().numpy(), 2)}  left palm {np.round(robot.data.body_pos_w[0, _l].cpu().numpy(), 2)}")
    # Where the open fingers are at the grasp pose, before closing: a tip
    # below the floor means the hand is already fighting the floor and the
    # close will launch the object rather than hold it (measured: the clamp
    # flew 38 cm the frame the fingers began to close).
    _tips = {}
    for _ln in (("right_hand_index_1_link", "right_hand_middle_1_link", "right_hand_thumb_2_link", PALM_LINK["right"]) if os.environ.get("HAND") != "inspire" else ("R_index_intermediate", "R_pinky_intermediate", "R_thumb_distal", PALM_LINK["right"])):
        _b = robot.find_bodies([_ln])[0]
        _tips[("palm" if _ln == PALM_LINK["right"] else _ln.replace("right_hand_", "").replace("_link", ""))] = robot.data.body_pos_w[0, _b[0]].cpu().numpy()
    _tip_min = min(v[2] for k, v in _tips.items() if k != "palm")
    if os.environ.get("TEST_VERBOSE"):
        # the links' actual meshes, not their origins: how low the flesh goes
        from pxr import Usd, UsdGeom as _UG
        _bc = _UG.BBoxCache(Usd.TimeCode.Default(), [_UG.Tokens.default_, _UG.Tokens.render], useExtentsHint=False)
        _rows = []
        for _pr in Usd.PrimRange(stage.GetPrimAtPath("/World/G1"), Usd.TraverseInstanceProxies()):
            _n = _pr.GetName()
            if _pr.GetTypeName() == "Xform" and _n.startswith("right_hand_") and _n.endswith("_link") \
                    and any(k in _n for k in ("palm", "index_1", "middle_1", "thumb_1", "thumb_2")):
                _r = _bc.ComputeWorldBound(_pr).ComputeAlignedRange()
                _rows.append(f"{_n.replace('right_hand_', '')} [{str(_pr.GetPath()).count('/')}] z {_r.GetMin()[2]:+.3f}..{_r.GetMax()[2]:+.3f}")
        print("[test]    link meshes at grasp: " + "; ".join(_rows))
    if os.environ.get("TEST_VERBOSE"):
        _rh = hand_ids[len(hand_ids) // 2:]
        print(f"[test]    right finger q {np.round(robot.data.joint_pos[0, _rh].cpu().numpy(), 2)} target {np.round(tgt[0, _rh].cpu().numpy(), 2)}; "
              f"wrist joints {np.round([robot.data.joint_pos[0, robot.find_joints([n])[0][0]].item() for n in ('right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint')], 2)}; "
              f"tips {({k: np.round(v, 3).tolist() for k, v in _tips.items()})}")
    snap(f"{int(k)}_grasp")
    _bp0 = obj_centre()
    print(f"[test]    at grasp: palm z {_tips['palm'][2]:+.3f}, lowest fingertip z {_tip_min:+.3f}, "
          f"palm - object {np.round(_tips['palm'] - _bp0, 3)}")
    # Close and lift the way the desk pick that held did: the fingers get
    # hold_after_close_frames (150 at 30 fps, 5 s) to settle on the object
    # before the lift, and the lift itself is slow (20 cm over 7 s there).
    # A 0.8 s close and a 15 cm/s lift let a pinch on a 39 mm box slip.
    SLOW = "--slow" in sys.argv
    n_ramp, n_settle, lift_x = (30, 150, 4) if SLOW else (20, 5, 1)
    # CLOSE_ORDER: thumb_first | fingers_first | together. Closing everything
    # at once pushed a standing bottle over with whichever finger touched
    # first (40 of 40 candidates); a hand cages before it squeezes.
    order = os.environ.get("CLOSE_ORDER", "together")
    thumb = np.array([("thumb" in n and n.startswith(("right", "R_"))) for n in hand_names], bool)      # right thumb joints
    # CLOSE_MODE=velocity: the close GraspGenX runs the Dex3 with
    # (end2end/robots/g1_right_arm.yaml gripper_control_mode: velocity,
    # dynamic_playback.py:641-661): stiffness 0, damping only, each joint
    # commanded at 0.25 rad/s towards closed, and the squeeze is kept on
    # through the hold and the lift. Its comment: "position mode snaps the
    # fingers to the closed angles and they bat the object away" -- which is
    # what every position-mode close here did (0/154 box, 0/40 hammer).
    # With stiffness 20 and the real 1.4 Nm effort cap the finger is torque-
    # saturated the whole way and arrives at the object at the velocity
    # limit. thumb_0 stays in position mode (GraspGenX gives it 0 velocity).
    velocity_mode = os.environ.get("CLOSE_MODE", "position") == "velocity"
    close_vel = float(os.environ.get("CLOSE_VEL", "0.25"))
    close_kd = float(os.environ.get("CLOSE_KD", "8.0"))
    _rh = hand_ids[len(hand_ids) // 2:]
    _vel_ids = [j for j, n in zip(_rh, hand_names[len(hand_names) // 2:]) if "thumb_0" not in n]
    _vel = torch.zeros(1, len(_vel_ids), device=sim.device)
    for c, (j, n) in enumerate([(j, n) for j, n in zip(_rh, hand_names[len(hand_names) // 2:]) if "thumb_0" not in n]):
        k7 = hand_names.index(n)
        _vel[0, c] = close_vel * float(np.sign(HAND_CLOSED[k7] - HAND_OPEN[k7]))
    _stiff0 = robot.data.joint_stiffness[:, _vel_ids].clone()
    _damp0 = robot.data.joint_damping[:, _vel_ids].clone()

    def squeeze(on):
        SQUEEZING[0], SQUEEZING[1], SQUEEZING[2] = on, _vel, _vel_ids
        if on:
            robot.write_joint_stiffness_to_sim(0.0, joint_ids=_vel_ids)
            robot.write_joint_damping_to_sim(close_kd, joint_ids=_vel_ids)
            robot.set_joint_velocity_target(_vel, joint_ids=_vel_ids)
        else:
            robot.write_joint_stiffness_to_sim(_stiff0, joint_ids=_vel_ids)
            robot.write_joint_damping_to_sim(_damp0, joint_ids=_vel_ids)
            robot.set_joint_velocity_target(torch.zeros_like(_vel), joint_ids=_vel_ids)
    if velocity_mode:
        squeeze(True)
    for i in range(n_ramp + n_settle):
        a = 0.0 if os.environ.get("NO_CLOSE") else min(1.0, (i + 1) / n_ramp)
        if order == "together":
            at, af = a, a
        else:
            first = min(1.0, (i + 1) / (n_ramp // 2))
            second = min(1.0, max(0.0, (i + 1 - n_ramp // 2) / (n_ramp // 2)))
            at, af = (first, second) if order == "thumb_first" else (second, first)
        h = np.array(HAND_OPEN, float)
        h[thumb] = (1 - at) * np.array(HAND_OPEN)[thumb] + at * np.array(HAND_CLOSED)[thumb]
        h[~thumb] = (1 - af) * np.array(HAND_OPEN)[~thumb] + af * np.array(HAND_CLOSED)[~thumb]
        put(roots[n_go - 1], dofs[n_go - 1], h)
        if os.environ.get("TEST_VERBOSE") and i % 30 == 0:
            _bp = obj_centre()
            print(f"[test]    close frame {i:3d}: right fingers {np.round(robot.data.joint_pos[0, _rh].cpu().numpy(), 2)} "
                  f"vel {np.round(robot.data.joint_vel[0, _rh].cpu().numpy(), 2)} object {np.round(_bp, 3)}")
    snap(f"{int(k)}_close")
    bz_closed = float(obj_centre()[2])
    if os.environ.get("TEST_VERBOSE"):
        _bp = box.data.root_pos_w[0].cpu().numpy()
        _hq = robot.data.joint_pos[0, hand_ids[7:]].cpu().numpy()
        _tips = {}
        for _ln in ((PALM_LINK["right"], "right_hand_index_1_link", "right_hand_middle_1_link", "right_hand_thumb_2_link") if os.environ.get("HAND") != "inspire"
                    else (PALM_LINK["right"], "R_index_intermediate", "R_pinky_intermediate", "R_thumb_distal")):
            _b = robot.find_bodies([_ln])[0]
            if _b:
                _tips[_ln.replace("right_hand_", "").replace("R_", "")] = robot.data.body_pos_w[0, _b[0]].cpu().numpy() - _bp
        print(f"[test]    right fingers at close (index0 index1 middle0 middle1 thumb0 thumb1 thumb2): "
              f"{np.round(_hq, 2)} of closed {np.round(HAND_CLOSED[len(HAND_CLOSED) // 2:], 2)}")
        print("[test]    box-relative: " + "  ".join(f"{k} {np.round(v, 3)}" for k, v in _tips.items()))
    for i in range(n_lift * lift_x):
        f = n_go + i / lift_x
        j, a = int(f), f - int(f)
        j1 = min(j + 1, n_go + n_lift - 1)
        put((1 - a) * roots[j] + a * roots[j1], (1 - a) * dofs[j] + a * dofs[j1], HAND_OPEN if os.environ.get("NO_CLOSE") else HAND_CLOSED)
    for _ in range(10):
        put(roots[-1], dofs[-1], HAND_OPEN if os.environ.get("NO_CLOSE") else HAND_CLOSED)
    snap(f"{int(k)}_lift")
    if velocity_mode:
        _hq = robot.data.joint_pos[0, _rh].cpu().numpy()
        print(f"[test]    fingers after lift (index0 index1 middle0 middle1 thumb0 thumb1 thumb2): {np.round(_hq, 2)}")
        squeeze(False)
    bp = obj_centre()
    dz = float(bp[2] - z_ref)
    held = dz > 0.05
    results.append((int(k), dz, held))
    print(f"[test] grasp #{int(k):2d} conf {float(d['conf'][k]):.3f} cuRobo {float(d['err'][k, n_go-1])*1000:5.1f} mm  "
          f"box at close {bz_closed - z_ref:+.3f}  after lift dz {dz:+.3f} m  -> {'HELD' if held else 'LOST'}")
    sys.stdout.flush()

held = [r for r in results if r[2]]
print(f"[test] {len(held)}/{len(results)} grasps held: " + ", ".join(f"#{r[0]} ({r[1]*1000:.0f} mm)" for r in held))
sys.stdout.flush()
os._exit(0)
