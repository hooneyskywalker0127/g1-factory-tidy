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
from build_reach_reference import HAND_OPEN, HAND_CLOSED, HAND_NAMES, PALM_LINK, HAND_KEEP, mujoco_names, robot_cfg, stiffen_mimic, soft_mimic  # noqa: E402
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
# SOLVER_IT/SOLVER_VIT: GraspGenX end2end/dynamic_playback.py:91-98 runs 100/50
# ("with only 10 iterations the constraint solver doesn't fully converge and grasps
# slip during the lift segment"). the original tester is fixed at 12/4, so every candidate
# it judged was judged under a solver NVIDIA calls too loose to hold a lift.
_SIT = int(os.environ.get("SOLVER_IT", "12")); _SVIT = int(os.environ.get("SOLVER_VIT", "4"))
cfg.spawn = cfg.spawn.replace(articulation_props=sim_utils.ArticulationRootPropertiesCfg(
    enabled_self_collisions=False, solver_position_iteration_count=_SIT,
    solver_velocity_iteration_count=_SVIT, fix_root_link=(os.environ.get("FIX_ROOT", "0") == "1")))
if os.environ.get("SOLVER_IT"): print(f"[solver] articulation iterations {_SIT}/{_SVIT} (GraspGenX 100/50)")
# ARM_KP_SCALE: with the body on its PD drives (PD_BODY) the stock arm gains let an
# outstretched arm sag -- measured on the crate approach, the left palm 10 cm under
# its reference while the IK was within 24 mm -- and the sagging hand dragged the crate.
if os.environ.get("ARM_KP_SCALE") and "arms" in cfg.actuators:
    _k = float(os.environ["ARM_KP_SCALE"])
    cfg.actuators["arms"] = cfg.actuators["arms"].replace(
        stiffness={n: v * _k for n, v in cfg.actuators["arms"].stiffness.items()} if isinstance(cfg.actuators["arms"].stiffness, dict) else cfg.actuators["arms"].stiffness * _k,
        damping={n: v * _k ** 0.5 for n, v in cfg.actuators["arms"].damping.items()} if isinstance(cfg.actuators["arms"].damping, dict) else cfg.actuators["arms"].damping * _k ** 0.5)
# ARM_KD: the only arm-drive value that diverges from the open source. GraspGenX's own dynamic
# playback runs arm_kp 2000 / arm_kd 100 (end2end/robots/g1_inspire_arm.yaml) = kd/kp 0.05; ours is
# 20/12000 = 0.0017, 30x less damping per unit stiffness. Against the measured arm inertia
# (arm+hand 3.54 kg at ~0.3 m -> I ~ 0.3 kg m2) critical damping at kp 12000 is 2*sqrt(kp*I) ~ 120,
# so kd 20 is zeta ~ 0.17. Absolute override, applied after ARM_KP_SCALE has scaled kd by sqrt(k).
if os.environ.get("ARM_KD") and "arms" in cfg.actuators:
    cfg.actuators["arms"] = cfg.actuators["arms"].replace(damping=float(os.environ["ARM_KD"]))
# ARM_EFFORT: the demand at right_shoulder_roll is kp*err = 12000 x 0.184 = 2208 Nm against a
# 300 Nm ceiling, and the achieved error barely moved across an 8x kp sweep (149/181/186 mrad at
# kp 3000/12000/24000) -- so the error is not a stiffness deficit. Raising the ceiling separates
# the two remaining causes: if the joint reaches its command, the clamp was binding; if it stalls
# at the same -0.68, something is hard-stopping the arm and no torque will fix it.
# NOTE the real hardware rating is effort="25" (g1_29dof_rev_1_0.urdf); the stock 300 is already 12x.
if os.environ.get("ARM_EFFORT") and "arms" in cfg.actuators:
    cfg.actuators["arms"] = cfg.actuators["arms"].replace(
        effort_limit=float(os.environ["ARM_EFFORT"]), effort_limit_sim=None)
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
# --- finger squeeze: the gains measured stable in hammer/v20 (kp 40 / kd 4.0 / effort 30)
# robot_cfg() sets damping = 0.2*(kp/10)**0.5, so at kp 40 it hands back kd 0.4 -- the
# ratio 0.01 that rang 0<->1571.78 N and spiked to 24,588 N in v19. GraspGenX's own
# ratio is kd/kp = 200/2000 = 0.1 (end2end/dynamic_playback.py:70-71).
if os.environ.get("HAND") == "inspire" and os.environ.get("HAND_KD"):
    _hz = cfg.actuators["hands"]
    _rep = dict(damping=float(os.environ["HAND_KD"]),
                effort_limit=float(os.environ.get("HAND_EFFORT", "200")))
    if os.environ.get("HAND_ARMATURE"): _rep["armature"] = float(os.environ["HAND_ARMATURE"])
    cfg.actuators["hands"] = _hz.replace(**_rep)
    print(f"[hand] squeeze (GraspGenX position-mode gains): kp {_hz.stiffness} "
          f"kd {_rep['damping']} effort {_rep['effort_limit']} armature {_rep.get('armature')}")
robot = Articulation(cfg)
stiffen_mimic(stage)   # HAND=inspire: rigid four-bar fingertips (build_reach_reference.py)
# --- finger colliders: GraspGenX end2end/robot_profiles.py UR10eInspireHandProfile
#     coacd_link_keywords = ("intermediate", "distal")
# Our asset ships every finger collider as a convex HULL, which rounds the phalanges
# the object is meant to sit between.
_coacd = os.environ.get("FINGER_COACD", "")
if _coacd:
    from pxr import Usd as _Usd2, UsdPhysics as _UsdPh2
    _kw = tuple(k for k in _coacd.split(",") if k); _nmesh = 0
    for _link in stage.GetPrimAtPath("/World/G1").GetChildren():
        if not (_link.GetName().startswith("R_") and any(k in _link.GetName() for k in _kw)): continue
        _col = stage.GetPrimAtPath(_link.GetPath().AppendChild("collisions"))
        if not (_col and _col.IsValid()): continue
        _col.SetInstanceable(False)
        for _m in _Usd2.PrimRange(_col):
            if _m.HasAPI(_UsdPh2.CollisionAPI):
                _UsdPh2.MeshCollisionAPI.Apply(_m).CreateApproximationAttr("convexDecomposition"); _nmesh += 1
    print(f"[hand] finger colliders -> convexDecomposition on {_kw}: {_nmesh} meshes (GraspGenX coacd_link_keywords)")

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
from plan_scene import dump_physics; dump_physics(stage)
sim.reset()
# --- thumb joint limits: the real Inspire hand has no backward travel ---------
# Measured in hammer/v21: R_thumb_intermediate_joint sat at -0.160 rad -- exactly our
# USD's lower limit -- from f530 to f750, while R_thumb_proximal_pitch_joint never left
# 0.00 against its 0.5 target. The asset's PhysX mimic (gearing -1.6) then pins the
# pitch at 0, so the thumb was a straight post for the whole grasp.
# The manufacturer's own description -- GraspGenX ext/gripper_descriptions/.../
# x_grippers/inspire_hand/gripper.urdf -- gives these joints NO backward travel:
#     thumb_intermediate_joint  limit 0 ~ 0.8   (mimic thumb_proximal_pitch x 1.334)
#     thumb_distal_joint        limit 0 ~ 0.4   (mimic thumb_proximal_pitch x 0.667)
# Our asset authors them -0.160 ~ 0.960 and -0.240 ~ 1.440. Raise the lower bounds to
# the URDF's 0 and leave the upper bounds and the gearing alone.
if os.environ.get("THUMB_LIMIT_URDF") == "1":
    _tid, _tn = robot.find_joints(["R_thumb_intermediate_joint", "R_thumb_distal_joint"])
    _lim = robot.data.joint_pos_limits[:, _tid, :].clone()
    for _i, _n in enumerate(_tn):
        print(f"[hand] {_n} limits {_lim[0, _i, 0]:.3f} .. {_lim[0, _i, 1]:.3f} rad -> "
              f"0.000 .. {_lim[0, _i, 1]:.3f} (inspire_hand/gripper.urdf has no backward travel)")
    _lim[:, :, 0] = 0.0
    robot.write_joint_position_limit_to_sim(_lim, joint_ids=_tid)

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
HANDS_SEQ = d["hands"] if "hands" in d.files else None
MARKS = {}
if "marks" in d.files:                                        # phase name -> first frame; keyed here by the frame each phase ENDS on
    import json as _json
    _m = _json.loads(str(d["marks"])); _ends = sorted(_m.items(), key=lambda kv: kv[1])
    MARKS = {(_ends[j + 1][1] if j + 1 < len(_ends) else n_go): nm for j, (nm, _) in enumerate(_ends)}       # per-frame finger targets (assist wrap: two hands on different clocks)
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
    if os.environ.get("ROOTDRIFT"):
        # The root is not welded: it is SET once per frame with zero velocity, then 33 substeps run
        # with the base free. Damping is not the cause (kd 20/120/600 -> 184.1/179.9/170.8 mrad) and
        # the load is not at the hand (fingers 0.8 N) nor the legs (80-280 N), yet shoulder_pitch's
        # incoming joint torque is 303.8 Nm against its own 48.6 Nm drive -- structural, not actuation.
        # cuRobo solved the arm against root7. Two numbers decide whether the base is the problem:
        #   drift = how far physics carried the base away from root7 during the last frame's substeps
        #   jump  = how far root7 itself moved since the previous frame
        # Millimetres means the base is consistent and the load is elsewhere; centimetres means the
        # arm is holding a pose relative to a base that is moving under it. Read-only.
        _k = globals().setdefault("_RD", [0, None])
        _ap = robot.data.root_pos_w[0].detach().cpu().numpy()
        _av = robot.data.root_lin_vel_w[0].detach().cpu().numpy()
        _aw = robot.data.root_ang_vel_w[0].detach().cpu().numpy()
        _cp = np.asarray(root7[:3], float)
        if _k[0] % 10 == 0:
            _jm = 0.0 if _k[1] is None else float(np.linalg.norm(_cp - _k[1])) * 1000
            print(f"[probe] root f{_k[0]:3d}  drift {np.linalg.norm(_ap - _cp) * 1000:7.2f} mm "
                  f"(dz {(_ap[2] - _cp[2]) * 1000:+7.2f})  jump {_jm:6.2f} mm  "
                  f"achieved_vel {np.linalg.norm(_av):6.3f} m/s  omega {np.linalg.norm(_aw):6.3f} rad/s",
                  flush=True)
        _k[1] = _cp.copy()
        _k[0] += 1
    if PD_BODY:
        # The way the one grasp that held was replayed (the desk box, pelvis
        # welded, every joint on its PD drive): no joint is written, the root
        # is placed once per frame, and PhysX sees one consistent
        # articulation. Written every substep, the body gave the simulated
        # fingers garbage velocities and contact kicks (see DIAGNOSIS.md).
        robot.write_root_state_to_sim(rs)
        for _ in range(substeps):
            robot.update(sim.get_physics_dt())
            soft_mimic(robot, tgt)
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
        soft_mimic(robot, tgt)
        robot.set_joint_position_target(tgt)
        if SQUEEZING[0]:
            robot.set_joint_velocity_target(SQUEEZING[1], joint_ids=SQUEEZING[2])
        robot.write_root_state_to_sim(rs)
        robot.write_joint_state_to_sim(tgt[:, write_ids], zero[:, write_ids], joint_ids=write_ids)
        robot.write_data_to_sim()
        sim.step()
    robot.update(sim.get_physics_dt())


# --pkl REF.pkl --hands H.npy: replay the RENDER's own data (a motion-lib clip + finger schedule, the files
# play_in_cell.py gets) through this tester's put() loop, then judge the lift. If this holds where the render
# loses, the difference is in play_in_cell's loop, not in the data (2026-09-28: #138 held 3/3 here, lost there).
if "--pkl" in sys.argv:
    import joblib
    _m = list(joblib.load(sys.argv[sys.argv.index("--pkl") + 1]).values())[0]
    _h = np.load(sys.argv[sys.argv.index("--hands") + 1])
    _rt, _rq, _dof = np.asarray(_m["root_trans_offset"]), np.asarray(_m["root_rot"]), np.asarray(_m["dof"])
    z_ref = float(obj_centre()[2])
    for i in range(len(_dof)):
        put(np.concatenate([_rt[i], _rq[i][[3, 0, 1, 2]]]), _dof[i], _h[min(i, len(_h) - 1)])
        if i % 5 == 0 or i == len(_dof) - 1:
            _b = robot.find_bodies([PALM_LINK["right"]])[0][0]; _rh = hand_ids[len(hand_ids) // 2:]
            print(f"[test]    pkl frame {i:4d}: object {np.round(obj_centre(), 3)} palm {np.round(robot.data.body_pos_w[0, _b].cpu().numpy(), 3)} "
                  f"root {np.round(robot.data.root_pos_w[0].cpu().numpy(), 3)} fingers {np.round(robot.data.joint_pos[0, _rh].cpu().numpy(), 2)}")
    _bp = obj_centre(); _dz = float(_bp[2] - z_ref)
    print(f"[test] pkl replay: after {len(_dof)} frames dz {_dz:+.3f} m -> {'HELD' if _dz > 0.05 else 'LOST'}")
    raise SystemExit(0)
def _jtrace(_i):
    """Commanded vs achieved at planned frame _i: joints, then the root, then the wrist
    in the root frame. At frame 254 (held) cuRobo FK and Isaac agree to 1.4 mm; at 195
    (the grasp dwell) they differ by 35.2 mm, so the disagreement is posture-dependent
    and has to be read at the posture that matters. Read-only."""
    # v35 measured a 19.7 mm gap in z (and 27-48 mm in x) between cuRobo's FK of the
    # commanded joints and where Isaac actually put right_wrist_yaw_link -- present in
    # free space before any contact. Print commanded vs achieved for every body joint
    # at the pose held through the close, so the joint that does not follow is a number
    # instead of a guess. Read-only: nothing here changes what is commanded.
    _cmd = np.asarray(dofs[_i], float)
    _ach = robot.data.joint_pos[0, body_ids].cpu().numpy()
    _lim = getattr(robot.data, "soft_joint_pos_limits", None)
    if _lim is None:
        _lim = getattr(robot.data, "joint_pos_limits", None)
    if _lim is None:
        _lo = np.full(len(names), -np.inf); _hi = np.full(len(names), np.inf)
    else:
        _lo = _lim[0, body_ids, 0].cpu().numpy(); _hi = _lim[0, body_ids, 1].cpu().numpy()
    _rows = sorted(range(len(names)), key=lambda j: -abs(_ach[j] - _cmd[j]))
    print("[probe] commanded vs achieved at the close pose (rad), worst first:")
    for j in _rows[:12]:
        _at = "  AT LIMIT" if min(abs(_cmd[j] - _lo[j]), abs(_cmd[j] - _hi[j])) < 0.02 else ""
        print(f"[probe]   {names[j]:<30s} cmd {_cmd[j]:+.4f}  got {_ach[j]:+.4f}  "
              f"d {(_ach[j]-_cmd[j])*1000:+7.1f} mrad  limits [{_lo[j]:+.3f} {_hi[j]:+.3f}]{_at}")
    _clamped = [names[j] for j in range(len(names)) if _cmd[j] < _lo[j] - 1e-4 or _cmd[j] > _hi[j] + 1e-4]
    print(f"[probe]   commanded OUTSIDE Isaac's limits: {_clamped if _clamped else 'none'}")
    print(f"[probe]   total |d| {abs(_ach - _cmd).sum()*1000:.1f} mrad over {len(names)} joints")
    # The arms track to <8 mrad while the wrist link is 20-50 mm off, so the error is
    # not in the arm chain's joints. Two candidates remain: the pelvis is not where the
    # plan put it (fix_root_link=True may make write_root_state_to_sim a no-op), or the
    # link geometry differs between cuRobo's yml and the Isaac USD. Comparing the root
    # and then the wrist IN THE ROOT FRAME separates them. Read-only.
    _rc = np.asarray(roots[_i], float)
    _rp = robot.data.root_pos_w[0].cpu().numpy(); _rqw = robot.data.root_quat_w[0].cpu().numpy()
    _Ri = R.from_quat(_rqw[[1, 2, 3, 0]]); _Rc = R.from_quat(_rc[[4, 5, 6, 3]])
    print(f"[probe]   root commanded pos {np.round(_rc[:3], 4)} quat {np.round(_rc[3:7], 4)}")
    print(f"[probe]   root Isaac     pos {np.round(_rp, 4)} quat {np.round(_rqw, 4)}")
    print(f"[probe]   root d pos {np.round((_rp - _rc[:3]) * 1000, 1)} mm  orientation "
          f"{np.degrees((_Ri * _Rc.inv()).magnitude()):.2f} deg")
    _wb = robot.find_bodies(["right_wrist_yaw_link"])[0][0]
    _ww = robot.data.body_pos_w[0, _wb].cpu().numpy()
    print(f"[probe]   right_wrist_yaw_link world {np.round(_ww, 4)}  in Isaac root frame "
          f"{np.round(_Ri.inv().apply(_ww - _rp), 4)}  in commanded root frame "
          f"{np.round(_Rc.inv().apply(_ww - _rc[:3]), 4)}")


results = []
for k in order:
    q = d["q"][k]
    HS = None if HANDS_SEQ is None else (HANDS_SEQ[k] if HANDS_SEQ.ndim == 3 else HANDS_SEQ)   # this candidate's finger schedule
    roots = np.concatenate([q[:, bcol[:3]], R.from_euler("XYZ", q[:, bcol[3:]]).as_quat()[:, [3, 0, 1, 2]]], axis=1)
    dofs = q[:, col]
    # reset: box back, body back to the kneel
    box.write_root_state_to_sim(box0)
    for _ in range(15):
        put(kneel_root, kneel_dof, HAND_OPEN)
    # SEAM_BLEND: ease from the kneel into the reach's first frame instead of stepping the shoulder 30 deg
    # in one frame -- measured (play_in_cell diag, 2026-09-27), the stiff arm (ARM_KP_SCALE 4) rang +-10 cm
    # at 3 Hz for a second after that step and swept the hammer before the descent had reached it.
    _nb = int(os.environ.get("SEAM_BLEND", "15"))
    for _j in range(_nb):
        _a = (_j + 1) / (_nb + 1)
        _r = (1 - _a) * kneel_root + _a * roots[0]; _r[3:7] /= np.linalg.norm(_r[3:7])
        put(_r, (1 - _a) * kneel_dof + _a * dofs[0], HAND_OPEN)
    for _ in range(int(os.environ.get("SETTLE_IDLE", "0"))):   # diagnostic: let the object fall asleep before the approach, as it does in the render
        put(roots[0], dofs[0], HAND_OPEN)
    if os.environ.get("TEST_VERBOSE") and _nb:
        _o = obj_centre(); _b = robot.find_bodies([PALM_LINK["right"]])[0][0]
        print(f"[test]    after the {_nb}-frame seam blend: object {np.round(_o, 3)}  right palm {np.round(robot.data.body_pos_w[0, _b].cpu().numpy(), 2)}")
    z_ref = float(obj_centre()[2])                 # where it rests before this candidate's reach
    if os.environ.get("TEST_VERBOSE"):
        print(f"[test]    object after the reset to the kneel: {np.round(obj_centre(), 3)} (placed at {np.round(box0[0, :3].cpu().numpy(), 3)})")
        _bq = box.data.root_quat_w[0].cpu().numpy(); _pq = box0[0, 3:7].cpu().numpy()
        _rot = np.degrees(2 * np.arccos(np.clip(abs(float(np.dot(_bq, _pq))), -1, 1)))
        _lo = float(box.data.body_pos_w[0, :, 2].min()) if hasattr(box.data, "body_pos_w") else float("nan")
        print(f"[test]    object settled: rotated {_rot:.1f} deg from where it was placed; quat now {np.round(_bq, 3)} placed {np.round(_pq, 3)}")
    for i in range(n_go):
        put(roots[i], dofs[i], HAND_OPEN if HS is None else HS[i])
        if os.environ.get("TRACE_FRAME") and i == int(os.environ["TRACE_FRAME"]):
            _jtrace(i)
        if os.environ.get("FINGER_TRACE") and i in [int(v) for v in os.environ["FINGER_TRACE"].split(",")]:
            # The one number that separates "the fingers close on nothing" from "the fingers
            # grip and it slips": if the achieved finger q reaches the commanded 1.47 rad, the
            # 23 mm handle is not between them. Read-only.
            _rh = hand_ids[len(hand_ids) // 2:]
            _fq = robot.data.joint_pos[0, _rh].cpu().numpy()
            _fc = np.asarray(HS[i] if HS is not None else HAND_OPEN, float)[len(hand_ids) // 2:]
            _bp = obj_centre(); _bq = box.data.root_quat_w[0].cpu().numpy()
            _Rb = R.from_quat(_bq[[1, 2, 3, 0]])
            _t = {}
            for _ln in ((PALM_LINK["right"], "right_hand_index_1_link", "right_hand_middle_1_link", "right_hand_thumb_2_link")
                        if os.environ.get("HAND") != "inspire"
                        else (PALM_LINK["right"], "R_index_intermediate", "R_pinky_intermediate", "R_thumb_distal")):
                _b = robot.find_bodies([_ln])[0]
                if _b:
                    _t[_ln.replace("right_hand_", "").replace("_link", "").replace("R_", "")] = robot.data.body_pos_w[0, _b[0]].cpu().numpy()
            print(f"[probe] f{i}: finger q   {np.round(_fq, 3)}")
            print(f"[probe] f{i}: commanded  {np.round(_fc, 3)}   max |d| {np.abs(_fq - _fc).max()*1000:6.1f} mrad"
                  f"   stalled {[k for k in range(len(_fq)) if abs(_fq[k]-_fc[k]) > 0.05]}")
            print(f"[probe] f{i}: object centroid {np.round(_bp, 4)}  tilt {np.degrees(_Rb.magnitude()):.1f} deg")
            print("[probe] f{}: link z  {}".format(i, "  ".join(f"{k} {v[2]:+.3f}" for k, v in _t.items())))
            print("[probe] f{}: link - object {}".format(i, "  ".join(f"{k} {np.round((v-_bp)*1000).astype(int)}" for k, v in _t.items())))
            # The shoulder that does not follow is the whole miss: 184 mrad at right_shoulder_roll
            # is ~83 mm at the wrist, and the wrist measured +80 mm in x from its target. Print the
            # right arm at every traced frame so free-space droop separates from contact reaction.
            _ac = np.asarray(dofs[i], float)
            _aa = robot.data.joint_pos[0, body_ids].cpu().numpy()
            _ai = [j for j, n in enumerate(names) if n.startswith("right_shoulder") or n.startswith("right_elbow") or n.startswith("right_wrist")]
            print("[probe] f{}: arm  {}".format(i, "  ".join(
            f"{names[j].replace('right_','').replace('_joint',''):<14s} {(_aa[j]-_ac[j])*1000:+7.1f}" for j in _ai)))
        if os.environ.get("WAIST_HISTORY") and i >= 120 and i % 5 == 0:
            # One snapshot of the waist cannot tell a joint pinned by its effort ceiling from a
            # joint oscillating around its command: stiffness 5000 with damping 5 is zeta ~ 0.02.
            # Capping the plan by 78 mrad made the achieved value 155 mrad worse, which a static
            # equilibrium cannot do, so read the whole dwell: position, velocity and the torque
            # PhysX actually applied (clamped at effort_limit 50). Read-only.
            _wn = [n for n in names if n.startswith("waist_")]
            _wi = [names.index(n) for n in _wn]; _wj = [body_ids[j] for j in _wi]
            _c = np.asarray(dofs[i], float)[_wi]
            _q = robot.data.joint_pos[0, _wj].cpu().numpy()
            _v = robot.data.joint_vel[0, _wj].cpu().numpy()
            _tt = getattr(robot.data, "applied_torque", None)
            _t = _tt[0, _wj].cpu().numpy() if _tt is not None else np.full(len(_wj), np.nan)
            print("[probe] waisth f%3d  " % i + "   ".join(
                f"{n.replace('waist_','').replace('_joint',''):5s} cmd{_c[j]:+.4f} got{_q[j]:+.4f}"
                f" d{(_q[j]-_c[j])*1000:+7.1f} vel{_v[j]:+7.2f} tau{_t[j]:+7.1f}"
                for j, n in enumerate(_wn)), flush=True)
        if os.environ.get("ARM_HISTORY") and i >= int(os.environ.get("ARM_FROM", "120")) and i % int(os.environ.get("ARM_STRIDE", "5")) == 0:
            # The waist tracks to 1 mrad and the root is pinned, yet the wrist link lands 17-21 mm
            # high (v37, candidate 3). Over a 0.7 m arm that is ~8 mrad spread across the arm joints
            # -- exactly the residual _jtrace already reports. A steady-state droop is either the
            # drive running out of torque or running out of stiffness, and those want opposite
            # fixes, so print both: the torque PhysX applied and the ceiling it was clamped to.
            _an = [n for n in names if n.startswith("right_") and ("shoulder" in n or "elbow" in n or "wrist" in n)]
            _ai = [names.index(n) for n in _an]; _aj = [body_ids[j] for j in _ai]
            _c = np.asarray(dofs[i], float)[_ai]
            _q = robot.data.joint_pos[0, _aj].cpu().numpy()
            _v = robot.data.joint_vel[0, _aj].cpu().numpy()
            _tt = getattr(robot.data, "applied_torque", None)
            _t = _tt[0, _aj].cpu().numpy() if _tt is not None else np.full(len(_aj), np.nan)
            _el = None
            for _src in ("joint_effort_limits", "joint_effort_limit", "default_joint_effort_limits"):
                _e = getattr(robot.data, _src, None)
                if _e is not None:
                    _el = _e[0, _aj].cpu().numpy(); break
            if _el is None:
                try: _el = robot.root_physx_view.get_dof_max_forces().cpu().numpy()[0][_aj]
                except Exception: _el = np.full(len(_aj), np.nan)
            if not globals().get("_ARM_GAINS_SHOWN"):
                globals()["_ARM_GAINS_SHOWN"] = True
                _ks = robot.data.joint_stiffness[0, _aj].cpu().numpy(); _kd = robot.data.joint_damping[0, _aj].cpu().numpy()
                # err plateaus at 76 mrad for any ceiling >= 1000 Nm with self-collision OFF: a wall at a
                # fixed angle that torque cannot pass is a joint limit, and this USD is not IsaacLab's
                # g1.usd. Print what PhysX actually enforces, next to what the URDF says (-2.2515 .. 1.5882).
                _pl = robot.data.joint_pos_limits[0, _aj].cpu().numpy()
                _sl = getattr(robot.data, "soft_joint_pos_limits", None)
                _sl = _sl[0, _aj].cpu().numpy() if _sl is not None else None
                for _j, _n in enumerate(_an):
                    _extra = f"  soft [{_sl[_j][0]:+.4f} {_sl[_j][1]:+.4f}]" if _sl is not None else ""
                    print(f"[probe] lim  {_n:<28s} hard [{_pl[_j][0]:+.4f} {_pl[_j][1]:+.4f}]{_extra}", flush=True)
                # NOT _v: that name holds the joint-velocity array this block's caller prints two
                # lines below, and clobbering it with a tensor is what raised Tensor.__format__.
                for _attr in ("joint_friction_coeff", "joint_friction", "joint_armature"):
                    _fv = getattr(robot.data, _attr, None)
                    if _fv is not None:
                        _fv = _fv[0, _aj].cpu().numpy()
                        print(f"[probe] {_attr}  " + "  ".join(
                            f"{_an[_j].replace('right_','').replace('_joint','')} {_fv[_j]:.5f}"
                            for _j in range(len(_an))), flush=True)
                print("[probe] arm drive  " + "   ".join(
                            f"{n.replace('right_','').replace('_joint','')} kp{_ks[j]:.0f} kd{_kd[j]:.1f} eff{_el[j]:.0f}"
                            for j, n in enumerate(_an)), flush=True)
            print("[probe] armh f%3d  " % i + "   ".join(
                f"{n.replace('right_','').replace('_joint',''):14s} cmd{_c[j]:+.4f} got{_q[j]:+.4f}"
                f" d{(_q[j]-_c[j])*1000:+7.1f} vel{_v[j]:+6.2f} tau{_t[j]:+7.1f}/{_el[j]:.0f}"
                for j, n in enumerate(_an)), flush=True)
        if os.environ.get("WRENCH") and i % 10 == 0:
            # ~900 Nm opposes right_shoulder_roll with self-collision OFF, friction 0, and the
            # command 1.19 rad inside its limit. The incoming joint force includes external contact
            # reactions, so walking it down the chain says WHERE the load enters: large at the hand
            # means the fingers are standing on the floor (the +14 mm "clearance" was a link ORIGIN,
            # not the collision mesh), large only at the shoulder means it is inertial. Read-only.
            try:
                _w = robot.root_physx_view.get_link_incoming_joint_force()[0].cpu().numpy()
            except Exception as _e:
                print(f"[probe] wrench unavailable: {_e}", flush=True); _w = None
            if _w is not None:
                _bn = list(robot.body_names)
                # The load does not enter at the fingers (0.5-2 N) and the arm carries 25-34x its own
                # weight. WRENCH=legs asks whether the legs are the other end of it: the pelvis is
                # welded by a per-frame root write while the leg drives miss their commanded kneel
                # (f254: left_ankle_pitch 789 / left_hip_yaw 765 / left_hip_pitch 528 / left_knee
                # 407 mrad, 29-joint sum 3182 mrad), so that weld-legs-floor loop could load the arm.
                # Measured answer: it does not. legs 80-280 N, pelvis 0.0, waist/torso 887-889 N.
                if os.environ.get("WRENCH") == "legs":
                    _want = [n for n in _bn if any(k in n for k in
                             ("pelvis", "torso", "waist", "hip", "knee", "ankle"))] + \
                            ["right_shoulder_roll_link", "right_wrist_yaw_link"]
                    _want = [n for n in _want if n in _bn]
                elif os.environ.get("WRENCH") == "hand":
                    # The earlier arm reading was cut off at "index_proximal F 0.8 T 0.0 th" and its
                    # filter never included right_hand_palm_link at all, so "the load is not at the
                    # hand" rested on a line whose end I had not seen. Self-collision is OFF but that
                    # does not disable collision with the FLOOR, and the error is posture-dependent
                    # (184 mrad at the low reach, 2.7 mrad in the lift) with the base rigid to 0.00 mm.
                    # Print every right-hand link with its world z, untruncated.
                    _want = [n for n in _bn if n.startswith("right_wrist") or n.startswith("right_hand")
                             or n.startswith("R_")]
                else:
                    _want = [n for n in _bn if n.startswith("right_shoulder") or n.startswith("right_elbow")
                             or n.startswith("right_wrist")] + \
                            [n for n in _bn if n.startswith("R_") and
                             any(k in n for k in ("hand_base", "index", "thumb"))]
                _row = []
                for _n in _want:
                    _b = _bn.index(_n)
                    _f = _w[_b]
                    _z = float(robot.data.body_pos_w[0, _b, 2])
                    _row.append(f"{_n.replace('right_','').replace('_link','').replace('R_','')} "
                                f"F{np.linalg.norm(_f[:3]):6.1f} T{np.linalg.norm(_f[3:6]):6.1f}"
                                + (f" z{_z:+.4f}" if os.environ.get("WRENCH") == "hand" else ""))
                if os.environ.get("WRENCH") == "hand":
                    print(f"[probe] wr f{i:3d}", flush=True)
                    for _e in _row:
                        print(f"[probe]      {_e}", flush=True)
                else:
                    print(f"[probe] wr f{i:3d}  " + "  ".join(_row), flush=True)
        if os.environ.get("SEG_GAP") and i % 10 == 0:
            # Link ORIGINS 300 mm apart say nothing about two 250-400 mm links crossing. The upper
            # arm and the thigh are capsules; measure segment-to-segment distance. shoulder_roll is
            # pinned at -300 Nm and cannot adduct past -0.69 while the thigh sits 251 mrad further
            # out than commanded -- the reaction pair a collision produces. Read-only.
            def _P(n):
                _b = robot.find_bodies([n])[0]
                return robot.data.body_pos_w[0, _b[0]].cpu().numpy() if _b else None
            def _s2s(p1, q1, p2, q2):
                _u, _v, _w = q1 - p1, q2 - p2, p1 - p2
                _a, _bb, _c = _u @ _u, _u @ _v, _v @ _v
                _d, _e = _u @ _w, _v @ _w
                _D = _a * _c - _bb * _bb
                if _D < 1e-9: _sc, _tc = 0.0, (_e / _c if _c > 1e-9 else 0.0)
                else: _sc, _tc = (_bb * _e - _c * _d) / _D, (_a * _e - _bb * _d) / _D
                _sc, _tc = min(max(_sc, 0.0), 1.0), min(max(_tc, 0.0), 1.0)
                return float(np.linalg.norm((p1 + _sc * _u) - (p2 + _tc * _v)))
            _segs = {"upperarm": ("right_shoulder_roll_link", "right_elbow_link"),
                     "forearm":  ("right_elbow_link", "right_wrist_roll_link"),
                     "thigh":    ("right_hip_roll_link", "right_knee_link"),
                     "shank":    ("right_knee_link", "right_ankle_roll_link")}
            _pt = {k: (_P(v[0]), _P(v[1])) for k, v in _segs.items()}
            _out = []
            for _x in ("upperarm", "forearm"):
                for _y in ("thigh", "shank"):
                    if all(z is not None for z in _pt[_x] + _pt[_y]):
                        _out.append(f"{_x}-{_y} {_s2s(*_pt[_x], *_pt[_y])*1000:5.0f}")
            print(f"[probe] seg  f{i:3d}  " + "   ".join(_out), flush=True)
        if os.environ.get("ARM_PHYS") and not globals().get("_PHYS_SHOWN"):
            # 226 Nm of load at f90 with nothing within 262 mm of the hand is impossible for a
            # 2 kg arm on a 0.3 m lever (6 Nm). Before blaming the drive, dump the physics the
            # way the convex-hull bug was found: mass and inertia per link. Read-only.
            globals()["_PHYS_SHOWN"] = True
            try:
                _m = robot.root_physx_view.get_masses().cpu().numpy()[0]
                _in = robot.root_physx_view.get_inertias().cpu().numpy()[0]
            except Exception as _e:
                print(f"[probe] phys: unavailable ({_e})"); _m = None
            if _m is not None:
                _sel = [j for j, n in enumerate(robot.body_names)
                        if n.startswith("R_") or n.startswith("right_shoulder") or n.startswith("right_elbow")
                        or n.startswith("right_wrist") or n in ("torso_link", "pelvis")]
                print(f"[probe] phys: total robot mass {_m.sum():.2f} kg over {len(_m)} links")
                for j in _sel:
                    _I = _in[j].reshape(3, 3) if _in[j].size == 9 else _in[j]
                    _d = np.diag(_I) if getattr(_I, "ndim", 1) == 2 else _I[:3]
                    print(f"[probe] phys:   {robot.body_names[j]:<26s} m {_m[j]:8.4f} kg   Idiag {np.round(_d, 5)}")
                _hb = [j for j, n in enumerate(robot.body_names) if n.startswith("R_")]
                print(f"[probe] phys: right hand = {len(_hb)} links, {_m[_hb].sum():.4f} kg;  "
                      f"right arm+hand = {_m[[j for j in _sel if robot.body_names[j] not in ('torso_link','pelvis')]].sum():.4f} kg")
        if os.environ.get("HAND_FLOOR") and i % 10 == 0:
            _hb = [j for j, n in enumerate(robot.body_names) if n.startswith("R_") or n.startswith("right_wrist")]
            if _hb:
                _z = robot.data.body_pos_w[0, _hb, 2].cpu().numpy()
                print(f"[probe] handz f{i:3d}  lowest {_z.min():+.4f} ({robot.body_names[_hb[int(_z.argmin())]]})", flush=True)
        if os.environ.get("LIMB_GAP") and i >= 90 and i % 10 == 0:
            # shoulder_roll sits at tau -300/300 with vel +1.5 rad/s for the whole dwell: a
            # saturated drive that still moves is being pushed, not drooping. The only body
            # near the right arm in this kneel is the right leg. Print the arm-to-leg link
            # distances so a collision is a number. Read-only.
            _A = ["right_shoulder_roll_link", "right_shoulder_yaw_link", "right_elbow_link", "right_wrist_roll_link"]
            _L = ["right_hip_roll_link", "right_hip_yaw_link", "right_knee_link", "torso_link", "pelvis"]
            _pos = {}
            for _n in _A + _L:
                _b = robot.find_bodies([_n])[0]
                if _b: _pos[_n] = robot.data.body_pos_w[0, _b[0]].cpu().numpy()
            _pr = [(float(np.linalg.norm(_pos[x] - _pos[y])), x, y) for x in _A if x in _pos for y in _L if y in _pos]
            _pr.sort()
            print("[probe] gap f%3d  " % i + "   ".join(
                f"{x.replace('right_','').replace('_link','')}-{y.replace('right_','').replace('_link','')} {d*1000:.0f}"
                for d, x, y in _pr[:5]), flush=True)
        if MARKS and (i + 1) in MARKS:                        # end of a scheduled phase (assist wrap): a picture and the object's pose
            _ph = MARKS[i + 1]; snap(f"{int(k)}_{_ph}")
            _o = obj_centre(); _bq = box.data.root_quat_w[0].cpu().numpy()
            _tilt = np.degrees(np.arccos(np.clip(1 - 2 * (_bq[1] ** 2 + _bq[2] ** 2), -1, 1)))
            _lb = robot.find_bodies([PALM_LINK["left"]])[0][0]; _rb = robot.find_bodies([PALM_LINK["right"]])[0][0]
            print(f"[test]    after {_ph:6s} (frame {i + 1:3d}): object {np.round(_o, 3)} tilt {_tilt:4.1f} deg  left palm {np.round(robot.data.body_pos_w[0, _lb].cpu().numpy(), 3)}  right palm {np.round(robot.data.body_pos_w[0, _rb].cpu().numpy(), 3)}")
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
    if os.environ.get("BOTH_HANDS") == "1":                     # where each hand sits before the close, object-relative
        _o0 = obj_centre()
        for _side, _lns in (("right", ("R_index_proximal", "R_index_intermediate", "R_pinky_intermediate", "R_thumb_distal", PALM_LINK["right"])),
                            ("left", ("L_index_proximal", "L_index_intermediate", "L_pinky_intermediate", "L_thumb_distal", PALM_LINK["left"]))):
            _row = []
            for _ln in _lns:
                _b = robot.find_bodies([_ln])[0]
                if _b:
                    _row.append(f"{_ln.replace('R_', '').replace('L_', '')} {np.round(robot.data.body_pos_w[0, _b[0]].cpu().numpy() - _o0, 3)}")
            print(f"[test]    at grasp, {_side} hand object-relative: " + "  ".join(_row))
    print(f"[test]    at grasp: palm z {_tips['palm'][2]:+.3f}, lowest fingertip z {_tip_min:+.3f}, "
          f"palm - object {np.round(_tips['palm'] - _bp0, 3)}")
    # Close and lift the way the desk pick that held did: the fingers get
    # hold_after_close_frames (150 at 30 fps, 5 s) to settle on the object
    # before the lift, and the lift itself is slow (20 cm over 7 s there).
    # A 0.8 s close and a 15 cm/s lift let a pinch on a 39 mm box slip.
    SLOW = "--slow" in sys.argv
    n_ramp, n_settle, lift_x = (int(os.environ.get("TEST_RAMP", "30")), 150, 4) if SLOW else (20, 5, 1)
    # CLOSE_ORDER: thumb_first | fingers_first | together. Closing everything
    # at once pushed a standing bottle over with whichever finger touched
    # first (40 of 40 candidates); a hand cages before it squeezes.
    order = os.environ.get("CLOSE_ORDER", "together")
    thumb = np.array([("thumb" in n and (os.environ.get("BOTH_HANDS") == "1" or n.startswith(("right", "R_")))) for n in hand_names], bool)      # right thumb joints (both with BOTH_HANDS)
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
    _hc = HAND_CLOSED if HS is None else HS[-1]       # what "closed" means after the sequence
    for i in range(0 if HANDS_SEQ is not None else n_ramp + n_settle):
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
    if os.environ.get("JOINT_TRACE"):
        _jtrace(n_go - 1)
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
        if os.environ.get("BOTH_HANDS") == "1":                 # the left hand too (two-hand grips)
            _lh = [robot.find_joints([n])[0][0] for n in hand_names[:len(hand_names) // 2]]
            _lt = {}
            for _ln in ((PALM_LINK["left"], "L_index_intermediate", "L_pinky_intermediate", "L_thumb_distal") if os.environ.get("HAND") == "inspire"
                        else (PALM_LINK["left"], "left_hand_index_1_link", "left_hand_middle_1_link", "left_hand_thumb_2_link")):
                _b = robot.find_bodies([_ln])[0]
                if _b:
                    _lt[_ln.replace("left_hand_", "").replace("L_", "")] = robot.data.body_pos_w[0, _b[0]].cpu().numpy() - _bp
            print(f"[test]    left fingers at close: {np.round(robot.data.joint_pos[0, _lh].cpu().numpy(), 2)} of closed {np.round(HAND_CLOSED[:len(HAND_CLOSED) // 2], 2)}")
            print("[test]    box-relative (left): " + "  ".join(f"{k} {np.round(v, 3)}" for k, v in _lt.items()))
    _l0 = int(d["lift_from"]) if "lift_from" in d.files and int(d["lift_from"]) > n_go else n_go   # crate solutions hold 30 frames before the lift
    for i in range(n_lift * lift_x):
        f = _l0 + i / lift_x
        j, a = int(f), f - int(f)
        j1 = min(j + 1, _l0 + n_lift - 1)
        put((1 - a) * roots[j] + a * roots[j1], (1 - a) * dofs[j] + a * dofs[j1], HAND_OPEN if os.environ.get("NO_CLOSE") else _hc)
    # TEST_HOLD_AFTER (frames, default 90): hold still after the lift before judging. GraspGen-X's player holds
    # after the lift for the same reason ("preventing premature-lift slip"); hammer #138 read HELD at the top of the
    # lift and slid out of a two-finger grip during the next second (260928/5지/hammer/v4, v5).
    for _ in range(int(os.environ.get("TEST_HOLD_AFTER", "90"))):
        put(roots[-1], dofs[-1], HAND_OPEN if os.environ.get("NO_CLOSE") else _hc)
    # TEST_RISE (m, default 0.35): after the hold, raise the whole body like the stand-up (pelvis 0.42 -> 0.79 m in
    # ~1.4 s of the planner's clip, here over TEST_RISE_FRAMES) -- the hammer and the drill both passed the static
    # hold and slid out of two-finger grips during exactly this (260928/5지/hammer/v5, drill/v4).
    _rise = float(os.environ.get("TEST_RISE", "0.35")); _nr = int(os.environ.get("TEST_RISE_FRAMES", "45"))
    if _rise > 0 and not os.environ.get("NO_CLOSE"):
        _z0 = float(obj_centre()[2])
        if os.environ.get("TEST_RISE_PKL"):
            # TEST_RISE_PKL: the planner's OWN stand-up (the first frames of a carry clip from walk_clip.py --rise-first: pelvis
            # 0.43 -> 0.78 m in 1 s with the torso pitching 14 -> 0 deg and the legs, waist and left arm moving), stretched
            # TEST_RISE_SLOW times as build_place_reference does, root as a delta from the reach's last frame, right arm and
            # fingers held. Sehoon: 일어설 때 문제라면 일어나는 것도 테스트에 넣어야 한다 -- the vertical raise was a stand-in.
            import joblib as _jl
            _m = list(_jl.load(os.environ["TEST_RISE_PKL"]).values())[0]
            _rt, _rq, _rd = np.asarray(_m["root_trans_offset"]), np.asarray(_m["root_rot"]), np.asarray(_m["dof"])
            _n_end = min(len(_rt) - 1, int(np.argmax(_rt[:, 2] > _rt[0, 2] + 0.9 * (_rt[:, 2].max() - _rt[0, 2]))) + 15)
            _slow = float(os.environ.get("TEST_RISE_SLOW", "2"))
            _rise = float(_rt[_n_end, 2] - _rt[0, 2])
            _arm = [j for j, n in enumerate(names) if n.startswith("right_")]
            _R0 = R.from_quat(_rq[0]); _Rl = R.from_quat(roots[-1][[4, 5, 6, 3]])
            _frames = np.arange(0, _n_end, 1.0 / _slow)
            for _f in list(_frames) + [float(_n_end)] * 30:
                _j = int(_f); _a = _f - _j; _j1 = min(_j + 1, _n_end)
                _t = (1 - _a) * _rt[_j] + _a * _rt[_j1]; _d = (1 - _a) * _rd[_j] + _a * _rd[_j1]
                _Ri = R.from_quat(_rq[_j]) * _R0.inv() * _Rl
                _r = roots[-1].copy(); _r[:3] = roots[-1][:3] + (_t - _rt[0]); _r[3:7] = _Ri.as_quat()[[3, 0, 1, 2]]
                _dd = _d.copy(); _dd[_arm] = dofs[-1][_arm]
                put(_r, _dd, _hc)
        else:
            for _i in range(_nr + 30):
                _r = roots[-1].copy(); _r[2] += _rise * min(1.0, (_i + 1) / _nr)
                put(_r, dofs[-1], _hc)
        _dz_r = float(obj_centre()[2]) - _z0
        print(f"[test]    stand-up test: body up {_rise:.2f} m, object followed {_dz_r:+.3f} m")
        if _dz_r < 0.5 * _rise:
            print(f"[test] grasp #{int(k):2d} conf {float(d['conf'][k]):.3f} cuRobo {float(d['err'][k, n_go - 1])*1000:5.1f} mm  slipped in the stand-up ({_dz_r:+.3f} of {_rise:.2f} m)  -> LOST")
            results.append((int(k), 0.0, False)); continue
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
