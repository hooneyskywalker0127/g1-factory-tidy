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
# slip during the lift segment"). Fable's tester is fixed at 12/4, so every candidate
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
        # v35 measured a 19.7 mm gap in z (and 27-48 mm in x) between cuRobo's FK of the
        # commanded joints and where Isaac actually put right_wrist_yaw_link -- present in
        # free space before any contact. Print commanded vs achieved for every body joint
        # at the pose held through the close, so the joint that does not follow is a number
        # instead of a guess. Read-only: nothing here changes what is commanded.
        _cmd = np.asarray(dofs[n_go - 1], float)
        _ach = robot.data.joint_pos[0, body_ids].cpu().numpy()
        _lim = getattr(robot.data, "soft_joint_pos_limits", None)
        if _lim is None:
            _lim = getattr(robot.data, "joint_pos_limits", None)
        if _lim is None:
            _lo = np.full(len(names), -np.inf); _hi = np.full(len(names), np.inf)
        else:
            _lo = _lim[0, body_ids, 0].cpu().numpy(); _hi = _lim[0, body_ids, 1].cpu().numpy()
        _rows = sorted(range(len(names)), key=lambda j: -abs(_ach[j] - _cmd[j]))
        print("[opus] commanded vs achieved at the close pose (rad), worst first:")
        for j in _rows[:12]:
            _at = "  AT LIMIT" if min(abs(_cmd[j] - _lo[j]), abs(_cmd[j] - _hi[j])) < 0.02 else ""
            print(f"[opus]   {names[j]:<30s} cmd {_cmd[j]:+.4f}  got {_ach[j]:+.4f}  "
                  f"d {(_ach[j]-_cmd[j])*1000:+7.1f} mrad  limits [{_lo[j]:+.3f} {_hi[j]:+.3f}]{_at}")
        _clamped = [names[j] for j in range(len(names)) if _cmd[j] < _lo[j] - 1e-4 or _cmd[j] > _hi[j] + 1e-4]
        print(f"[opus]   commanded OUTSIDE Isaac's limits: {_clamped if _clamped else 'none'}")
        print(f"[opus]   total |d| {abs(_ach - _cmd).sum()*1000:.1f} mrad over {len(names)} joints")
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
