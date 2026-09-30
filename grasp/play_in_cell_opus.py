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
from build_reach_reference_opus import HAND_NAMES, PALM_LINK, HAND_KEEP, robot_cfg, stiffen_mimic, soft_mimic  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
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
# CCD=0: CCD came from the placed-hand tester (plan_scene.py:84-91), not from a source.
# GraspGenX (Newton/MuJoCo, dynamic_playback.py:1153-1170) and IsaacLab's dexsuite
# (dexsuite_env_cfg.py:425-430) grasp without it. The scene flag gates the per-body one.
# CONTACT_LAST=1: IsaacLab's PhysxCfg.solve_articulation_contact_last (simulation_cfg.py:46-58),
# documented "for gripping scenarios": solve dynamic contact after the joint drives. Default off.
sim = SimulationContext(sim_utils.SimulationCfg(dt=1.0 / 1000.0, device="cpu", physx=sim_utils.PhysxCfg(
    enable_ccd=os.environ.get("CCD", "1") == "1",
    solve_articulation_contact_last=os.environ.get("CONTACT_LAST", "0") == "1")))
print(f"[phys] scene CCD {os.environ.get('CCD', '1') == '1'}  "
      f"contact_last {os.environ.get('CONTACT_LAST', '0') == '1'}")
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
# --hold-only: the one question with nothing else in the way -- can the robot
# hold the pose the walk ends in, under physics, with the arm doing nothing?
#
# Every failure in this project lands on the same frame: the one where the
# walk clip stops and physics starts. 171 mm of slide, a 9.8x leap, a 12.5x
# spike, a robot in the splits at 23.5 s. Welded, it gets dragged onto the
# stand; unwelded, it collapses. Those are one problem seen from two sides,
# and neither side can be judged while an arm is also swinging.
HOLD_ONLY = "--hold-only" in sys.argv
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
cfg = robot_cfg(G1_29DOF_CFG.replace(prim_path="/World/G1"), sim_utils, ImplicitActuatorCfg)

# --- squeeze: GraspGenX's own position-mode finger drive -----------------------
# Read from the open source rather than guessed (Sehoon 2026-09-28: "그립이지? 그럼
# graspgenx 코드를 읽어"). NVIDIA GraspGenX end2end/dynamic_playback.py:68-71 and :689
#   FINGER_KP_DEFAULT = 2000.0, FINGER_KD_DEFAULT = 200.0, finger_effort_limit = 200.0
# and end2end/robot_profiles.py UR10eInspireHandProfile: POSITION mode, closing to
# thumb_proximal_pitch 0.6 / four proximals 1.47, no gripper armature, CoACD on the
# intermediate/distal links, mu 3.0 on the fingertip pads.
# Our robot_cfg() default is kp 10 / kd 0.2*(kp/10)**0.5 / effort 30. The close target
# and the fingertip friction already matched NVIDIA's; only the squeeze was 50x softer,
# and kd 0.2 leaves nothing resisting back-drive when the stand-up loads the contact --
# which is the grip visibly loosening around 25 s in hammer/v9 and drill/v6.
if os.environ.get("HAND") == "inspire" and os.environ.get("HAND_KD"):
    _hz = cfg.actuators["hands"]
    _rep = dict(damping=float(os.environ["HAND_KD"]),
                effort_limit=float(os.environ.get("HAND_EFFORT", "200")))
    if os.environ.get("HAND_ARMATURE"):
        _rep["armature"] = float(os.environ["HAND_ARMATURE"])
    cfg.actuators["hands"] = _hz.replace(**_rep)
    print(f"[hand] squeeze (GraspGenX position-mode gains): kp {_hz.stiffness} "
          f"kd {_rep['damping']} effort {_rep['effort_limit']} "
          f"armature {_rep.get('armature', getattr(_hz, 'armature', None))}")

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
if os.environ.get("HAND") == "inspire" and os.environ.get("HAND_VEL"):
    _hv = cfg.actuators["hands"]
    cfg.actuators["hands"] = _hv.replace(velocity_limit=None,
                                         velocity_limit_sim=float(os.environ["HAND_VEL"]))
    print(f"[hand] velocity_limit_sim {os.environ['HAND_VEL']} rad/s "
          f"(asset 0.5, manufacturer URDF 5.0; cfg velocity_limit {_hv.velocity_limit} is discarded)")

# --- mimic followers at NVIDIA's ratio, not the master's stiffness -------------
# Measured, hammer v21/v22: R_thumb_intermediate_joint parked at -0.160 rad -- exactly
# our USD's lower limit -- for f530..f750 while its master R_thumb_proximal_pitch_joint
# never left 0.00 against a 0.5 rad target. Raising that lower bound to the URDF's 0
# (v22, THUMB_LIMIT_URDF) did not free the joint: the thumb contact spiked to 23,366 N
# at f760 (v21 peak 1,854 N) and the hammer ended 0.2497 m below its start. Two
# controllers act on one joint -- the asset's PhysX mimic constraint and, because
# SOFT_MIMIC puts .*_intermediate_joint / .*_thumb_distal_joint in the same actuator
# group, a position drive at the master's own stiffness. build_reach_reference.py:89-91
# records the same symptom ("driven as well, they ran to their -0.34 limit while the
# proximals closed to 1.1").
# GraspGenX end2end/dynamic_playback.py:69-80 drives followers far softer than masters:
#     FINGER_KP 2000 / FINGER_KD 200   (masters)
#     MIMIC_KP    50 / MIMIC_KD    10  (followers)  -> kp x0.025, kd x0.05
# with the reason stated there: "Keep these gentle so the PD doesn't fight Newton's
# mimic constraint". MIMIC_SPLIT=1 applies that ratio to the follower joints only and
# leaves soft_mimic()'s per-substep target rewriting untouched.
if os.environ.get("HAND") == "inspire" and os.environ.get("MIMIC_SPLIT") == "1":
    from isaaclab.actuators import ImplicitActuatorCfg as _IAC
    _h = cfg.actuators["hands"]
    _foll = [".*_intermediate_joint", ".*_thumb_distal_joint"]
    _mast = [e for e in _h.joint_names_expr if e not in _foll]
    if len(_mast) == len(_h.joint_names_expr):
        print("[hand] MIMIC_SPLIT: no follower joints in the hands group (SOFT_MIMIC off?) -- nothing to split")
    else:
        _fkp = float(_h.stiffness) * float(os.environ.get("MIMIC_KP_RATIO", "0.025"))
        _fkd = float(_h.damping) * float(os.environ.get("MIMIC_KD_RATIO", "0.05"))
        cfg.actuators["hands"] = _h.replace(joint_names_expr=_mast)
        cfg.actuators["hands_mimic"] = _IAC(
            joint_names_expr=_foll, effort_limit=_h.effort_limit,
            velocity_limit=_h.velocity_limit, stiffness=_fkp, damping=_fkd,
            armature=_h.armature)
        print(f"[hand] MIMIC_SPLIT: masters {_mast} kp {_h.stiffness} kd {_h.damping}; "
              f"followers {_foll} kp {_fkp} kd {_fkd} "
              f"(GraspGenX MIMIC_KP 50 / FINGER_KP 2000 = 0.025, MIMIC_KD 10 / FINGER_KD 200 = 0.05)")
# ------------------------------------------------------------------------------

# --- contact solver: GraspGenX's own iteration counts --------------------------
# NVIDIA GraspGenX end2end/dynamic_playback.py:91-98:
#   SOLVER_ITERATIONS = 100, SOLVER_LS_ITERATIONS = 50, with the comment
#   "With only 10 iterations the constraint solver doesn't fully converge and
#    grasps slip during the lift segment."
# Their object mass (0.2 kg) and friction (object mu 10, finger mu 3) we already
# match; the iteration count is the one setting we never did (12/4 here).
SOLVER_IT = int(os.environ.get("SOLVER_IT", "12"))
SOLVER_VIT = int(os.environ.get("SOLVER_VIT", "4"))
if os.environ.get("SOLVER_IT"):
    print(f"[solver] articulation iterations {SOLVER_IT}/{SOLVER_VIT} (GraspGenX 100/50)")
# ------------------------------------------------------------------------------

cfg.spawn = cfg.spawn.replace(collision_props=sim_utils.CollisionPropertiesCfg(
    contact_offset=0.002, rest_offset=0.0))   # fingers as thin as they are: no phantom floor contact
cfg.spawn = cfg.spawn.replace(
    articulation_props=sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=False, solver_position_iteration_count=SOLVER_IT,
        solver_velocity_iteration_count=SOLVER_VIT,
        fix_root_link=not (WBC or SONIC)))
# Dex3-1 finger torque, from Unitree's own URDF: every hand joint in
# g1_29dof_with_hand_rev_1_0.urdf is <limit effort="1.4" velocity="12">.
# IsaacLab's G1_29DOF_CFG leaves the hands at effort_limit=300, 214x the real
# actuator, so the closing fingers drive straight through the object and PhysX
# ejects it instead of stalling them on contact. The trajectory commands the
# fingers all the way to their joint limits (end2end/tasks.py ramps to
# close_vals), so what stops them has to be the actuator, not the command.
# ARM_KP_SCALE: with the body on its PD drives (PD_BODY) the stock arm gains let an
# outstretched arm sag -- measured on the crate approach, the left palm 10 cm under
# its reference while the IK was within 24 mm -- and the sagging hand dragged the crate.
if os.environ.get("ARM_KP_SCALE") and "arms" in cfg.actuators:
    _k = float(os.environ["ARM_KP_SCALE"])
    cfg.actuators["arms"] = cfg.actuators["arms"].replace(
        stiffness={n: v * _k for n, v in cfg.actuators["arms"].stiffness.items()} if isinstance(cfg.actuators["arms"].stiffness, dict) else cfg.actuators["arms"].stiffness * _k,
        damping={n: v * _k ** 0.5 for n, v in cfg.actuators["arms"].damping.items()} if isinstance(cfg.actuators["arms"].damping, dict) else cfg.actuators["arms"].damping * _k ** 0.5)
# ARM_EFFORT: measured in the probe, not guessed. At the low reach right_shoulder_roll holds
# 184 mrad from its command with the drive clamped at its 300 Nm ceiling; an 8x kp sweep barely
# moved it (149/181/186 mrad at kp 3000/12000/24000), so it is not a stiffness deficit, and the
# base is rigid to 0.00 mm, self-collision is off, joint friction is 0 and no hand link carries
# more than 1.4 N. Raising the ceiling is the one lever that moved the number: eff 300 -> 181.5,
# eff 1000 -> 76.4, eff 4000 -> 75.5 mrad (demand -901 Nm, unclamped), i.e. ~105 mrad ~ 47 mm off
# the 80 mm execution error. It is a simulation-fidelity concession, not a physical fix: the real
# actuator is rated effort="25" in g1_29dof_rev_1_0.urdf, so the stock 300 is already 12x and
# 1000 is 40x. Measured side effect: shoulder_pitch's own error worsens 9.6 -> 21.3 mrad.
if os.environ.get("ARM_EFFORT") and "arms" in cfg.actuators:
    cfg.actuators["arms"] = cfg.actuators["arms"].replace(
        effort_limit=float(os.environ["ARM_EFFORT"]), effort_limit_sim=None)
# WAIST_EFFORT: the same lever as ARM_EFFORT, on the group it was never applied to. With FIX_ROOT=1
# the root is welded, so only the waist and the right arm can move the hand -- and on v44's close pose
# (test_th44.txt, "commanded vs achieved at the close pose") the four worst hand-relevant joints are
# waist_yaw 92.6, right_shoulder_roll 74.8, waist_roll 65.4, waist_pitch 53.6 mrad. The arm's 74.8 is
# already at the floor ARM_EFFORT can buy (eff 1000 -> 76.4, eff 4000 -> 75.5 mrad, above), but the
# waist's 211.6 mrad over three joints has never been touched: ARM_EFFORT only replaces
# cfg.actuators["arms"]. IsaacLab's own G1_29DOF_CFG (isaaclab_assets/robots/unitree.py:478-502) gives
# the waist group effort_limit yaw 88.0, roll 50.0, pitch 50.0 Nm -- 11x to 20x below the arm's current
# 1000 -- while already giving it a HIGHER stiffness than the arm (5000 vs 3000), which is why the
# ceiling and not kp is the lever here, exactly as the ARM_EFFORT probe found.
# What the shortfall costs, measured on v44: at the wrap pose Isaac puts right_wrist_yaw_link 86.2 mm
# from the commanded target (79.1 mm of it along world x, the finger direction) while cuRobo's own
# residual at the same frame is 13.5 mm. That drops the handle to palm-frame x ~51 mm -- inboard of the
# thumb root (69.1 mm) and 85 mm short of the index knuckle (136.5 mm; measured from our own asset,
# R_index_proximal_joint sits 178.0 mm off right_wrist_yaw_link, minus WRIST_TO_PALM 41.5 mm). That is
# why R_thumb_proximal and R_thumb_proximal_base are the only links ever to register force in v43 or
# v44, and why the fingers close to a full fist on air.
if os.environ.get("WAIST_EFFORT") and "waist" in cfg.actuators:
    cfg.actuators["waist"] = cfg.actuators["waist"].replace(
        effort_limit=float(os.environ["WAIST_EFFORT"]), effort_limit_sim=None)
if os.environ.get("HAND") == "inspire": pass
else: cfg.actuators["hands"] = cfg.actuators["hands"].replace(
    effort_limit=1.4, velocity_limit=12.0)
# The legs, when the pelvis is welded: PhysX's own PD, not IsaacLab's
# DC-motor model.
#
# This is the jittering that starts the moment the walk ends, in every
# delivered video. G1_29DOF_CFG drives the legs with DCMotorCfg, whose torque
# limit shrinks with the joint's velocity -- and the velocity it is handed is
# PhysX's estimate, which the SimulationContext itself warns is noisy at this
# setting (measured: 20-26 rad/s reported on joints that had not moved). In
# the bent stance the walk ends in, gravity loads the knees and hips; each
# bad velocity sample collapses the torque limit, the leg drops, the next
# sample restores it, the leg snaps back at the velocity clamp. With the legs
# straight down (the config's default pose) the load is small and nothing
# shows, which is why the no-walk picks never had it.
#
# Measured over the 830-frame pick, leg joint motion between rendered frames:
#   DC motor, desk as one block          384 mrad/frame mean   HELD
#   DC motor, desk as slab + legs         98                   HELD
#   DC motor, pelvis 2 cm higher         285                   LOST
#   DC motor, forces every iteration     113                   LOST
#   implicit PD, same gains               0.09                 HELD
# The gains are the config's own; only the torque model changes. --dc-legs
# keeps the old model, for comparison.
if not (WBC or SONIC) and "--dc-legs" not in sys.argv:
    from isaaclab.actuators import ImplicitActuatorCfg
    for _g in ("legs", "feet"):
        _a = cfg.actuators[_g]
        cfg.actuators[_g] = ImplicitActuatorCfg(
            joint_names_expr=_a.joint_names_expr, stiffness=_a.stiffness,
            damping=_a.damping, armature=getattr(_a, "armature", None),
            effort_limit_sim=_a.effort_limit)
    print("[play] legs and feet on PhysX's implicit PD, the config's gains")
if SONIC:
    # SONIC's own gains, computed the way policy_parameters.hpp computes them:
    # stiffness = armature * (2*pi*10)^2, damping = 2 * 2 * armature * (2*pi*10),
    # with the ankles at twice that. Driving its actions through anything else
    # is driving a different robot than the one it was trained on.
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from sonic_control import JOINTS as _SJ, STIFFNESS as _SK, DAMPING as _SD
    if os.environ.get("SONIC_ACTUATORS", "0") == "1":
        # The body actuators SONIC was trained on, loaded from its own training
        # config rather than patched onto G1_29DOF_CFG's: GR00T-WholeBodyControl
        # gear_sonic/envs/manager_env/robots/g1.py:239-354 (G1_CYLINDER_MODEL_12_DEX_CFG,
        # all ImplicitActuatorCfg, with its effort/velocity limits and armature).
        # Loaded by file path so gear_sonic's package __init__ is not imported.
        import importlib.util as _ilu
        _spec = _ilu.spec_from_file_location("_sonic_g1", "/home/sehoon/Projects/GR00T-WholeBodyControl/"
                                             "gear_sonic/envs/manager_env/robots/g1.py")
        _sg1 = _ilu.module_from_spec(_spec)
        _spec.loader.exec_module(_sg1)
        # SONIC_LEGS_ONLY=1: decoupled_wbc's split (g1_decoupled_whole_body_policy.py:118-143):
        # the policy drives the legs, the 17 upper-body joints (waist + arms, the deploy's
        # has_upper_body set, policy_parameters.hpp:80) take their targets directly. Those
        # keep IsaacLab's G1_29DOF_CFG arm/waist actuators (unitree.py:388-533).
        # SONIC_LOWER_N: how many of sonic_control JOINTS[:N] the policy drives -- 15 (legs + waist,
        # decoupled_wbc's lower body, g1_decoupled_whole_body_policy.py:141-143) or 12 (legs only;
        # waist + arms = the deploy's 17-joint has_upper_body set, policy_parameters.hpp:80, driven
        # straight to the clip). The rest keep IsaacLab's G1_29DOF_CFG actuators (unitree.py:388-533).
        _legs_only = os.environ.get("SONIC_LEGS_ONLY", "0") == "1"
        _lower_n = int(os.environ.get("SONIC_LOWER_N", "15")) if _legs_only else 29
        _sonic_groups = ("legs", "feet") if _lower_n <= 12 else (("legs", "feet", "waist") if _lower_n <= 15 else ("legs", "feet", "waist", "arms"))
        for _g in _sonic_groups:
            cfg.actuators.pop(_g, None)
        for _g, _act in _sg1.G1_CYLINDER_MODEL_12_DEX_CFG.actuators.items():
            if _g not in _sonic_groups:
                continue
            cfg.actuators["sonic_" + _g] = _act
        print(f"[sonic] body actuators from gear_sonic g1.py: {sorted(_sonic_groups)}")
    _kp = dict(zip(_SJ, _SK))
    _kd = dict(zip(_SJ, _SD))
    if os.environ.get("SONIC_LEGS_ONLY", "0") == "1":
        _ln = int(os.environ.get("SONIC_LOWER_N", "15"))
        _kp = {_n: _v for _n, _v in _kp.items() if _SJ.index(_n) < _ln}
        _kd = {_n: _v for _n, _v in _kd.items() if _SJ.index(_n) < _ln}
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
# The pelvis is welded for the pick, and it has to be welded at the height the
# leg pose actually stands at. --legs-from takes the walk's last leg angles;
# those angles put the pelvis at the height the walk ended on, not at the
# config's spawn height. Measured: 0.7622 against 0.7500, so the weld drove
# the feet 12.2 mm into the floor and held them there. The contact pushes out,
# the PD pushes back, and the legs shake for the whole pick -- 21.6 rad/s of
# joint speed while the arm worked. Raising the leg gains only makes the fight
# harder. Take the height from the same clip the angles came from.
_PELVIS_Z = cfg.init_state.pos[2]
if "--legs-from" in sys.argv:
    import joblib as _jl
    _lw = list(_jl.load(sys.argv[sys.argv.index("--legs-from") + 1]).values())[0]
    _PELVIS_Z = float(np.asarray(_lw["root_trans_offset"])[-1, 2])
    print(f"[play] pelvis welded at {_PELVIS_Z:.4f} m, the height the walk "
          f"ends at (config spawn is {cfg.init_state.pos[2]:.4f})")
if "--pick-stand" in sys.argv:
    _i = sys.argv.index("--pick-stand")
    STAND = (float(sys.argv[_i + 1]), float(sys.argv[_i + 2]), _PELVIS_Z)
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
            enabled_self_collisions=False, solver_position_iteration_count=SOLVER_IT,
            solver_velocity_iteration_count=SOLVER_VIT, fix_root_link=(os.environ.get("FIX_ROOT", "0") == "1")))
    _q0 = walk["quat"][0]
    cfg.init_state = cfg.init_state.replace(
        pos=tuple(float(v) for v in walk["pos"][0]),
        rot=(float(_q0[3]), float(_q0[0]), float(_q0[1]), float(_q0[2])))
elif DRIVE is not None:
    cfg = cfg.replace(spawn=cfg.spawn.replace(
        articulation_props=cfg.spawn.articulation_props.replace(
            fix_root_link=(os.environ.get("FIX_ROOT", "0") == "1"))))
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
# --- CONTACT_FORCE: measure how hard the fingertips push the object ----------
# IsaacLab's own dexterous-hand task (manager_based/manipulation/dexsuite/config/
# kuka_allegro/dexsuite_kuka_allegro_env_cfg.py:43-56) hangs a ContactSensor on every
# fingertip, filtered to the object, and calls the grasp good when the thumb and one
# opposing finger each exceed 1.0 N (mdp/rewards.py:50-71, threshold passed at :111);
# it clips the observation at 20 N with the note "contact force in finger tips is under
# 20N normally". We have never measured this number. A ContactSensor only reports if the
# body was spawned with contact reporting on. Unset -> the run is byte-for-byte unchanged.
# cfg.spawn.replace(activate_contact_sensors=True) is the documented way and it does not
# work here: it is the only configuration that fails when nothing else is running.
# Measured 2026-09-29, five launches: CONTACT_FORCE unset + cameras on succeeds (ae43c0),
# CONTACT_FORCE=1 + cameras on fails alone (contact43c), with 12 errors
# "failed to find internal joint object for PhysxMimicJointAPI at /World/G1/joints/*"
# then "Failed to create articulation at: /World/G1/root_joint" out of sim.reset().
# IsaacLab 2.3.2 schemas.py:551-554 is why the flag touches anything but the fingertips:
# inside `if child_prim.HasAPI(UsdPhysics.RigidBodyAPI)` it writes the sleep threshold to
# `prim`, the path the caller passed, not `child_prim` -- so every rigid body found stamps
# physxRigidBody:sleepThreshold onto /World/G1, the articulation root Xform. That the write
# is what breaks the mimic joints is unverified; what is verified is that the flag breaks
# the run and that we do not need it. A ContactSensor only requires the reporting API on
# the bodies it reads, so apply it to exactly those and leave the root alone.
robot = Articulation(cfg)
if SONIC and os.environ.get("FIX_ROOT", "0") != "1":
    # No weld: the pelvis is SONIC's to move. fix_root_link=False only disables the
    # USD's authored /World/G1/root_joint (IsaacLab schemas.py:180-184), and a disabled
    # joint whose world frame does not match the spawn pose still breaks the parse
    # ("disjointed body transforms" -> mimic joints -> "Failed to create articulation").
    # SONIC's own robot has no world joint at all (gear_sonic g1.py:201 fix_base=False),
    # so remove ours the same way: deactivate the prim before sim.reset().
    # In this asset the ArticulationRootAPI sits on that joint (measured: root_joint is a
    # PhysicsFixedJoint with PhysicsArticulationRootAPI, body0 = world at (0,0,0), body1 =
    # pelvis), so the root API moves to the pelvis -- where IsaacLab's URDF importer puts
    # it for a floating base -- and our articulation properties are applied to it again.
    from pxr import UsdPhysics as _UP, PhysxSchema as _PS
    _rj = stage.GetPrimAtPath("/World/G1/root_joint")
    _pel = stage.GetPrimAtPath("/World/G1/pelvis")
    assert _rj.IsValid() and _pel.IsValid(), "root_joint/pelvis not where the asset had them"
    _rj.SetActive(False)
    _UP.ArticulationRootAPI.Apply(_pel)
    _PS.PhysxArticulationAPI.Apply(_pel)
    sim_utils.schemas.modify_articulation_root_properties("/World/G1", cfg.spawn.articulation_props)
    print("[sonic] /World/G1/root_joint deactivated, articulation root on the pelvis: "
          "floating base, nothing welds it")
# The report API goes on the OBJECT, not on the hand. Measured 2026-09-29, alone on the
# machine: with it on the 12 /World/G1/R_* bodies the articulation dies in sim.reset()
# ("failed to find internal joint object for PhysxMimicJointAPI", 12 of them) whether the
# ContactSensor is built (contact43d) or not (cf_A, CF_SENSOR=0), and the 12 errors name
# the L_ joints as well, which that pass never touched. Those links are the mimic children.
# A sensor only needs the API on the bodies it reads, and force_matrix_w is
# (envs, bodies, filters, 3) -- so one sensor on the object, filtered to each fingertip,
# gives the same per-finger newtons without putting anything on the articulation.
if os.environ.get("CONTACT_FORCE"):
    import re as _re
    _fpat = _re.compile("^" + os.environ.get("CONTACT_LINKS", "/World/G1/R_.*") + "$")
    _flinks = sorted(str(_p.GetPath()) for _p in stage.Traverse()
                     if _fpat.match(str(_p.GetPath())) and _p.HasAPI(UsdPhysics.RigidBodyAPI))
    print(f"[force] {len(_flinks)} filter links found")
stiffen_mimic(stage)   # HAND=inspire: rigid four-bar fingertips (build_reach_reference.py)

# --- MIMIC_URDF_RATIO: the thumb's four-bar ratios, from the manufacturer URDF -------
# Measured in hammer/v23: at f480, with the four fingers still only at 0.38 rad,
# R_thumb_intermediate already sat at -0.16 and R_thumb_distal at -0.24 (both their USD
# lower limits), and R_thumb_proximal_pitch then held 0.000 rad against its 0.5 rad
# target from f520 to f760 -- snapping to 0.5 / 0.8 / 1.2 at f800, the frame after the
# hammer left. Every contact in that window landed on R_thumb_intermediate /
# R_thumb_distal, never the pad. The four fingers meanwhile curled to 0.96..1.03 rad
# and stalled on the handle, which is what a working wrap looks like.
#
# The ratios we drive the thumb with are not this hand's. gripper_descriptions/
# x_grippers/inspire_hand/gripper_spherical_dof.urdf declares:
#     thumb_intermediate_joint  mimic thumb_proximal_pitch  mult 1.334   limit 0 .. 0.8
#     thumb_distal_joint        mimic thumb_proximal_pitch  mult 0.667   limit 0 .. 0.4
#     thumb_proximal_pitch_joint (master)                                limit 0 .. 0.6
# Our USD authors those two at gearing -1.6 / -2.4 and soft_mimic mirrors it with
# +1.6 / +2.4, so a 0.5 rad close commands thumb_distal to 1.2 rad: 3x past the real
# joint's whole 0.4 rad travel, at 3.6x the URDF's multiplier. The tip curls under
# faster than the intermediate wraps.
#
# Sign convention, from NVIDIA's own test (omni.physx.tests PhysxMimicJointAPI.py,
# test_prismatic_simple / test_revolute_simple): with gearing = 1.0 and offset = 0.0 the
# assertions are linkAPos[axis] == -linkBPos[axis] and linkAAngleDegree ==
# -linkBAngleDegree, and in that setup both joints share one axis and one parent with
# localPos0 at each link's rest position, so q_follower = -gearing * q_reference
# - offset. The asset's negative gearings therefore drive the followers the same
# direction soft_mimic's positive table does; only the magnitude is wrong.
if os.environ.get("HAND") == "inspire" and os.environ.get("MIMIC_URDF_RATIO") == "1":
    import build_reach_reference_opus as _brr
    _URDF_MULT = {"thumb_intermediate_joint": 1.334, "thumb_distal_joint": 0.667}
    _brr._MIMIC[:] = [(a, b, _URDF_MULT.get(a.split("_", 1)[1], r)) for a, b, r in _brr._MIMIC]
    print("[hand] MIMIC_URDF_RATIO: soft_mimic thumb ratios -> "
          + ", ".join(f"{a} {r}" for a, b, r in _brr._MIMIC if "thumb" in a))
    from pxr import Usd as _Usd
    _ng = 0
    for _prim in _Usd.PrimRange(stage.GetPrimAtPath("/World/G1")):
        _k = next((k for k in _URDF_MULT if _prim.GetName().endswith(k)), None)
        if _k is None:
            continue
        for _a in _prim.GetAttributes():
            _nm = _a.GetName()
            if _nm.startswith("physxMimicJoint:") and _nm.endswith(":gearing"):
                _old = _a.Get()
                _a.Set(-_URDF_MULT[_k])
                print(f"[hand] MIMIC_URDF_RATIO: {_prim.GetName()} {_nm} {_old} -> {-_URDF_MULT[_k]}")
                _ng += 1
    print(f"[hand] MIMIC_URDF_RATIO: {_ng} thumb mimic gearings rewritten"
          + ("  <<< EXPECTED 4, CHECK PRIM NAMES" if _ng != 4 else ""))

# --- finger colliders: GraspGenX decomposes the intermediate/distal links ------
# end2end/robot_profiles.py UR10eInspireHandProfile:
#     coacd_link_keywords = ("intermediate", "distal")
#     "The thumb tip / distal and the *_intermediate links have the concavity
#      that matters for object contact."
# Our IsaacLab G1+Inspire asset ships every finger collider as a convex HULL
# (measured on assets/g1_inspire/g1_29dof_inspire_hand.usd: 50 collider meshes,
# all physics:approximation = convexHull, none authored otherwise), so the
# concave curl of each finger is filled in solid in physics. Measured in hammer
# v14: R_thumb_proximal_pitch held 0.000 rad against its 0.5 rad target for 290
# frames (480-770) and reached 0.5 within 30 frames once the object was gone.
# The USD is instanceable, so each link's "collisions" scope is de-instanced
# first (plan_scene.keep_only_hand_collisions:266 documents that constraint).
# Unset -> nothing is touched and the run is unchanged.
_coacd = os.environ.get("FINGER_COACD", "")
if _coacd:
    from pxr import Usd as _Usd2, UsdPhysics as _UsdPh2
    _kw = tuple(k for k in _coacd.split(",") if k)
    _cdone = []
    _nmesh = 0
    for _link in stage.GetPrimAtPath("/World/G1").GetChildren():
        if not (_link.GetName().startswith(("R_", "right_wrist")) and any(k in _link.GetName() for k in _kw)):
            continue
        _col = stage.GetPrimAtPath(_link.GetPath().AppendChild("collisions"))
        if not (_col and _col.IsValid()):
            continue
        _col.SetInstanceable(False)
        for _m in _Usd2.PrimRange(_col):
            if _m.HasAPI(_UsdPh2.CollisionAPI):
                _UsdPh2.MeshCollisionAPI.Apply(_m).CreateApproximationAttr("convexDecomposition")
                _nmesh += 1
                _cdone.append(_link.GetName())
    print(f"[hand] finger colliders -> convexDecomposition on {_kw}: {_nmesh} meshes "
          f"(GraspGenX coacd_link_keywords)")
    print(f"[hand] coacd links ({len(set(_cdone))}): {sorted(set(_cdone))}", flush=True)

# ------------------------------------------------------------------------------
if os.environ.get("BODY_COLLISION", "0") == "0" and "--hands" in sys.argv:
    from plan_scene import keep_only_hand_collisions
    print(f"[walk] body does not collide with the cell; hand links kept: {len(keep_only_hand_collisions(stage, keep=HAND_KEEP))}")

# Finger pad friction, from GraspGenX's own --finger_mu default of 3.0. Left
# alone the stage runs on PhysX's 0.5, and a grasp generated under mu 3 on the
# pads slips the moment the fingers touch the box.
_fm = sim_utils.RigidBodyMaterialCfg(static_friction=FINGER_MU, friction_combine_mode="max",   # as the tester
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

# --- contact solver on the object body, same source ---------------------------
_oit = int(os.environ.get("OBJ_SOLVER_IT", os.environ.get("SOLVER_IT", "0")))
if _oit:
    from pxr import PhysxSchema as _PxS
    _tp = stage.GetPrimAtPath(os.environ.get("TARGET_PRIM", "/World/GraspTarget"))
    if _tp and _tp.IsValid():
        _ovit = int(os.environ.get("OBJ_SOLVER_VIT", str(SOLVER_VIT)))
        _api = _PxS.PhysxRigidBodyAPI.Apply(_tp)
        _api.CreateSolverPositionIterationCountAttr(_oit)
        _api.CreateSolverVelocityIterationCountAttr(_ovit)
        print(f"[solver] object {_tp.GetPath()} iterations {_oit}/{_ovit}")

# --- depenetration cap on the object body -------------------------------------
# Every IsaacLab manipulation config that grasps a rigid body caps the velocity
# the solver may introduce to push a penetrating pair apart:
# max_depenetration_velocity=5.0 in manipulation/lift/config/franka/joint_pos_env_cfg.py:58,
# direct/factory/factory_tasks_cfg.py and manipulation/deploy/gear_assembly/*.
# Ours never set it (v12's dump_physics lists no such attribute on
# /World/GraspTarget), so the solver may introduce any velocity it likes.
# Unset here -> the attribute is not authored and the run is unchanged.
_mdv = os.environ.get("OBJ_MAX_DEPEN_VEL", "")
if _mdv:
    from pxr import PhysxSchema as _PxS2
    _tp2 = stage.GetPrimAtPath(os.environ.get("TARGET_PRIM", "/World/GraspTarget"))
    if _tp2 and _tp2.IsValid():
        _PxS2.PhysxRigidBodyAPI.Apply(_tp2).CreateMaxDepenetrationVelocityAttr(float(_mdv))
        print(f"[phys] object maxDepenetrationVelocity {_mdv} m/s (IsaacLab manipulation cfgs use 5.0)")
# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------

from plan_scene import floor_slab

floor_slab(stage)

# Track the object in Isaac too: the Newton replay is a different engine, and
# what matters is what this one does. Wrapped before reset -- an asset made
# after it is never initialised.
target_body = None
if "object" in meta:
    from isaaclab.assets import RigidObject, RigidObjectCfg  # noqa: E402
    target_body = RigidObject(RigidObjectCfg(prim_path=os.environ.get("TARGET_PRIM", "/World/GraspTarget"),
                                             spawn=None))

from plan_scene import dump_physics; dump_physics(stage)
# The sensor has to exist before sim.reset(): PhysX builds its scene there. One sensor
# with a regex over the right-hand links gives force_matrix_w of shape (1, B, 1, 3) --
# per-link force against the one filtered body, the object -- with sensor.body_names
# giving the B ordering (contact_sensor_data.py:97-105).
_fsensor = None
# CF_SENSOR=0 keeps the contact-report API on the fingertips but builds no ContactSensor:
# it splits "the API application broke the articulation" from "the sensor object did".
if os.environ.get("CONTACT_FORCE") and target_body is not None and os.environ.get("CF_SENSOR", "1") != "0":
    from isaaclab.sensors import ContactSensor, ContactSensorCfg  # noqa: E402
    _tprim = os.environ.get("TARGET_PRIM", "/World/GraspTarget")
    # applied here, not at robot spawn: the target prim does not exist yet up there
    # (pxr.Tf.ErrorException "Invalid prim 'null prim'", contact43e).
    PhysxSchema.PhysxContactReportAPI.Apply(
        stage.GetPrimAtPath(_tprim)).CreateThresholdAttr().Set(0.0)
    _fsensor = ContactSensor(ContactSensorCfg(
        prim_path=_tprim, filter_prim_paths_expr=_flinks,
        history_length=0, update_period=0.0))
    print(f"[force] contact sensor on {_tprim} filtered to {len(_flinks)} links: "
          + ", ".join(n.rsplit("/", 1)[-1] for n in _flinks))


def _force_line(i):
    """Per-fingertip contact force against the object, in newtons.

    DexSuite's criterion (mdp/rewards.py:50-71) is thumb > 1.0 N AND one opposing finger
    > 1.0 N; its observation clips at 20 N as the normal range. Printing the magnitudes
    lets us compare, instead of assuming what the squeeze is doing.
    """
    if _fsensor is None:
        return
    _fsensor.update(sim.get_physics_dt(), force_recompute=True)
    _fm = _fsensor.data.force_matrix_w
    if _fm is None:
        print(f"[force] frame {i:5d} no filtered contact data"); return
    _mag = _fm[0, 0, :, :].norm(dim=-1).cpu().numpy()
    _nm = [n.rsplit("/", 1)[-1] for n in _flinks]
    # Control for the instrument itself: the object rests on /World/FloorSlab, a static
    # collider that cannot be a filter, so its weight shows up only in the net force.
    # 0.2 kg (plan_scene.py:83) -> 1.96 N. A net near 0 means the sensor is not reporting
    # and a row of zeros below would say nothing about whether the fingers touch.
    _net = float(_fsensor.data.net_forces_w[0, 0].norm())
    _hit = [(n, m) for n, m in zip(_nm, _mag) if m > 0.01]
    _hit.sort(key=lambda t: -t[1])
    _thumb = max([m for n, m in zip(_nm, _mag) if "thumb" in n], default=0.0)
    _other = max([m for n, m in zip(_nm, _mag) if "thumb" not in n], default=0.0)
    print(f"[force] frame {i:5d} net {_net:6.2f} N  sum {_mag.sum():7.2f} N  max {_mag.max():6.2f} N  "
          f"thumb {_thumb:6.2f} N  best-opposing {_other:6.2f} N  "
          f"dexsuite_good={bool(_thumb > 1.0 and _other > 1.0)}  "
          + ", ".join(f"{n} {m:.2f}" for n, m in _hit[:6]))

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


obj_start = None
if target_body is not None:
    target_body.update(0.0)
    obj_start = target_body.data.root_pos_w[0].cpu().numpy()
    print(f"[obj ] start {np.round(obj_start,4)}")

# the plan's joint list is the Dex3 arm+hand; with another hand (HAND=inspire) the hand
# entries do not exist -- they are only needed when the plan drives the arm (not --clip-arms)
missing = [n for n in names if n not in robot.joint_names]       # find_joints raises on a missing name
names = [n for n in names if n not in missing]
ids = [robot.find_joints([n])[0][0] for n in names]
if missing and "--clip-arms" not in sys.argv:
    print(f"[play] joints missing on the Isaac G1: {missing}")
    raise SystemExit("the plan drives joints this hand does not have")
if missing:
    print(f"[play] plan joints not on this hand, ignored (--clip-arms): {len(missing)}")

# Frame the reach: far enough back to see the robot, the object it goes for,
# and the rack behind them. Aim at the object when the plan carries one.
tgt = np.array([-1.30, -0.75, 0.95])
if "object" in meta:
    tgt = 0.5 * (tgt + np.array(T_torso @ np.array(
        meta["object"]["transform_in_torso"]))[:3, 3])
# --cam-eye DX DY DZ: where the room camera stands relative to what it looks
# at. The default looks from the south-west, which is the right side of the
# desk for a pick from the desk; for a box on the floor north-east of the
# desk it puts the desk and the crate between the camera and the robot.
if "--look-at" in sys.argv:   # what the room camera watches, e.g. the crate's seen centre
    tgt = np.array([float(sys.argv[sys.argv.index("--look-at") + k]) for k in (1, 2, 3)])
_ce = (np.array([float(sys.argv[sys.argv.index("--cam-eye") + k]) for k in (1, 2, 3)])
       if "--cam-eye" in sys.argv else np.array([-1.65, -2.05, 1.05]))
eye = tgt + _ce
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
    _body_ids = [j for j in range(robot.num_joints) if "hand" not in robot.joint_names[j]]
    if "--arm-pd" in sys.argv:
        # The right arm stays a PD articulation, as in every pick that held;
        # written into place every substep it is a fixture the fingers press
        # the object against, and the contact solver throws the object.
        _body_ids = [j for j in _body_ids if not (robot.joint_names[j].startswith("right_")
                     and ("shoulder" in robot.joint_names[j] or "elbow" in robot.joint_names[j]
                          or "wrist" in robot.joint_names[j]))]
        print(f"[walk] right arm PD-driven; {len(_body_ids)} joints written")
    print(f"[walk] replaying {len(walk['dof'])} frames: legs from the clip, "
          f"arm held at the plan's start pose, waist eased upright over the "
          f"last {_waist_n}")
    # --clip-arms: the clip carries the arms too (a whole-body reach solved
    # by cuRobo's retargeter), so nothing is overridden; --hands SCHEDULE.npy
    # drives the fourteen finger joints from a (frames, 14) table in GR00T's
    # G1_HAND_JOINTS order, the same file the evaluator's SONIC_HAND_SCHEDULE
    # hook reads.
    CLIP_ARMS = "--clip-arms" in sys.argv
    _hands = None
    _vmode = False
    if "--hands" in sys.argv:
        _hands = np.load(sys.argv[sys.argv.index("--hands") + 1])
        _hand_names = list(HAND_NAMES)
        _hand_ids = [robot.find_joints([n])[0][0] for n in _hand_names]
        print(f"[walk] fingers from {os.path.basename(sys.argv[sys.argv.index('--hands') + 1])}: "
              f"{_hands.shape}")
        # CLOSE_MODE=velocity: the Dex3 close the way GraspGenX runs it
        # (end2end/robots/g1_right_arm.yaml gripper_control_mode: velocity):
        # while the schedule's right hand is away from open, the right
        # fingers run with stiffness 0 and damping CLOSE_KD at CLOSE_VEL rad/s
        # towards closed, and squeeze at whatever they meet. Position mode
        # "snaps the fingers to the closed angles and they bat the object
        # away" (their comment; measured here 0/154, 0/40).
        _vmode = os.environ.get("CLOSE_MODE", "position") == "velocity"
        _open_r = _hands[0, len(_hand_names) // 2:].copy()
        _closed_r = _hands[int(np.argmax(np.abs(_hands[:, len(_hand_names) // 2:] - _open_r).sum(1))), len(_hand_names) // 2:]
        _vel_ids = [j for j, n in zip(_hand_ids[len(_hand_names) // 2:], _hand_names[len(_hand_names) // 2:]) if "thumb_0" not in n]
        _vel = torch.tensor([[float(os.environ.get("CLOSE_VEL", "0.25")) * float(np.sign(c - o))
                              for (j, n), c, o in zip(zip(_hand_ids[len(_hand_names) // 2:], _hand_names[len(_hand_names) // 2:]), _closed_r, _open_r)
                              if "thumb_0" not in n]], dtype=torch.float32, device=sim.device)
        _stiff0 = robot.data.joint_stiffness[:, _vel_ids].clone()
        _damp0 = robot.data.joint_damping[:, _vel_ids].clone()
        _squeezing = False
        if _vmode:
            print(f"[walk] velocity-mode close: kd {os.environ.get('CLOSE_KD', '8.0')}, "
                  f"{_vel[0].cpu().numpy()} rad/s")
        # --- YAW_EFFORT: the held joint is the thumb's abduction ----------------
        # The one joint in _vel_ids whose close velocity is 0 is
        # R_thumb_proximal_yaw (open 1.308 == closed 1.308, so sign(c-o) == 0).
        # GraspGenX holds the same joint the same way -- robots/g1_inspire_arm.yaml
        # gripper_close_velocity right_hand_thumb_0_joint: 0.0, in velocity mode at
        # finger_velocity_kd (dynamic_playback.py:647, default 800) -- but that
        # profile's `dynamic: finger_effort_limit` is 1000.0, not the 200 default
        # (:689) and not our 30.
        # Measured here, hammer v57 (CLOSE_KD 800, effort 30), the delivered
        # evidence/render.log: the yaw is back-driven 1.30 -> 0.98 rad (-18 deg)
        # over f650..f800 while the four proximals hold near 1.0, i.e. the hand's
        # only opposing member is the one that gives way. 0.32 rad in 5.0 s =
        # 0.064 rad/s; at kd 800 holding that rate needs 51 Nm, above the 30 Nm
        # ceiling, and 30/800 = 0.038 rad/s is the drift rate the ceiling allows --
        # the same order as the 0.064 measured. So the ceiling, not kd, is what
        # lets the thumb spread. It cannot cause a close-time impulse the way
        # HAND_EFFORT 200 did in v59: this joint's velocity target is 0.
        if _vmode and os.environ.get("YAW_EFFORT"):
            _held = [j for j, v in zip(_vel_ids, _vel[0].tolist()) if abs(v) < 1e-9]
            if _held:
                robot.write_joint_effort_limit_to_sim(float(os.environ["YAW_EFFORT"]),
                                                      joint_ids=_held)
                print(f"[hand] YAW_EFFORT: {os.environ['YAW_EFFORT']} Nm on the held-velocity "
                      f"joint(s) {[robot.joint_names[j] for j in _held]}; the closing joints "
                      f"stay at {os.environ.get('HAND_EFFORT', '200')} Nm")
    if os.environ.get("ROOT_SUB", "0") == "1":
        _ps = np.asarray(walk["pos"], dtype=np.float64)
        _st = np.linalg.norm(np.diff(_ps, axis=0), axis=1) * 1000.0
        _ns = max(1, round((1.0 / FPS) / sim.get_physics_dt()))
        print(f"[walk] ROOT_SUB: the pelvis moves across {_ns} substeps, not in one jump. "
              f"biggest frame step {_st.max():.2f} mm at frame {int(_st.argmax()) + 1} "
              f"-> {_st.max() / _ns:.2f} mm per substep (finger contact offset 2.00 mm)")
    if os.environ.get("ROOT_VEL", "0") == "1":
        _ps = np.asarray(walk["pos"], dtype=np.float64)
        _sp = np.linalg.norm(np.diff(_ps, axis=0), axis=1) * FPS
        print(f"[walk] ROOT_VEL: writing the clip's root velocity, not zero "
              f"(IsaacLab humanoid_amp_env.py:165-166). clip root speed "
              f"mean {_sp.mean():.3f} m/s, max {_sp.max():.3f} m/s at frame {int(_sp.argmax()) + 1}")
    for _i in range(int(os.environ.get("DIAG_IDLE_FRAMES", "0"))):        # diagnostic: step with no writes at all, watch the object
        for _ss in range(max(1, round((1.0 / FPS) / sim.get_physics_dt()))):
            sim.step()
        if target_body is not None:
            target_body.update(sim.get_physics_dt())
            print(f"[obj ] idle {_i} pos {np.round(target_body.data.root_pos_w[0].cpu().numpy(), 4)}  vel {np.round(target_body.data.root_lin_vel_w[0].cpu().numpy(), 3)}")
    # --- settle before the stand-up: GraspGenX's settle_frames, on the clip -------
    # GraspGenX end2end/dynamic_playback.py:1049 `settle_frames: int = 30`, :1310-1312
    # "Stepping %d trajectory frames + %d settle frames": the demo steps extra frames
    # with the trajectory held before it judges the grasp, because the close only
    # reaches its contact equilibrium after the motion stops (:128-129 "physics
    # naturally limits how far the close actually goes when there's contact").
    # Measured, hammer v25: the clip already holds still at f720..f750 (palm z 0.3090,
    # root z 0.4210, object z 0.2680 constant for 40 frames) and the hammer survives it
    # with continuous contact; the stand-up starts between f750 and f760 (root z 0.4210
    # -> 0.4280 -> 0.4680) and the object is lost at f760, the stand-up's first frame,
    # with thumb_proximal_pitch still rising (0.350 of its 0.500 target, max at f760).
    # SETTLE_AT/SETTLE_FRAMES repeat one clip frame so the close keeps pressing while
    # the body stays where the grasp was formed. Diagnostics keep counting i, so the
    # settle shows up as the stretch where the body does not move.
    _settle_at = int(os.environ.get("SETTLE_AT", "-1"))
    _settle_n = max(0, int(os.environ.get("SETTLE_FRAMES", "0")))
    if _settle_at >= 0 and _settle_n:
        print(f"[walk] settle: clip frame {_settle_at} held for {_settle_n} extra frames "
              f"(GraspGenX dynamic_playback.py:1049 settle_frames=30, :1310)")
    def _clip_i(_i):
        if _settle_at < 0 or not _settle_n or _i <= _settle_at:
            return _i
        return _settle_at if _i <= _settle_at + _settle_n else _i - _settle_n
    _root_prev = None          # ROOT_SUB: last frame's root, to interpolate from
    _wsonic = None
    if SONIC:
        # No root write anywhere below: SONIC tracks the whole clip -- walk, kneel,
        # reach, stand-up -- and the pelvis goes where the legs put it. This is
        # GR00T's own deployment loop (g1_deploy_onnx_ref.cpp, ported in
        # sonic_control.py): reference in, 29 joint targets out at 50 Hz.
        from sonic_control import (SonicTracker, JOINTS as _SJW, CONTROL_DT as _SDT,
                                   clip_to_reference)
        assert list(walk["names"]) == list(_SJW), "clip joint order is not SONIC's"
        _wref, _wq, _wz = clip_to_reference(
            {"dof": walk["dof"], "root_trans_offset": walk["pos"], "root_rot": walk["quat"]}, FPS)
        _wsonic = SonicTracker(_wref, ref_quat=_wq, ref_root_z=_wz)
        _wsonic_ids = [robot.find_joints([n])[0][0] for n in _SJW]
        _wsonic_dec = max(1, round(_SDT / sim.get_physics_dt()))
        # The one initial condition: the robot starts in the clip's first pose
        # (spawned there by init_state; joints set once here, before any step).
        for _c, _j in enumerate(_wsonic_ids):
            tgt_q[0, _j] = float(walk["dof"][0, _c])
        robot.write_joint_state_to_sim(tgt_q, torch.zeros_like(tgt_q))
        robot.set_joint_position_target(tgt_q)
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim.get_physics_dt())
        _wsonic.prime(robot.data.joint_pos[0, _wsonic_ids].cpu().numpy().astype(np.float64),
                      robot.data.joint_vel[0, _wsonic_ids].cpu().numpy().astype(np.float64),
                      robot.data.root_quat_w[0].cpu().numpy().astype(np.float64),
                      robot.data.root_ang_vel_b[0].cpu().numpy().astype(np.float64))
        _wsonic_k = 0          # physics substeps since the clip started
        print(f"[sonic] tracking the whole clip: {len(_wref)} reference frames at "
              f"{1/_SDT:.0f} Hz, every {_wsonic_dec} physics steps; no root writes")

    def _wsonic_tick(t):
        """One 50 Hz SONIC tick at reference index t; writes the 29 body targets into tgt_q."""
        _out = _wsonic.step(min(t, _wsonic.T - 1),
                            robot.data.joint_pos[0, _wsonic_ids].cpu().numpy().astype(np.float64),
                            robot.data.joint_vel[0, _wsonic_ids].cpu().numpy().astype(np.float64),
                            robot.data.root_quat_w[0].cpu().numpy().astype(np.float64),
                            robot.data.root_ang_vel_b[0].cpu().numpy().astype(np.float64))
        for _c, _j in enumerate(_wsonic_ids):
            if _wsonic_legs_only and _c >= _wsonic_lower_n:
                continue             # upper body: the clip's own targets, below
            if _c < 12 and not _legs_from_policy:
                continue             # static phase: the legs follow the clip too (below)
            tgt_q[0, _j] = float(_out[_c])
    # SONIC_LEGS_ONLY=1: decoupled_wbc's split -- the policy moves the lower body (12 leg
    # + 3 waist joints, sonic_control JOINTS[:15]; g1_decoupled_whole_body_policy.py:141-143),
    # the 14 arm joints are PD-driven straight to the clip, as IsaacLab's own G1
    # locomanipulation drives the arms under its leg policy (pink_task_space_actions.py:307-321).
    # SONIC still observes them.
    _wsonic_legs_only = _wsonic is not None and os.environ.get("SONIC_LEGS_ONLY", "0") == "1"
    _wsonic_lower_n = int(os.environ.get("SONIC_LOWER_N", "15"))
    if _wsonic_legs_only:
        print(f"[sonic] legs only: JOINTS[:{_wsonic_lower_n}] from the policy, the other {29 - _wsonic_lower_n} from the clip")
    # SONIC_LEGS_DIRECT=A,B: between clip frames A and B (the kneel is static -- both knees on the
    # floor, no balance to keep) the legs too are PD-driven to the clip, so the whole-body reach
    # cuRobo planned (which tilts the pelvis, v2: 19 deg) is executed by the body that was planned,
    # as IsaacLab's fixed-base upper-body env drives every joint to its target. The policy takes the
    # legs back at B for the stand-up. Measured, v2: with the legs on the policy the reach's pelvis
    # tilt was not executed and the hand closed 16 cm above the hammer.
    def _dump_state(path, frame):
        """Where the robot really is (root pose, 29+ joints, object pose), for planning from
        there: the deployment perceives after arriving; floor_object_chain.sh step 5 "look
        again from the kneel" is the offline form of the same thing."""
        import json as _json
        robot.update(sim.get_physics_dt())
        _rp_, _rq_ = robot.data.root_pos_w[0].cpu().numpy(), robot.data.root_quat_w[0].cpu().numpy()
        _dq = robot.data.joint_pos[0].cpu().numpy()
        _dump = {"root_pos": [float(v) for v in _rp_], "root_quat_wxyz": [float(v) for v in _rq_],
                 "dof": {n: float(_dq[j]) for j, n in enumerate(robot.joint_names)}, "frame": int(frame)}
        if target_body is not None:
            target_body.update(sim.get_physics_dt())
            _dump["object_pos"] = [float(v) for v in target_body.data.root_pos_w[0].cpu().numpy()]
            _dump["object_quat_wxyz"] = [float(v) for v in target_body.data.root_quat_w[0].cpu().numpy()]
        _json.dump(_dump, open(path, "w"), indent=1)
        print(f"[walk] state at frame {frame} dumped to {path}")
    # DUMP_STATE_AT=frame: the same dump mid-clip (e.g. at the end of the reach, before the close)
    _dump_at = int(os.environ.get("DUMP_STATE_AT", "-1"))
    _legs_direct = ([int(v) for v in os.environ["SONIC_LEGS_DIRECT"].split(",")]
                    if os.environ.get("SONIC_LEGS_DIRECT") else None)
    _legs_from_policy = True
    if _legs_direct:
        print(f"[sonic] legs direct from the clip between frames {_legs_direct[0]} and {_legs_direct[1]} (static kneel)")
    for i in range(min(len(walk["dof"]) + _settle_n, int(os.environ.get("WALK_MAX_FRAMES", "1000000")))):   # WALK_MAX_FRAMES: a short diagnostic run
        ci = _clip_i(i)
        if i == _dump_at and os.environ.get("DUMP_STATE"):
            _dump_state(os.environ["DUMP_STATE"], i)
        _was_policy = _legs_from_policy
        _legs_from_policy = not (_legs_direct and _legs_direct[0] <= ci < _legs_direct[1])
        if _legs_direct and _was_policy != _legs_from_policy:
            # cuRobo executes its plans open loop on position drives at kp 1047.2 / kd 52.36
            # (curobo v0.7.7 examples/isaac_sim/helper.py:96-97, JOINT_DRIVE_POSITION), i.e. the
            # controller, not the planner, is what makes the executed pose match the plan. The
            # policy's soft leg gains (kp ~99) sagged 0.2 rad under the torso in v3 (pelvis 7.8 cm
            # back, 4.3 cm low; hand 10 cm off). So while the legs follow the plan they get
            # cuRobo's drive gains; the policy's own gains come back at the handover.
            _leg_ids = [_wsonic_ids[_c] for _c in range(12)]
            if not _legs_from_policy:
                _kp_c, _kd_c = [float(v) for v in os.environ.get("SONIC_LEGS_DIRECT_GAINS", "1047.19751,52.35988").split(",")]
                _leg_kp0 = robot.data.joint_stiffness[0, _leg_ids].clone()
                _leg_kd0 = robot.data.joint_damping[0, _leg_ids].clone()
                robot.write_joint_stiffness_to_sim(torch.full((1, 12), _kp_c, device=sim.device), joint_ids=_leg_ids)
                robot.write_joint_damping_to_sim(torch.full((1, 12), _kd_c, device=sim.device), joint_ids=_leg_ids)
                print(f"[sonic] frame {i}: legs on cuRobo's execution drives kp {_kp_c:.0f} kd {_kd_c:.1f} (were kp {_leg_kp0.mean():.0f})")
            else:
                robot.write_joint_stiffness_to_sim(_leg_kp0.unsqueeze(0), joint_ids=_leg_ids)
                robot.write_joint_damping_to_sim(_leg_kd0.unsqueeze(0), joint_ids=_leg_ids)
                print(f"[sonic] frame {i}: legs back on the policy's gains")
        for k, jid in enumerate(walk_ids):
            if (_wsonic is None or (_wsonic_legs_only and k >= _wsonic_lower_n)
                    or (k < 12 and not _legs_from_policy)):   # under SONIC the lower-body targets are its output
                tgt_q[0, jid] = float(walk["dof"][ci, k])
        if not CLIP_ARMS:
            for k, jid in enumerate(ids):
                tgt_q[0, jid] = _plan_q[k]
        if _hands is not None:
            for k, jid in enumerate(_hand_ids):
                tgt_q[0, jid] = float(_hands[min(ci, len(_hands) - 1), k])
        if ci >= _waist_from and not CLIP_ARMS:
            a = (ci - _waist_from + 1) / _waist_n
            for j in _waist:
                tgt_q[0, j] = ((1.0 - a) * float(tgt_q[0, j])
                               + a * float(robot.data.default_joint_pos[0, j]))
        q = walk["quat"][ci]
        if _wsonic is None:
          robot.write_root_state_to_sim(torch.tensor(
            [[float(walk["pos"][ci][0]), float(walk["pos"][ci][1]),
              float(walk["pos"][ci][2]), float(q[3]), float(q[0]),
              float(q[1]), float(q[2]), 0, 0, 0, 0, 0, 0]],
            dtype=torch.float32, device=sim.device))
        if _hands is not None:
            # body written, fingers simulated: a finger whose state is written
            # every step passes through the box instead of pressing on it.
            # And the fingers need the frame's full 33 ms of physics, not the
            # one millisecond step a kinematic replay gets away with -- at one
            # step per frame the hand closed 1.4 s of PD in a 1403-frame clip
            # and the box never moved.
            # PD_BODY (below): no joint is written, the way the tester replays a
            # held grasp (test_grasps_in_isaac.py put(): "no joint is written, the
            # root is placed once per frame"). Written once a frame here, the arm
            # was snapped onto the plan instead of PD-tracking it, and the #77
            # hammer grasp that held in the tester flipped the hammer in the render.
            if os.environ.get("PD_BODY", "1") != "1":
                robot.write_joint_state_to_sim(tgt_q[:, _body_ids], zero[:, _body_ids],
                                               joint_ids=_body_ids)
            robot.set_joint_position_target(tgt_q)
            if _vmode:
                _want = bool(np.abs(_hands[min(ci, len(_hands) - 1), len(_hand_names) // 2:] - _open_r).max() > 1e-6)
                if _want != _squeezing:
                    _squeezing = _want
                    print(f"[walk] frame {i}: fingers {'squeeze (velocity)' if _want else 'release (position)'}")
                    robot.write_joint_stiffness_to_sim(0.0 if _want else _stiff0, joint_ids=_vel_ids)
                    robot.write_joint_damping_to_sim(float(os.environ.get("CLOSE_KD", "8.0")) if _want else _damp0,
                                                     joint_ids=_vel_ids)
                robot.set_joint_velocity_target(_vel if _squeezing else torch.zeros_like(_vel), joint_ids=_vel_ids)
                # --- GRIP_EFFORT: the grip ceiling is set at close time and never raised ---
                # Measured, hammer v61 (the best run on record, 320 frames of
                # dexsuite_good=T): R_thumb_proximal_pitch -- the thumb's closing
                # master -- sits at q 0.000 rad against a 0.6 rad target from the
                # close (f519) to the loss (f925), the whole 400 frames. It is
                # blocked by the object, not by a constraint: v14 measured the same
                # joint reaching 0.5 rad within 30 frames once the object was gone.
                # So the thumb presses at whatever HAND_EFFORT allows -- 30 Nm.
                # In v61's rise that ceiling stayed put while the load quadrupled:
                #   f915 net 143 N -> f920 net 655 N (4.6x), with the object already
                #   chattering at 0.34/0.60/1.10 m/s inside the grip while the palm
                #   moved 0.30 m/s, and then 400 kN at f930.
                # GraspGenX's profile for this exact robot and hand --
                # end2end/robots/g1_inspire_arm.yaml, `dynamic: finger_effort_limit:
                # 1000.0` -- is 33x ours, and is not the 200 default of
                # dynamic_playback.py:689 either.
                # v59 raised HAND_EFFORT to 200 globally AT CLOSE TIME and the close
                # impulse ejected the object. This write lands at GRIP_EFFORT_FRAME,
                # long after the hand has come to rest on the object (v61: contact
                # equilibrium f605, rise f909), so it cannot make that impulse.
                # Unset -> nothing is written and the run is unchanged.
                if os.environ.get("GRIP_EFFORT") and i == int(os.environ.get("GRIP_EFFORT_FRAME", "850")):
                    robot.write_joint_effort_limit_to_sim(float(os.environ["GRIP_EFFORT"]),
                                                          joint_ids=_vel_ids)
                    print(f"[hand] GRIP_EFFORT: frame {i}, {os.environ['GRIP_EFFORT']} Nm on the "
                          f"{len(_vel_ids)} closing joints (was "
                          f"{os.environ.get('HAND_EFFORT', '200')} Nm)", flush=True)
            # ROOT_VEL: write the clip's own root velocity instead of zero.
            # IsaacLab's own motion replay does exactly this --
            # direct/humanoid_amp/humanoid_amp_env.py:165-166 fills
            # root_state[:, 7:10] / [10:13] from the clip's
            # body_linear_velocities / body_angular_velocities, and its
            # motion_loader carries a velocity per frame beside every pose.
            # Ours wrote zeros, so the solver is told the pelvis is standing
            # still while it is in fact being teleported. Measured in v55:
            #   hold  f700-900 (stable)  root steps 1.07 mm/frame, object
            #                            rotates 0.1-3 deg/sample
            #   rise  f915-920           2.63 mm   ->  9.3 deg
            #   rise  f920-925           6.79 mm   -> 44.8 deg
            #   rise  f925-930 (ejected) 13.00 mm (max 14.5) -> 94 deg,
            #                            object leaves at 420 m/s
            # The finger contact offset is 2 mm: the stable hold stays inside
            # it, the 14.5 mm step is 7x past it, and the declared velocity is
            # 0 where the true one is 0.44 m/s.
            _rv = [0.0] * 6
            if os.environ.get("ROOT_VEL", "0") == "1" and 0 < ci < len(walk["pos"]) - 1:
                _p0, _p1 = walk["pos"][ci - 1], walk["pos"][ci + 1]
                _rv[:3] = [float((_p1[k] - _p0[k]) * FPS / 2.0) for k in range(3)]
                _a, _b = walk["quat"][ci - 1], walk["quat"][ci + 1]      # xyzw
                # dq = b * conj(a); omega_world = 2 * dq.xyz / dt (small angle)
                _ax, _ay, _az, _aw = (-_a[0], -_a[1], -_a[2], _a[3])
                _bx, _by, _bz, _bw = _b
                _dx = _bw * _ax + _bx * _aw + _by * _az - _bz * _ay
                _dy = _bw * _ay - _bx * _az + _by * _aw + _bz * _ax
                _dz = _bw * _az + _bx * _ay - _by * _ax + _bz * _aw
                _dw = _bw * _aw - _bx * _ax - _by * _ay - _bz * _az
                _sg = 1.0 if _dw >= 0 else -1.0                          # shortest arc
                _rv[3:] = [float(2.0 * _sg * v * FPS / 2.0) for v in (_dx, _dy, _dz)]
            _root = torch.tensor(
                [[float(walk["pos"][ci][0]), float(walk["pos"][ci][1]),
                  float(walk["pos"][ci][2]), float(q[3]), float(q[0]),
                  float(q[1]), float(q[2])] + _rv],
                dtype=torch.float32, device=sim.device)
            # PD_BODY (default 1, as in test_grasps_in_isaac.py): the joints
            # run on their PD drives and the root is placed once per frame,
            # so the render replays exactly what the tester verified. Written
            # every substep, the body gave the fingers garbage velocities and
            # a hand that sat a few mm away from where the PD-driven arm had
            # held the clamp (render: the clamp never moved; tester: +79 mm).
            _pd_body = os.environ.get("PD_BODY", "1") == "1"
            # ROOT_SUB: move the pelvis across the frame's substeps instead of
            # in one jump.  Physics runs at dt = 1/1000 (:109), so a 30 fps
            # frame is 33 substeps, and the root was written once -- the pelvis
            # crossed the whole frame's distance inside a single 1 ms step and
            # then stood still for 32 ms.  Measured in v55/v56 (identical runs):
            #   hold f700-900 (stable)   1.07 mm/frame -> object rotates 0.1-3 deg
            #   rise f925-930 (ejected) 14.53 mm/frame -> 94 deg, object at 420 m/s
            # 14.53 mm in 1 ms is 14.5 m/s, and the finger contact offset is
            # 2 mm (:261): one step buries the fingers 7x past the contact
            # margin.  Spread over 33 substeps it is 0.44 mm per step -- the
            # clip's true speed, and inside the margin.  Joints stay on PD.
            _nsub = max(1, round((1.0 / FPS) / sim.get_physics_dt()))
            _root_sub = (os.environ.get("ROOT_SUB", "0") == "1" and _root_prev is not None
                         and _pd_body and os.environ.get("SKIP_ROOT_WRITE") != "1")
            if _wsonic is not None:
                _root_sub = False
            if (_wsonic is None and _pd_body and not _root_sub
                    and os.environ.get("SKIP_ROOT_WRITE") != "1"):   # SKIP_ROOT_WRITE: diagnostic only
                robot.write_root_state_to_sim(_root)
            for _ss in range(_nsub):
                # write_joint_state_to_sim with a subset pushes the whole joint
                # buffer to PhysX (articulation.py:616, 646): refreshed once a
                # frame, it reset the simulated fingers every substep and they
                # moved at 1/33 of their drive. Refresh it first.
                robot.update(sim.get_physics_dt())
                if _wsonic is not None:
                    if _wsonic_k % _wsonic_dec == 0:
                        # the clip frame's time on SONIC's 50 Hz clock
                        _wsonic_tick(int(round((ci + _ss / _nsub) / FPS / _SDT)))
                    _wsonic_k += 1
                soft_mimic(robot, tgt_q)
                robot.set_joint_position_target(tgt_q)
                if _vmode:
                    robot.set_joint_velocity_target(_vel if _squeezing else torch.zeros_like(_vel), joint_ids=_vel_ids)
                if _root_sub:
                    _a = float(_ss + 1) / _nsub
                    _ri = _root_prev * (1.0 - _a) + _root * _a
                    _q0, _q1 = _root_prev[0, 3:7], _root[0, 3:7]
                    if float((_q0 * _q1).sum()) < 0.0:      # shortest arc
                        _ri[0, 3:7] = _q0 * (_a - 1.0) + _q1 * _a
                    _ri[0, 3:7] = _ri[0, 3:7] / _ri[0, 3:7].norm()
                    robot.write_root_state_to_sim(_ri)
                if not _pd_body and _wsonic is None:
                    robot.write_root_state_to_sim(_root)
                    robot.write_joint_state_to_sim(tgt_q[:, _body_ids], zero[:, _body_ids],
                                                   joint_ids=_body_ids)
                robot.write_data_to_sim()
                sim.step()
            _root_prev = _root.clone()
            robot.update(sim.get_physics_dt())
            _every = int(os.environ.get("OBJ_EVERY", "100"))
            if i % _every == 0 and target_body is not None:
                target_body.update(sim.get_physics_dt())
                _op = target_body.data.root_pos_w[0].cpu().numpy()
                print(f"[obj ] frame {i:5d} pos {np.round(_op, 4)}  moved "
                      f"{np.linalg.norm(_op - obj_start):.4f} m  vel {np.round(target_body.data.root_lin_vel_w[0].cpu().numpy(), 3)}"
                      f"  quat {np.round(target_body.data.root_quat_w[0].cpu().numpy(), 3)}")
                _force_line(i)
                _out = []
                for _ln in (PALM_LINK["right"],
                            "left_ankle_roll_link", "right_ankle_roll_link", "left_knee_link", "right_knee_link"):
                    _b = robot.find_bodies([_ln])[0]
                    if _b:
                        _out.append(f"{_ln.replace('_link', '').replace('right_hand_', 'R.')} "
                                    f"{np.round(robot.data.body_pos_w[0, _b[0]].cpu().numpy() - _op, 2)}")
                print(f"[hand] frame {i:5d} rel. to object: " + "  ".join(_out))
                _bR = robot.find_bodies([PALM_LINK["right"]])[0][0]
                print(f"[abs ] frame {i:5d}: object {np.round(_op, 3)} palm {np.round(robot.data.body_pos_w[0, _bR].cpu().numpy(), 3)} root {np.round(robot.data.root_pos_w[0].cpu().numpy(), 3)}")
                if _every <= 10:                       # diagnostic: which link is nearest the object's mesh centroid
                    import trimesh as _tm
                    if "_c_off" not in globals():
                        globals()["_c_off"] = np.asarray(_tm.load(meta["object"]["mesh"], force="mesh").centroid, np.float32)
                    _q = target_body.data.root_quat_w[0].cpu().numpy(); _w, _x, _y, _z = _q
                    _Rm = np.array([[1 - 2 * (_y * _y + _z * _z), 2 * (_x * _y - _z * _w), 2 * (_x * _z + _y * _w)],
                                    [2 * (_x * _y + _z * _w), 1 - 2 * (_x * _x + _z * _z), 2 * (_y * _z - _x * _w)],
                                    [2 * (_x * _z - _y * _w), 2 * (_y * _z + _x * _w), 1 - 2 * (_x * _x + _y * _y)]])
                    _cen = _op + _Rm @ _c_off
                    _bp = robot.data.body_pos_w[0].cpu().numpy(); _dd = np.linalg.norm(_bp - _cen, axis=1); _o = np.argsort(_dd)[:3]
                    print(f"[near] frame {i:5d} centroid {np.round(_cen, 3)} nearest links: " + ", ".join(f"{robot.body_names[j]} {_dd[j]*100:.1f} cm" for j in _o))
                _rh = [j for j in _hand_ids[len(_hand_ids) // 2:]]
                print(f"[hand] frame {i:5d} right finger q {np.round(robot.data.joint_pos[0, _rh].cpu().numpy(), 2)} "
                      f"target {np.round(tgt_q[0, _rh].cpu().numpy(), 2)}; wrist joints "
                      f"{np.round([robot.data.joint_pos[0, robot.find_joints([n])[0][0]].item() for n in ('right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint')], 2)}")
            _shoot()
            continue
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
    if os.environ.get("DUMP_STATE"):
        _dump_state(os.environ["DUMP_STATE"], i)
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
if walk is None and "--legs-from" in sys.argv and STIFF_LEGS:
    # The pick renders on its own: --legs-from puts the walk's last leg pose on
    # the robot and the pelvis is welded, so there is no walk branch and none
    # of the leg handling below ever runs. The legs then hang on G1's default
    # PD gains and take the arm's reaction through them -- measured, the leg
    # joints moving at 21.6 rad/s while the arm worked, which is the tremble.
    #
    # Stiffness is how IsaacLab holds a joint still without writing it, and it
    # costs the solver nothing extra. Same factor the walk path uses.
    _lids = [robot.find_joints([n])[0][0] for n in robot.joint_names
             if ("hip" in n or "knee" in n or "ankle" in n)]
    _lids = [j for j in _lids if j not in set(ids)]
    _sf = 50.0
    robot.write_joint_stiffness_to_sim(
        robot.data.joint_stiffness[:, _lids] * _sf, joint_ids=_lids)
    robot.write_joint_damping_to_sim(
        robot.data.joint_damping[:, _lids] * _sf, joint_ids=_lids)
    print(f"[legs] {len(_lids)} leg joints at x{_sf:.0f} gains so they hold "
          f"themselves under the arm")
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
    if target_body is not None and obj_start is not None:
        target_body.update(sim.get_physics_dt())
        _op = target_body.data.root_pos_w[0].cpu().numpy()
        _d = _op - obj_start
        print(f"[eval] end pos {np.round(_op, 4)}  dxy {np.linalg.norm(_d[:2]):.4f} m"
              f"  dz {_d[2]:+.4f} m  -> {'HELD' if _d[2] > 0.05 else 'LOST'}")
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
    if walk is not None:
        # One reference for the whole thing: the planner's walk. The arm is
        # not spliced in here -- it goes in per tick, the way
        # g1_deploy_onnx_ref.cpp:780 does it, replacing the 17 upper-body
        # joints of every frame the encoder is shown. So there is no combined
        # motion to build and no seam between walking and reaching.
        from sonic_control import clip_to_reference
        _clip = {"dof": walk["dof"], "root_trans_offset": walk["pos"],
                 "root_rot": walk["quat"]}
        _wref, _wq, _wz = clip_to_reference(_clip, FPS)
        # After the walk the reference holds its last frame; SONIC's lookahead
        # clamps at T-1 on its own, so the pick just keeps stepping past it.
        _hold = max(0, _n50 - len(_wref))
        if _hold:
            _wref = np.vstack([_wref, np.tile(_wref[-1], (_hold, 1))])
            _wq = np.vstack([_wq, np.tile(_wq[-1], (_hold, 1))])
            _wz = np.concatenate([_wz, np.full(_hold, _wz[-1])])
        sonic = SonicTracker(_wref, ref_quat=_wq, ref_root_z=_wz)
        print(f"[sonic] reference is the walk itself: {len(_wref)} frames at "
              f"{1/CONTROL_DT:.0f} Hz, root z {_wz.min():.3f}..{_wz.max():.3f}")
    else:
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


def _sonic_step(plan_i=0):
    """One 50 Hz tick of the tracker; writes all 29 targets."""
    global sonic_t
    q = robot.data.joint_pos[0, sonic_ids].cpu().numpy().astype(np.float64)
    dq = robot.data.joint_vel[0, sonic_ids].cpu().numpy().astype(np.float64)
    quat = robot.data.root_quat_w[0].cpu().numpy().astype(np.float64)
    om = robot.data.root_ang_vel_b[0].cpu().numpy().astype(np.float64)
    # The 17 upper-body joints the deployment overwrites, IsaacLab order,
    # from the robot's own measured pose with the plan's arm written over it:
    # GR00T targets what a joint is currently holding when nothing commands it
    # (run_g1_control_loop.py), and the plan commands only the right arm.
    from sonic_control import UPPER_BODY_IL, MUJOCO_TO_ISAACLAB
    _up = np.array([q[_hw] for _hw in range(29)])
    for _k, _c in _plan_to_sonic.items():
        _up[_c] = float(traj[min(plan_i, traj.shape[0] - 1), _k])
    _up_il = np.array([_up[i] for i in range(29)])
    _il = np.zeros(29)
    for _hw in range(29):
        _il[MUJOCO_TO_ISAACLAB[_hw]] = _up_il[_hw]
    sonic.set_upper_body(_il[UPPER_BODY_IL])
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

_leg_ids = [robot.find_joints([n])[0][0] for n in robot.joint_names
            if ("hip" in n or "knee" in n or "ankle" in n)]
_leg_v = []
# ...and how far they actually move between rendered frames. PhysX's joint
# velocity is a noisy estimate (the SimulationContext warns about it at
# startup); what the eye sees is displacement, so record that too.
_leg_q = []
if HOLD_ONLY:
    _sec = 6.0
    _every = max(1, round((1.0 / FPS) / sim.get_physics_dt()))
    _n = int(_sec / sim.get_physics_dt())
    _z0 = float(robot.data.root_pos_w[0, 2].item())
    _hv = []
    print(f"[hold] standing still for {_sec:.0f}s from pelvis z {_z0:.4f}, "
          f"arm pinned at the plan's first waypoint")
    for _i in range(_n):
        if WBC and _i % wbc_dec == 0:
            _wbc_step()
        for _k in range(len(ids)):
            tgt_q[0, ids[_k]] = float(traj[0, _k])
        robot.set_joint_position_target(tgt_q)
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim.get_physics_dt())
        if _i % _every == 0:
            _shoot()
        if _leg_ids:
            _hv.append(float(np.abs(
                robot.data.joint_vel[0, _leg_ids].cpu().numpy()).max()))
        if _i % int(0.5 / sim.get_physics_dt()) == 0:
            _z = float(robot.data.root_pos_w[0, 2].item())
            _w = np.asarray(_hv[-500:]) if _hv else np.zeros(1)
            print(f"[hold] t {_i * sim.get_physics_dt():4.1f}s  "
                  f"pelvis z {_z:.4f}  legs mean {_w.mean():6.3f} "
                  f"max {_w.max():6.3f} rad/s")
    _z = float(robot.data.root_pos_w[0, 2].item())
    _w = np.asarray(_hv) if _hv else np.zeros(1)
    print(f"[hold] {'STOOD' if _z > _z0 - 0.10 else 'FELL'}: "
          f"pelvis {_z0:.4f} -> {_z:.4f} m")
    print(f"[hold] legs over {len(_w)} steps: mean {_w.mean():.3f}, "
          f"p95 {np.percentile(_w, 95):.3f}, max {_w.max():.3f} rad/s "
          f"-- nothing is commanding them, so this is the floor")
    if not NO_VIDEO:
        import imageio.v2 as _iio
        for _fr, _sfx in ((frames, ""), (head_frames, "_wrist"), (eye_frames, "_head")):
            if not _fr:
                continue
            _out = OUT.replace(".mp4", f"{_sfx}.mp4")
            _w = _iio.get_writer(_out, fps=FPS, quality=8)
            for _f in _fr:
                _w.append_data(_f)
            _w.close()
            print(f"[play] wrote {_out}: {len(_fr)} frames")
    sys.stdout.flush()
    os._exit(0)

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
                _sonic_step(i)
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
    if _leg_ids:
        # Every frame. This used to sit inside the "every two hundredth frame"
        # diagnostic block and reported five samples out of eight hundred.
        _leg_v.append(float(np.abs(
            robot.data.joint_vel[0, _leg_ids].cpu().numpy()).max()))
        _leg_q.append(robot.data.joint_pos[0, _leg_ids].cpu().numpy().copy())
    if i % 20 == 0 and target_body is not None:
        target_body.update(sim.get_physics_dt())
        op = target_body.data.root_pos_w[0].cpu().numpy()
        print(f"[obj ] frame {i:5d} pos {np.round(op,4)}  moved "
              f"{np.linalg.norm(op - obj_start):.4f} m")
        _force_line(i)
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
    if _leg_v:
        _lv = np.asarray(_leg_v)
        print(f"[legs] joint speed while the arm works: mean {_lv.mean():.3f}"
              f", p95 {np.percentile(_lv, 95):.3f}, max {_lv.max():.3f} rad/s"
              f"  ({len(_lv)} frames)")
    if len(_leg_q) > 1:
        _dq = np.abs(np.diff(np.asarray(_leg_q), axis=0)) * 1000.0   # mrad
        _worst = int(np.argmax(_dq.max(axis=0)))
        _mv = np.asarray(_leg_q).ptp(axis=0) * 1000.0
        print(f"[legs] motion between frames: mean {_dq.max(axis=1).mean():.2f}"
              f", p95 {np.percentile(_dq.max(axis=1), 95):.2f}, max "
              f"{_dq.max():.2f} mrad/frame; worst joint "
              f"{robot.joint_names[_leg_ids[_worst]]}; total swing per joint "
              f"{np.round(_mv, 1)} mrad")
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
