# Which frame of a plan puts the target inside the wrist camera's view?
#
# grasp/capture_rgbd.py --wrist shoots one frame. Finding a frame where the
# hand actually looks at the object by shooting one at a time costs an Isaac
# launch each. This walks the plan once and reports, per frame, where the
# object falls in the wrist camera's image.
#
#   python grasp/probe_wrist_view.py results/v15_mesh.json [step]
import math
import os
import sys

from isaaclab.app import AppLauncher

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "map"))
sys.path.insert(0, os.path.join(REPO, "grasp"))

PLAN = sys.argv[1]
STEP = int(sys.argv[2]) if len(sys.argv) > 2 else 10
STAND = (-1.30, -0.60, -90.0)

# Same published mount grasp/capture_rgbd.py uses.
WRIST_PARENT = "right_wrist_yaw_link"
WRIST_MOUNT_XYZ = (0.07, 0.0, 0.0)
WRIST_MOUNT_RPY = (1.5707963267948966, 0.7853981633974483, 1.5707963267948966)
WRIST_CAM_XYZ = (0.008, 0.091, 0.002)
WRIST_CAM_RPY = (1.9792033717615698, 0.3490658503988659, 0.0)
# D405 depth, as capture_rgbd.py writes it.
W, H, HFOV = 640, 480, 87.0

app = AppLauncher(headless=True).app

import json  # noqa: E402

import numpy as np  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
import omni.usd  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab_assets.robots.unitree import G1_29DOF_CFG  # noqa: E402
from pxr import Gf, Sdf, UsdGeom  # noqa: E402
import torch  # noqa: E402

from plan_scene import build as build_plan_scene, torso_pose  # noqa: E402

sim = SimulationContext(sim_utils.SimulationCfg(dt=1.0 / 120.0, device="cpu"))
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
stage.DefinePrim("/World/Cell", "Xform").GetReferences().AddReference(
    Sdf.Reference(os.path.join(REPO, "map", "cell.usd")))
for _ in range(3):
    app.update()

yaw = math.radians(STAND[2])
cfg = G1_29DOF_CFG.replace(prim_path="/World/G1")
cfg.spawn = cfg.spawn.replace(
    articulation_props=sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=False, solver_position_iteration_count=8,
        solver_velocity_iteration_count=4, fix_root_link=True))
cfg.init_state = cfg.init_state.replace(
    pos=(STAND[0], STAND[1], cfg.init_state.pos[2]),
    rot=(math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)))
robot = Articulation(cfg)
for _ in range(3):
    app.update()

meta = json.load(open(PLAN))
T_torso = torso_pose((STAND[0], STAND[1], cfg.init_state.pos[2]), yaw)
build_plan_scene(stage, app, meta, T_torso)
sim.reset()

traj = np.load(os.path.splitext(PLAN)[0] + ".npy")
names = meta["joint_names"]
ids = [robot.find_joints([n])[0][0] for n in names]

# Where the object sits, in cell coordinates.
obj_prim = stage.GetPrimAtPath("/World/GraspTarget")
box = UsdGeom.Imageable(obj_prim).ComputeWorldBound(0.0, "default").ComputeAlignedBox()
obj_c = np.array([(box.GetMin()[i] + box.GetMax()[i]) / 2.0 for i in range(3)])
print(f"[probe] object centre {np.round(obj_c, 4)}")


def _rpy(r, p_, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p_),
                              math.sin(p_), math.cos(y), math.sin(y))
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def _T(xyz, rpy_):
    M = np.eye(4)
    M[:3, :3] = _rpy(*rpy_)
    M[:3, 3] = xyz
    return M


T_wc = (_T(WRIST_MOUNT_XYZ, WRIST_MOUNT_RPY)
        @ _T(WRIST_CAM_XYZ, WRIST_CAM_RPY)
        @ _T((0, 0, 0), (-math.pi / 2, 0, 0)))

fx = (W / 2.0) / math.tan(math.radians(HFOV) / 2.0)
fy = fx
bid = robot.find_bodies([WRIST_PARENT])[0][0]

print(f"[probe] {traj.shape[0]} frames, every {STEP}")
print("  frame   dist   u,v (image 640x480)   in view   cam pos")
best = []
for f in range(0, traj.shape[0], STEP):
    q = robot.data.default_joint_pos.clone()
    for k, jid in enumerate(ids):
        q[0, jid] = float(traj[f, k])
    robot.write_joint_state_to_sim(q, torch.zeros_like(q))
    robot.write_data_to_sim()
    sim.step()

    bp = robot.data.body_pos_w[0, bid].cpu().numpy().astype(np.float64)
    bq = robot.data.body_quat_w[0, bid].cpu().numpy().astype(np.float64)
    R = np.array(Gf.Matrix3d(Gf.Quatd(bq[0], Gf.Vec3d(*bq[1:])))).reshape(3, 3).T
    Tw = np.eye(4)
    Tw[:3, :3], Tw[:3, 3] = R, bp
    Tw = Tw @ T_wc

    rel = np.linalg.inv(Tw) @ np.append(obj_c, 1.0)
    x, y, z = rel[:3]
    if z <= 0.02:
        continue
    u, v = fx * x / z + W / 2.0, fy * y / z + H / 2.0
    ok = 0 <= u < W and 0 <= v < H
    if ok:
        best.append((f, z, u, v, Tw[:3, 3].copy()))
        print(f"  {f:5d}  {z:5.3f}  ({u:6.1f},{v:6.1f})   yes     "
              f"{np.round(Tw[:3, 3], 3)}")

if not best:
    print("[probe] the object never enters the wrist camera's view")
else:
    # closest to the image centre, among frames where it is in view
    c = min(best, key=lambda b: (b[2] - W / 2) ** 2 + (b[3] - H / 2) ** 2)
    print(f"[probe] best centred: frame {c[0]}, {c[1]:.3f} m away, "
          f"({c[2]:.0f},{c[3]:.0f})")
sys.stdout.flush()
os._exit(0)
