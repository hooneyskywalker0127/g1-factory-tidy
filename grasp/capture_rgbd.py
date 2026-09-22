# One RGB-D frame from G1's head camera, written in the format GraspGenX reads.
#
# GraspGenX already knows how to turn an RGB-D capture into per-object point
# clouds: `graspgenx.utils.scene_loaders.load_realworld_scene` unprojects the
# depth, splits the objects apart by their segmentation ids, and
# `build_scene_pc_excluding_object` gives the rest of the scene back as the
# collision cloud. All it wants is a directory:
#
#   <out>/meta_data.json   intrinsics, camera_pose, label_map
#   <out>/depth.npy
#   <out>/rgb.png
#   <out>/seg.png
#
# So nothing here segments anything. The prims get semantic labels, Isaac
# renders the three buffers, and this writes them out.
#
# With --plan TRAJ.json the plan's own table and target are put in the cell
# first, at the same offsets from the torso grasp/play_in_cell.py uses, and the
# arm is held at the plan's start pose. That makes the capture a picture of the
# very scene the plan was made for, so the grasps can be re-predicted from it.
#
# --arm-at F holds the arm at frame F of results/g1_graspgen.npy before the
# shutter. A wrist camera only sees the work if the arm is pointing at it.
#
#   python grasp/capture_rgbd.py <look_x> <look_y> <look_yaw> \
#       --plan results/g1_graspgen.json --plan-stand <x> <y> <yaw> --out DIR
import math
import os
import sys

from isaaclab.app import AppLauncher

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "map"))
sys.path.insert(0, os.path.join(REPO, "grasp"))

_args = [a for a in sys.argv[1:] if not a.startswith("--")]
STAND_X = float(_args[0]) if len(_args) > 0 else -1.30
STAND_Y = float(_args[1]) if len(_args) > 1 else -0.60
STAND_YAW = float(_args[2]) if len(_args) > 2 else -90.0
OUT = (sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv
       else os.path.join(REPO, "results", "capture"))
PLAN = sys.argv[sys.argv.index("--plan") + 1] if "--plan" in sys.argv else None
ARM_AT = (int(sys.argv[sys.argv.index("--arm-at") + 1])
          if "--arm-at" in sys.argv else None)
# Where the plan will be executed from. The scene is anchored to THAT torso,
# not to wherever the robot stands to look: the object has to sit 0.35 m in
# front of the reaching pose, which is inside the depth sensor's 0.40 m blind
# zone, so the picture is taken from further back.
_ps = sys.argv.index("--plan-stand") if "--plan-stand" in sys.argv else None
PLAN_STAND = ((float(sys.argv[_ps + 1]), float(sys.argv[_ps + 2]),
               float(sys.argv[_ps + 3])) if _ps else (STAND_X, STAND_Y, STAND_YAW))

# --wrist puts the camera on the hand instead of the head, the way Unitree's
# own G1 datasets and the published mounts do. The chain is
# right_wrist_yaw_link -> mount -> d405 -> optical, with the numbers taken
# verbatim from Zulkhuu/g1-wrist-camera-tools' config/camera_mounts.yaml
# (right side) and its stated convention that the D405 optical +Z maps to the
# CAD body's local +Y.
WRIST = "--wrist" in sys.argv
WRIST_PARENT = "right_wrist_yaw_link"
WRIST_MOUNT_XYZ = (0.07, 0.0, 0.0)
WRIST_MOUNT_RPY = (1.5707963267948966, 0.7853981633974483, 1.5707963267948966)
WRIST_CAM_XYZ = (0.008, 0.091, 0.002)
WRIST_CAM_RPY = (1.9792033717615698, 0.3490658503988659, 0.0)
# D405 depth: 640x480, and it sees from 0.07 m -- far closer than the D435i
D405_W, D405_H, D405_NEAR = 640, 480, 0.07

# Same D435i geometry the rest of the repo uses (map/find_box.py): 86 deg
# horizontal depth FoV, pitched 30 deg down because G1 has no neck joint.
H_FOV = 86.0
APERTURE = 20.955
FOCAL = APERTURE / (2.0 * math.tan(math.radians(H_FOV) / 2.0))
# 15 deg, not the 30 map/find_box.py uses. 30 is tuned for objects on the
# floor; it puts the top of the frame 2.2 deg BELOW horizontal, so anything at
# or above the camera is cut off -- and a table the arm can reach sits above
# this robot's eyes (camera 0.80, object top 0.888). At 15 deg the top of the
# frame is 12.8 deg above horizontal, which covers the table and still looks
# down far enough to see the floor from about 1 m.
CAM_PITCH_DEG = 15.0
# 848x480, one of the D435i's own depth modes. map/find_box.py runs at 424x240
# to save render time, but GraspGenX's outlier removal (K=20 neighbours, 14 mm)
# throws the whole cloud away at that density -- a 0.16 m box a metre out came
# back as 301 points and 142 of them were dropped as outliers.
W, H = 848, 480

app = AppLauncher(headless=True, enable_cameras=True).app

import json  # noqa: E402

import numpy as np  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
import omni.usd  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sensors import Camera, CameraCfg  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab.sim.utils import add_labels  # noqa: E402
from isaaclab.utils.math import convert_camera_frame_orientation_convention  # noqa: E402
from isaaclab_assets.robots.unitree import G1_29DOF_CFG  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom  # noqa: E402
import torch  # noqa: E402

import cell_layout as L  # noqa: E402
from plan_scene import build as build_plan_scene, torso_pose  # noqa: E402
from props import spawn_props  # noqa: E402
from vision import camera_pose  # noqa: E402

sim = SimulationContext(sim_utils.SimulationCfg(dt=1.0 / 120.0, device="cpu"))
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
stage.DefinePrim("/World/Cell", "Xform").GetReferences().AddReference(
    Sdf.Reference(os.path.join(REPO, "map", "cell.usd")))
light = sim_utils.DomeLightCfg(intensity=900.0)
light.func("/World/light", light)
for _ in range(3):
    app.update()
placed = spawn_props(stage, app)

yaw = math.radians(STAND_YAW)
cfg = G1_29DOF_CFG.replace(prim_path="/World/G1")
# fixed root, as map/find_box.py does: left free the robot sags over the 60
# settle steps and the head camera ends up looking at the ceiling.
cfg.spawn = cfg.spawn.replace(
    articulation_props=sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=False, solver_position_iteration_count=8,
        solver_velocity_iteration_count=4, fix_root_link=True))
cfg.init_state = cfg.init_state.replace(
    pos=(STAND_X, STAND_Y, cfg.init_state.pos[2]),
    rot=(math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)))
robot = Articulation(cfg)
for _ in range(3):
    app.update()

meta = json.load(open(PLAN)) if PLAN else {}
if meta:
    px, py, pyaw = PLAN_STAND
    plan_torso = torso_pose((px, py, cfg.init_state.pos[2]), math.radians(pyaw))
    build_plan_scene(stage, app, meta, plan_torso)
    for i, name in enumerate(
            [s["name"] for s in meta.get("support", [])] + ["object"], start=1):
        path = ("/World/GraspTarget" if name == "object"
                else f"/World/PlanSupport_{name}")
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            label = "table" if name != "object" else "obj_plan"
            for sub in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
                add_labels(sub, [label], overwrite=True)
    print(f"[cap] plan scene from {PLAN}, anchored to stand {PLAN_STAND}")

# Label everything the capture needs to tell apart. The names follow the
# sample scenes in assets/sample_data/real_world: `ground`, `table`, `robot`,
# and one `obj_<n>` per thing that can be picked.
add_labels(stage.GetPrimAtPath("/World/Cell"), ["ground"])
add_labels(stage.GetPrimAtPath("/World/G1"), ["robot"])
props_root = stage.GetPrimAtPath("/World/Props")
obj_prims = list(props_root.GetChildren()) if props_root.IsValid() else []
for i, prim in enumerate(obj_prims, start=1):
    # the label has to sit on the prims that carry geometry, not only on the
    # group Xform above them, or the renderer never sees it
    # every prim under it, overwriting: the referenced assets carry their own
    # labels ("box"), and the renderer merges them with ours into "box,obj_1",
    # which load_realworld_scene then does not recognise as an obj_* entry.
    for sub in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
        add_labels(sub, [f"obj_{i}"], overwrite=True)
print(f"[cap] labelled {len(obj_prims)} pickable prims: "
      + ", ".join(p.GetName() for p in obj_prims))

head = "head_link"
_p = math.radians(CAM_PITCH_DEG) / 2.0
if WRIST:
    # Build the wrist chain as fixed prims so the camera rides the hand. The
    # transforms are the published mount's, composed here rather than folded
    # into one guessed offset.
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

    # optical +Z along the body's +Y, per the mount repo
    T_wc = (_T(WRIST_MOUNT_XYZ, WRIST_MOUNT_RPY)
            @ _T(WRIST_CAM_XYZ, WRIST_CAM_RPY)
            @ _T((0, 0, 0), (-math.pi / 2, 0, 0)))
    q = Gf.Matrix4d(T_wc.T.tolist()).ExtractRotationQuat()
    cam_path = f"/World/G1/{WRIST_PARENT}/wrist_cam"
    W, H = D405_W, D405_H
    NEAR = D405_NEAR
    cam = Camera(CameraCfg(
        prim_path=cam_path, update_period=0.0, width=W, height=H,
        data_types=["rgb", "distance_to_image_plane", "semantic_segmentation"],
        colorize_semantic_segmentation=False,
        offset=CameraCfg.OffsetCfg(
            pos=tuple(float(v) for v in T_wc[:3, 3]),
            rot=(float(q.GetReal()), *[float(v) for v in q.GetImaginary()]),
            convention="ros"),
        spawn=sim_utils.PinholeCameraCfg(clipping_range=(0.01, 20.0)),
    ))
    print(f"[cap] wrist camera on {WRIST_PARENT} at "
          f"{np.round(T_wc[:3, 3], 4)}, looking {np.round(T_wc[:3, 2], 3)}")
else:
    NEAR = 0.40
    cam = Camera(CameraCfg(
        prim_path=f"/World/G1/{head}/head_cam",
        update_period=0.0, width=W, height=H,
        data_types=["rgb", "distance_to_image_plane", "semantic_segmentation"],
        # ids, not colours: load_realworld_scene indexes seg.png by label id.
        colorize_semantic_segmentation=False,
        offset=CameraCfg.OffsetCfg(pos=(0.08, 0.0, 0.05),
                                   rot=(math.cos(_p), 0.0, math.sin(_p), 0.0),
                                   convention="world"),
        spawn=sim_utils.PinholeCameraCfg(focal_length=FOCAL,
                                         horizontal_aperture=APERTURE,
                                         clipping_range=(0.01, 20.0)),
    ))

sim.reset()

if ARM_AT is not None and PLAN:
    q = np.load(os.path.splitext(PLAN)[0] + ".npy")
    names = meta["joint_names"]
    ids = [robot.find_joints([n])[0][0] for n in names]
    tgt = robot.data.default_joint_pos.clone()
    f = min(ARM_AT, q.shape[0] - 1)
    for k, jid in enumerate(ids):
        tgt[0, jid] = float(q[f, k])
    robot.write_joint_state_to_sim(tgt, torch.zeros_like(tgt))
    robot.set_joint_position_target(tgt)
    robot.write_data_to_sim()
    print(f"[cap] arm held at plan frame {f}")

for _ in range(60):
    robot.write_data_to_sim()
    sim.step()
for _ in range(3):
    app.update()
    cam.update(sim.get_physics_dt())

depth = cam.data.output["distance_to_image_plane"][0].cpu().numpy().squeeze()
rgb = cam.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8)
seg = cam.data.output["semantic_segmentation"][0].cpu().numpy().squeeze()
info = cam.data.info[0]["semantic_segmentation"]

# idToLabels is {id: {"class": "obj_3"}}; the loader wants {name: id}.
id_to_labels = info["idToLabels"] if isinstance(info, dict) else {}
label_map = {}
for sid, entry in id_to_labels.items():
    name = entry.get("class") if isinstance(entry, dict) else str(entry)
    if not name:
        continue
    # The renderer merges an asset's own label with ours, so a box comes back
    # as "box,obj_1". load_realworld_scene keys objects off names starting
    # with "obj_", so keep that token and drop the asset's.
    tokens = [t for t in str(name).split(",") if t]
    obj = next((t for t in tokens if t.startswith("obj_")), None)
    label_map[obj or str(name)] = int(sid)

# Neither sensor returns anything closer than its minimum range -- 0.40 m on
# the head's D435i, 0.07 m on the wrist's D405. The loader treats depth <= 0
# as invalid, which is the same thing the hardware does.
depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
depth[depth < NEAR] = 0.0

# camera_pose as a 4x4 world transform in the ros convention the loader
# unprojects with.
# vision.camera_pose reads the stage instead of cam.data.pos_w/quat_w: those
# buffers are filled through Fabric, which is off on a cpu device, and come
# back all zeros -- an all-zero quaternion turns every unprojected point NaN.
if WRIST:
    # Compose it off the wrist body instead of reading the stage: the stage
    # transform of an articulation link does not follow the joints here, so a
    # stage read gives the camera's pose at spawn no matter where the arm is.
    bid = robot.find_bodies([WRIST_PARENT])[0][0]
    bp = robot.data.body_pos_w[0, bid].cpu().numpy().astype(np.float64)
    bq = robot.data.body_quat_w[0, bid].cpu().numpy().astype(np.float64)  # wxyz
    Rw = np.array(Gf.Matrix3d(Gf.Quatd(bq[0], Gf.Vec3d(*bq[1:])))).reshape(3, 3).T
    T_world = np.eye(4)
    T_world[:3, :3] = Rw
    T_world[:3, 3] = bp
    T_world = T_world @ T_wc
    pos = T_world[:3, 3]
    qc = Gf.Matrix4d(T_world.T.tolist()).ExtractRotationQuat()
    quat_ros = np.array([qc.GetReal(), *qc.GetImaginary()], dtype=np.float64)
else:
    pos, quat_ros = camera_pose(stage, f"/World/G1/{head}/head_cam")
pos = pos.astype(np.float64)
w, x, y, z = quat_ros
R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
              [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
              [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
cam_pose = np.eye(4)
cam_pose[:3, :3] = R
cam_pose[:3, 3] = pos

K = cam.data.intrinsic_matrices[0].cpu().numpy().astype(np.float64)

os.makedirs(OUT, exist_ok=True)
np.save(os.path.join(OUT, "depth.npy"), depth.astype(np.float32))
Image.fromarray(rgb).save(os.path.join(OUT, "rgb.png"))
Image.fromarray(seg.astype(np.int32), mode="I").save(os.path.join(OUT, "seg.png"))
# Also write the plan-frame transform. The capture is in cell coordinates; the
# planner works in a world whose origin is the robot's torso at the height
# robots/*.yaml gives. Anything reading this capture for planning has to move
# the clouds across, and the only thing needed for that is where the torso
# stood when the scene was placed.
plan_from_cell = np.linalg.inv(plan_torso) if meta else np.eye(4)

json.dump({
    "plan_from_cell": plan_from_cell.tolist(),
    "intrinsics": K.tolist(),
    "camera_pose": cam_pose.tolist(),
    "label_map": label_map,
    "scene_bounds": [-L.FLOOR[0] / 2, -L.FLOOR[1] / 2, 0.0,
                     L.FLOOR[0] / 2, L.FLOOR[1] / 2, 2.0],
}, open(os.path.join(OUT, "meta_data.json"), "w"), indent=2)

valid = depth > 0
print(f"[cap] {W}x{H}  depth valid {valid.sum()}/{depth.size} px")
print(f"[cap] labels seen: {sorted(label_map.items(), key=lambda kv: kv[1])}")
print(f"[cap] camera at {np.round(pos, 3)}")
print(f"[cap] wrote {OUT}")
sys.stdout.flush()
os._exit(0)
