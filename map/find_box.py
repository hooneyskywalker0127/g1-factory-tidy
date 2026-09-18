# Find the box on the floor from the head camera alone.
#
# No coordinates are handed to the robot. The depth image is unprojected into
# world points, the floor is subtracted, and what is left standing on it is the
# box. Its centre and size come out of those points.
#
# The true position IS read at the end -- but only to score the estimate. It
# never enters the estimate itself.
#
# G1 is spawned with a fixed root here. It has no balance controller in this
# scene and is face down in a second otherwise, and a camera on a fallen robot
# measures nothing.
#
#   python map/find_box.py [x] [y] [yaw_deg]
import os
import sys

from isaaclab.app import AppLauncher

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "map"))
OUT = os.path.join(REPO, "results")
CELL = os.path.join(REPO, "map", "cell.usd")

STAND_X = float(sys.argv[1]) if len(sys.argv) > 1 else -2.00
STAND_Y = float(sys.argv[2]) if len(sys.argv) > 2 else -0.20
STAND_YAW = float(sys.argv[3]) if len(sys.argv) > 3 else -90.0   # faces -y

app = AppLauncher(headless=True, enable_cameras=True).app

import math  # noqa: E402

import numpy as np  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
import omni.usd  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sensors import Camera, CameraCfg  # noqa: E402
from isaaclab.sensors.camera.utils import create_pointcloud_from_depth  # noqa: E402
from isaaclab.utils.math import convert_camera_frame_orientation_convention  # noqa: E402
import torch  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab_assets.robots.unitree import G1_MINIMAL_CFG  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Sdf, UsdGeom  # noqa: E402

import cell_layout as L  # noqa: E402
from props import spawn_props  # noqa: E402

W, H = 424, 240
H_FOV, NEAR_DEPTH, APERTURE = 86.0, 0.40, 20.955
FOCAL = APERTURE / (2.0 * math.tan(math.radians(H_FOV) / 2.0))

# G1 has no neck joint -- head_link is fixed to the torso -- so where the camera
# looks is decided by how it is bolted on, and nothing else. Level, it cannot
# see its own feet: at 0.79 m with a 55.7 deg vertical field, the floor only
# enters the frame 1.5 m out, and a box at 0.9 m sits on the bottom edge with
# its near face cut off. Pitched 30 deg down the floor starts at 0.49 m, just
# outside the depth sensor's own 0.40 m limit.
CAM_PITCH_DEG = 30.0
_p = math.radians(CAM_PITCH_DEG) / 2.0
CAM_ROT = (math.cos(_p), 0.0, math.sin(_p), 0.0)   # pitch down about +y

FLOOR_EPS = 0.03        # points this close to z=0 are the floor itself
MAX_OBJ_Z = 0.80        # taller than anything that can sit on this floor
SELF_R = 0.55           # points this close to the camera are the robot's arms

sim = SimulationContext(sim_utils.SimulationCfg(dt=1.0 / 120.0, device="cpu"))
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
stage.DefinePrim("/World/Cell", "Xform").GetReferences().AddReference(
    Sdf.Reference(CELL))
light = sim_utils.DomeLightCfg(intensity=900.0)
light.func("/World/light", light)
for _ in range(3):
    app.update()

spawn_props(stage, app)

yaw = math.radians(STAND_YAW)
cfg = G1_MINIMAL_CFG.replace(prim_path="/World/G1")
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

cam = Camera(CameraCfg(
    prim_path="/World/G1/head_link/head_cam",
    update_period=0.0, width=W, height=H,
    data_types=["rgb", "distance_to_image_plane"],
    offset=CameraCfg.OffsetCfg(pos=(0.08, 0.0, 0.05), rot=CAM_ROT,
                               convention="world"),
    spawn=sim_utils.PinholeCameraCfg(focal_length=FOCAL,
                                     horizontal_aperture=APERTURE,
                                     clipping_range=(0.01, 20.0)),
))

sim.reset()
for _ in range(90):
    sim.step()
cam.update(sim.get_physics_dt())

# The sensor's own pose buffers read (0,0,0) with an all-zero quaternion on a
# CPU device -- Fabric is disabled there (the run log says so) and that is the
# path they are filled through. Unprojecting with that zero quaternion turns
# every point into NaN, which is what emptied the first point cloud. The pose is
# read off the stage instead, and converted from USD's camera convention
# (looking down -z) to the ros one the unprojection expects.
m = UsdGeom.XformCache().GetLocalToWorldTransform(
    stage.GetPrimAtPath("/World/G1/head_link/head_cam"))
t = m.ExtractTranslation()
r = m.ExtractRotationQuat()
pos = np.array([t[0], t[1], t[2]], dtype=np.float32)
q_gl = torch.tensor([[r.GetReal(), *r.GetImaginary()]], dtype=torch.float32)
quat = convert_camera_frame_orientation_convention(
    q_gl, origin="opengl", target="ros")[0].numpy()
sensor_pos = cam.data.pos_w[0].cpu().numpy()
print(f"[find] camera at ({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:.3f}) from stage"
      f"  (sensor buffer said {sensor_pos[0]:+.3f}, {sensor_pos[1]:+.3f}, "
      f"{sensor_pos[2]:+.3f})")

depth = cam.data.output["distance_to_image_plane"][0].cpu().numpy().squeeze()
K = cam.data.intrinsic_matrices[0].cpu().numpy()
d = depth.copy()
d[~np.isfinite(d)] = 0.0
d[d < NEAR_DEPTH] = 0.0                     # the hardware's near limit
pts = create_pointcloud_from_depth(K, d, position=pos, orientation=quat)
pts = np.asarray(pts)
pts = pts[np.isfinite(pts).all(axis=1)]
print(f"[find] {len(pts)} points from {int((d > 0).sum())} valid depth pixels")

# --- subtract the floor and the robot's own arms -------------------------
near_cam = np.linalg.norm(pts - pos, axis=1) < SELF_R
inside = ((np.abs(pts[:, 0]) < L.FLOOR[0] / 2.0 - 0.12) &
          (np.abs(pts[:, 1]) < L.FLOOR[1] / 2.0 - 0.12))
standing = (pts[:, 2] > FLOOR_EPS) & (pts[:, 2] < MAX_OBJ_Z)
obj = pts[standing & inside & ~near_cam]
print(f"[find] floor removed -> {len(obj)} points standing on it")

if len(obj) < 30:
    print("[find] nothing found -- the box is not in view from here")
    sys.stdout.flush()
    os._exit(1)

lo, hi = obj.min(axis=0), obj.max(axis=0)
ctr = (lo + hi) / 2.0
print(f"[find] ESTIMATE  centre ({ctr[0]:+.3f}, {ctr[1]:+.3f})  "
      f"size {hi[0]-lo[0]:.3f} x {hi[1]-lo[1]:.3f} x {hi[2]:.3f}")

# --- score it against the truth (scoring only) ---------------------------
bx, by, brz = L.FLOOR_BOXES[0]
bw, bd, bh = L.FLOOR_BOX_SIZE
# the box is turned on the floor, so its axis-aligned extent is wider than its
# own sides -- that is what a point cloud can be compared against
c, sn = abs(math.cos(math.radians(brz))), abs(math.sin(math.radians(brz)))
aabb_w, aabb_d = bw * c + bd * sn, bw * sn + bd * c
err = math.hypot(ctr[0] - bx, ctr[1] - by)
print(f"[find] TRUTH     centre ({bx:+.3f}, {by:+.3f})  "
      f"aabb {aabb_w:.3f} x {aabb_d:.3f} x {bh:.3f}  (box {bw:.2f} turned {brz:.0f} deg)")
print(f"[find] centre error {err * 1000:.0f} mm, height error "
      f"{(hi[2] - bh) * 1000:+.0f} mm")

os.makedirs(OUT, exist_ok=True)
rgb = cam.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8)
Image.fromarray(rgb).save(os.path.join(OUT, "find_box_rgb.png"))
print(f"[find] wrote {OUT}/find_box_rgb.png")
sys.stdout.flush()
os._exit(0)
