# What does the robot actually see? Stands G1 in the cell and saves one frame
# from a head camera built to the real robot's depth sensor.
#
# The real G1 carries an Intel RealSense D435i in its head: depth FoV 86 x 57
# degrees, and no depth closer than 0.40 m. Both are kept here. The near limit
# is NOT done with the camera's clipping plane -- that would delete the colour
# image too, and the real sensor still sees colour up close, it just stops
# returning depth. So the frame is rendered normally and the depth below 0.40 m
# is thrown away afterwards, which is what the hardware does.
#
# Resolution is half the sensor's 848 x 480 depth mode. Dropping resolution is
# not a spec violation -- a policy gets a downsampled image on the real robot
# too -- and it is the one place a one-person project should save render time.
#
#   python map/look_from_g1.py [x] [y] [yaw_deg]
import os
import sys

from isaaclab.app import AppLauncher

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "map"))
OUT = os.path.join(REPO, "results")
CELL = os.path.join(REPO, "map", "cell.usd")

STAND_X = float(sys.argv[1]) if len(sys.argv) > 1 else -2.20
STAND_Y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.00
STAND_YAW = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0   # 0 = facing +x

app = AppLauncher(headless=True, enable_cameras=True).app

import math  # noqa: E402

import numpy as np  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
import omni.usd  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sensors import Camera, CameraCfg  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab_assets.robots.unitree import G1_MINIMAL_CFG  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Sdf, UsdGeom  # noqa: E402

from props import spawn_props  # noqa: E402

# --- D435i, as the hardware is specified ---------------------------------
W, H = 424, 240                  # half of the sensor's 848 x 480 depth mode
H_FOV = 86.0                     # degrees, depth
NEAR_DEPTH = 0.40                # metres -- closer than this returns nothing
APERTURE = 20.955                # mm, the renderer's default sensor width
FOCAL = APERTURE / (2.0 * math.tan(math.radians(H_FOV) / 2.0))

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
cfg.init_state = cfg.init_state.replace(
    pos=(STAND_X, STAND_Y, cfg.init_state.pos[2]),
    rot=(math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)))
robot = Articulation(cfg)
for _ in range(3):
    app.update()

# --- find the head to hang the camera off ---------------------------------
links = [p.GetName() for p in stage.GetPrimAtPath("/World/G1").GetChildren()]
head = next((n for n in links if "head" in n.lower()), None)
if head is None:
    head = next((n for n in links if "torso" in n.lower()), None)
print(f"[look] G1 links: {', '.join(links)}")
print(f"[look] camera goes on: {head}")

cam = Camera(CameraCfg(
    prim_path=f"/World/G1/{head}/head_cam",
    update_period=0.0,
    width=W, height=H,
    data_types=["rgb", "distance_to_image_plane"],
    # "world" convention: the camera looks straight down the head link's own +x
    # with +z up, so an identity rotation is the robot looking where it faces.
    # The ros convention was what rolled the first frame over.
    offset=CameraCfg.OffsetCfg(pos=(0.08, 0.0, 0.05), rot=(1.0, 0.0, 0.0, 0.0),
                               convention="world"),
    spawn=sim_utils.PinholeCameraCfg(focal_length=FOCAL,
                                     horizontal_aperture=APERTURE,
                                     clipping_range=(0.01, 20.0)),
))

sim.reset()
for _ in range(60):                       # let it stand before the shutter
    sim.step()
cam.update(sim.get_physics_dt())

p = cam.data.pos_w[0].cpu().numpy()
q = cam.data.quat_w_world[0].cpu().numpy()          # w, x, y, z
w, x, y, z = q
fwd = (1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y))
up = (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))
print(f"[look] camera at ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})  "
      f"forward ({fwd[0]:+.2f}, {fwd[1]:+.2f}, {fwd[2]:+.2f})  "
      f"up ({up[0]:+.2f}, {up[1]:+.2f}, {up[2]:+.2f})")

os.makedirs(OUT, exist_ok=True)
rgb = cam.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8)
Image.fromarray(rgb).save(os.path.join(OUT, "g1_view_rgb.png"))

d = cam.data.output["distance_to_image_plane"][0].cpu().numpy().squeeze()
valid = np.isfinite(d) & (d >= NEAR_DEPTH)          # the hardware's near limit
lost = float((np.isfinite(d) & (d < NEAR_DEPTH)).mean())
vis = np.zeros_like(d)
if valid.any():
    lo, hi = d[valid].min(), min(d[valid].max(), 6.0)
    vis[valid] = 1.0 - np.clip((d[valid] - lo) / max(hi - lo, 1e-6), 0, 1)
Image.fromarray((vis * 255).astype(np.uint8)).save(
    os.path.join(OUT, "g1_view_depth.png"))

fov_v = 2.0 * math.degrees(math.atan(APERTURE * H / W / (2.0 * FOCAL)))
print(f"[look] {W}x{H}, FoV {H_FOV:.1f} x {fov_v:.1f} deg, focal {FOCAL:.2f} mm")
print(f"[look] standing at ({STAND_X:+.2f}, {STAND_Y:+.2f}) yaw {STAND_YAW:.0f} deg")
print(f"[look] depth valid on {100 * valid.mean():.1f}% of pixels; "
      f"{100 * lost:.1f}% lost to the 0.40 m near limit")
print(f"[look] wrote {OUT}/g1_view_rgb.png and g1_view_depth.png")
sys.stdout.flush()
# app.close() does not return here -- the frames are already on disk by this
# point, so the process is ended outright rather than left hanging.
os._exit(0)
