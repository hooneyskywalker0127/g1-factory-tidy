# Build the tidy cell's STATIC shell and save it as map/cell.usd.
#
# Static only: floor, the two walls with their doorway, and the racks. Nothing
# the robot is meant to pick up lives in here -- those are spawned live as rigid
# bodies once physics is up, because adding prims to the stage after the env
# exists invalidates PhysX's articulation view (detachShape storm on the first
# step). The shell has to be fixed before physics starts, the clutter does not.
#
# Rack collision is left ON here. humanoid-swarm-sim kills it in its props bake
# because those props are visual dressing over a map that already has physics --
# here the rack IS the physics: a crate set on a deck has to land on a board.
#
#   python map/build_cell.py [out.usd]
import os
import sys

from isaaclab.app import AppLauncher

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "common"))
sys.path.insert(0, os.path.join(REPO, "map"))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(REPO, "map", "cell.usd")

app = AppLauncher(headless=True).app

import omni.usd  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402

import cell_layout as L  # noqa: E402
from rack_component import find_decks, spawn_rack  # noqa: E402

ROOT = "/World/Cell"

omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")
UsdGeom.Xform.Define(stage, ROOT)
stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))


def mat(name, rgb, rough=0.6):
    m = UsdShade.Material.Define(stage, f"{ROOT}/Looks/{name}")
    s = UsdShade.Shader.Define(stage, f"{ROOT}/Looks/{name}/Shader")
    s.CreateIdAttr("UsdPreviewSurface")
    s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), "surface")
    return m


def box(path, size, centre, material):
    c = UsdGeom.Cube.Define(stage, path)
    c.GetSizeAttr().Set(1.0)
    x = UsdGeom.Xformable(c)
    x.AddTranslateOp().Set(Gf.Vec3d(*centre))
    x.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(material)
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())          # static collider
    return c


FLOOR_MAT = mat("floor", (0.40, 0.41, 0.43), rough=0.75)
WALL_MAT = mat("wall", (0.66, 0.66, 0.63), rough=0.85)

fx, fy = L.FLOOR
box(f"{ROOT}/Floor", (fx, fy, 0.02), (0.0, 0.0, -0.01), FLOOR_MAT)

# north wall: full span, sat just inside the floor edge
box(f"{ROOT}/WallN", (fx, L.WALL_T, L.WALL_H),
    (0.0, fy / 2.0 - L.WALL_T / 2.0, L.WALL_H / 2.0), WALL_MAT)

# west wall: the same span minus the doorway, so two segments
wall_x = -fx / 2.0 + L.WALL_T / 2.0
for i, (y0, y1) in enumerate([(-fy / 2.0, L.DOOR_CY - L.DOOR_W / 2.0),
                              (L.DOOR_CY + L.DOOR_W / 2.0, fy / 2.0)]):
    box(f"{ROOT}/WallW_{i}", (L.WALL_T, y1 - y0, L.WALL_H),
        (wall_x, (y0 + y1) / 2.0, L.WALL_H / 2.0), WALL_MAT)


def ensure_collision(prim):
    """Turn the rack's own colliders back on, or give it some if it has none."""
    on = 0
    for p in Usd.PrimRange(prim):
        att = p.GetAttribute("physics:collisionEnabled")
        if att:
            att.Set(True)
            on += 1
        elif UsdGeom.Mesh(p):
            UsdPhysics.CollisionAPI.Apply(p)
            api = UsdPhysics.MeshCollisionAPI.Apply(p)
            api.CreateApproximationAttr().Set("none")   # static, so the real mesh
            on += 1
    return on


cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
wall_inner_y = fy / 2.0 - L.WALL_T

for i, rx in enumerate(L.RACK_X):
    path = f"{ROOT}/Rack_{i}"
    prim = spawn_rack(stage, app, path, translate=(rx, 0.0, 0.0))
    cache.Clear()
    bb = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    lo, hi = bb.GetMin(), bb.GetMax()
    # slide it so the rear face is RACK_GAP off the wall and x is centred on rx
    dx = rx - (lo[0] + hi[0]) / 2.0
    dy = (wall_inner_y - L.RACK_GAP) - hi[1]
    UsdGeom.Xformable(prim).GetOrderedXformOps()[0].Set(
        Gf.Vec3d(rx + dx, dy, 0.0))
    for _ in range(2):
        app.update()
    cache.Clear()
    bb = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    lo, hi = bb.GetMin(), bb.GetMax()
    n = ensure_collision(prim)
    decks = find_decks(prim)
    print(f"[cell] Rack_{i}  x {lo[0]:+.3f}..{hi[0]:+.3f}  "
          f"y {lo[1]:+.3f}..{hi[1]:+.3f}  z {lo[2]:.3f}..{hi[2]:.3f}  "
          f"colliders {n}  decks {len(decks)}")
    for d in decks:
        print(f"         deck z={d['z']:.3f}  boards " +
              "  ".join(f"x {b[0]:+.3f}..{b[1]:+.3f} y {b[2]:+.3f}..{b[3]:+.3f}"
                        for b in d["boards"]))

stage.Export(OUT)
print(f"[cell] floor {fx} x {fy} m, walls N+W, doorway {L.DOOR_W} m "
      f"at west x={-fx / 2.0:+.2f}, y={L.DOOR_CY:+.2f}")
print(f"[cell] wrote {OUT}")
app.close()
