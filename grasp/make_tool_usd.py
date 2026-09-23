# Turn a Dexonomy object into a USD the Isaac scene can spawn.
#
# The grasps were synthesised against THIS mesh at THIS scale, so the same
# geometry has to appear in the cell -- pulling the original YCB asset instead
# would put the fingers somewhere the synthesiser never saw.
#
# The convex pieces the synthesiser used for collision are reused as collision
# geometry, so contact in Isaac matches contact in MuJoCo. The visual mesh is
# the simplified one.
#
#   python grasp/make_tool_usd.py <object_name> <scale> [out.usd]
import glob
import json
import os
import sys

import numpy as np
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

DEX = os.path.expanduser("~/Projects/Dexonomy")
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

name = sys.argv[1] if len(sys.argv) > 1 else "ddg_gd_drill_poisson_000"
scale = float(sys.argv[2]) if len(sys.argv) > 2 else 0.20
out = sys.argv[3] if len(sys.argv) > 3 else os.path.join(
    REPO, "assets", "tools", f"{name}_s{int(scale*1000):03d}.usd")

src = os.path.join(DEX, "assets", "object", "DGN_5k", "processed_data", name)
info = json.load(open(os.path.join(src, "info", "simplified.json")))
# The grasps were validated against a 0.1 kg object -- that is Dexonomy's
# obj_mass default, and every succ_grasp held at that weight. Giving the USD a
# different mass would test a grasp nobody verified.
mass = 0.1

stage = Usd.Stage.CreateNew(out)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
root = UsdGeom.Xform.Define(stage, "/Tool")
stage.SetDefaultPrim(root.GetPrim())
UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(float(mass))

mat = UsdShade.Material.Define(stage, "/Tool/Looks/tool")
sh = UsdShade.Shader.Define(stage, "/Tool/Looks/tool/Shader")
sh.CreateIdAttr("UsdPreviewSurface")
sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.85, 0.55, 0.15))
sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")


def add_mesh(path, obj_path, collide):
    m = trimesh.load(obj_path, process=False, force="mesh")
    v = np.asarray(m.vertices, dtype=np.float32) * scale
    f = np.asarray(m.faces, dtype=np.int32)
    g = UsdGeom.Mesh.Define(stage, path)
    g.CreatePointsAttr([Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in v])
    g.CreateFaceVertexCountsAttr([3] * len(f))
    g.CreateFaceVertexIndicesAttr(f.flatten().tolist())
    lo_, hi_ = v.min(axis=0), v.max(axis=0)
    g.CreateExtentAttr([Gf.Vec3f(*map(float, lo_)), Gf.Vec3f(*map(float, hi_))])
    UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(g.GetPrim()).CreateApproximationAttr(
            "convexHull")           # each piece is already convex
    else:
        g.CreatePurposeAttr(UsdGeom.Tokens.guide)
    return v


pieces = sorted(glob.glob(os.path.join(src, "urdf", "meshes", "*.obj")))
allv = []
for i, p in enumerate(pieces):
    allv.append(add_mesh(f"/Tool/piece_{i:03d}", p, collide=True))
v = np.concatenate(allv)
lo, hi = v.min(axis=0), v.max(axis=0)
stage.GetRootLayer().Save()
print(f"[tool] {name} x{scale}  {len(pieces)} convex pieces, mass {mass*1000:.0f} g")
print(f"[tool] size {(hi-lo)[0]*1000:.0f} x {(hi-lo)[1]*1000:.0f} x {(hi-lo)[2]*1000:.0f} mm, "
      f"origin at {(-lo/(hi-lo)).round(2)}")
print(f"[tool] wrote {out}")
