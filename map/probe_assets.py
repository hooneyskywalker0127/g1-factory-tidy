# Measure candidate props before picking one, instead of guessing from the name.
#
# Prints each asset's size as it actually arrives on a metres stage, and where
# its origin sits in its own bounding box -- the origin decides whether a
# placement can key off the base, and a size around 60 instead of 0.6 is how a
# centimetre-authored asset announces itself (USD does not rescale a reference
# to the host stage's units).
#
#   python map/probe_assets.py
import os
import sys

from isaaclab.app import AppLauncher

app = AppLauncher(headless=True).app

import omni.usd  # noqa: E402
from pxr import Sdf, Usd, UsdGeom  # noqa: E402

BASE = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
        "/Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/Props")
NAMES = ["SM_CardBoxA_01", "SM_CardBoxA_02", "SM_CardBoxB_01", "SM_CardBoxB_02",
         "SM_CardBoxC_01", "SM_CardBoxC_02", "SM_CardBoxD_01", "SM_CardBoxD_02",
         "SM_CardBoxD_03", "SM_CratePlastic_A_01", "SM_CratePlastic_B_01",
         "SM_CratePlastic_C_01"]

omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

for i, name in enumerate(NAMES):
    path = f"/World/P{i}"
    url = f"{BASE}/{name}.usd"
    stage.DefinePrim(path, "Xform").GetReferences().AddReference(Sdf.Reference(url))
    for _ in range(3):
        app.update()
    cache.Clear()
    r = cache.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
    lo, hi = r.GetMin(), r.GetMax()
    if r.IsEmpty():
        print(f"{name:<22} EMPTY (failed to load)")
        continue
    print(f"{name:<22} size {hi[0]-lo[0]:.3f} x {hi[1]-lo[1]:.3f} x {hi[2]-lo[2]:.3f}"
          f"   origin at x{-lo[0]/(hi[0]-lo[0]):.2f} y{-lo[1]/(hi[1]-lo[1]):.2f}"
          f" z{-lo[2]/(hi[2]-lo[2]):.2f}")

app.close()
