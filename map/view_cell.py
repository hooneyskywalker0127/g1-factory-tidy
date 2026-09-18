# Open the cell in the Isaac Sim GUI and just sit there so it can be looked at.
#
# cell.usd is REFERENCED into a throwaway stage rather than opened directly, so
# nothing typed in the viewport (or a stray Ctrl+S) can write back into the
# built map. Lights live here and not in cell.usd for the same reason the props
# do not: the map is geometry, whoever loads it brings their own lighting.
#
#   python map/view_cell.py [file.usd]
import os
import sys

from isaaclab.app import AppLauncher

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "map"))
CELL = sys.argv[1] if len(sys.argv) > 1 else os.path.join(REPO, "map", "cell.usd")
if not os.path.isfile(CELL):
    raise SystemExit(f"missing {CELL} -- build it with: python map/build_cell.py")

app = AppLauncher(headless=False).app

import omni.usd  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux  # noqa: E402

omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
stage.DefinePrim("/World/Cell", "Xform").GetReferences().AddReference(
    Sdf.Reference(CELL))

dome = UsdLux.DomeLight.Define(stage, "/World/Light/Dome")
dome.CreateIntensityAttr(900.0)
sun = UsdLux.DistantLight.Define(stage, "/World/Light/Sun")
sun.CreateIntensityAttr(1800.0)
sun.CreateAngleAttr(1.0)
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-50.0, 0.0, -35.0))

from props import spawn_props  # noqa: E402

nb, nc = spawn_props(stage, app)
print(f"[view] {nb} boxes, {nc} crates")

print(f"[view] {CELL} -- close the window to quit")
while app.is_running():
    app.update()
app.close()
