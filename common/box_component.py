# Box component -- Environments/Simple_Warehouse/Props/SM_CardBoxA_01.
#
# The shipping box the rest of the plant is already dressed with, used here in
# place of the yellow stacking crate: at 0.170 m the crate's lid sits barely
# above the robot's ankles, and a squat down to it does not read as one.
#
# Unlike the PackingTable crate this one is authored in metres already
# (metersPerUnit 1.0), so it is referenced at true size with no unit scale.
# Measured off the asset:
#   bbox min (-0.350, -0.250, 0.000)  max (0.350, 0.250, 0.500) m
# so it is centred on x/y with its base on z=0 -- same convention as
# tray_component, and placement can key off the base without a probe pass.
from pxr import Gf, Sdf, UsdGeom

BOX_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
    "/Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/Props"
    "/SM_CardBoxA_01.usd"
)

SIZE = (0.700, 0.500, 0.500)         # outer, metres, as authored


def spawn_box(stage, app, path, centre_xy=(0.0, 0.0), base_z=0.0, rotz=0.0,
              width=None, height=None, depth=None):
    """Reference the box centred on `centre_xy` with its base on `base_z`.

    Ops go on a wrapper Xform because the referenced default prim already
    carries its own. `rotz` turns it about its own centre -- 90 swaps which way
    the 0.700 m length runs, which is what decides whether the long face or the
    short one is presented to the robot.

    `width` overrides that length in metres, by scaling the asset's own x. It
    is how the box is made exactly as wide as the robot's hands are apart:
    cardboard is a printed box, so stretching one axis reads as a different
    carton rather than as a distorted model, and depth and height stay true.

    `depth` does it to y, the axis that ends up along the robot's approach.
    The grip is across the box, on the two side faces, so depth buys the demo
    nothing -- and it costs: every centimetre of it is floor the G1 cannot put
    a knee on when it kneels down beside it.

    `height` does the same to z. It exists because the squat demo turns on how
    HIGH the thing on the floor is: probe_aiw_floor.py measures the AI Worker
    reaching 0.343 m forward with its hand at 0.30..0.35 and only -0.051 m below
    that, against a box face 0.435 m out, so a carton whose lid is under ~0.32
    is out of its range at every height -- while at 0.500 m the same carton is
    comfortably in it. The demo only reads if the box is a low one.
    """
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(centre_xy[0], centre_xy[1], base_z))
    if rotz:
        xf.AddRotateZOp().Set(rotz)
    if width or height or depth:
        xf.AddScaleOp().Set(Gf.Vec3f(width / SIZE[0] if width else 1.0,
                                     depth / SIZE[1] if depth else 1.0,
                                     height / SIZE[2] if height else 1.0))
    stage.DefinePrim(path + "/Asset", "Xform").GetReferences().AddReference(
        Sdf.Reference(BOX_URL))
    for _ in range(2):
        app.update()
    return prim
