# Tray component -- Props/PackingTable/props/SM_Crate_A07_Yellow_01.
#
# The crate is authored in centimetres (metersPerUnit 0.01) while this project
# works in metres, and USD does not rescale a reference to the host stage's
# units. Referenced raw onto a metres stage it arrives 100x oversized -- which
# is the whole reason it first dwarfed the shelf. Scaling by 0.01 is a unit
# conversion, not a resize.
#
# At true size it is 0.601 x 0.402 x 0.170 m, measured off the asset:
#   bbox min (-30.073, -20.089, 0.0)  max (30.073, 20.089, 17.001) cm
# so it is centred on x/y and its floor sits on z=0 -- placement can key off
# the base directly without a probe pass.
#
# That is a real vented stacking crate, though not the same one as the 570 x
# 410 x 180 mm spec sheet it was picked against. Within 5% on every axis, so it
# is left at its own size; forcing the spec numbers would bend the mesh for no
# visible gain.
from pxr import Gf, Sdf, UsdGeom

TRAY_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
    "/Assets/Isaac/5.1/Isaac/Props/PackingTable/props"
    "/SM_Crate_A07_Yellow_01/SM_Crate_A07_Yellow_01.usd"
)

UNIT = 0.01                          # centimetre asset -> metre stage
SIZE = (0.601, 0.402, 0.170)         # outer, metres, after the conversion

# Inside face of the crate, measured off the same mesh: the floor panel's top
# sits 7.4 mm above the crate's base and the walls leave 577 x 377 mm clear.
# Anything dropped in the crate keys off these, not off the outer box, or it
# ends up buried in the floor or poking through a wall.
INNER_FLOOR = 0.0074
INNER = (0.577, 0.377)


def spawn_tray(stage, app, path, centre_xy=(0.0, 0.0), base_z=0.0, rotz=0.0,
               width=None, depth=None, height=None):
    """Reference the crate centred on `centre_xy` with its floor on `base_z`.

    The crate's own origin is centred in x/y with the floor at z=0, so the
    translate is the target position as-is. Ops go on a wrapper Xform because
    the referenced default prim already carries its own. `rotz` turns it about
    its own centre -- 90 swaps which way the 601 mm length runs, which is what
    decides whether it fits a shelf or a robot's arms.
    """
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(centre_xy[0], centre_xy[1], base_z))
    if rotz:
        xf.AddRotateZOp().Set(rotz)
    # The stock crate is 0.170 m tall, which is under a squatting G1's palms --
    # it can only be patted, not gripped. Scaling it up in z gives the walls
    # something to close on while keeping the same moulded crate; width and
    # depth follow so the palms land on its two ends.
    xf.AddScaleOp().Set(Gf.Vec3f(
        UNIT * (width / SIZE[0] if width else 1.0),
        UNIT * (depth / SIZE[1] if depth else 1.0),
        UNIT * (height / SIZE[2] if height else 1.0)))
    stage.DefinePrim(path + "/Asset", "Xform").GetReferences().AddReference(
        Sdf.Reference(TRAY_URL))
    for _ in range(2):
        app.update()
    return prim
