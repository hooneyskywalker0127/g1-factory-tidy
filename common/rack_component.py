# Rack component -- Environments/Hospital/Props/SM_MedShelf_01d.
#
# Already modelled at real-world size (metersPerUnit 1.0, 0.910 x 0.805 x
# 2.012 m), so it is referenced as-is with no scaling.
#
# The awkward part is working out where a crate may actually be set down. The
# asset is a single mesh -- no per-board prims to query -- and it is a
# DOUBLE-SIDED shelf: every deck is two boards with a ~53 mm slot down the
# middle, not one continuous surface. Deck 3 measures
#
#   board  y -0.397 .. -0.028      slot  y -0.028 .. +0.025      board  y +0.025 .. +0.393
#
# so anything centred on the deck straddles the slot instead of resting on a
# board. The boards are recovered here by walking the mesh's horizontal faces
# and unioning their y intervals, which gives real material coverage; a vertex
# histogram cannot, because a flat board has no vertices in its interior.
#
# Deck heights come from the same faces rather than from histogram bin edges --
# the bin edge overshot the true surface by ~4 mm and left crates floating.
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom

RACK_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
    "/Assets/Isaac/5.1/Isaac/Environments/Hospital/Props/SM_MedShelf_01d.usd"
)

FLAT = 1e-4      # a face this level in z counts as horizontal, metres
SAME_Z = 0.002   # faces this close in z are the same surface plane, metres
MERGE = 0.05     # planes this close belong to one board (top face vs underside)
JOIN = 0.005     # y intervals this close are one continuous board, metres


def spawn_rack(stage, app, path="/World/Rack", translate=(0.0, 0.0, 0.0)):
    """Reference the shelf under a wrapper Xform that owns the placement.

    The referenced default prim carries its own xformOps, so the wrapper keeps
    this from either duplicating an op or fighting the asset's own transform.
    """
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*translate))
    stage.DefinePrim(path + "/Asset", "Xform").GetReferences().AddReference(
        Sdf.Reference(RACK_URL))
    for _ in range(3):
        app.update()
    return prim


def _union(intervals):
    """Merge overlapping (lo, hi) spans into the ranges they actually cover."""
    out = []
    for lo, hi in sorted(intervals):
        if out and lo <= out[-1][1] + JOIN:
            out[-1][1] = max(out[-1][1], hi)
        else:
            out.append([lo, hi])
    return [tuple(v) for v in out]


def _horizontal_faces(prim):
    """Every horizontal face under `prim`, as (z, x_lo, x_hi, y_lo, y_hi)."""
    faces = []
    for p in Usd.PrimRange(prim):
        mesh = UsdGeom.Mesh(p)
        if not mesh:
            continue
        pts = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        idx = mesh.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts:
            continue
        m = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        pts = np.array([tuple(m.Transform(v)) for v in pts])
        idx = np.asarray(idx)
        off = 0
        for c in counts:
            v = pts[idx[off:off + c]]
            off += c
            if v[:, 2].max() - v[:, 2].min() < FLAT:
                faces.append((v[0, 2], v[:, 0].min(), v[:, 0].max(),
                              v[:, 1].min(), v[:, 1].max()))
    return faces


def find_decks(prim):
    """Deck surfaces, lowest first, as {"z", "boards": [(x0, x1, y0, y1), ...]}.

    Only the top plane of each board is kept -- the underside sits a few
    centimetres below and would otherwise be reported as its own deck.
    """
    faces = _horizontal_faces(prim)
    if not faces:
        return []

    planes = {}
    for z, x0, x1, y0, y1 in faces:
        planes.setdefault(round(z / SAME_Z) * SAME_Z, []).append((x0, x1, y0, y1))

    # A board is a few planes stacked within its own thickness; keep the top.
    tops, group = [], []
    for z in sorted(planes) + [None]:
        if group and (z is None or z - group[-1] > MERGE):
            tops.append(group[-1])
            group = []
        if z is not None:
            group.append(z)

    decks = []
    for z in tops:
        near = [f for f in faces if abs(f[0] - z) <= SAME_Z]
        if not near:
            continue
        boards = []
        for y0, y1 in _union([(f[3], f[4]) for f in near]):
            on = [f for f in near if f[3] < y1 + JOIN and f[4] > y0 - JOIN]
            xs = _union([(f[1], f[2]) for f in on])
            if not xs:
                continue
            boards.append((xs[0][0], xs[-1][1], y0, y1))
        if boards:
            decks.append({"z": z, "boards": boards})
    return decks
