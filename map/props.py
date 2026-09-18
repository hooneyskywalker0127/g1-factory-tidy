# The movable contents of the cell: cardboard boxes and stacking crates.
#
# Kept out of cell.usd on purpose. Everything in here is a rigid body -- the
# robot picks it up -- and rigid bodies cannot be baked into the static shell:
# they would either be frozen as scenery or arrive already falling. The shell is
# built once and never changes; this table is the scene's state, and a task
# resets it.
#
# The crate is assets/empty_crate.usd, which already arrives as its own rigid
# body (2 kg, convexDecomposition colliders, its own friction material). The box
# is the warehouse CardBoxA, which is plain geometry, so the body and colliders
# are applied here.
import math
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "common"))
sys.path.insert(0, os.path.join(REPO, "map"))

import cell_layout as L  # noqa: E402
from box_component import BOX_URL, spawn_box  # noqa: E402
from rack_component import find_decks  # noqa: E402

CRATE_USD = os.path.join(REPO, "assets", "empty_crate.usd")
CRATE_MASS = 2.0
BOX_MASS = 2.0
CRATE_H = 0.170          # tray_component's measured height, for deck headroom

# The shelf cartons, sized off map/probe_assets.py. Same warehouse set as the
# floor box, just the small ones that clear a 0.374 m deck.
_WH = BOX_URL.rsplit("/", 1)[0]
SHELF_BOX_URL = {"D1": f"{_WH}/SM_CardBoxD_01.usd",
                 "D3": f"{_WH}/SM_CardBoxD_03.usd"}
SHELF_BOX_MASS = 1.0


def _place(stage, path, url, xy, z, rotz):
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(xy[0], xy[1], z))
    if rotz:
        xf.AddRotateZOp().Set(rotz)
    stage.DefinePrim(path + "/Asset", "Xform").GetReferences().AddReference(
        Sdf.Reference(url))
    return prim


def _make_body(prim, mass):
    """Box geometry arrives bare -- give it one body and hull colliders."""
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    for p in Usd.PrimRange(prim):
        if UsdGeom.Mesh(p):
            UsdPhysics.CollisionAPI.Apply(p)
            UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr(
                "convexHull")


def _usable_decks(rack):
    """Decks a crate could be set on, lowest first -- the index tables use these.

    Clearance is fixed at the crate's height even when a box is going on the
    deck, so that deck 2 means the same shelf in every layout table.
    """
    decks = find_decks(rack)
    out = []
    for k, d in enumerate(decks[:-1]):
        if sum(b[3] - b[2] for b in d["boards"]) < 0.6 * 0.805:
            continue                                  # a cross rail, not a deck
        if decks[k + 1]["z"] - d["z"] < CRATE_H + 0.02:
            continue                                  # no headroom
        d["headroom"] = decks[k + 1]["z"] - d["z"]
        out.append(d)
    return out


def _overhang_note(over_front, over_side, depth):
    """A little hanging off the front is how a crate is left to be picked up
    again; hanging off a side, or off the front by a quarter of itself, is not."""
    bits = []
    if over_side > 0.001:
        bits.append(f"OFF THE SIDE by {over_side:.3f}")
    if over_front > depth * 0.25:
        bits.append(f"OFF THE FRONT by {over_front:.3f}")
    elif over_front > 0.001:
        bits.append(f"front overhang {over_front:.3f}")
    return ("  " + ", ".join(bits)) if bits else ""


def _front_board(deck):
    """The board on the room side. The deck's other board is across the wire
    partition, and the middle of a deck is the partition itself."""
    return min(deck["boards"], key=lambda b: b[2])


def _on_board(board, foot, rotz, dx):
    """Where to put something so it rests on `board` and clears the partition.

    Turning a footprint grows the depth it needs, so the turn is taken into
    account here rather than measured after the fact.
    """
    r = math.radians(rotz)
    depth = abs(foot[0] * math.sin(r)) + abs(foot[1] * math.cos(r))
    width = abs(foot[0] * math.cos(r)) + abs(foot[1] * math.sin(r))
    cx = (board[0] + board[1]) / 2.0 + dx
    cy = board[3] - L.SHELF_CLEAR - depth / 2.0
    over_front = max(0.0, board[2] - (cy - depth / 2.0))
    over_side = max(0.0, (cx + width / 2.0) - board[1],
                    board[0] - (cx - width / 2.0))
    return cx, cy, over_front, over_side


def _extent(cache, prim):
    cache.Clear()
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    return r.GetMin(), r.GetMax()


def _report_overlaps(cache, prims):
    """Anything already interpenetrating at spawn gets launched on the first
    step, so say so here rather than let it look like a physics mystery."""
    cache.Clear()
    boxes = {}
    for name, p in prims.items():
        r = cache.ComputeWorldBound(p).ComputeAlignedRange()
        boxes[name] = (r.GetMin(), r.GetMax())
    names = sorted(boxes)
    hits = 0
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            lo_a, hi_a = boxes[a]
            lo_b, hi_b = boxes[b]
            gap = [max(lo_a[k], lo_b[k]) - min(hi_a[k], hi_b[k]) for k in range(3)]
            if max(gap) < 0.0:                     # overlapping on all three axes
                over = [-g for g in gap]
                print(f"[props] OVERLAP {a} / {b}  by "
                      f"{over[0]:.3f} x {over[1]:.3f} x {over[2]:.3f} m")
                hits += 1
    print(f"[props] {hits} overlapping pair(s) at spawn")
    return hits


def spawn_props(stage, app, root="/World/Props"):
    """Put the boxes and crates in the cell. Returns how many of each."""
    if not os.path.isfile(CRATE_USD):
        raise SystemExit(f"missing {CRATE_USD} -- copy it from humanoid-swarm-sim "
                         f"or rebuild with common/make_empty_crate_usd.py")
    UsdGeom.Xform.Define(stage, root)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    spawned = {}

    for i, (x, y, rz) in enumerate(L.FLOOR_BOXES):
        w, d, h = L.FLOOR_BOX_SIZE
        p = spawn_box(stage, app, f"{root}/Box_{i}", centre_xy=(x, y), base_z=0.0,
                      rotz=rz, width=w, depth=d, height=h)
        _make_body(p, L.FLOOR_BOX_MASS)
        spawned[f"Box_{i}"] = p
        lo, hi = _extent(cache, p)
        print(f"[props] Box_{i}   x {lo[0]:+.2f}..{hi[0]:+.2f}  "
              f"y {lo[1]:+.2f}..{hi[1]:+.2f}  z {lo[2]:.2f}..{hi[2]:.2f}")

    for i, (x, y, rz) in enumerate(L.FLOOR_CRATES):
        p = _place(stage, f"{root}/Crate_{i}", CRATE_USD, (x, y), 0.0, rz)
        app.update()
        spawned[f"Crate_{i}"] = p
        lo, hi = _extent(cache, p)
        print(f"[props] Crate_{i} x {lo[0]:+.2f}..{hi[0]:+.2f}  "
              f"y {lo[1]:+.2f}..{hi[1]:+.2f}  z {lo[2]:.2f}..{hi[2]:.2f}")

    # --- what is already put away ----------------------------------------
    racks = sorted((p for p in stage.Traverse()
                    if p.GetName().startswith("Rack_") and p.GetTypeName() == "Xform"
                    and "/Asset" not in str(p.GetPath())),
                   key=lambda p: p.GetName())
    decks = {i: _usable_decks(r) for i, r in enumerate(racks)}
    for i, ds in decks.items():
        print(f"[props] Rack_{i} usable decks: " +
              "  ".join(f"{k}:z={d['z']:.3f}/h={d['headroom']:.3f}"
                        for k, d in enumerate(ds)))

    for n, (ri, di) in enumerate(L.DECK_CRATES):
        if ri not in decks or di >= len(decks[ri]):
            print(f"[props] no Rack_{ri} deck {di} -- skipped shelved crate {n}")
            continue
        deck = decks[ri][di]
        board = _front_board(deck)
        cx, cy, of, os_ = _on_board(board, L.CRATE_FOOT_SHELVED,
                                    L.CRATE_SHELVED_ROTZ, 0.0)
        p = _place(stage, f"{root}/Shelved_{n}", CRATE_USD, (cx, cy),
                   deck["z"] + 0.002, L.CRATE_SHELVED_ROTZ)
        app.update()
        spawned[f"Shelved_{n}"] = p
        lo, hi = _extent(cache, p)
        note = _overhang_note(of, os_, L.CRATE_FOOT_SHELVED[1])
        print(f"[props] Shelved_{n} crate on Rack_{ri} deck {di} z={deck['z']:.3f}  "
              f"x {lo[0]:+.3f}..{hi[0]:+.3f}  y {lo[1]:+.3f}..{hi[1]:+.3f}  "
              f"z {lo[2]:.3f}..{hi[2]:.3f}{note}")

    for n, (ri, di, dx, rz, kind) in enumerate(L.SHELF_BOXES):
        if ri not in decks or di >= len(decks[ri]):
            print(f"[props] no Rack_{ri} deck {di} -- skipped shelf box {n}")
            continue
        deck = decks[ri][di]
        board = _front_board(deck)
        cx, cy, of, os_ = _on_board(board, L.SHELF_BOX_FOOT[kind], rz, dx)
        p = _place(stage, f"{root}/ShelfBox_{n}", SHELF_BOX_URL[kind],
                   (cx, cy), deck["z"] + 0.002, rz)
        app.update()
        _make_body(p, SHELF_BOX_MASS)
        spawned[f"ShelfBox_{n}"] = p
        lo, hi = _extent(cache, p)
        note = _overhang_note(of, os_, L.SHELF_BOX_FOOT[kind][1])
        head = max(0.0, (lo[2] + deck["headroom"]) - hi[2])
        print(f"[props] ShelfBox_{n} {kind} on Rack_{ri} deck {di}  "
              f"x {lo[0]:+.3f}..{hi[0]:+.3f}  y {lo[1]:+.3f}..{hi[1]:+.3f}  "
              f"z {lo[2]:.3f}..{hi[2]:.3f}  headroom left {head:.3f}{note}")

    _report_overlaps(cache, spawned)
    return (len(L.FLOOR_BOXES) + len(L.SHELF_BOXES),
            len(L.FLOOR_CRATES) + len(L.DECK_CRATES))
