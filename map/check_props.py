# Does the scene hold still? Loads the cell, spawns the props, steps physics,
# and reports how far each one moved.
#
# The map is only "built" if nothing is launched at t=0. Interpenetration at
# spawn looks exactly like a physics bug at runtime -- a crate on a shelf that
# is 3 mm inside the board leaves as a projectile -- so this measures it instead
# of leaving it to the eye in the viewport.
#
#   python map/check_props.py [seconds]
import os
import sys

from isaaclab.app import AppLauncher

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "map"))
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
CELL = os.path.join(REPO, "map", "cell.usd")

app = AppLauncher(headless=True).app

import isaaclab.sim as sim_utils  # noqa: E402
import omni.usd  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from pxr import Sdf, Usd, UsdGeom  # noqa: E402

from props import spawn_props  # noqa: E402

# use_fabric off so the USD transforms follow the sim and can be read back here
sim = SimulationContext(sim_utils.SimulationCfg(dt=1.0 / 120.0, device="cpu",
                                                use_fabric=False))
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
stage.DefinePrim("/World/Cell", "Xform").GetReferences().AddReference(
    Sdf.Reference(CELL))
for _ in range(3):
    app.update()

spawn_props(stage, app)

props = {p.GetName(): p for p in stage.GetPrimAtPath("/World/Props").GetChildren()}
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])


def centres():
    cache.Clear()
    out = {}
    for n, p in props.items():
        r = cache.ComputeWorldBound(p).ComputeAlignedRange()
        lo, hi = r.GetMin(), r.GetMax()
        out[n] = ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, lo[2])
    return out


start = centres()
sim.reset()
for _ in range(int(SECONDS * 120)):
    sim.step(render=False)
end = centres()

print(f"\n[check] after {SECONDS:.1f} s of physics "
      f"(base_z = the bottom of the object, not its centre)")
worst = 0.0
for n in sorted(props):
    s, e = start[n], end[n]
    d = ((e[0] - s[0]) ** 2 + (e[1] - s[1]) ** 2) ** 0.5
    dz = e[2] - s[2]
    worst = max(worst, d)
    flag = "   MOVED" if d > 0.05 or abs(dz) > 0.05 else ""
    print(f"[check] {n:<12} xy drift {d:.3f} m   base_z {s[2]:.3f} -> {e[2]:.3f}"
          f"  ({dz:+.3f}){flag}")
print(f"[check] worst xy drift {worst:.3f} m")
app.close()
