# Bake the plan's table + target box into a USD for SONIC's SONIC_PROPS_USD.
#
# grasp/plan_scene.py already places exactly these two, from the plan's own
# numbers, relative to a torso pose. The SONIC tracking env spawns props from
# a USD file, so this runs the same builder on an empty stage and exports it.
#
#   python grasp/bake_props_usd.py TORSO.json OUT.usd [--plan results/g1_graspgen.json]
import json
import os
import sys

from isaaclab.app import AppLauncher

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "grasp"))

TORSO_JSON = sys.argv[1]
OUT = sys.argv[2]
PLAN = (sys.argv[sys.argv.index("--plan") + 1] if "--plan" in sys.argv
        else os.path.join(REPO, "results", "g1_graspgen.json"))

app = AppLauncher(headless=True).app

import numpy as np  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
import omni.usd  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from pxr import UsdGeom  # noqa: E402

from plan_scene import build as build_plan_scene  # noqa: E402

sim = SimulationContext(sim_utils.SimulationCfg(dt=1.0 / 120.0, device="cpu"))
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
for _ in range(3):
    app.update()

T_torso = np.array(json.load(open(TORSO_JSON))["torso"], dtype=np.float64)
meta = json.load(open(PLAN))
build_plan_scene(stage, app, meta, T_torso)
# The same scene variants props.py takes from the environment, so the
# evaluator sees the room the language run saw (a crate on the desk).
if os.environ.get("TIDY_CRATE_ON_DESK"):
    sys.path.insert(0, os.path.join(REPO, "map"))
    from props import _place, CRATE_USD  # noqa: E402
    cx, cy, cz, crz = (float(v) for v in os.environ["TIDY_CRATE_ON_DESK"].split(","))
    _place(stage, "/World/DeskCrate", CRATE_USD, (cx, cy), cz + 0.002, crz)
    print("[bake] crate on the desk")
for _ in range(5):
    app.update()

# A referenced USD contributes nothing unless it names a default prim, which
# is why the first bake spawned an empty /World/FactoryProps.
stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
os.makedirs(os.path.dirname(OUT), exist_ok=True)
stage.Export(OUT)
print("[bake] prims:", [str(x.GetPath()) for x in stage.GetPrimAtPath("/World").GetChildren()])
print(f"[bake] torso {np.round(T_torso[:3, 3], 4)}")
print(f"[bake] wrote {OUT}")
sys.stdout.flush()
os._exit(0)
