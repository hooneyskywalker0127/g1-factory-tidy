"""walk_clip.py with the planner's walking mode chosen by WALK_MODE (default 2, WALK).
    WALK_MODE=21 python walk_clip_mode_wb.py <walk_clip.py arguments>
Planner modes (GR00T planner_onnx.md:106-151): 1 slowWalk, 2 walk, 3 run, 21 objectCarrying
("Walking with hands reaching out", V2 planner). walk_clip.py is read-only here, so the mode
constant it reads inside walk_to() is set on the imported module and its main() is called.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import walk_clip as W  # noqa: E402

W.WALK = int(os.environ.get("WALK_MODE", "2"))
print(f"[walk] planner walking mode {W.WALK}")
# TARGET_VEL: the planner's own speed override (planner_onnx.md:85-92, "target_vel", m/s; <= 0 = the mode's default).
# walk_clip's _inputs() leaves it at -1. GR00T's own loop doubles the requested speed because "the actual speed is
# usually 0.5 of the target speed" (motionbricks/.../demo/full_agent.py:227-228) -- the tracker executes about half
# of what the planner asks, which is what the drill's short approach walks showed (26-56%, 0-0.18 m of 0.5-0.7 m).
if os.environ.get("TARGET_VEL"):
    _tv = float(os.environ["TARGET_VEL"]); _inputs0 = W._inputs

    def _inputs_tv(mode, seed, height, *a, **k):
        d = _inputs0(mode, seed, height, *a, **k)
        d["target_vel"] = W.np.array([_tv], W.np.float32)
        return d
    W._inputs = _inputs_tv
    print(f"[walk] planner target_vel {_tv} m/s")
W.main()
