"""walk_clip.py with the planner's walking mode chosen by WALK_MODE (default 2, WALK).
    WALK_MODE=21 python walk_clip_mode_opus.py <walk_clip.py arguments>
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
W.main()
