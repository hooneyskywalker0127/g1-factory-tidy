"""Small operations on motion-lib clips (results/motion/*.pkl), for the perceive-and-replan chain.
    python clip_tools_wb.py cut IN.pkl A B HOLD OUT_NAME        frames [A, B) then HOLD copies of frame B-1
    python clip_tools_wb.py concat A.pkl B.pkl OUT_NAME           A then B, verbatim (no blend, no shift)
    python clip_tools_wb.py from_state STATE.json N OUT_NAME      N identical frames of a DUMP_STATE pose
                                                                    (walk_clip --from-clip reads the last 4 as context)
Clips are qpos rows [xyz, quat wxyz, 29 dof (MuJoCo order)] through build_place_reference.clip_qpos and
gen_planner_motion.qpos_to_motion_lib, the same round trip rise_reference_wb.py uses.
"""
import json
import os
import sys

import joblib
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_reach_reference import mujoco_names, qpos_to_motion_lib  # noqa: E402
from build_place_reference import clip_qpos  # noqa: E402

FPS = 30


def save(q, name):
    path = os.path.join("results", "motion", f"{name}.pkl")
    joblib.dump({name: qpos_to_motion_lib(np.asarray(q, np.float32), FPS)}, path, compress=True)
    print(f"[clip] {len(q)} frames -> {path}; root {np.round(q[0][:3], 3)} -> {np.round(q[-1][:3], 3)}")


cmd = sys.argv[1]
if cmd == "cut":
    q = clip_qpos(sys.argv[2]); a, b, hold = int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5])
    save(np.concatenate([q[a:b], np.repeat(q[b - 1:b], hold, axis=0)]), sys.argv[6])
elif cmd == "concat":
    save(np.concatenate([clip_qpos(sys.argv[2]), clip_qpos(sys.argv[3])]), sys.argv[4])
elif cmd == "from_state":
    st = json.load(open(sys.argv[2])); n = int(sys.argv[3])
    row = np.array(list(st["root_pos"]) + list(st["root_quat_wxyz"]) + [st["dof"][k] for k in mujoco_names()], np.float32)
    save(np.repeat(row[None], n, axis=0), sys.argv[4])
else:
    raise SystemExit(__doc__)
