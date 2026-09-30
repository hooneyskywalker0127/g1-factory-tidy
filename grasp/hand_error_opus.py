"""How far the executed hand is from the planned grasp when the fingers start to close.
    python hand_error_opus.py PICK_RUN.log REACH_PICK.npz STATE_K.json
The same correction the chain applies to the walk and the kneel (aim the goal by the measured stop
error): the render's arm reaches the planned wrist within ~2.5 cm, and for a fingertip pinch on a
4 cm handle that is the difference between catching and shoving (drill v3 #76: tester HELD at the
exact pose, render LOST). Measured in the run's own frame: the log's "[hand] frame N rel. to
object: right_wrist_yaw [dx dy dz]" line nearest the close frame (wrist relative to the object)
against the plan's wrist target relative to the object pose the reach was solved for (STATE_K).
Prints the error and an OBJECT_POSE_NOW string shifted by -error for the next reach solve.
"""
import json
import re
import sys

import numpy as np

log, npz, state = sys.argv[1:4]
d = np.load(npz)
close_at = None
for l in open(log, errors="ignore"):
    m = re.search(r"hands close at frame (\d+)", l)
    if m:
        close_at = int(m.group(1))
if close_at is None:
    close_at = int(sys.argv[4]) if len(sys.argv) > 4 else None
assert close_at is not None, "pass the close frame as the 4th argument"
best = None
for l in open(log, errors="ignore"):
    m = re.match(r"\[hand\] frame\s+(\d+) rel\. to object: right_wrist_yaw \[([^\]]*)\]", l)
    if m and int(m.group(1)) <= close_at and (best is None or int(m.group(1)) > best[0]):
        best = (int(m.group(1)), np.array([float(v) for v in m.group(2).split()]))
assert best is not None, "no [hand] rel. to object line before the close"
st = json.load(open(state))
obj = np.asarray(st["object_pos"], float)
planned_rel = d["grasp"][:3, 3] - obj
err = best[1] - planned_rel
now = ",".join("%.5f" % v for v in list(obj - err) + list(st["object_quat_wxyz"]))
print(f"[hand-error] frame {best[0]} (close at {close_at}): wrist-object measured {np.round(best[1], 3)} planned "
      f"{np.round(planned_rel, 3)} -> error {np.round(err, 3)} ({np.linalg.norm(err) * 1000:.0f} mm)")
print(f"[hand-error] OBJECT_POSE_NOW shifted by -error: {now}")
