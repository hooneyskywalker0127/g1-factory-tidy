# Turn a GraspGenX end2end trajectory.json into what grasp/play_in_cell.py eats.
#
# The plan is made outside Isaac (GraspGenX + cuRobo, in the graspgenx env) on
# an arm-only G1 whose root is torso_link. What comes back is joint angles over
# time plus the target's pose in that same torso frame. Both transfer: the
# joint names are the 29-dof G1's own names, and the object is re-placed in the
# cell at the same offset from the robot's torso, so the same angles reach it.
#
#   python grasp/traj_from_graspgen.py <trajectory.json> [out_stem]
import json
import os
import sys

import numpy as np

# cspace order written by GraspGenX end2end/build_g1_right_arm.py
ARM = ["right_shoulder_pitch_joint", "right_shoulder_roll_joint",
       "right_shoulder_yaw_joint", "right_elbow_joint",
       "right_wrist_roll_joint", "right_wrist_pitch_joint",
       "right_wrist_yaw_joint"]
HAND = ["right_hand_thumb_0_joint", "right_hand_thumb_1_joint",
        "right_hand_thumb_2_joint", "right_hand_index_0_joint",
        "right_hand_index_1_joint", "right_hand_middle_0_joint",
        "right_hand_middle_1_joint"]

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src = sys.argv[1]
stem = sys.argv[2] if len(sys.argv) > 2 else os.path.join(REPO, "results", "g1_graspgen")

d = json.load(open(src))
frames = d["frames"]
q = np.array([f["joint_position"] for f in frames], dtype=np.float32)
names = ARM + HAND
if q.shape[1] != len(names):
    raise SystemExit(f"{q.shape[1]} columns but {len(names)} joint names")

# torso_link is the plan's root; it is parts[0] and does not move.
torso = np.array(frames[0]["parts"][0]["transform"], dtype=np.float64)
if frames[0]["parts"][0]["name"] != "torso_link":
    raise SystemExit("parts[0] is not torso_link: " + frames[0]["parts"][0]["name"])
# Where the object is depends on the playback mode the plan was replayed in:
# kinematic leaves it in `static`, dynamic moves it every frame so it lives in
# `objects` + `frames[*].object_poses`. Either way we want its mesh and its
# pose at frame 0 -- Isaac re-simulates it from there as a rigid body.
if "object" in d.get("static", {}):
    mesh = d["static"]["object"]["mesh_rel"]
    obj = np.array(d["static"]["object"]["transform"], dtype=np.float64)
else:
    entry = d["objects"][0]
    mesh = entry["mesh_rel"]
    obj = np.array(frames[0]["object_poses"][entry["id"]], dtype=np.float64)
obj_in_torso = np.linalg.inv(torso) @ obj

def _abs(rel):
    return rel if os.path.isabs(rel) else os.path.join(
        d.get("base_dir", os.path.dirname(src)), rel)


# The plan assumed the object rests on something. Carry that support across
# too, or the object is spawned in mid-air in the cell and simply falls.
support = []
for name, item in (d.get("static") or {}).items():
    if name == "object":
        continue
    support.append({
        "name": name,
        "mesh": _abs(item["mesh_rel"]),
        "transform_in_torso": (np.linalg.inv(torso)
                               @ np.array(item["transform"], float)).tolist(),
    })

mesh = _abs(mesh)
meta = {
    "joint_names": names,
    "fps": d["fps"],
    "phases": [f["phase"] for f in frames],
    "object": {"mesh": mesh, "transform_in_torso": obj_in_torso.tolist()},
    "support": support,
}
np.save(stem + ".npy", q)
json.dump(meta, open(stem + ".json", "w"), indent=1)
print(f"[traj] {q.shape[0]} frames x {q.shape[1]} joints @ {d['fps']} fps -> {stem}.npy")
print(f"[traj] object in torso frame: {np.round(obj_in_torso[:3, 3], 4)}")
print(f"[traj] object mesh: {mesh}")
print(f"[traj] support: {[x['name'] for x in support]}")
