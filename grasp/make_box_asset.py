# Write the floor box into the object format Dexonomy's synthesis reads.
#
# Dexonomy takes objects pre-processed by MeshProcess: a simplified mesh, a
# convex decomposition as both mujoco xml and urdf, an info json carrying the
# object's bounding box and mass, and one scene_cfg naming all of those plus the
# scale to apply. A box is already convex, so its decomposition is one piece and
# the whole thing can be written directly instead of run through MeshProcess.
#
# The mesh is authored at true size and the scene scale left at 1.0, so the
# numbers in cell_layout are the numbers the grasp is synthesised against.
#
#   python grasp/make_box_asset.py [dexonomy_root]
import os
import sys

import numpy as np
import trimesh

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "map"))
ROOT = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    "~/Projects/Dexonomy")

import cell_layout as L  # noqa: E402

NAME = "g1tidy_floorbox"
SET = "G1TIDY"
W, D, H = L.FLOOR_BOX_SIZE
MASS = L.FLOOR_BOX_MASS

obj_root = os.path.join(ROOT, "assets", "object", SET)
proc = os.path.join(obj_root, "processed_data", NAME)
cfg = os.path.join(obj_root, "scene_cfg", NAME, "floating")
for d in [os.path.join(proc, "mesh"), os.path.join(proc, "urdf", "meshes"),
          os.path.join(proc, "info"), cfg,
          os.path.join(obj_root, "valid_split")]:
    os.makedirs(d, exist_ok=True)

box = trimesh.creation.box(extents=(W, D, H))
for name in ["mesh/simplified.obj", "mesh/normalized.obj",
             "urdf/meshes/convex_piece_000.obj"]:
    box.export(os.path.join(proc, name))

with open(os.path.join(proc, "urdf", "coacd.xml"), "w") as f:
    f.write('''<mujoco model="MuJoCo Model">
  <compiler angle="radian" meshdir="."/>

  <asset>
    <mesh name="convex_piece_000.obj" file="meshes/convex_piece_000.obj"/>
  </asset>

  <worldbody>
    <body name="object">
      <geom name="object_visual_0" type="mesh" contype="0" conaffinity="0" density="0" mesh="convex_piece_000.obj"/>
      <geom name="object_collision_0" type="mesh" mesh="convex_piece_000.obj"/>
    </body>
  </worldbody>
</mujoco>
''')

with open(os.path.join(proc, "urdf", "coacd.urdf"), "w") as f:
    f.write('''<robot name="root">
  <link name="link_convex_piece_000.obj">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="meshes/convex_piece_000.obj" scale="1.0 1.0 1.0"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="meshes/convex_piece_000.obj" scale="1.0 1.0 1.0"/>
      </geometry>
    </collision>
  </link>
</robot>
''')

import json  # noqa: E402
with open(os.path.join(proc, "info", "simplified.json"), "w") as f:
    json.dump({"gravity_center": [0.0, 0.0, 0.0],
               "obb": [W, D, H],
               "scale": 1.0,
               "density": MASS / (W * D * H),
               "mass": MASS}, f, indent=1)

rel = f"../../../processed_data/{NAME}"
scene = {"scene": {NAME: {"type": "rigid_object",
                          "file_path": f"{rel}/mesh/simplified.obj",
                          "xml_path": f"{rel}/urdf/coacd.xml",
                          "urdf_path": f"{rel}/urdf/coacd.urdf",
                          "info_path": f"{rel}/info/simplified.json",
                          "scale": np.array([1.0, 1.0, 1.0]),
                          "pose": np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])}},
         "scene_id": f"{NAME}/floating/scale010",
         "task": {"type": "force_closure", "obj_name": NAME}}
np.save(os.path.join(cfg, "scale010.npy"), scene, allow_pickle=True)

with open(os.path.join(obj_root, "valid_split", "all.json"), "w") as f:
    json.dump([NAME], f)

print(f"[asset] {NAME}: {W*1000:.0f} x {D*1000:.0f} x {H*1000:.0f} mm, "
      f"{MASS} kg, density {MASS / (W * D * H):.1f}")
print(f"[asset] wrote {obj_root}")
