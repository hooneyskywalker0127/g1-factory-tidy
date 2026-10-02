"""The floor crate as GraspGenX sees it without a camera: its mesh surface sampled at the crate's pose.

    python crate_mesh_scene_wb.py CRATE.obj X Y Z OUT_DIR [N_POINTS]

GraspGenX samples the known mesh when there is no observation (end2end/e2e_grasp_demo.py:694-705,
trimesh.sample.sample_surface, num_sample_points 2000). For the 0.6 m crate 2000 points are too sparse for
GraspMoE's outlier removal (k 20 within the threshold: all 2000 removed), so N_POINTS is passed (crate: 20000; the
260928 camera look saw the crate in 8344 px). The crate is written as a GraspGenX JSON point-cloud
scene (graspgenx/utils/scene_loaders.py:128 load_graspgenx_json_scene) with the floor as the rest of the scene,
so the e2e demo's own collision filter (filter_colliding_grasps against the scene) drops grasps through the floor.
The camera clouds of 260928 (gg1-gg3) saw the honeycomb walls through and every pair closed on air.
"""
import json, os, sys
import numpy as np, trimesh

mesh, x, y, z, out = sys.argv[1], *map(float, sys.argv[2:5]), sys.argv[5]
m = trimesh.load(mesh, force="mesh")
pc, _ = trimesh.sample.sample_surface(m, int(sys.argv[6]) if len(sys.argv) > 6 else 2000, seed=0)
pc = pc + [x, y, z]
g = np.arange(-0.7, 0.7, 0.01)
fx, fy = np.meshgrid(x + g, y + g)
floor = np.stack([fx.ravel(), fy.ravel(), np.zeros(fx.size)], 1)
os.makedirs(out, exist_ok=True)
json.dump({"object_info": {"pc": pc.tolist(), "pc_color": np.full((len(pc), 3), 200).tolist()},
           "scene_info": {"full_pc": [floor.tolist()], "img_color": np.full((len(floor), 3), 128).tolist()}},
          open(os.path.join(out, "crate_scene.json"), "w"))
print(f"[crate] {len(pc)} crate points (bounds {pc.min(0).round(3)} {pc.max(0).round(3)}), {len(floor)} floor points -> {out}")
