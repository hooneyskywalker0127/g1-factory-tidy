"""Dexonomy's synthesised Dex3 grasps on a tool, brought into the cell.

Dexonomy (assets/object/TOOLS, output/tools_unitree_g1) ships grasps for the
same right Dex3 hand ("unitree_g1") on hammers, pliers, a drill: full palm
pose + 7 finger joints per grasp, by grasp TYPE (1_Large_Diameter = the
wrap a person puts around a handle). GraspGenX has no such taxonomy and no
part conditioning; this is the other open-source answer to "grasp the
handle". The tool is laid on the floor the way every object here is
(longest axis flat, thinnest up), the grasps are carried with it, and the
ones whose palm or contacts would be under the floor are dropped.

    python grasp/dexonomy_grasps.py ddg_gd_hammer_poisson_005 0.14 RUN_DIR [--type 1_Large_Diameter]

Writes RUN_DIR/<obj>_flat.obj, RUN_DIR/scene.json (from results/fable5) and
RUN_DIR/grasps_cell.json: palm poses in the CELL frame (reach_from_pose.py
--cell-frame).
"""
import glob
import json
import os
import sys

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as R

DEX = "/home/sehoon/Projects/Dexonomy"
PLACE = np.array([-0.30, 0.05, 0.162])        # where results/fable5/scene.json puts the object origin (torso @ transform_in_torso)


def tabletop(name, scale, out, exp="floor_tools"):
    """Dexonomy's TABLETOP synthesis (a floor plane in the scene, so no finger
    goes under the object; results in output/<exp>/succ_grasp are the ones
    its MuJoCo evaluation held). The grasps are in Dexonomy's world, where
    the floor is z=0 and the object rests at xy=0 in its stable pose; that
    pose is baked into the mesh here, so in the cell the object only moves
    to PLACE's xy and every grasp moves with it.

        python grasp/dexonomy_grasps.py ddg_gd_hammer_poisson_005 0.14 RUN_DIR --tabletop [--exp floor_tools]
    """
    os.makedirs(out, exist_ok=True)
    files = sorted(glob.glob(f"{DEX}/output/{exp}/succ_grasp/*/{name}/tabletop/*/*_grasp.npy"))
    if not files:
        files = sorted(glob.glob(f"{DEX}/output/{exp}/grasp_data/*/{name}/tabletop/*/*_grasp.npy"))
        print(f"[dex] no succ_grasp for {name}; falling back to {len(files)} unevaluated grasps")
    if not files:
        # Dexonomy's MuJoCo refinement rejected every tabletop init (reason
        # not surfaced by its worker logging); its INIT stage already applied
        # the floor half-space filter (hand skeleton >= 2 cm above the plane)
        # and the grasp-type template, so the inits are used as candidates
        # and the cell's own physics test is the judge, as for GraspGenX.
        files = sorted(glob.glob(f"{DEX}/output/{exp}_unitree_g1/init_data/*/{name}/tabletop/*/*_grasp.npy"))
        print(f"[dex] no refined grasps for {name}; using {len(files)} floor-filtered inits")
    if not files:
        raise SystemExit(f"[dex] nothing synthesised for {name} in output/{exp}")
    # the resting pose with the most candidates
    from collections import Counter
    poses = [tuple(np.round(np.asarray(np.load(f, allow_pickle=True).item()["obj_pose"]).ravel(), 4)) for f in files]
    best_pose = Counter(poses).most_common(1)[0][0]
    files = [f for f, pz in zip(files, poses) if pz == best_pose]
    g0 = np.load(files[0], allow_pickle=True).item()
    pose = np.asarray(g0["obj_pose"]).ravel()                  # x y z qw qx qy qz, world
    Rw = R.from_quat([pose[4], pose[5], pose[6], pose[3]]).as_matrix()
    m = trimesh.load(f"{DEX}/assets/object/TOOLS/processed_data/{name}/mesh/normalized.obj", force="mesh")
    m.apply_scale(scale)
    T = np.eye(4); T[:3, :3] = Rw; T[:3, 3] = pose[:3]
    m.apply_transform(T)
    m.apply_translation([0.0, 0.0, -PLACE[2]])                  # plan_scene puts the mesh origin at PLACE
    mesh_path = os.path.abspath(os.path.join(out, f"{name}_tabletop.obj")); m.export(mesh_path)
    sc = json.load(open("results/fable5/scene.json")); sc["object"]["mesh"] = mesh_path
    json.dump(sc, open(os.path.join(out, "scene.json"), "w"))
    print(f"[dex] {name} x{scale} resting pose baked: bottom z {m.bounds[0, 2] + PLACE[2]:+.3f}, extents {np.round(m.extents, 3)}")
    grasps, conf, names, opens, closes = [], [], [], [], []
    # Dexonomy joint order (dex_3_1_r.xml): thumb_0 thumb_1 thumb_2 middle_0 middle_1 index_0 index_1
    # ours (build_reach_reference): index_0 index_1 middle_0 middle_1 thumb_0 thumb_1 thumb_2
    perm = [5, 6, 3, 4, 0, 1, 2]
    for f in files:
        g = np.load(f, allow_pickle=True).item()
        if not np.allclose(np.asarray(g["obj_pose"]).ravel(), pose, atol=1e-4):
            continue                                            # another resting pose: another object placement
        q = np.asarray(g["grasp_qpos"])[0]
        Rg = R.from_quat([q[4], q[5], q[6], q[3]]).as_matrix()
        G = np.eye(4); G[:3, :3] = Rg; G[:3, 3] = q[:3] + np.array([PLACE[0], PLACE[1], 0.0])
        grasps.append(G.tolist()); conf.append(1.0); names.append("/".join(f.split("/")[-5:]))
        opens.append(np.asarray(g["pregrasp_qpos"])[0][7:][perm].tolist() if "pregrasp_qpos" in g else None)
        closes.append(np.asarray(g["squeeze_qpos"])[0][7:][perm].tolist() if "squeeze_qpos" in g else np.asarray(g["grasp_qpos"])[0][7:][perm].tolist())
    json.dump({"frame": "cell", "tool_frame": "right_hand_palm_link",
               "grasp_to_tool_transform": {"translation": [0, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]},
               "grasps": grasps, "confidence": conf, "files": names, "hand_open": opens, "hand_closed": closes},
              open(os.path.join(out, "grasps_cell.json"), "w"), indent=1)
    print(f"[dex] {len(grasps)} tabletop grasps at this resting pose -> {out}/grasps_cell.json")


def main():
    name, scale, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
    if "--tabletop" in sys.argv:
        tabletop(name, scale, out, sys.argv[sys.argv.index("--exp") + 1] if "--exp" in sys.argv else "floor_tools")
        return
    gtype = sys.argv[sys.argv.index("--type") + 1] if "--type" in sys.argv else "1_Large_Diameter"
    os.makedirs(out, exist_ok=True)
    m = trimesh.load(f"{DEX}/assets/object/TOOLS/processed_data/{name}/mesh/normalized.obj", force="mesh")
    m.apply_scale(scale)
    order = np.argsort(m.extents)
    Rm = np.eye(3)[[order[1], order[2], order[0]]]
    if np.linalg.det(Rm) < 0:
        Rm[0] *= -1
    T = np.eye(4); T[:3, :3] = Rm
    m.apply_transform(T)
    t_flat = np.array([-m.centroid[0], -m.centroid[1], -0.16 - m.bounds[0, 2]])
    m.apply_translation(t_flat)
    mesh_path = os.path.abspath(os.path.join(out, f"{name}_flat.obj"))
    m.export(mesh_path)
    sc = json.load(open("results/fable5/scene.json")); sc["object"]["mesh"] = mesh_path
    json.dump(sc, open(os.path.join(out, "scene.json"), "w"))
    print(f"[dex] {name} x{scale}: lying extents {np.round(m.extents, 3)}")
    grasps, conf, names = [], [], []
    sc_tag = f"scale{int(round(scale * 100)):03d}"
    for f in sorted(glob.glob(f"{DEX}/output/tools_unitree_g1/grasp_data/{gtype}/{name}/floating/{sc_tag}/*_grasp.npy")):
        g = np.load(f, allow_pickle=True).item()
        q = np.asarray(g["grasp_qpos"])[0]
        p_obj, quat = q[:3], q[3:7]                       # MuJoCo order w x y z
        R_obj = R.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_matrix()
        p_cell = Rm @ p_obj + t_flat + PLACE
        R_cell = Rm @ R_obj
        c_cell = (Rm @ np.asarray(g["obj_cpn_w"])[:, :3].T).T + t_flat + PLACE
        low = min(float(c_cell[:, 2].min()), float(p_cell[2]))
        ok = low > 0.0
        print(f"[dex]   {os.path.basename(f):16s} palm z {p_cell[2]:+.3f}, lowest contact z {c_cell[:, 2].min():+.3f} -> {'kept' if ok else 'under the floor'}")
        if not ok:
            continue
        G = np.eye(4); G[:3, :3] = R_cell; G[:3, 3] = p_cell
        grasps.append(G.tolist()); conf.append(1.0); names.append(os.path.basename(f))
    json.dump({"frame": "cell", "tool_frame": "right_hand_palm_link",
               "grasp_to_tool_transform": {"translation": [0, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]},
               "grasps": grasps, "confidence": conf, "files": names, "joints_closed": None},
              open(os.path.join(out, "grasps_cell.json"), "w"), indent=1)
    print(f"[dex] {len(grasps)} grasps above the floor -> {out}/grasps_cell.json")


if __name__ == "__main__":
    main()
