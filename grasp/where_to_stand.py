"""Where does the robot have to stand for a grasp to be in reach?

This is the step between seeing a thing and walking to it. GraspGenX gives a
set of grasps in the cell's own floor frame (grasp/run_far_pick.sh exports them
with e2e_grasp_demo.py --export-grasps); cuRobo answers, for each one, whether
a standing G1 can put its right palm there and if so where its pelvis has to be
to do it.

The answering is done by cuRobo's own IK on curobo_assets/g1_base_arm.yml,
built by GraspGenX's build_g1_right_arm.py --floating-base: the right-arm chain
carved out of the G1 the way the arm-only config already is, but rooted one
link lower at the pelvis and with cuRobo's floating base (base_j_x, base_j_y,
base_j_ztheta) underneath. Solving that config returns the base position as
the first three joint values, in metres and radians, in the cell frame -- which
is a place to stand.

Two things this is NOT:

  - It is not motion planning. Asking plan_grasp for a trajectory with the base
    free makes cuRobo try to slide the pelvis across the floor at its 1 m/s
    joint limit, and it returns None. Walking is GR00T's job, not the arm
    planner's; this only has to say WHERE.

  - It is not a reachability heuristic of ours. The arm envelope is whatever
    the URDF and the solver say it is.

The grasps arrive in the planner's world -- the capture's plan_from_cell and
then the robot YAML's robot_base_pose, which is what e2e_grasp_demo.py applies
to them. Both are undone here, because the config this solves is rooted at
base_link on the CELL floor: a base solution then reads straight off as a place
to stand in the cell, which is what the walk takes.

    conda activate graspgenx
    python grasp/where_to_stand.py GRASPS.json CAPTURE_DIR OUT.json [--base-z Z]
"""

import json
import math
import os
import sys

import numpy as np
import torch

CUROBO_CFG = os.environ.get("STAND_CFG") or (
    "/home/sehoon/Projects/GraspGenX/end2end/curobo_assets/g1_base_arm.yml")
# The stand was being chosen with a robot that has no legs.
#
# g1_base_arm.yml carries collision spheres for 22 links -- pelvis, torso,
# head, the right arm and its fingers -- and none for the hips, knees or
# ankles. So the clearance test could not see a leg, and it passed a stand
# whose pelvis sits 59 mm from the desk. Measured by forward kinematics at the
# pose the walk actually ends in, both ankles and both knees were then 110 to
# 160 mm INSIDE the desk, whose collision shape is a solid block from the
# floor to its top. The contact solver spends every step pushing them out, at
# the joint velocity limit: 19.9 rad/s with the legs pinned, 26.1 rad/s with
# GR00T's balance policy driving them, which is the shaking.
#
# cuRobo ships the whole robot: 35 links, 400 spheres, twelve of them on the
# legs. Use it for the clearance test.
FULL_CFG = ("/home/sehoon/Projects/g1-dex3-tabletop/third_party/curobo/curobo/"
            "content/configs/robot/unitree_g1_29dof_retarget.yml")
TOOL = "right_hand_palm_link"


def whole_body_clear(stands, arm_sol, arm_names, obstacles, stance_json,
                     pelvis_z):
    """Which of these stands can the WHOLE robot occupy, legs included.

    The stand is where the robot will be standing when it reaches, and it will
    be standing in the pose the walk leaves it in -- hips back about 30
    degrees, knees bent, feet apart. That pose puts the feet a long way from
    the pelvis, which is why a clearance test run on a legless model passes
    stands the robot cannot physically be in.

    Build the real configuration: the base at the candidate stand, the legs at
    the stance the walk ends in, the arm at the solution that reaches the
    grasp. Then ask cuRobo, with the desk it is already given.
    """
    import json as _json
    import yaml as _yaml
    from curobo.collision_checking import (RobotCollisionChecker,
                                           RobotCollisionCheckerCfg)
    # The retarget config's top level is "kinematics"; load_from_config wants
    # it under "robot_cfg", the way the arm config ships it.
    _full = _yaml.safe_load(open(FULL_CFG))
    if "robot_cfg" not in _full:
        _full = {"robot_cfg": _full}
    names = _full["robot_cfg"]["kinematics"]["cspace"]["joint_names"]
    stance = _json.load(open(stance_json))
    q = np.zeros((len(stands), len(names)), np.float32)
    for j, nm in enumerate(names):
        if nm in stance:
            q[:, j] = float(stance[nm])
    ai = {nm: k for k, nm in enumerate(arm_names)}
    for j, nm in enumerate(names):
        if nm in ai:
            q[:, j] = arm_sol[:, ai[nm]]
    q[:, names.index("base_j_x")] = stands[:, 0]
    q[:, names.index("base_j_y")] = stands[:, 1]
    q[:, names.index("base_j_z")] = pelvis_z
    q[:, names.index("base_j_ztheta")] = stands[:, 2]
    chk = RobotCollisionChecker(RobotCollisionCheckerCfg.load_from_config(
        robot_config=_full, scene_model={"cuboid": obstacles},
        n_cuboids=max(4, len(obstacles))))
    ok = chk.validate(torch.tensor(q, dtype=torch.float32,
                                   device="cuda").unsqueeze(1))
    return ok.view(-1).cpu().numpy().astype(bool)


def _xyzw_to_matrix(t, q):
    """Same convention e2e_grasp_demo.py uses for grasp_to_tool_transform."""
    import trimesh.transformations as tra

    T = np.eye(4)
    T[:3, 3] = t
    if not (abs(q[0]) < 1e-9 and abs(q[1]) < 1e-9
            and abs(q[2]) < 1e-9 and abs(q[3] - 1) < 1e-9):
        T[:3, :3] = tra.quaternion_matrix([q[3], q[0], q[1], q[2]])[:3, :3]
    return T


def _wrap(a):
    """Into (-pi, pi]. base_j_ztheta's limits are +-16 rad, so the solver is
    free to hand back a heading that has gone round several times."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# What the pick is built around: the object sits this far in front of the
# reaching pose. capture_rgbd.py uses the same number to decide how far back
# to shoot from, since it is inside the depth camera's 0.40 m blind zone.
PLAN_REACH = 0.35


def _desk_from_grasps(tools):
    """The desk, positioned from what the camera saw rather than an assumed stand.

    The box rests on it, so the desk is directly under the grasps: their own
    spread gives the xy and the mesh the plan used gives the extents. Carrying
    the plan's support transform across instead ties the desk to whichever
    stand you assume, which put it 0.23 m out as soon as vision picked a
    different one.
    """
    import trimesh
    plan_meta = json.load(open(sys.argv[sys.argv.index("--path") + 1]))
    obstacles = {}
    ctr_xy = tools[:, :3, 3].mean(axis=0)[:2]
    for sup in plan_meta.get("support", []):
        mesh = trimesh.load(sup["mesh"], force="mesh")
        ext = np.asarray(mesh.extents, dtype=np.float64)
        top = float(tools[:, 2, 3].min()) - 0.10
        ctr = np.array([ctr_xy[0], ctr_xy[1], top - ext[2] / 2.0])
        obstacles[sup["name"]] = {
            "dims": [float(v) for v in ext],
            "pose": [float(ctr[0]), float(ctr[1]), float(ctr[2]),
                     1.0, 0.0, 0.0, 0.0],
        }
        print(f"[stand] obstacle '{sup['name']}' {np.round(ext, 3)} "
              f"at {np.round(ctr, 3)} (top {top:.3f})")
    return obstacles, plan_meta


def main():
    src, cap, dst = sys.argv[1], sys.argv[2], sys.argv[3]
    base_z = (float(sys.argv[sys.argv.index("--base-z") + 1])
              if "--base-z" in sys.argv else 0.98)
    d = json.load(open(src))
    grasps = np.asarray(d["grasps"], dtype=np.float64)
    conf = np.asarray(d["confidence"], dtype=np.float64)
    g2t = d.get("grasp_to_tool_transform") or {}
    T_offset = _xyzw_to_matrix(g2t.get("translation", [0, 0, 0]),
                               g2t.get("quaternion_xyzw", [0, 0, 0, 1]))
    # e2e_grasp_demo.py: T_target = inv(robot_base_T) @ grasp @ T_offset, and
    # this config's robot_base_T is identity, so the palm goal is the grasp
    # with the gripper-convention offset on it and nothing else.
    tools = np.array([g @ T_offset for g in grasps])

    # planner world -> cell. e2e_grasp_demo.py moved the capture across with
    # robot_base_T @ plan_from_cell; undoing both puts the grasps back where
    # the camera found them, which is the frame the floating base is rooted in.
    meta = json.load(open(os.path.join(cap, "meta_data.json")))
    pfc = np.asarray(meta.get("plan_from_cell") or np.eye(4), dtype=np.float64)
    base_T = np.eye(4)
    base_T[2, 3] = base_z
    to_cell = np.linalg.inv(pfc) @ np.linalg.inv(base_T)
    tools = np.array([to_cell @ T for T in tools])
    print(f"[stand] {len(tools)} grasps, conf {conf.min():.3f}..{conf.max():.3f}")
    print(f"[stand] in cell coords: palm z {tools[:, 2, 3].min():.3f}"
          f"..{tools[:, 2, 3].max():.3f}")

    from curobo.inverse_kinematics import (InverseKinematics,
                                           InverseKinematicsCfg)
    from curobo._src.types.tool_pose import GoalToolPose
    import trimesh.transformations as tra

    # One IK problem per grasp, solved in one batch: cuRobo sizes its buffers
    # at construction, so it has to be told how many are coming.
    ik = InverseKinematics(InverseKinematicsCfg.create(
        robot=CUROBO_CFG, num_seeds=64, max_batch_size=len(tools)))
    names = ik.kinematics.joint_names
    bx, by, bz = (names.index("base_j_x"), names.index("base_j_y"),
                  names.index("base_j_ztheta"))

    pos = torch.tensor(tools[:, :3, 3], dtype=torch.float32, device="cuda")
    quat = torch.tensor(
        np.array([tra.quaternion_from_matrix(T) for T in tools]),
        dtype=torch.float32, device="cuda")          # wxyz, as cuRobo wants

    n = len(tools)
    # The floating-base build declares a second tool frame, torso_link, so the
    # body can be given an opinion of its own (cuRobo takes one goal and one
    # ToolPoseCriteria per frame). Here it is given no opinion at all: a
    # placeholder goal with disabled criteria, so it constrains nothing.
    #
    # Which way the body ends up facing is then read off the answers rather
    # than asked for. It has to be: "stand so the object is in front of you"
    # depends on where you end up standing, so asking for a heading and
    # re-asking from the answer chases its own tail -- measured, 75 -> 57 ->
    # 107 degrees off instead of settling. Every solution here already reaches
    # the grasp, so scoring them costs nothing and cannot fail.
    frames = list(ik.kinematics.tool_frames)
    pos_f = pos.reshape(n, 1, 1, 1, 3)
    quat_f = quat.reshape(n, 1, 1, 1, 4)
    if len(frames) > 1:
        from curobo._src.cost.tool_pose_criteria import ToolPoseCriteria
        ik.update_tool_pose_criteria(
            {f: (ToolPoseCriteria() if f == TOOL else ToolPoseCriteria.disabled())
             for f in frames})
        pos_f = pos_f.repeat(1, 1, len(frames), 1, 1)
        quat_f = quat_f.repeat(1, 1, len(frames), 1, 1)
    pose = GoalToolPose(tool_frames=frames, position=pos_f.contiguous(),
                        quaternion=quat_f.contiguous())
    # Every seed that converged is a place to stand, not only the cheapest
    # one per grasp. With the base free the solver has no opinion about
    # heading, so the one solution it ranks first per grasp faces wherever
    # its seed happened to start -- measured, the nearest any of 80 such
    # stands came to facing the object was 20.7 degrees, and plan_grasp from
    # there got the palm to the box and could not close on it. Asking for
    # SEEDS solutions per grasp (solve_pose's own return_seeds) gives the
    # ranking below real choices; all of them reach the grasp.
    SEEDS = 16
    r = ik.solve_pose(goal_tool_poses=pose, return_seeds=SEEDS)
    ok = r.success.view(-1).cpu().numpy().astype(bool)
    sol = r.solution.view(n * SEEDS, len(names)).cpu().numpy()
    err = r.position_error.view(-1).cpu().numpy()
    grasp_of = np.repeat(np.arange(n), SEEDS)      # candidate -> grasp
    conf = conf[grasp_of]
    tools = tools[grasp_of]
    n = n * SEEDS

    print(f"[stand] reachable with the base free: {int(ok.sum())}/{n} "
          f"stands ({len(set(grasp_of[ok]))} of {n // SEEDS} grasps)")
    if "base_j_z" in names and ok.any():
        _dz = sol[ok, names.index("base_j_z")]
        print(f"[stand] pelvis height across them: {0.75 + _dz.min():.3f}"
              f"..{0.75 + _dz.max():.3f} m (URDF stands at 0.750)")
        _wp = names.index("waist_pitch_joint")
        print(f"[stand] waist pitch across them: {sol[ok, _wp].min():+.2f}"
              f"..{sol[ok, _wp].max():+.2f} rad")

    # Does it have to walk at all? Ask, rather than assume.
    #
    # The pipeline has always planned a stand and then walked to it, which
    # gets the right answer without ever testing the question a person asks
    # first: can I reach this from where I am? Pin the base where the robot
    # actually is and solve the same grasps again -- same solver, same
    # collision world, the only difference being that the base cannot move.
    n_grasps = n // SEEDS
    if "--from" in sys.argv:
        _f = sys.argv[sys.argv.index("--from") + 1:][:3]
        _sx, _sy, _syaw = float(_f[0]), float(_f[1]), math.radians(float(_f[2]))
        _seed = ik.kinematics.default_joint_position.view(1, -1).clone()
        _seed[0, bx], _seed[0, by], _seed[0, bz] = _sx, _sy, _syaw
        # Two 80-problem, 64-seed solvers do not fit next to each other on a
        # shared GPU -- measured, the second one asked for 556 MiB with 446
        # free and the whole stand step died. Everything the first one was
        # for is already in numpy; let it go before building the second.
        del ik, r
        import gc as _gc
        _gc.collect()
        torch.cuda.empty_cache()
        import yaml as _yaml
        _rc = _yaml.safe_load(open(CUROBO_CFG))["robot_cfg"]
        _rc["kinematics"].setdefault("asset_root_path",
                                     os.path.dirname(CUROBO_CFG))
        _lk = dict(_rc["kinematics"].get("lock_joints") or {})
        _lk.update({"base_j_x": _sx, "base_j_y": _sy, "base_j_ztheta": _syaw})
        if "base_j_z" in names:
            _lk["base_j_z"] = 0.0
        _rc["kinematics"]["lock_joints"] = _lk
        _here = InverseKinematics(InverseKinematicsCfg.create(
            robot=_rc, num_seeds=64, max_batch_size=n_grasps))
        _hf = list(_here.kinematics.tool_frames)
        _hp = pos.reshape(n_grasps, 1, 1, 1, 3)
        _hq = quat.reshape(n_grasps, 1, 1, 1, 4)
        if len(_hf) > 1:
            from curobo._src.cost.tool_pose_criteria import ToolPoseCriteria
            _here.update_tool_pose_criteria(
                {f: (ToolPoseCriteria() if f == TOOL else ToolPoseCriteria.disabled())
                 for f in _hf})
            _hp = _hp.repeat(1, 1, len(_hf), 1, 1)
            _hq = _hq.repeat(1, 1, len(_hf), 1, 1)
        _r = _here.solve_pose(goal_tool_poses=GoalToolPose(
            tool_frames=_hf, position=_hp.contiguous(), quaternion=_hq.contiguous()))
        _n_here = int(_r.success.view(-1).sum())
        del _here, _r
        _gc.collect()
        torch.cuda.empty_cache()
        print(f"[stand] from where it is now ({_sx:.2f}, {_sy:.2f}): "
              f"{_n_here}/{n_grasps} grasps reachable")
        if _n_here == 0:
            print("[stand] nothing is in reach from here -- it has to walk")
        else:
            print(f"[stand] {_n_here} already in reach; walking is not required")
    if not ok.any():
        # Nothing converged. With --nearest, take the stands whose best seed
        # came closest and say how close: the body can still be put there and
        # the rest of the reach found with the legs (GR00T's own kneel and
        # squat modes), which this arm-and-waist model does not have.
        if "--nearest" not in sys.argv:
            raise SystemExit("[stand] nothing is reachable even with the base free")
        _cut = np.percentile(err, 10)
        ok = err <= _cut
        print(f"[stand] nothing converged; --nearest keeps the {int(ok.sum())} "
              f"stands within {_cut*1000:.1f} mm of a grasp "
              f"(best {err.min()*1000:.1f} mm)")

    stands = np.stack([sol[:, bx], sol[:, by],
                       np.array([_wrap(v) for v in sol[:, bz]])], axis=1)
    # Of the places the arm can reach from, stand where the object is in front
    # of you. Every one of these is reachable -- 80 of 80 on this scene -- so
    # the choice costs nothing in reach, and it decides whether the arm works
    # across the body or straight ahead. The bearing is measured from each
    # candidate stand to the grasps the camera produced, so nothing is written
    # down here either.
    # A stand the arm can reach from is not yet a stand the body can occupy.
    # cuRobo's own reachable set includes places inside the desk -- measured on
    # one run, the best-scoring stand sat 131 mm inside its footprint, and the
    # route planner then correctly refused to walk there and the walk fell back
    # to going straight. So ask the collision checker which of these poses the
    # robot can actually be in, and choose among those.
    #
    # RobotCollisionChecker is cuRobo's public API for exactly this ("use this
    # module when building custom collision-aware pipelines outside the main
    # solvers"), and it is given the same desk the route planner is given.
    obstacles, plan_meta, stand_deg = None, None, None
    if "--path" in sys.argv:
        obstacles, plan_meta = _desk_from_grasps(tools)
        stand_deg = sys.argv[sys.argv.index("--from") + 1:][:3] \
            if "--from" in sys.argv else None
        from curobo.collision_checking import (RobotCollisionChecker,
                                               RobotCollisionCheckerCfg)
        chk = RobotCollisionChecker(RobotCollisionCheckerCfg.load_from_config(
            robot_config=CUROBO_CFG, scene_model={"cuboid": obstacles},
            n_cuboids=max(4, len(obstacles))))
        free = chk.validate(torch.tensor(sol, dtype=torch.float32,
                                         device="cuda").unsqueeze(1))
        free = free.view(-1).cpu().numpy().astype(bool)
        print(f"[stand] {int((ok & free).sum())}/{int(ok.sum())} reachable "
              f"stands are also clear of the scene (arm and pelvis only)")
        # And now with the legs. --stance is the walk's own _end.json; the
        # robot will be standing in that pose when it reaches, and it is the
        # pose that puts the feet out where the desk is.
        if "--stance" in sys.argv:
            _sj = sys.argv[sys.argv.index("--stance") + 1]
            _pz = float(sys.argv[sys.argv.index("--pelvis-z") + 1]) \
                if "--pelvis-z" in sys.argv else 0.7622
            _st = np.stack([sol[:, bx], sol[:, by],
                            np.array([_wrap(v) for v in sol[:, bz]])], axis=1)
            whole = whole_body_clear(_st, sol, names, obstacles, _sj, _pz)
            print(f"[stand] {int((ok & free & whole).sum())}/"
                  f"{int((ok & free).sum())} of those also clear with the "
                  f"legs in the stance the walk ends in")
            free = free & whole
        if (ok & free).any():
            ok = ok & free
        else:
            print("[stand] none of them is clear; keeping the reachable set")

    centre = tools[:, :3, 3].mean(axis=0)
    bearing = np.arctan2(centre[1] - stands[:, 1], centre[0] - stands[:, 0])
    off = np.abs(np.arctan2(np.sin(bearing - stands[:, 2]),
                            np.cos(bearing - stands[:, 2])))
    # Bucket the angle so near-equal headings are separated by confidence
    # rather than by a fraction of a degree.
    # ...and among those, stand near enough that the arm is not at full
    # stretch. Dropping the stands inside the desk also pushes the survivors
    # outward, and the first run after that filter chose one 0.473 m from the
    # object against 0.351 m for the stand that works: cuRobo's IK still
    # reaches the grasp from there, but plan_grasp falls back to "approach
    # only" -- it can get the palm to the box and cannot then close and lift.
    # Reach is what the arm has left over, so it belongs in the ranking next
    # to facing. The distance is to the grasps the camera produced, bucketed
    # at 5 cm so it separates stands rather than sorting on millimetres.
    # The distance to aim for is not ours either: the pick is built around the
    # object sitting PLAN_REACH in front of the reaching pose -- the same
    # number capture_rgbd.py shoots from behind, because it is inside the
    # depth camera's blind zone. Rank by how near a stand comes to that, then
    # by facing. Measured the other way round, facing first, the winner was
    # 0.425 m out and plan_grasp could only manage "approach only".
    reach = np.linalg.norm(stands[:, :2] - centre[:2], axis=1)
    # Ranking used to prefer stands PLAN_REACH (0.35 m) from the grasps. That
    # is a number typed here, and it is the one that put the pelvis 59 mm from
    # the desk with the legs inside it. What decides a stand is whether the
    # robot can be there and reach from there, both of which are already
    # measured: the clearance filter above and cuRobo's own IK error. Rank on
    # those and on facing the object, and let distance fall where it falls.
    idx = sorted(np.nonzero(ok)[0],
                 key=lambda i: (round(math.degrees(off[i]) / 10.0),
                                -conf[i], err[i]))[0]
    print(f"[stand] chosen stand is {reach[idx]:.3f} m from the grasps; "
          f"the clear ones span {reach[ok].min():.3f}..{reach[ok].max():.3f} m")
    print(f"[stand] object sits {math.degrees(off[ok].min()):.1f}"
          f"..{math.degrees(off[ok].max()):.1f} deg off the body's front "
          f"across the reachable stands; taking "
          f"{math.degrees(off[idx]):.1f} deg")
    sx, sy, syaw = stands[idx]
    print(f"[stand] best grasp #{grasp_of[idx]} conf {conf[idx]:.3f} "
          f"palm {np.round(tools[idx][:3, 3], 3)} err {err[idx]*1000:.2f} mm")
    print(f"[stand] STAND AT x={sx:.3f} y={sy:.3f} yaw={math.degrees(syaw):.1f} deg")
    # How far the robot would have to go, and how spread out the answers are --
    # a tight cluster means the choice of grasp barely moves the stand.
    d_reach = stands[ok]
    print(f"[stand] {len(d_reach)} reachable stands span "
          f"x {d_reach[:, 0].min():.2f}..{d_reach[:, 0].max():.2f}  "
          f"y {d_reach[:, 1].min():.2f}..{d_reach[:, 1].max():.2f}")

    base_path = None
    if "--path" in sys.argv:
        # And how to get there without walking through the desk.
        #
        # Handed only a destination the walk goes straight at it, and on this
        # cell that line clips the desk -- measured, 23 of the 88 moving
        # frames had the robot's body inside its footprint, closest 0.086 m.
        # cuRobo already knows where the desk is; plan_cspace with the graph
        # planner behind it answers with a route around, and walk_clip.py
        # feeds those waypoints to the planner as specific targets.
        #
        # The desk comes from the plan the pick was made against -- its mesh
        # for the extents, its transform for the pose -- so nothing about the
        # scene is described twice.
        PATH_ATTEMPTS = int(sys.argv[sys.argv.index("--path-attempts") + 1]) \
            if "--path-attempts" in sys.argv else 5
        from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
        from curobo.types import JointState
        # Where the desk is, from what the camera saw rather than from any
        # assumed stand. The box rests on it, so the desk is directly under
        # the grasps: their own spread gives the xy, and the mesh the plan
        # used gives the extents. Carrying the plan's support transform
        # across instead ties the desk to whichever stand you assume and puts
        # it 0.23 m out as soon as vision picks a different one.
        mp = MotionPlanner(MotionPlannerCfg.create(
            robot=CUROBO_CFG, scene_model={"cuboid": obstacles},
            collision_cache={"cuboid": max(4, len(obstacles))}))
        # The motion planner orders its joints its own way; the IK solver's
        # indices do not carry over. Reading the base columns with the wrong
        # ones returns arm angles dressed up as a route -- it ended 2.2 m from
        # the goal facing the wrong way, with 2 m jumps between waypoints.
        mnames = mp.joint_names
        mbx, mby, mbz = (mnames.index("base_j_x"), mnames.index("base_j_y"),
                         mnames.index("base_j_ztheta"))
        q_home = list(mp.default_joint_state.position.view(-1).cpu().numpy())
        start_q = list(q_home)
        if stand_deg:
            start_q[mbx], start_q[mby] = float(stand_deg[0]), float(stand_deg[1])
            start_q[mbz] = math.radians(float(stand_deg[2]))
        goal_q = list(q_home)
        goal_q[mbx], goal_q[mby], goal_q[mbz] = float(sx), float(sy), float(syaw)
        res = mp.plan_cspace(
            JointState.from_position(
                torch.tensor([goal_q], dtype=torch.float32, device="cuda"),
                joint_names=mp.joint_names),
            JointState.from_position(
                torch.tensor([start_q], dtype=torch.float32, device="cuda"),
                joint_names=mp.joint_names),
            max_attempts=PATH_ATTEMPTS)
        if res is not None and bool(res.success.any()):
            # The interpolated plan comes back as [1, 1, horizon, dof] and
            # its dof is the config's FULL cspace -- 20 here, the ten free
            # joints and the ten the builder locked -- not the ten
            # planner.joint_names lists. Reshaping to len(joint_names)
            # interleaves the two halves, and the "route" that falls out
            # alternates between two poses and never reaches the goal.
            q = res.get_interpolated_plan().position
            q = q.reshape(-1, q.shape[-1])
            base_path = q[:, [mbx, mby, mbz]].cpu().numpy()
            base_path[:, 2] = [_wrap(float(v)) for v in base_path[:, 2]]
            print(f"[stand] collision-free base path: {len(base_path)} waypoints, "
                  f"{np.abs(np.diff(base_path[:, :2], axis=0)).sum():.2f} m travelled")
        else:
            why = "plan_cspace returned None" if res is None else str(
                getattr(res, "status", "no status"))
            print(f"[stand] no collision-free base path ({why}); the walk will "
                  f"have to steer at the stand directly")

    # Every stand that survived the filters, in ranked order. One stand is a
    # guess: the arm reaches the grasp from it -- cuRobo's IK said 0.00 mm --
    # and plan_grasp still returned None, because reaching a pose and planning
    # approach, close and lift through it are different questions. Only
    # plan_grasp answers the second one, so hand it candidates until one works
    # rather than betting the run on the top of a ranking.
    _ranked = sorted(np.nonzero(ok)[0],
                     key=lambda i: (round(math.degrees(off[i]) / 10.0),
                                    -conf[i], err[i]))
    _cands = [{"x": float(stands[i][0]), "y": float(stands[i][1]),
               "yaw_deg": float(math.degrees(_wrap(stands[i][2]))),
               "off_deg": float(math.degrees(off[i])),
               "reach_m": float(reach[i]), "conf": float(conf[i]),
               "ik_err_mm": float(err[i] * 1000.0)}
              for i in _ranked[:24]]
    json.dump({
        "stand": {"x": float(sx), "y": float(sy),
                  "yaw_deg": float(math.degrees(syaw))},
        "candidates": _cands,
        "base_path": (base_path.tolist() if base_path is not None else None),
        "grasp_index": int(grasp_of[idx]),
        "confidence": float(conf[idx]),
        "tool_pose": tools[idx].tolist(),
        "reachable": int(len(set(grasp_of[ok]))),
        "total": int(n // SEEDS),
    }, open(dst, "w"), indent=1)
    print(f"[stand] wrote {dst}")


if __name__ == "__main__":
    main()
