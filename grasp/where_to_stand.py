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

    conda activate graspgenx
    python grasp/where_to_stand.py results/far_grasps.json results/stand.json
"""

import json
import math
import sys

import numpy as np
import torch

CUROBO_CFG = ("/home/sehoon/Projects/GraspGenX/end2end/curobo_assets/"
              "g1_base_arm.yml")
TOOL = "right_hand_palm_link"


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


def main():
    src, dst = sys.argv[1], sys.argv[2]
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
    print(f"[stand] {len(tools)} grasps, conf {conf.min():.3f}..{conf.max():.3f}")

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
    pose = GoalToolPose(tool_frames=[TOOL],
                        position=pos.reshape(n, 1, 1, 1, 3),
                        quaternion=quat.reshape(n, 1, 1, 1, 4))
    r = ik.solve_pose(goal_tool_poses=pose)
    ok = r.success.view(-1).cpu().numpy().astype(bool)
    sol = r.solution.view(n, -1, len(names))[:, 0].cpu().numpy()
    err = r.position_error.view(n, -1)[:, 0].cpu().numpy()

    print(f"[stand] reachable with the base free: {int(ok.sum())}/{n}")
    if not ok.any():
        raise SystemExit("[stand] nothing is reachable even with the base free")

    stands = np.stack([sol[:, bx], sol[:, by],
                       np.array([_wrap(v) for v in sol[:, bz]])], axis=1)
    # Of the grasps that can be reached, take the one the grasp model liked
    # best. Ties in confidence are broken by IK error.
    idx = sorted(np.nonzero(ok)[0], key=lambda i: (-conf[i], err[i]))[0]
    sx, sy, syaw = stands[idx]
    print(f"[stand] best grasp #{idx} conf {conf[idx]:.3f} "
          f"palm {np.round(tools[idx][:3, 3], 3)} err {err[idx]*1000:.2f} mm")
    print(f"[stand] STAND AT x={sx:.3f} y={sy:.3f} yaw={math.degrees(syaw):.1f} deg")
    # How far the robot would have to go, and how spread out the answers are --
    # a tight cluster means the choice of grasp barely moves the stand.
    d_reach = stands[ok]
    print(f"[stand] {len(d_reach)} reachable stands span "
          f"x {d_reach[:, 0].min():.2f}..{d_reach[:, 0].max():.2f}  "
          f"y {d_reach[:, 1].min():.2f}..{d_reach[:, 1].max():.2f}")

    json.dump({
        "stand": {"x": float(sx), "y": float(sy),
                  "yaw_deg": float(math.degrees(syaw))},
        "grasp_index": int(idx),
        "confidence": float(conf[idx]),
        "tool_pose": tools[idx].tolist(),
        "reachable": int(ok.sum()),
        "total": int(n),
    }, open(dst, "w"), indent=1)
    print(f"[stand] wrote {dst}")


if __name__ == "__main__":
    main()
