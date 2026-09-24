"""One whole-body solve for the reach, with cuRobo's own retargeter.

The pick has been planned in the arm's joint space and executed on a robot
whose pelvis was welded to the world, because an arm swinging on a free-
standing G1 falls over. That weld is what makes the walk and the pick two
different things, and the join between them is where the leap lives.

cuRobo ships the alternative, and this project had not read it:
MotionRetargeter (examples/getting_started/humanoid_retargeting.py) solves for
a whole body that tracks per-link pose targets, with

  - a floating base through ``extra_links`` -- six virtual joints between
    base_link and pelvis, the same mechanism our base_j_* locks already use,
  - per-link weights through ``ToolPoseCriteria``; the guide's own advice is
    "feet and hips get high weight to maintain balance, while mid-chain links
    like shoulders get low weight",
  - a velocity-limited warm start from the previous frame, which exists to
    stop the solution jumping to a distant answer between frames,
  - and 35 DOF out: [x, y, z, roll, pitch, yaw] then the 29 body joints.

So the feet and pelvis can be told to hold the stance the walk ended in while
the wrist follows the grasp, solved as one sequence, with no weld and nothing
to join afterwards.

Stage 1 (--check) is the harness only: take the pose the walk ends in, ask for
exactly the link poses that pose produces, and see whether the solver gives it
back. If it cannot reproduce a pose it was handed, nothing built on top of it
is worth rendering.
"""
import json
import math
import sys

import numpy as np
import torch

from curobo.motion_retargeter import (MotionRetargeter, MotionRetargeterCfg,
                                      SequenceGoalToolPose)
from curobo.types import JointState, ToolPoseCriteria

# Feet and pelvis hold the stance; the wrist does the work. The retarget
# config is built on g1_29dof and has no hands, so its right-arm tool frame is
# the wrist, not the palm.
# The solver wants a target for every tool frame the robot config declares,
# so all fourteen are supplied and the weights decide what matters. The
# guide's own advice: "feet and hips get high weight to maintain balance,
# while mid-chain links like shoulders get low weight since the elbow and
# wrist targets already constrain the arm."
# torso_link is held too, and not for balance: traj_from_graspgen.py records
# that "torso_link is the plan's root; it is parts[0] and does not move", so
# every angle in the grasp plan is measured from a torso that stays put. Let
# the waist drift and the plan's own numbers stop meaning what they meant.
HOLD = ["pelvis", "torso_link", "left_hip_roll_link", "right_hip_roll_link",
        "left_ankle_roll_link", "right_ankle_roll_link",
        "left_knee_link", "right_knee_link"]
WORK = ["right_wrist_yaw_link"]
LOOSE = ["left_shoulder_roll_link", "left_elbow_link",
         "left_wrist_yaw_link", "right_shoulder_roll_link",
         "right_elbow_link"]
STAND_Z = 0.7503                      # the height the walk ends at


def build(self_collision=False, use_mpc=False):
    crit = {}
    for n in HOLD + WORK:
        crit[n] = ToolPoseCriteria.track_position_and_orientation(
            xyz=[1.0, 1.0, 1.0], rpy=[0.067, 0.067, 0.067])
    for n in LOOSE:
        crit[n] = ToolPoseCriteria.track_position_and_orientation(
            xyz=[0.1, 0.1, 0.1], rpy=[0.007, 0.007, 0.007])
    cfg = MotionRetargeterCfg.create(
        robot="unitree_g1_29dof_retarget.yml",
        tool_pose_criteria=crit, num_envs=1,
        self_collision_check=self_collision, use_mpc=use_mpc)
    return MotionRetargeter(cfg), cfg


def walk_end_state(jn):
    """The 35-DOF configuration the walk actually finishes in."""
    end = json.load(open("results/motion/vz_end.json"))
    st = json.load(open("results/stand_walked.json"))["stand"]
    q = torch.zeros(1, len(jn), dtype=torch.float32, device="cuda")
    for i, n in enumerate(jn):
        if n in end:
            q[0, i] = float(end[n])
    for n, v in (("base_j_x", st["x"]), ("base_j_y", st["y"]),
                 ("base_j_z", STAND_Z),
                 ("base_j_ztheta", math.radians(st["yaw_deg"]))):
        q[0, jn.index(n)] = v
    return q


def plan_wrist_in_cell(torso_cell_pos, torso_cell_quat):
    """The grasp plan's wrist pose per frame, placed in the cell.

    The plan is solved for an arm-only G1 whose root is torso_link, and the
    exported trajectory gives every link's transform in that same frame. The
    cell's torso pose comes from the walk's own end configuration, so the two
    meet at the torso and the wrist target lands where the plan meant it.
    """
    d = json.load(open("results/run_vz/trajectory.json"))
    frames = d["frames"]
    names = [p["name"] for p in frames[0]["parts"]]
    ti, wi = names.index("torso_link"), names.index("right_wrist_yaw_link")
    torso_plan = np.array(frames[0]["parts"][ti]["transform"], float)
    inv_plan = np.linalg.inv(torso_plan)

    T = np.eye(4)
    T[:3, :3] = quat_to_mat(torso_cell_quat)
    T[:3, 3] = torso_cell_pos
    out = np.zeros((len(frames), 4, 4))
    for i, f in enumerate(frames):
        out[i] = T @ inv_plan @ np.array(f["parts"][wi]["transform"], float)
    return out


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(m):
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([0.25 * s, (m[2, 1] - m[1, 2]) / s,
                         (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s])
    i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
    if i == 0:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return np.array([(m[2, 1] - m[1, 2]) / s, 0.25 * s,
                         (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s])
    if i == 1:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return np.array([(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s,
                         0.25 * s, (m[1, 2] + m[2, 1]) / s])
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return np.array([(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s,
                     (m[1, 2] + m[2, 1]) / s, 0.25 * s])


def main():
    r, cfg = build()
    jn = r.kinematics.joint_names
    q = walk_end_state(jn)
    ks = r.kinematics.compute_kinematics(JointState.from_position(q, joint_names=jn))
    all_frames = list(ks.tool_frames)
    pick = [all_frames.index(n) for n in cfg.tool_frames]
    pos = ks.tool_poses.position[:, :, pick]          # (1, 1, L, 3)
    quat = ks.tool_poses.quaternion[:, :, pick]
    print(f"[check] asking for the walk's own end pose back, "
          f"{len(cfg.tool_frames)} tool frames")

    # SequenceGoalToolPose: (num_frames, num_envs, num_links, num_goalset, D)
    seq = SequenceGoalToolPose(
        tool_frames=cfg.tool_frames,
        position=pos.permute(1, 0, 2, 3).unsqueeze(3).contiguous(),
        quaternion=quat.permute(1, 0, 2, 3).unsqueeze(3).contiguous())
    res = r.solve_sequence(seq)
    sol = res.joint_state.position[0, 0].cpu().numpy()
    want = q[0].cpu().numpy()
    d = np.abs(sol - want)
    body = [i for i, n in enumerate(jn) if not n.startswith("base_j")]
    print(f"[check] base error   {np.round(d[:6], 4)}")
    print(f"[check] joint error  max {d[body].max()*1000:.1f} mrad, "
          f"mean {d[body].mean()*1000:.1f} mrad")
    worst = body[int(np.argmax(d[body]))]
    print(f"[check] worst joint  {jn[worst]} {d[worst]*1000:.1f} mrad")


if __name__ == "__main__":
    main()
