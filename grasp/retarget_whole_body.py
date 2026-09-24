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
HOLD = ["pelvis", "left_hip_roll_link", "right_hip_roll_link",
        "left_ankle_roll_link", "right_ankle_roll_link",
        "left_knee_link", "right_knee_link"]
WORK = ["right_wrist_yaw_link"]
LOOSE = ["torso_link", "left_shoulder_roll_link", "left_elbow_link",
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
