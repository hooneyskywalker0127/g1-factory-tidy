# Does a synthesised grasp actually pick the tool up?
#
# Everything before this has been geometry: a hand pose that holds in Dexonomy's
# own simulator, on a floating hand. Here the pose is put on the end of G1's arm
# in Isaac, the fingers close on it, and the arm lifts. The object either comes
# with it or it does not, and that is the first number in this project that says
# whether the robot can do the job.
#
# The arm is the 29-dof G1: 7 joints to the palm (3 shoulder, elbow, 3 wrist).
# The 23-dof one cannot be used -- it has no wrist, so 5 joints cannot reach an
# arbitrary 6-dof pose, and the synthesised grasp specifies all six.
#
# The root is fixed. Balance is a separate problem and a robot that falls over
# proves nothing about the grasp.
#
#   python grasp/pick_tool.py [n_grasps]
import glob
import os
import sys

from isaaclab.app import AppLauncher

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
N = int(sys.argv[1]) if len(sys.argv) > 1 else 5
TOOL_USD = os.path.join(REPO, "assets", "tools",
                        "ddg_gd_drill_poisson_000_s200.usd")
GRASPS = sorted(glob.glob(os.path.expanduser(
    "~/Projects/Dexonomy/output/tools_unitree_g1/succ_grasp/1_Large_Diameter/"
    "ddg_gd_drill_poisson_000/floating/scale020/*_grasp.npy")))

app = AppLauncher(headless=True).app

import numpy as np  # noqa: E402
import torch  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
import isaaclab.utils.math as math_utils  # noqa: E402
import omni.usd  # noqa: E402
from isaaclab.assets import Articulation, RigidObject, RigidObjectCfg  # noqa: E402
from isaaclab.controllers import DifferentialIKController  # noqa: E402
from isaaclab.controllers import DifferentialIKControllerCfg  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab_assets.robots.unitree import G1_29DOF_CFG  # noqa: E402
from pxr import UsdGeom  # noqa: E402

ARM = ["right_shoulder_pitch_joint", "right_shoulder_roll_joint",
       "right_shoulder_yaw_joint", "right_elbow_joint",
       "right_wrist_roll_joint", "right_wrist_pitch_joint",
       "right_wrist_yaw_joint"]
# same names on both sides, so no mapping table is needed here
FINGERS = ["right_hand_thumb_0_joint", "right_hand_thumb_1_joint",
           "right_hand_thumb_2_joint", "right_hand_middle_0_joint",
           "right_hand_middle_1_joint", "right_hand_index_0_joint",
           "right_hand_index_1_joint"]
PALM = "right_hand_palm_link"
# The tool sits on a bench, not on the floor. With the root fixed the robot
# cannot bend, and the floor is out of the arm's reach: the shoulder is at
# z = 1.0, so a tool at z = 0.2 half a metre out is 0.92 m away and the arm is
# shorter than that. Reaching the floor is the legs' and waist's problem, and
# mixing it in here would make a failed grasp and an unreachable goal look the
# same. The bench puts the tool where the arm can get to it.
TOOL_XY = (0.36, -0.22)
BENCH_TOP = 0.80
LIFT = 0.20                      # how far the wrist is raised to test the hold

sim = SimulationContext(sim_utils.SimulationCfg(dt=1.0 / 200.0, device="cpu"))
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
ground = sim_utils.GroundPlaneCfg()
ground.func("/World/ground", ground)
light = sim_utils.DomeLightCfg(intensity=900.0)
light.func("/World/light", light)
bench = sim_utils.CuboidCfg(size=(0.7, 0.7, BENCH_TOP),
                            collision_props=sim_utils.CollisionPropertiesCfg(),
                            visual_material=sim_utils.PreviewSurfaceCfg(
                                diffuse_color=(0.45, 0.45, 0.48)))
bench.func("/World/Bench", bench,
           translation=(TOOL_XY[0] + 0.12, TOOL_XY[1], BENCH_TOP / 2.0))

cfg = G1_29DOF_CFG.replace(prim_path="/World/G1")
cfg.spawn = cfg.spawn.replace(
    articulation_props=sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=False, solver_position_iteration_count=12,
        solver_velocity_iteration_count=4, fix_root_link=True))
robot = Articulation(cfg)
tool = RigidObject(RigidObjectCfg(
    prim_path="/World/Tool",
    spawn=sim_utils.UsdFileCfg(usd_path=TOOL_USD),
    init_state=RigidObjectCfg.InitialStateCfg(
        pos=(TOOL_XY[0], TOOL_XY[1], BENCH_TOP + 0.20)),
))
ik = DifferentialIKController(
    DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False,
                                ik_method="dls"),
    num_envs=1, device=sim.device)
sim.reset()

arm_ids = [robot.find_joints([n])[0][0] for n in ARM]
fing_ids = [robot.find_joints([n])[0][0] for n in FINGERS]
palm_id = robot.find_bodies([PALM])[0][0]
jac_id = palm_id - 1                    # fixed base: jacobian rows start at 1
print(f"[pick] arm joints {arm_ids}, palm body {palm_id}")
print(f"[pick] {len(GRASPS)} synthesised grasps available, trying {min(N, len(GRASPS))}")


def settle(steps, arm_t=None, fing_t=None):
    tgt = robot.data.joint_pos_target.clone()
    if arm_t is not None:
        tgt[0, arm_ids] = arm_t
    if fing_t is not None:
        tgt[0, fing_ids] = fing_t
    for _ in range(steps):
        robot.set_joint_position_target(tgt)
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim.get_physics_dt())
        tool.update(sim.get_physics_dt())


DEBUG = os.environ.get("PICK_DEBUG") == "1"


def pin_tool(pose7):
    tool.write_root_pose_to_sim(pose7)
    tool.write_root_velocity_to_sim(torch.zeros((1, 6)))


def solve_ik(goal_pos, goal_quat, pose7, iters=150):
    """Solve for an arm configuration that puts the palm on the goal.

    Done kinematically: the joints are teleported each iteration and the result
    read back, so the jacobian, the palm pose and the configuration being
    updated all describe the SAME arm. Running the solver against a moving,
    lagging arm instead mixes two configurations and it neither converges nor
    stays put -- the commands ran 5 rad from the joints and the palm sat 700 mm
    off the goal however long it ran.

    The tool is pinned through the solve so the teleporting arm cannot bat it
    off the bench before the real attempt starts.
    """
    q = robot.data.joint_pos.clone()
    best_q, best_err = q[0, arm_ids].clone(), 1e9
    for _ in range(iters):
        root_pos, root_quat = robot.data.root_pos_w[:1], robot.data.root_quat_w[:1]
        p_b, q_b = math_utils.subtract_frame_transforms(
            root_pos, root_quat, goal_pos, goal_quat)
        ik.set_command(torch.cat([p_b, q_b], dim=-1))
        jac = robot.root_physx_view.get_jacobians()[:, jac_id, :, arm_ids]
        ee_p, ee_q = math_utils.subtract_frame_transforms(
            root_pos, root_quat,
            robot.data.body_pos_w[:1, palm_id], robot.data.body_quat_w[:1, palm_id])
        err = torch.norm(robot.data.body_pos_w[0, palm_id] - goal_pos[0]).item()
        if err < best_err:
            best_err, best_q = err, robot.data.joint_pos[0, arm_ids].clone()
        q_des = ik.compute(ee_p, ee_q, jac, robot.data.joint_pos[:, arm_ids])
        q[0, arm_ids] = q_des[0]
        robot.write_joint_state_to_sim(q, torch.zeros_like(q))
        pin_tool(pose7)
        sim.step()
        robot.update(sim.get_physics_dt())
    return best_q, best_err


def move_arm(q_goal, fingers, steps=200, pose7=None):
    """Drive the arm to a configuration over `steps` ticks, holding fingers."""
    start = robot.data.joint_pos[0, arm_ids].clone()
    tgt = robot.data.joint_pos_target.clone()
    tgt[0, fing_ids] = fingers
    for k in range(steps):
        a = (k + 1) / steps
        tgt[0, arm_ids] = start + (q_goal - start) * a
        robot.set_joint_position_target(tgt)
        robot.write_data_to_sim()
        if pose7 is not None:
            pin_tool(pose7)
        sim.step()
        robot.update(sim.get_physics_dt())
        tool.update(sim.get_physics_dt())
    return torch.norm(robot.data.body_pos_w[0, palm_id]).item()


picked = 0
tried = 0
for gi, gf in enumerate(GRASPS[:N]):
    dd = np.load(gf, allow_pickle=True).item()
    q = dd["grasp_qpos"][0]
    wrist_p_o = torch.tensor(q[:3], dtype=torch.float32).unsqueeze(0)
    wrist_q_o = torch.tensor(q[3:7], dtype=torch.float32).unsqueeze(0)
    grasp_f = torch.tensor(dd["grasp_qpos"][0][7:], dtype=torch.float32)
    pre_f = torch.tensor(dd["pregrasp_qpos"][0][7:], dtype=torch.float32)
    sqz_f = torch.tensor(dd["squeeze_qpos"][0][7:], dtype=torch.float32)

    # same start every attempt
    state = tool.data.default_root_state.clone()
    state[0, 0:3] = torch.tensor([TOOL_XY[0], TOOL_XY[1], BENCH_TOP + 0.20])
    tool.write_root_pose_to_sim(state[:, :7])
    tool.write_root_velocity_to_sim(torch.zeros_like(state[:, 7:]))
    robot.write_joint_state_to_sim(robot.data.default_joint_pos.clone(),
                                   torch.zeros_like(robot.data.default_joint_pos))
    settle(250, arm_t=robot.data.default_joint_pos[0, arm_ids],
           fing_t=torch.zeros(len(fing_ids)))
    tool.update(sim.get_physics_dt())
    rest = tool.data.root_state_w[:, :7].clone()
    rest_z = rest[0, 2].item()

    goal_p, goal_q = math_utils.combine_frame_transforms(
        rest[:, :3], rest[:, 3:7], wrist_p_o, wrist_q_o)
    shoulder = robot.data.body_pos_w[0, robot.find_bodies(
        ["right_shoulder_roll_link"])[0][0]]
    reach = torch.norm(goal_p[0] - shoulder).item()

    q_sol, ik_err = solve_ik(goal_p, goal_q, rest)
    robot.write_joint_state_to_sim(robot.data.default_joint_pos.clone(),
                                   torch.zeros_like(robot.data.default_joint_pos))
    pin_tool(rest)
    settle(60, fing_t=pre_f)
    move_arm(q_sol, pre_f, steps=260, pose7=rest)
    settle(120, fing_t=grasp_f)
    settle(150, fing_t=sqz_f)

    lift_goal = goal_p.clone()
    lift_goal[0, 2] += LIFT
    q_lift, _ = solve_ik(lift_goal, goal_q, rest)
    robot.write_joint_state_to_sim(robot.data.joint_pos.clone(),
                                   torch.zeros_like(robot.data.joint_pos))
    move_arm(q_lift, sqz_f, steps=260)
    settle(150, fing_t=sqz_f)

    tool.update(sim.get_physics_dt())
    rise = tool.data.root_pos_w[0, 2].item() - rest_z
    ok = rise > LIFT * 0.5
    picked += int(ok)
    tried += 1
    print(f"[pick] grasp {gi}: reach {reach*1000:.0f} mm, ik solved to "
          f"{ik_err*1000:5.1f} mm, tool rose {rise*1000:+6.1f} mm  "
          f"{'HELD' if ok else 'dropped'}")

print(f"[pick] {picked}/{tried} picked up")
sys.stdout.flush()
os._exit(0)
