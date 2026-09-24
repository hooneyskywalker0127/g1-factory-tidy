# Why the fingers did not close: a minimal reproduction. VD_MODE=fixed|root|root_step|root_update|rootpose|rootframe.
# root = the tester's write pattern (1/33 finger speed); root_update = the fix (robot.update every substep).
from isaaclab.app import AppLauncher
app = AppLauncher(headless=True, enable_cameras=False).app
import torch, numpy as np, os
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext
from isaaclab_assets.robots.unitree import G1_29DOF_CFG
MODE = os.environ.get("VD_MODE", "root")     # root: floating root written each step (the tester's pattern)
sim = SimulationContext(sim_utils.SimulationCfg(dt=1.0 / 1000.0, device="cpu"))
cfg = G1_29DOF_CFG.replace(prim_path="/World/G1")
cfg.spawn = cfg.spawn.replace(articulation_props=sim_utils.ArticulationRootPropertiesCfg(
    enabled_self_collisions=False, solver_position_iteration_count=12, solver_velocity_iteration_count=4, fix_root_link=(MODE == "fixed")))
cfg.actuators["hands"] = cfg.actuators["hands"].replace(effort_limit=1.4, velocity_limit=12.0)
robot = Articulation(cfg); sim.reset(); robot.update(0.0)
names = [f"right_hand_{j}_joint" for j in ("index_0", "index_1", "middle_0", "middle_1", "thumb_1", "thumb_2")]
ids = [robot.find_joints([n])[0][0] for n in names]
body = [j for j in range(robot.num_joints) if "hand" not in robot.joint_names[j]]
tgt = robot.data.default_joint_pos.clone(); zero = torch.zeros_like(tgt)
rs = robot.data.root_state_w[:1].clone(); rs[0, 7:] = 0.0
vel = torch.tensor([[0.25, 0.25, 0.25, 0.25, -0.25, -0.25]])
def put(squeeze):
    robot.set_joint_position_target(tgt)
    if squeeze: robot.set_joint_velocity_target(vel, joint_ids=ids)
    if MODE == "rootframe": robot.write_root_state_to_sim(rs)          # once per frame
    for _ in range(33):
        if MODE in ("root", "root_step", "root_update"): robot.write_root_state_to_sim(rs)   # every substep (the tester)
        if MODE == "root_step" and squeeze:
            robot.set_joint_position_target(tgt); robot.set_joint_velocity_target(vel, joint_ids=ids)
        if MODE == "rootpose": robot.write_root_pose_to_sim(rs[:, :7])  # pose only, every substep
        if MODE == "root_update": robot.update(sim.get_physics_dt())   # buffers current before the partial write
        robot.write_joint_state_to_sim(tgt[:, body], zero[:, body], joint_ids=body)
        robot.write_data_to_sim(); sim.step()
    robot.update(sim.get_physics_dt())
for i in range(15): put(False)
print("[vd] before squeeze q", np.round(robot.data.joint_pos[0, ids].numpy(), 3), flush=True)
robot.write_joint_stiffness_to_sim(0.0, joint_ids=ids); robot.write_joint_damping_to_sim(8.0, joint_ids=ids)
for i in range(60):
    put(True)
    if i in (0, 15, 30, 59):
        print(f"[vd] {MODE} t={(i+1)*33/1000:.2f}s q {np.round(robot.data.joint_pos[0, ids].numpy(), 3)} stiff {robot.data.joint_stiffness[0, ids].tolist()[:2]} vel_tgt {robot.data.joint_vel_target[0, ids].tolist()[:2]}", flush=True)
os._exit(0)
