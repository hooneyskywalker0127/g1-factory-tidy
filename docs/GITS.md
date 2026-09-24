# The three gits, read properly

Written after finding four transcription bugs one at a time. Everything here
is quoted from the repositories, with the file it came from, so the next
question gets answered by looking here rather than by another experiment.

## GR00T-WholeBodyControl

### Frames and conventions (docs/source/references/conventions.md)
- Isaac Lab and MuJoCo: Z-up, right-handed, X forward, Y left.
- Quaternions are **wxyz everywhere** except scipy (xyzw). Isaac Lab's
  `root_quat_w` / `body_quat_w` are wxyz.
- 6D rotation = **first two columns of the rotation matrix**. Both
  `mat[..., :2].reshape(-1)` (commands.py:1962) and the C++
  (g1_deploy_onnx_ref.cpp:679, which cites the same numpy) flatten **row-wise**:
  `[R00, R01, R10, R11, R20, R21]`.
- Joint order: `y_data = x_data[..., X_TO_Y]`, so
  `G1_MUJOCO_TO_ISAACLAB_DOF[i]` is the **MuJoCo index of IsaacLab slot i**.
  The names read backwards from the usual convention; the arrays are in
  gear_sonic/envs/manager_env/robots/g1.py.
- Motion PKL `dof` and `pose_aa` are **MuJoCo order**. Isaac Lab is IsaacLab
  order. IsaacLab's is breadth-first, left and right interleaved.

### What the encoder is given (sonic_release/config.yaml, observation_config.yaml)
- The g1 encoder takes exactly two terms: `command_multi_future_nonflat` and
  `motion_anchor_ori_b_mf_nonflat`. The experiment preset overrides the
  tokenizer to `unitoken_all_noz` -- **no z**, which is why the release's g1
  mode does not list root height.
- `command_multi_future` = `cat([joint_pos_multi_future, joint_vel_multi_future])`
  (commands.py:897), i.e. all future positions then all future velocities,
  which is exactly the C++'s two contiguous fields.
- `joint_pos_multi_future` = `motion_lib.get_dof_pos(...)` -- **absolute**
  reference angles, IsaacLab order, not offsets.
- `motion_anchor_ori_b_mf` = reference root rotation **seen from the robot's
  anchor body**, full orientation including pitch and roll:
      root_rot_dif = quat_mul(quat_inv(robot_anchor_quat_w), ref_root_quat)
  `anchor_body: pelvis` (config.yaml:377), and Isaac's G1 root IS the pelvis.
- Observations not in a mode's `required_observations` are **zero-filled**
  (observation_config.md). Filling them anyway is not neutral by contract,
  even where it measures as harmless.
- `encoder_mode_4` is the mode **ID** plus three zeros, not a one-hot. g1 is
  mode 0, so the field is all zeros.

### What the decoder is given
- Deployment order (policy/release/observation_config.yaml, and the C++
  concatenates in the order listed): token_state(64),
  his_base_angular_velocity(30), his_body_joint_positions(290),
  his_body_joint_velocities(290), his_last_actions(290), his_gravity_dir(30)
  = 994. The file's own header comment says 436 -- it is stale, written for a
  4-frame history.
- Training composes the same terms in a different order
  (config/manager_env/observations/policy/local_dir_hist.yaml: gravity_dir,
  base_ang_vel, joint_pos, joint_vel, actions). The deployment YAML is the one
  that pairs with the exported ONNX.
- `body_joint_positions` is **default-subtracted** despite the doc calling it
  "current joint positions": g1_deploy_onnx_ref.cpp:2829
      body_q[i] = q[mujoco_to_isaaclab[i]] - default_angles[mujoco_to_isaaclab[i]]
  and it is stored already in IsaacLab order.
- `last_actions` is the raw previous policy output.

### Gains and scales (include/policy_parameters.hpp)
- `default_angles`: hips -0.312, knees 0.669, ankle pitch -0.363, waist 0,
  shoulders 0.2 / +-0.2, elbows 0.6, wrists 0.
- `action_scale = 0.25 * effort_limit / stiffness`, per motor type.
- `stiffness = armature * (2*pi*10)^2`, `damping = 2 * 2 * armature * (2*pi*10)`.
- **Six** joints are driven at twice nominal, not four: both ankle pitches,
  both ankle rolls, **waist_roll and waist_pitch** (kps lines 148-158).
- Release notes, 2026-08-31: `--motor-kp-scale 4,10=1.5 --motor-kd-scale
  4,10=1.5` on the ankle pitches, against stumbling. Not in the 14 July policy.

### The reference motion (motion_reference.md)
- A motion is a **folder of CSVs** at 50 Hz: joint_pos, joint_vel (IsaacLab
  order, 29), body_quat (wxyz, root first), body_pos, metadata.txt with
  `Body part indexes:`. Minimum viable set is those five, root-only.
- `reference/convert_motions.py` converts a joblib pkl to that layout.
- `visualize_motion.py --motion_dir ...` checks a motion before deploying it.
- Current reference tracking is **joint-based only, encoder mode 0**; SMPL
  tracking (mode 2) would need a code change.

### The planner (planner_onnx.md)
- Inputs: `context_mujoco_qpos [1,4,36]`, `target_vel`, `mode`,
  `movement_direction [1,3]`, `facing_direction [1,3]`, `height`.
  All in the **world frame**; the model canonicalises internally.
- `mode`: 0 idle, 1 slowWalk, 2 walk, 3 run, 4 squat (needs `height`,
  ~0.4-0.8 m), 21 objectCarrying, 22 crouch.
- `movement_direction` below 1e-5 falls back to `facing_direction` for
  in-place turning -- this is why a 1e-6 "hold" drifts.
- `has_specific_target=1` takes `specific_target_positions [1,4,3]` and
  `specific_target_headings [1,4]` as waypoint constraints, overriding the
  spring model's target. The last heading is the primary target.
- Output `mujoco_qpos [1,N,36]` is **not truncated** -- slice to
  `num_pred_frames`. Global world frame, wxyz root quaternion.

### decoupled_wbc (decoupled_wbc.md, g1_decoupled_whole_body_policy.py)
- One call takes both: `navigate_cmd` and `target_upper_body_pose`.
      q[upper] = target_upper_body_pose
      lower = lower_body_policy.get_action(time, q_arms, base_height_command,
                                           torso_orientation_rpy, navigate_cmd)
      q[lower] = lower["body_action"][0][:len(lower)]
- Walking and standing are **two networks chosen by the command's magnitude**,
  on the same observation, with no reset:
      if norm(cmd) < 0.05: policy_1 (stand) else: policy_2 (walk)
- Its own sources for `navigate_cmd` are a keyboard and a teleop stream --
  there is no goal-following controller in the repository.
- Uncommanded upper-body joints are targeted at **what the robot currently
  holds**, not at a configured pose.

### Upper body over a walking reference
- `g1_deploy_onnx_ref.cpp:780`, inside the routine that gathers the encoder's
  future frames: when `has_upper_body_data_`, the **17** joints at
  `upper_body_joint_isaaclab_order_in_isaaclab_index`
  = {2,5,8,11,12,15,16,19,20,21,22,23,24,25,26,27,28} are replaced by the
  externally-provided targets, per frame, before encoding.
- So a planned arm over a walk is not two motions to join: every frame the
  encoder sees already has the commanded arm in it.

## cuRobo

### MotionRetargeter (examples/getting_started/humanoid_retargeting.py)
The part of cuRobo this project had not read at all, and the one written for
exactly our shape of problem: produce a whole-body G1 motion that tracks
per-link pose targets.

- **Floating base without touching the URDF**: `extra_links` with
  `child_link_name` inserts six virtual joints (3 prismatic + 3 revolute)
  between `base_link` and `pelvis`, re-parenting the pelvis under them. This
  is the same mechanism our `base_j_x` / `base_j_y` / `base_j_ztheta` lock
  uses -- it is cuRobo's own, not a hack.
- **Output is 35 DOF**: `[x, y, z, roll, pitch, yaw]` then the 29 body joints.
  So one solve gives both where the robot stands and how it is posed.
- **Per-link weights** via `ToolPoseCriteria` in
  `MotionRetargeterCfg.tool_pose_criteria`: "feet and hips get high weight to
  maintain balance, while mid-chain links like shoulders get low weight since
  the elbow and wrist targets already constrain the arm."
- **Two-phase solve**: frame 0 is a global IK with 64 seeds and no velocity
  limit; frames 1..N warm-start from the previous solution with a velocity
  limit, "which prevents it from jumping to a distant solution even if one has
  lower cost". That velocity limit is the mechanism against exactly the kind
  of jump this project has been fighting by hand.
- Three levels: IK without self-collision, IK with it, and `use_mpc=True`
  which "optimizes a trajectory over a planning horizon, producing smoother
  results with acceleration and jerk costs".
- `solve_sequence(seq)` offline, `solve_frame(tool_pose)` streaming.
- Quaternions are wxyz, as everywhere else.
- A prebuilt G1 config ships at
  `curobo/content/configs/robot/unitree_g1_29dof_retarget.yml`, and
  `build_robot_model` regenerates it with chosen `--tool-frames`.

What this means for the pipeline: the walk and the reach do not have to be two
motions. Tool-pose targets for the feet and pelvis over the walk, plus the
wrist target from the grasp, solved as one sequence, give a single 35-DOF
trajectory -- which is also the shape a SONIC reference wants (root pose plus
29 joints).

## GraspGenX

### The pipeline as its own README describes it (end2end/README.md)
GraspGenX grasps -> cuRobo plans to a **collision-free** grasp -> Newton/MuJoCo
replays under gravity and contacts with PD-controlled joints -> MP4 + USD.

- `clutter_task` "scores grasps for collision-freedom (gripper mesh vs
  table/bin/neighbors/target, fcl) and **feeds cuRobo only the collision-free
  grasps**". The `pick_and_lift` path we run does not call that filter -- it is
  only invoked from clutter_task -- so our G1 run hands cuRobo raw candidates.
- Physics notes that are deliberate, not accidents:
  - Bin collision is hollow primitives, not a solid cuboid.
  - Contact gap/margin stays at Newton defaults; a tight 1 mm band caused
    resting-contact jitter that tipped tall objects.
  - `condim = 3`.
  - **"The PD target is initialized to the home pose so the initial settle
    holds home (no start-of-demo jump)."** The same class of artifact this
    project fought at the walk/pick seam, solved by initialising the target
    rather than by moving the robot.
  - Velocity-mode gripper close waits `--hold_after_close_frames` for the
    fingers to settle before lifting; "premature lift slips the object out".
- `trajectory.json` holds per-frame link transforms and object poses and is
  decoupled from the sim, which is why re-rendering never needs a re-plan.
- Config layers: `robots/<name>.yaml` carries the cuRobo robot config, tool
  frame, default joint config, `grasp_to_tool_transform`, base pose, gripper
  name and PD gains; `envs/<name>.yaml` carries table/bin assets,
  `object_slots`, `placement_region` and the render camera.

### The model (README.md)
- One model across grippers, conditioned on the gripper's **swept volume**; it
  generalises zero-shot to grippers it never saw. `unitree_g1` is a shipped
  gripper description.
- Default planner is **GraspMoE**: diffusion samples union OBB heuristic
  grasps, all scored by the discriminator. `--planner diffusion` drops the OBB
  half; `--moe_obb_density sparse|dense` controls the OBB sweep.
- Scene-point-cloud inference filters grasps that would collide with the rest
  of the scene; `--no-filter_collisions` turns that off.
- Inputs are either a partial point cloud from a depth camera or a mesh. Ours
  is the first, which is the mode the model was built for.
