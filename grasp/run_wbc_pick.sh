#!/usr/bin/env bash
# See, then reach. The arm target is not written down anywhere in this file:
# the robot photographs the cell, GraspGenX predicts a grasp from that photo,
# and that grasp is what the reach clip aims at. No object in the picture, no
# grasp, no motion -- the run stops instead of reaching at empty air.
#
#   bash grasp/run_wbc_pick.sh
set -uo pipefail
REPO=/home/sehoon/Documents/GitHub/g1-factory-tidy
SWARM=/home/sehoon/Documents/GitHub/humanoid-swarm-sim
SONIC_REPO=/home/sehoon/Projects/GR00T-WholeBodyControl
GGX=/home/sehoon/Projects/GraspGenX
CAP=$REPO/results/capture_wbc
MOTION=$REPO/results/motion
RUN=$GGX/end2end/runs/wbc_vision
source /home/sehoon/miniconda3/etc/profile.d/conda.sh
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1

step() { echo; echo "===== $* ====="; }

# 1. The scene the robot will actually work in, and a photo of it from the
#    robot's own head camera. Same stand the plan is executed from.
step "1/5 capture"
conda activate env_isaaclab
python "$REPO/grasp/capture_rgbd.py" -1.30 -0.60 -90 \
    --plan "$REPO/results/g1_graspgen.json" --plan-stand -1.30 -0.60 -90 \
    --out "$CAP" 2>&1 | grep -aE "^\[cap\]|Traceback|Error"
if ! grep -q "obj_plan" "$CAP/meta_data.json" 2>/dev/null; then
    echo "STOP: the camera did not see a pickable object -- nothing to reach for"
    exit 1
fi

# 2. Grasps from that photo, and a cuRobo plan to the one it can reach.
step "2/5 grasp from the photo"
conda activate graspgenx
mkdir -p "$RUN"
cd "$GGX"
PYOPENGL_PLATFORM=egl PYGLET_HEADLESS=true \
python end2end/e2e_grasp_demo.py \
    --robot_config end2end/robots/g1_right_arm.yaml \
    --env_config   end2end/envs/g1_reach_test.yaml \
    --mesh_file    assets/sample_data/hope_objects/GranolaBars.obj \
    --capture_dir  "$CAP" --capture_object obj_plan \
    --task pick_and_lift --playback_mode kinematic --no-viser \
    --num_grasps 200 --topk 80 --grasp_threshold 0.7 --planner graspmoe \
    --hold_after_close_frames 150 --seed 0 \
    --export-trajectory "$RUN/trajectory.json" > "$RUN/run.log" 2>&1
if [ ! -f "$RUN/trajectory.json" ]; then
    echo "STOP: no reachable grasp came out of that photo (see $RUN/run.log)"
    exit 1
fi
grep -aE "chose goalset_idx|Collision filter|GraspGen returned" "$RUN/run.log" | tail -3

# 3. Where that grasp puts the palm, in the plan frame.
step "3/5 palm from the grasp"
conda activate env_isaaclab
read -r PX PY PZ < <(python - "$RUN/trajectory.json" <<'PY'
import json, sys, numpy as np
g = np.array(json.load(open(sys.argv[1]))["annotations"]["target_grasp_transform"], float)
print(" ".join(f"{v:.6f}" for v in g[:3, 3]))
PY
)
echo "palm (plan frame): $PX $PY $PZ"

# 4. The stance from SONIC's own planner, with the right arm bent onto it.
step "4/5 reach clip"
cd "$SONIC_REPO"
OMP_NUM_THREADS=1 python "$SWARM/common/gen_planner_motion.py" \
    --chain "0:12" --name wbc_idle --fps 30 --inplace 2>&1 | tail -2
python "$REPO/grasp/reach_clip.py" \
    "$SWARM/common/data/motion_lib/planner/wbc_idle.pkl" \
    "$MOTION" wbc_pick "$PX" "$PY" "$PZ" 2>&1 | grep -aE "^\[reach\]"

# 5. Track it whole-body in our cell, with the table and the box in the scene.
step "5/5 tracking"
python - "$MOTION/wbc_pick.pkl" "$MOTION/torso.json" <<'PY'
import joblib, json, os, sys
import numpy as np
from scipy.spatial.transform import Rotation as R
sys.path[:0] = ["/home/sehoon/Documents/GitHub/humanoid-swarm-sim/common",
                "/home/sehoon/Documents/GitHub/humanoid-swarm-sim/demos/squat",
                "/home/sehoon/Projects/GR00T-WholeBodyControl/gear_sonic/data_process"]
from foot_height import load_urdf
from make_squat_demo_motion import ARMS
joints, order, _ = load_urdf()
m = list(joblib.load(sys.argv[1]).values())[0]
dof = np.asarray(m["dof"]); rt = np.asarray(m["root_trans_offset"]); rq = np.asarray(m["root_rot"])
f = len(dof) - 1; idx = {n: i for i, n in enumerate(order)}
T = np.eye(4); T[:3, :3] = R.from_quat(rq[f]).as_matrix(); T[:3, 3] = rt[f]
for jn in [j for j in ARMS["R"] if "waist" in j]:
    j = joints[jn]
    L = np.eye(4); L[:3, :3] = R.from_euler("xyz", j["rpy"]).as_matrix(); L[:3, 3] = j["xyz"]
    T = T @ L
    if j["axis"] is not None:
        Rr = np.eye(4); Rr[:3, :3] = R.from_rotvec(j["axis"] * dof[f, idx[jn]]).as_matrix(); T = T @ Rr
json.dump({"torso": T.tolist()}, open(sys.argv[2], "w"))
PY
python "$REPO/grasp/bake_props_usd.py" "$MOTION/torso.json" "$MOTION/wbc_props.usd" \
    2>&1 | grep -aE "^\[bake\]|^\[scene\]"

export SONIC_REPO SONIC_FREE_CAM=1
export SONIC_FACTORY_USD="$REPO/map/cell.usd"
export SONIC_PROPS_USD="$MOTION/wbc_props.usd" SONIC_PROPS_XY="0.0,0.0"
rm -rf /tmp/wbc_pick
HANDS=$(python - "$REPO/results/g1_graspgen.npy" "$REPO/results/g1_graspgen.json" <<'PY'
import json, sys, numpy as np
q = np.load(sys.argv[1]); names = json.load(open(sys.argv[2]))["joint_names"]
order = ["left_hand_index_0_joint","left_hand_index_1_joint","left_hand_middle_0_joint",
         "left_hand_middle_1_joint","left_hand_thumb_0_joint","left_hand_thumb_1_joint",
         "left_hand_thumb_2_joint","right_hand_index_0_joint","right_hand_index_1_joint",
         "right_hand_middle_0_joint","right_hand_middle_1_joint","right_hand_thumb_0_joint",
         "right_hand_thumb_1_joint","right_hand_thumb_2_joint"]
print("[" + ",".join(f"{float(q[-1][names.index(n)]) if n in names else 0.0:.4f}" for n in order) + "]")
PY
)
python gear_sonic/eval_agent_trl_drive.py \
    +checkpoint="$SONIC_REPO/sonic_release/last.pt" \
    +headless=True ++eval_callbacks=im_eval ++run_eval_loop=False ++num_envs=1 \
    ++manager_env.config.render_results=True \
    "++manager_env.config.save_rendering_dir=/tmp/wbc_pick" \
    "~manager_env/recorders=empty" "+manager_env/recorders=render" \
    ++manager_env.observations.policy.enable_corruption=False \
    ++manager_env.observations.tokenizer.enable_corruption=False \
    ++manager_env.commands.motion.encoder_sample_probs.g1=1.0 \
    ++manager_env.commands.motion.encoder_sample_probs.teleop=0.0 \
    ++manager_env.commands.motion.encoder_sample_probs.smpl=0.0 \
    ++manager_env.commands.motion.debug_vis=False \
    ++manager_env.commands.motion.start_from_first_frame=True \
    "++manager_env.config.eval_camera_abs=[2.6,-2.4,1.8,0.2,-0.1,0.95]" \
    "++manager_env.commands.motion.hand_default_positions=$HANDS" \
    ++manager_env.config.terrain_type=usd ++manager_env.config.env_spacing=9.0 \
    "++manager_env.config.train_only_terminations=[anchor_pos,anchor_ori_full,ee_body_pos,foot_pos_xyz]" \
    "++manager_env.commands.motion.motion_lib_cfg.max_unique_motions=1" \
    "++manager_env.commands.motion.motion_lib_cfg.filter_motion_keys=[wbc_pick]" \
    "++manager_env.commands.motion.motion_lib_cfg.motion_file=$MOTION" \
    "++manager_env.commands.motion.motion_lib_cfg.smpl_motion_file=dummy" 2>&1 \
    | grep -aoE "Terminated: [0-9]+ \| max frames: [0-9]+ \| steps [0-9]+" | tail -1
echo "WBC PICK DONE"
