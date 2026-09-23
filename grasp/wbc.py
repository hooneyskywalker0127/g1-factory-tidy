# GR00T-WholeBodyControl's lower-body policy, driven from Isaac Lab.
#
# The policy ships as ONNX with the whole interface written down, so it does
# not have to run in the MuJoCo stack it was deployed in:
#
#   decoupled_wbc/sim2mujoco/resources/robots/g1/policy/
#       GR00T-WholeBodyControl-Balance.onnx   (loco_cmd ~ 0)
#       GR00T-WholeBodyControl-Walk.onnx      (loco_cmd nonzero)
#   decoupled_wbc/sim2mujoco/resources/robots/g1/g1_gear_wbc.yaml
#   decoupled_wbc/sim2mujoco/scripts/run_mujoco_gear_wbc.py
#
# Everything here mirrors that script: the same 86-dim observation in the same
# order, the same scales, the same 6-frame history, the same
# target = default + action * action_scale.
#
# 15 actions = 12 leg joints + 3 waist joints. The 14 arm joints are left to
# whoever owns the upper body -- for us, the GraspGenX trajectory.
import collections
import os

import numpy as np
import onnxruntime as ort
import yaml

WBC_ROOT = ("/home/sehoon/Projects/GR00T-WholeBodyControl/decoupled_wbc/"
            "sim2mujoco/resources/robots/g1")

# Joint order of the policy's own MuJoCo model (g1_gear_wbc.xml, with the 14
# hand joints commented out): legs, waist, then both arms. 29 total, and
# 13 + 2*29 + 15 = 86, which is the observation width the ONNX expects.
LOWER_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
]
ARM_JOINTS = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
POLICY_JOINTS = LOWER_JOINTS + ARM_JOINTS
N_JOINTS = len(POLICY_JOINTS)      # 29
N_ACTIONS = len(LOWER_JOINTS)      # 15
# The arms are PD'd separately in the reference script, with these gains.
ARM_KP, ARM_KD = 100.0, 0.5


def _quat_rotate_inverse(q, v):
    """Rotate v into the frame of quaternion q (wxyz). Same algebra as
    run_mujoco_gear_wbc.py's quat_rotate_inverse."""
    w, x, y, z = q
    qc = np.array([w, -x, -y, -z])
    return np.array([
        v[0] * (qc[0] ** 2 + qc[1] ** 2 - qc[2] ** 2 - qc[3] ** 2)
        + v[1] * 2 * (qc[1] * qc[2] - qc[0] * qc[3])
        + v[2] * 2 * (qc[1] * qc[3] + qc[0] * qc[2]),
        v[0] * 2 * (qc[1] * qc[2] + qc[0] * qc[3])
        + v[1] * (qc[0] ** 2 - qc[1] ** 2 + qc[2] ** 2 - qc[3] ** 2)
        + v[2] * 2 * (qc[2] * qc[3] - qc[0] * qc[1]),
        v[0] * 2 * (qc[1] * qc[3] - qc[0] * qc[2])
        + v[1] * 2 * (qc[2] * qc[3] + qc[0] * qc[1])
        + v[2] * (qc[0] ** 2 - qc[1] ** 2 - qc[2] ** 2 + qc[3] ** 2),
    ], dtype=np.float32)


class LowerBodyWBC:
    """Wraps the released ONNX policy. Call ``step`` at the policy rate
    (simulation_dt * control_decimation = 50 Hz) and apply ``targets`` to the
    15 lower joints through a PD with the config's own gains."""

    def __init__(self, root=WBC_ROOT):
        with open(os.path.join(root, "g1_gear_wbc.yaml")) as f:
            self.cfg = yaml.safe_load(f)
        c = self.cfg
        self.balance = ort.InferenceSession(
            os.path.join(root, c["policy_path"]),
            providers=["CPUExecutionProvider"])
        self.walk = ort.InferenceSession(
            os.path.join(root, c["walk_policy_path"]),
            providers=["CPUExecutionProvider"])
        self.single_obs_dim = c["num_obs"] // c["obs_history_len"]   # 86
        assert self.single_obs_dim == 13 + 2 * N_JOINTS + N_ACTIONS, \
            f"observation width {self.single_obs_dim} does not match 29 joints"
        self.history = collections.deque(
            [np.zeros(self.single_obs_dim, dtype=np.float32)] * c["obs_history_len"],
            maxlen=c["obs_history_len"])
        self.action = np.zeros(N_ACTIONS, dtype=np.float32)
        self.default_angles = np.array(c["default_angles"], dtype=np.float32)
        self.targets = self.default_angles.copy()
        # zero-padded to 29, exactly as the reference script does
        self.padded_defaults = np.zeros(N_JOINTS, dtype=np.float32)
        self.padded_defaults[:len(self.default_angles)] = self.default_angles
        self.kps = np.array(c["kps"], dtype=np.float32)
        self.kds = np.array(c["kds"], dtype=np.float32)
        self.dt = c["simulation_dt"]
        self.decimation = c["control_decimation"]

    def command(self, loco=(0.0, 0.0, 0.0), height=None, rpy=(0.0, 0.0, 0.0)):
        """7-wide command: scaled base velocity, base height, torso rpy."""
        c = self.cfg
        cmd = np.zeros(7, dtype=np.float32)
        cmd[:3] = np.asarray(loco, dtype=np.float32) * np.asarray(c["cmd_scale"])
        cmd[3] = c["height_cmd"] if height is None else height
        cmd[4:7] = rpy
        return cmd, np.linalg.norm(np.asarray(loco, dtype=np.float32))

    def step(self, qj, dqj, base_quat_wxyz, base_ang_vel, cmd, loco_norm):
        """qj/dqj: the 29 policy joints, in POLICY_JOINTS order.
        base_quat_wxyz / base_ang_vel: pelvis, in its own frame."""
        c = self.cfg
        obs = np.zeros(self.single_obs_dim, dtype=np.float32)
        obs[0:7] = cmd
        obs[7:10] = np.asarray(base_ang_vel) * c["ang_vel_scale"]
        obs[10:13] = _quat_rotate_inverse(
            np.asarray(base_quat_wxyz, dtype=np.float32),
            np.array([0.0, 0.0, -1.0], dtype=np.float32))
        obs[13:13 + N_JOINTS] = (np.asarray(qj) - self.padded_defaults) * c["dof_pos_scale"]
        obs[13 + N_JOINTS:13 + 2 * N_JOINTS] = np.asarray(dqj) * c["dof_vel_scale"]
        obs[13 + 2 * N_JOINTS:] = self.action

        self.history.append(obs)
        flat = np.concatenate(list(self.history)).astype(np.float32)[None, :]

        # The reference switches policies on the locomotion command's size.
        sess = self.balance if loco_norm <= 0.05 else self.walk
        self.action = np.asarray(
            sess.run(None, {sess.get_inputs()[0].name: flat})[0]
        ).squeeze().astype(np.float32)
        self.targets = self.action * c["action_scale"] + self.default_angles
        return self.targets
