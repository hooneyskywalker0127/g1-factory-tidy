"""GR00T's own balance policy, so the robot can stand while the arm reaches.

The pick used to be rendered with the pelvis welded to the world, because a
robot whose legs are inert falls over the moment the arm swings. Welding it is
not a thing a real G1 can do, and it is what makes the seam between the walk
and the pick look like a teleport -- one phase has a body, the next has a
fixture.

GR00T-WholeBodyControl ships the answer. decoupled_wbc's lower-body policy is
given the arm configuration and keeps the robot standing under it:

    lower_body_action = self.lower_body_policy.get_action(
        time, q_arms, base_height_command, torso_orientation_rpy, navigate_cmd)

and sim2mujoco/scripts/run_mujoco_gear_wbc.py is a complete reference for
driving it, down to a demo that reaches the right arm at a can on a table while
the policy holds the stance. This is that loop, with MuJoCo's state replaced by
arguments so it can be stepped from Isaac.

Everything here -- the observation layout, the scales, the gains, the default
angles -- is read from decoupled_wbc's own g1_gear_wbc.yaml and its script.
Nothing is tuned.
"""
import os

import numpy as np
import onnxruntime as ort
import yaml

WBC = ("/home/sehoon/Projects/GR00T-WholeBodyControl/decoupled_wbc/"
       "sim2mujoco/resources/robots/g1")

# The 29 joints in the order the policy was trained on: both legs, the waist,
# then the arms and hands. walk_clip.py writes its _end.json in this same
# order, because both read it from GR00T's own URDF.
LEG_WAIST = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
]

# The arms the policy was trained with: seven joints a side, no fingers. The
# G1 in this cell carries Dex3-1 hands, so the articulation has 43 joints and
# the policy knows 29 of them -- observing the other fourteen is what made the
# first attempt hand it a 43-long vector.
ARMS = [f"{s}_{j}_joint" for s in ("left", "right")
        for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                  "wrist_roll", "wrist_pitch", "wrist_yaw")]
POLICY_JOINTS = LEG_WAIST + ARMS


def _quat_rotate_inverse(q, v):
    """Verbatim from run_mujoco_gear_wbc.py; quaternion is (w, x, y, z)."""
    w, x, y, z = q
    c = np.array([w, -x, -y, -z])
    return np.array([
        v[0] * (c[0] ** 2 + c[1] ** 2 - c[2] ** 2 - c[3] ** 2)
        + v[1] * 2 * (c[1] * c[2] - c[0] * c[3])
        + v[2] * 2 * (c[1] * c[3] + c[0] * c[2]),
        v[0] * 2 * (c[1] * c[2] + c[0] * c[3])
        + v[1] * (c[0] ** 2 - c[1] ** 2 + c[2] ** 2 - c[3] ** 2)
        + v[2] * 2 * (c[2] * c[3] - c[0] * c[1]),
        v[0] * 2 * (c[1] * c[3] - c[0] * c[2])
        + v[1] * 2 * (c[2] * c[3] + c[0] * c[1])
        + v[2] * (c[0] ** 2 - c[1] ** 2 - c[2] ** 2 + c[3] ** 2),
    ])


class BalanceWBC:
    """Steps GR00T's balance policy and returns leg and waist joint targets.

    One instance per run. Feed it the robot's state at the policy's own rate
    (50 Hz: the reference runs 5 ms physics with a decimation of 4) and apply
    what it returns as position targets, with the gains it was trained under.
    """

    def __init__(self, height_cmd=None):
        cfg = yaml.safe_load(open(os.path.join(WBC, "g1_gear_wbc.yaml")))
        self.cfg = cfg
        self.n_act = int(cfg["num_actions"])            # 12 legs + 3 waist
        self.hist_len = int(cfg["obs_history_len"])
        self.obs_dim = int(cfg["num_obs"]) // self.hist_len
        self.default = np.asarray(cfg["default_angles"], np.float32)
        self.kps = np.asarray(cfg["kps"], np.float32)
        self.kds = np.asarray(cfg["kds"], np.float32)
        self.action_scale = float(cfg["action_scale"])
        self.dof_pos_scale = float(cfg["dof_pos_scale"])
        self.dof_vel_scale = float(cfg["dof_vel_scale"])
        self.ang_vel_scale = float(cfg["ang_vel_scale"])
        self.cmd_scale = np.asarray(cfg["cmd_scale"], np.float32)
        # The navigation command, [vx, vy, wz]. This is the whole difference
        # between walking and standing: g1_gear_wbc_policy.py picks the policy
        # off its magnitude, on the same observation, with no reset in between.
        #
        #     if np.linalg.norm(self.cmd) < 0.05:
        #         policy = self.policy_1      # standing
        #     else:
        #         policy = self.policy_2      # walking
        #
        # So there is no walk phase and no pick phase to join -- there is one
        # loop whose command decays to zero on arrival. Pinning this to zero,
        # the way this file used to, is what left the walk to a separate clip
        # and put a seam between them.
        self.loco_cmd = np.zeros(3, np.float32)
        self.SWITCH = 0.05
        self.height_cmd = float(cfg["height_cmd"] if height_cmd is None
                                else height_cmd)
        self.rpy_cmd = np.asarray(cfg.get("rpy_cmd", [0.0, 0.0, 0.0]), np.float32)

        self.stand_sess = ort.InferenceSession(
            os.path.join(WBC, cfg["policy_path"]),
            providers=["CPUExecutionProvider"])
        self.walk_sess = ort.InferenceSession(
            os.path.join(WBC, cfg["walk_policy_path"]),
            providers=["CPUExecutionProvider"])
        self.in_name = self.stand_sess.get_inputs()[0].name
        self.walking = False

        self.action = np.zeros(self.n_act, np.float32)
        self.hist = [np.zeros(self.obs_dim, np.float32)] * self.hist_len
        self.target = self.default.copy()

    def _observe(self, qj, dqj, quat_wxyz, omega_body):
        n = len(qj)
        cmd = np.zeros(7, np.float32)
        cmd[:3] = self.loco_cmd * self.cmd_scale
        cmd[3] = self.height_cmd
        cmd[4:7] = self.rpy_cmd

        padded = np.zeros(n, np.float32)
        padded[:len(self.default)] = self.default

        o = np.zeros(self.obs_dim, np.float32)
        o[0:7] = cmd
        o[7:10] = np.asarray(omega_body, np.float32) * self.ang_vel_scale
        o[10:13] = _quat_rotate_inverse(quat_wxyz, np.array([0.0, 0.0, -1.0]))
        o[13:13 + n] = (qj - padded) * self.dof_pos_scale
        o[13 + n:13 + 2 * n] = dqj * self.dof_vel_scale
        o[13 + 2 * n:13 + 2 * n + self.n_act] = self.action
        return o

    def step(self, qj, dqj, quat_wxyz, omega_body, nav_cmd=None):
        """qj/dqj: all 29 joints in LEG_WAIST + arm order. Returns 15 targets.

        nav_cmd is [vx, vy, wz] in the body frame. Give it a velocity and the
        robot walks; let it fall below 0.05 and the same observation goes to
        the standing policy instead.
        """
        if nav_cmd is not None:
            self.loco_cmd = np.asarray(nav_cmd, np.float32)
        self.hist = self.hist[1:] + [self._observe(qj, dqj, quat_wxyz, omega_body)]
        obs = np.concatenate(self.hist)[None].astype(np.float32)
        self.walking = bool(np.linalg.norm(self.loco_cmd) >= self.SWITCH)
        sess = self.walk_sess if self.walking else self.stand_sess
        self.action = sess.run(None, {self.in_name: obs})[0].squeeze()
        self.target = self.action * self.action_scale + self.default
        return self.target
