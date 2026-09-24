"""SONIC's tracking controller, so the robot executes a motion instead of miming it.

Up to now this pipeline used SONIC's *planner* -- gen_planner_motion.py makes a
reference clip -- and then replayed that clip kinematically, writing the root
pose and joint angles straight into the simulator. The robot was never
balancing; it was a puppet. That is why the pick had to weld the pelvis to the
world, why unwelding it fell over, and why the join between the two phases
looked like a teleport.

SONIC is a motion *tracking* model. GR00T-WholeBodyControl's README: "SONIC
uses motion tracking as a scalable training task", and the deployment stack
"plays back pre-loaded reference motions -- sequences of joint positions,
velocities, and full body kinematics that the policy tracks". Give it the
reference and it produces the whole body, 29 joints, at 50 Hz.

Everything here is read off the deployment implementation, not guessed:

  policy/release/model_encoder.onnx     1762 -> 64  latent motion token
  policy/release/model_decoder.onnx      994 -> 29  joint actions
  policy/release/observation_config.yaml            which fields, in order
  src/g1/g1_deploy_onnx_ref/include/policy_parameters.hpp
                                        default angles, action scale, gains
  src/g1/g1_deploy_onnx_ref/src/g1_deploy_onnx_ref.cpp
                                        each field's width, and
                                        q_target = default + action * scale
"""
import os

import numpy as np
import onnxruntime as ort

DEPLOY = "/home/sehoon/Projects/GR00T-WholeBodyControl/gear_sonic_deploy"
POLICY = os.path.join(DEPLOY, "policy", "release")

# IsaacLab joint order, the order every SONIC vector is in.
JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
N = len(JOINTS)

DEFAULT = np.array([
    -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
    -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
    0.0, 0.0, 0.0,
    0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
    0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
], np.float64)

_W = 10.0 * 2.0 * np.pi                 # natural frequency, 10 Hz
_ARM = {"5020": 0.003609725, "7520_14": 0.010177520,
        "7520_22": 0.025101925, "4010": 0.00425}
_EFF = {"5020": 25.0, "7520_14": 88.0, "7520_22": 139.0, "4010": 5.0}
_MOTOR = (["7520_22", "7520_22", "7520_14", "7520_22", "5020", "5020"] * 2
          + ["7520_14", "5020", "5020"]
          + ["5020"] * 4 + ["5020", "4010", "4010"]
          + ["5020"] * 4 + ["5020", "4010", "4010"])
STIFFNESS = np.array([_ARM[m] * _W * _W for m in _MOTOR])
DAMPING = np.array([2.0 * 2.0 * _ARM[m] * _W for m in _MOTOR])
# 0.25 * effort_limit / stiffness, as policy_parameters.hpp computes it.
ACTION_SCALE = np.array([0.25 * _EFF[m] / (_ARM[m] * _W * _W) for m in _MOTOR])
# Ankle pitch and roll are driven at twice the nominal stiffness.
for _i in (4, 5, 10, 11):
    STIFFNESS[_i] *= 2.0
    DAMPING[_i] *= 2.0

CONTROL_DT = 0.02                       # 50 Hz, the controller's own rate
HIST = 10                               # history frames, step 1
LOOK = 10                               # reference lookahead frames
STRIDE = 5                              # ...every 5th frame (step5)

# Encoder input, in the order observation_config.yaml lists the enabled fields.
_ENC = [("encoder_mode_4", 4),
        ("motion_joint_positions_10frame_step5", LOOK * N),
        ("motion_joint_velocities_10frame_step5", LOOK * N),
        ("motion_root_z_position_10frame_step5", LOOK),
        ("motion_root_z_position", 1),
        ("motion_anchor_orientation", 6),
        ("motion_anchor_orientation_10frame_step5", LOOK * 6),
        ("motion_joint_positions_lowerbody_10frame_step5", 120),
        ("motion_joint_velocities_lowerbody_10frame_step5", 120),
        ("vr_3point_local_target", 9),
        ("vr_3point_local_orn_target", 12),
        ("smpl_joints_10frame_step1", 720),
        ("smpl_anchor_orientation_10frame_step1", 60),
        ("motion_joint_positions_wrists_10frame_step1", 60)]
ENC_DIM = sum(d for _, d in _ENC)       # 1762

# Decoder input, same source.
_DEC = [("token_state", 64),
        ("his_base_angular_velocity_10frame_step1", HIST * 3),
        ("his_body_joint_positions_10frame_step1", HIST * N),
        ("his_body_joint_velocities_10frame_step1", HIST * N),
        ("his_last_actions_10frame_step1", HIST * N),
        ("his_gravity_dir_10frame_step1", HIST * 3)]
DEC_DIM = sum(d for _, d in _DEC)       # 994

# The lower body the encoder asks for: 12 leg joints over 10 frames.
LOWER = list(range(12))


def _to_il(v_hw):
    """Hardware/MuJoCo-ordered joint vector -> IsaacLab order."""
    return np.array([v_hw[ISAACLAB_TO_MUJOCO[i]] for i in range(len(v_hw))])


def quat_rotate_inverse(q_wxyz, v):
    """Rotate v into the body frame of q. Same form as the deploy code."""
    w, x, y, z = q_wxyz
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


def quat_mul(a, b):
    """Hamilton product, w x y z -- quat_mul_d in the deployment."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([aw*bw - ax*bx - ay*by - az*bz,
                     aw*bx + ax*bw + ay*bz - az*by,
                     aw*by - ax*bz + ay*bw + az*bx,
                     aw*bz + ax*by - ay*bx + az*bw])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_to_6d(q_wxyz):
    """First two columns of the rotation matrix, flattened ROW-WISE.

    g1_deploy_onnx_ref.cpp:679, with its own note that this matches
    ``.as_matrix()[..., :2].reshape(1, -1)``:

        {R[0][0], R[0][1],  R[1][0], R[1][1],  R[2][0], R[2][1]}

    This file used to return the same six numbers column-wise, which is a
    different vector for every rotation that is not symmetric.
    """
    w, x, y, z = q_wxyz
    r00 = 1 - 2 * (y * y + z * z)
    r01 = 2 * (x * y - w * z)
    r10 = 2 * (x * y + w * z)
    r11 = 1 - 2 * (x * x + z * z)
    r20 = 2 * (x * z - w * y)
    r21 = 2 * (y * z + w * x)
    return np.array([r00, r01, r10, r11, r20, r21])


class SonicTracker:
    """Runs SONIC over a reference motion and returns 29 joint targets.

    reference: (T, 29) joint positions at 50 Hz, IsaacLab order.
    ref_quat:  (T, 4) root orientation, w x y z. Optional; identity if absent.
    ref_root_z:(T,) root height. Optional; the standing height if absent.
    """

    def __init__(self, reference, ref_quat=None, ref_root_z=None):
        self.ref = np.asarray(reference, np.float64)
        self.T = len(self.ref)
        self.ref_vel = np.zeros_like(self.ref)
        self.ref_vel[1:] = (self.ref[1:] - self.ref[:-1]) / CONTROL_DT
        self.ref_il = np.stack([_to_il(r) for r in self.ref])
        self.ref_vel_il = np.stack([_to_il(r) for r in self.ref_vel])
        self.ref_quat = (np.tile([1.0, 0, 0, 0], (self.T, 1))
                         if ref_quat is None else np.asarray(ref_quat, np.float64))
        self.ref_z = (np.full(self.T, 0.78) if ref_root_z is None
                      else np.asarray(ref_root_z, np.float64))

        self.enc = ort.InferenceSession(
            os.path.join(POLICY, "model_encoder.onnx"),
            providers=["CPUExecutionProvider"])
        self.dec = ort.InferenceSession(
            os.path.join(POLICY, "model_decoder.onnx"),
            providers=["CPUExecutionProvider"])
        self.enc_in = self.enc.get_inputs()[0].name
        self.dec_in = self.dec.get_inputs()[0].name

        self.h_q = [np.zeros(N) for _ in range(HIST)]
        self.h_dq = [np.zeros(N) for _ in range(HIST)]
        self.h_act = [np.zeros(N) for _ in range(HIST)]
        self.h_omega = [np.zeros(3) for _ in range(HIST)]
        self.h_grav = [np.array([0.0, 0.0, -1.0]) for _ in range(HIST)]
        self.action = np.zeros(N)
        # The externally-commanded upper body, 17 values in IsaacLab order, or
        # None to track the reference's own arms.
        self.upper = None

    def set_upper_body(self, vals_il_17):
        """Run a planned arm over a walking reference.

        g1_deploy_onnx_ref.cpp:780, inside the routine that gathers the
        encoder's future frames:

            if (has_upper_body_data_) {
              for (size_t i = 0; i < 17; i++) {
                current_motion_joint_pos[
                    upper_body_joint_isaaclab_order_in_isaaclab_index[i]]
                  = upper_body_joint_positions_buffer_[i];
              }
            }

        So the arm command does not get blended in afterwards and it is not a
        second reference to join: every frame the encoder looks at already has
        the commanded arm in it. The legs keep walking to the planner's clip
        and the arm does what cuRobo asked, in one motion.
        """
        self.upper = (None if vals_il_17 is None
                      else np.asarray(vals_il_17, np.float64))

    def _future(self, t, arr, width, upper=False):
        """LOOK frames from t, every STRIDE, clamped at the end of the clip."""
        out = np.zeros((LOOK, width))
        for k in range(LOOK):
            i = min(t + k * STRIDE, self.T - 1)
            out[k] = arr[i]
        if upper and self.upper is not None:
            out[:, UPPER_BODY_IL] = self.upper
        return out.reshape(-1)

    def _encode(self, t, base_quat):
        v = np.zeros(ENC_DIM, np.float32)
        o = {}
        p = 0
        for name, d in _ENC:
            o[name] = (p, d)
            p += d

        def put(name, vals):
            s, d = o[name]
            a = np.asarray(vals, np.float32).reshape(-1)
            v[s:s + min(d, a.size)] = a[:d]

        # encoder_mode_4 is the mode ID as a scalar with three zeros after it,
        # not a one-hot -- GatherEncoderMode writes
        # buf[offset] = GetEncodeMode() and zeroes the rest. g1 is mode 0, so
        # this field is all zeros. Writing a one-hot puts a 1 in the first slot,
        # which says mode 1 (teleop) while the rest of the vector is g1 data,
        # and the token that comes back is meaningless: measured, the first
        # action asked the joints for 4.0 rad off the default and the robot
        # exploded.
        # g1 mode fills FOUR fields and no others. observation_config.yaml:
        #
        #   encoder_modes:
        #     - name: "g1"
        #       mode_id: 0
        #       required_observations:
        #         - encoder_mode_4
        #         - motion_joint_positions_10frame_step5
        #         - motion_joint_velocities_10frame_step5
        #         - motion_anchor_orientation_10frame_step5
        #
        # The root-height, lower-body, vr_3point, smpl and wrist slots belong
        # to the teleop and smpl modes. This file used to fill five of them
        # with real numbers while running as g1, which puts data where the
        # model was trained to see zeros.
        #
        # encoder_mode_4 is the mode ID as a scalar with three zeros after it,
        # not a one-hot -- GatherEncoderMode writes buf[offset] =
        # GetEncodeMode() and zeroes the rest. g1 is mode 0, so the field is
        # all zeros; a one-hot there says mode 1, teleop.
        put("encoder_mode_4", [0.0, 0.0, 0.0, 0.0])
        put("motion_joint_positions_10frame_step5",
            self._future(t, self.ref_il, N, upper=True))
        put("motion_joint_velocities_10frame_step5",
            self._future(t, self.ref_vel_il, N))
        # The anchor orientation is the reference's root rotation seen from
        # the robot's own base, not the reference's world rotation.
        # g1_deploy_onnx_ref.cpp:673, orientation_mode 0:
        #
        #     base_to_ref_quat = quat_mul_d(quat_conjugate_d(base_quat),
        #                                   new_ref_root_rot);
        #
        # Feeding the world quaternion instead hands the policy the robot's
        # whole heading change as an orientation error -- this walk turns
        # about 60 degrees -- and it spends that on the legs and waist.
        # apply_delta_heading is identity here: the reference is generated in
        # the same world frame the robot stands in, so there is no motion
        # frame to align.
        put("motion_anchor_orientation_10frame_step5",
            np.stack([quat_to_6d(quat_mul(
                quat_conj(base_quat),
                self.ref_quat[min(t + k * STRIDE, self.T - 1)]))
                for k in range(LOOK)]))
        return self.enc.run(None, {self.enc_in: v[None]})[0].reshape(-1)

    def prime(self, q, dq, quat_wxyz, omega_body):
        """Fill the history with the state the robot is actually in.

        The deployment runs a state logger and only activates the policy once
        GetLatest() has real frames to return. Starting with ten frames of
        zeros tells the policy the robot was at its default pose with no
        gravity a fifth of a second ago, and the first action it returns is
        enormous -- measured, 3.6 rad off the default, which in a simulator is
        an explosion.
        """
        grav = quat_rotate_inverse(quat_wxyz, np.array([0.0, 0.0, -1.0]))
        self.h_q = [_to_il(np.asarray(q) - DEFAULT) for _ in range(HIST)]
        self.h_dq = [_to_il(np.asarray(dq)) for _ in range(HIST)]
        self.h_omega = [np.asarray(omega_body).copy() for _ in range(HIST)]
        self.h_grav = [grav.copy() for _ in range(HIST)]
        self.h_act = [np.zeros(N) for _ in range(HIST)]

    def step(self, t, q, dq, quat_wxyz, omega_body):
        """One 50 Hz tick. Returns 29 joint position targets."""
        grav = quat_rotate_inverse(quat_wxyz, np.array([0.0, 0.0, -1.0]))
        # The model speaks IsaacLab order; DEFAULT, ACTION_SCALE and JOINTS
        # here are the hardware order the deployment's own arrays are written
        # in (g1_deploy_onnx_ref.cpp indexes default_angles with the hardware
        # index and the model output with isaaclab_to_mujoco). Reorder on the
        # way in, and back on the way out.
        self.h_q = self.h_q[1:] + [_to_il(np.asarray(q) - DEFAULT)]
        self.h_dq = self.h_dq[1:] + [_to_il(np.asarray(dq))]
        self.h_omega = self.h_omega[1:] + [np.asarray(omega_body)]
        self.h_grav = self.h_grav[1:] + [grav]

        token = self._encode(t, np.asarray(quat_wxyz, np.float64))
        v = np.concatenate([
            token,
            np.concatenate(self.h_omega),
            np.concatenate(self.h_q),
            np.concatenate(self.h_dq),
            np.concatenate(self.h_act),
            np.concatenate(self.h_grav),
        ]).astype(np.float32)
        assert v.size == DEC_DIM, f"decoder input {v.size}, expected {DEC_DIM}"

        self.action = self.dec.run(None, {self.dec_in: v[None]})[0].reshape(-1)
        self.h_act = self.h_act[1:] + [self.action.copy()]
        act_hw = np.array([self.action[MUJOCO_TO_ISAACLAB[i]] for i in range(N)])
        return DEFAULT + act_hw * ACTION_SCALE


# The two index maps, from the deployment's own policy_parameters.hpp. They
# were named backwards here, and one of the three uses read the wrong one.
#
# policy_parameters.hpp lists the same 17 upper-body joints twice, once with
# each indexing, which pins the direction down with no guessing:
#
#   upper_body_joint_mujoco_order_in_mujoco_index   = {12,13,14,...,28}
#   upper_body_joint_mujoco_order_in_isaaclab_index = { 2, 5, 8,...,28}
#
# so MuJoCo 12 (waist_yaw) is IsaacLab 2. IsaacLab's order is not
# legs-waist-arms; it is breadth-first, left and right interleaved.
ISAACLAB_TO_MUJOCO = [0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10,
                      16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28]
MUJOCO_TO_ISAACLAB = [0, 3, 6, 9, 13, 17, 1, 4, 7, 10, 14, 18, 2, 5, 8,
                      11, 15, 19, 21, 23, 25, 27, 12, 16, 20, 22, 24, 26, 28]

# The 17 joints an external arm command replaces in the reference, IsaacLab
# indices, verbatim from policy_parameters.hpp:80. This is how GR00T runs a
# planned arm over a walking reference -- g1_deploy_onnx_ref.cpp:780 swaps
# them in while it gathers the encoder's future frames, so there is no
# separate reference to build and no seam to join.
UPPER_BODY_IL = [2, 5, 8, 11, 12, 15, 16, 19, 20, 21, 22, 23, 24, 25, 26,
                 27, 28]


def clip_to_reference(clip, fps):
    """A walk clip as a SONIC reference: (T50, 29) hardware order at 50 Hz.

    Returns joint positions, root quaternion (w x y z) and root height, all
    resampled onto the controller's clock. motion_reference.md: "Each row is
    one timestep at 50 Hz".
    """
    dof = np.asarray(clip["dof"], np.float64)            # (T, 29) MuJoCo order
    root = np.asarray(clip["root_trans_offset"], np.float64)
    quat_xyzw = np.asarray(clip["root_rot"], np.float64)  # clips store xyzw
    n = len(dof)
    # Stays in MuJoCo order. SonicTracker takes its reference in the hardware
    # order the deployment's own default_angles and action_scale are written
    # in and converts with _to_il; reordering here as well converted it twice.
    iso = dof

    src = np.arange(n) / float(fps)
    n50 = max(2, int(round(n / float(fps) / CONTROL_DT)))
    dst = np.arange(n50) * CONTROL_DT
    pos = np.stack([np.interp(dst, src, iso[:, c]) for c in range(N)], axis=1)
    z = np.interp(dst, src, root[:, 2])
    q = np.stack([np.interp(dst, src, quat_xyzw[:, c]) for c in range(4)], axis=1)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    return pos, q[:, [3, 0, 1, 2]], z                     # xyzw -> wxyz
    # pos is MuJoCo/hardware order, matching JOINTS/DEFAULT above.
