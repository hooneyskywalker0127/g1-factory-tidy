"""From the pose a clip ends in, can the whole body put the right palm on a grasp?

The arm-only planner stops 43-71 mm short of a box on the floor even with
the pelvis dropped to 0.37 m and the waist at its limit. GR00T's planner has
the poses that get a person's hand to the floor -- squat, kneel on one knee,
kneel on both -- and cuRobo's MotionRetargeter (unitree_g1_29dof_retarget.yml,
35 DOF with the six base joints) solves the whole body at once. So: take the
clip's last frame as the body, hold the feet where the clip put them, leave
the pelvis and torso loose, and ask for the wrist on each grasp.

    python grasp/reach_from_pose.py CLIP.pkl GRASPS.json CAPTURE_DIR [--out SEQ.npz]

Prints the wrist error per grasp. With --out, solves the reach as a sequence
(pre-grasp above the grasp -> grasp -> lift) for the best grasp and writes the
35-DOF frames, which is the shape a SONIC reference wants.
"""
import json
import math
import os
import sys

import joblib
import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from retarget_whole_body import quat_to_mat, mat_to_quat  # noqa: E402
from curobo.motion_retargeter import (MotionRetargeter, MotionRetargeterCfg,  # noqa: E402
                                      SequenceGoalToolPose)
from curobo.types import JointState, ToolPoseCriteria  # noqa: E402


def build():
    """Feet hold the floor; the wrist does the work; everything else, the
    pelvis included, is loose. The kneel is the planner's, but the last few
    centimetres of a reach to the floor come from the body leaning over the
    hand, and a pelvis held at full weight kept the wrist 17 mm off."""
    hold = ["left_ankle_roll_link", "right_ankle_roll_link"]
    work = ["right_wrist_yaw_link"]
    loose = ["pelvis", "torso_link", "left_hip_roll_link", "right_hip_roll_link",
             "left_knee_link", "right_knee_link", "left_shoulder_roll_link",
             "left_elbow_link", "left_wrist_yaw_link", "right_shoulder_roll_link",
             "right_elbow_link"]
    if os.environ.get("BIMANUAL") == "1":          # a crate between both palms
        work = ["left_wrist_yaw_link", "right_wrist_yaw_link"]
        loose.remove("left_wrist_yaw_link")
    crit = {}
    for n in hold + work:
        crit[n] = ToolPoseCriteria.track_position_and_orientation(
            xyz=[1.0, 1.0, 1.0], rpy=[0.067, 0.067, 0.067])
    # Not one loose weight for everything below the wrist. With the whole
    # body at 0.005 the wrist landed within 9 mm and the body did anything it
    # liked to get there: a leg swung back, the left arm went up, the kneel
    # became a one-legged stand (frame 1320 of the first replay). The guide's
    # split: legs and pelvis matter for balance, the reaching arm's own
    # shoulder and elbow can go where the wrist needs them, the rest stays put.
    w_body = float(os.environ.get("BODY_W", "0.3"))    # pelvis, hips, knees, torso
    w_left = float(os.environ.get("LEFT_W", "0.1"))    # the other arm
    w_arm = float(os.environ.get("ARM_W", "0.02"))     # right shoulder, elbow
    for n in loose:
        lw = (w_arm if n.startswith("right_") and "hip" not in n and "knee" not in n
              else (w_arm if os.environ.get("BIMANUAL") == "1" else w_left)
              if n.startswith("left_") and ("shoulder" in n or "elbow" in n or "wrist" in n)
              else w_body)
        crit[n] = ToolPoseCriteria.track_position_and_orientation(
            xyz=[lw] * 3, rpy=[lw * 0.07] * 3)
    cfg = MotionRetargeterCfg.create(robot=os.environ.get("RETARGET_CFG", "unitree_g1_29dof_retarget.yml"),
                                     tool_pose_criteria=crit, num_envs=1,
                                     self_collision_check=True)
    return MotionRetargeter(cfg), cfg

SWARM = "/home/sehoon/Documents/GitHub/humanoid-swarm-sim"
# wrist -> palm, from the G1 URDF's right_hand_palm_joint (fixed)
WRIST_TO_PALM = np.array([0.0415, -0.003, 0.0])


def _clip_end(path):
    m = list(joblib.load(path).values())[0]
    dof = np.asarray(m["dof"])[-1]                # 29, MuJoCo order
    root = np.asarray(m["root_trans_offset"])[-1]
    q = np.asarray(m["root_rot"])[-1]             # xyzw
    return dof, root, np.array([q[3], q[0], q[1], q[2]])


def _joint_names_mujoco():
    import os as _os
    cwd = _os.getcwd()
    _os.chdir("/home/sehoon/Projects/GR00T-WholeBodyControl")
    try:
        sys.path.insert(0, os.path.join(SWARM, "common"))
        from foot_height import load_urdf
        return load_urdf()[1]
    finally:
        _os.chdir(cwd)


def _rpy_from_quat(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def grasps_in_cell(grasps_json, cap_dir, base_z=0.98):
    import trimesh.transformations as tra
    d = json.load(open(grasps_json))
    g2t = d["grasp_to_tool_transform"]
    T = np.eye(4)
    T[:3, 3] = g2t["translation"]
    qx = g2t["quaternion_xyzw"]
    T[:3, :3] = tra.quaternion_matrix([qx[3], qx[0], qx[1], qx[2]])[:3, :3]
    meta = json.load(open(os.path.join(cap_dir, "meta_data.json")))
    pfc = np.asarray(meta["plan_from_cell"])
    B = np.eye(4)
    B[2, 3] = base_z
    if d.get("frame") == "cell" or "--cell-frame" in sys.argv:
        # already palm poses in the cell (grasp/dexonomy_grasps.py)
        palms = np.array([np.asarray(g) @ T for g in d["grasps"]])
    else:
        palms = np.array([np.linalg.inv(pfc) @ np.linalg.inv(B) @ np.asarray(g) @ T
                          for g in d["grasps"]])
    # GRASP_ROLL_DEG: turn every palm about its own approach axis (+y). On a
    # lying 3.7 cm flashlight the palm came down right, 5 cm above it, with
    # the fingers running ALONG the cylinder and the thumb pushing its side
    # (results/fable7b/snap_12_grasp_A.png); a pinch closes across an axis,
    # not along it. A 90 deg roll about the approach is the test of whether
    # the grasp-to-palm mapping for this hand is a quarter turn off.
    roll = math.radians(float(os.environ.get("GRASP_ROLL_DEG", "0")))
    if roll:
        Ry = np.eye(4)
        Ry[:3, :3] = np.array([[math.cos(roll), 0, math.sin(roll)], [0, 1, 0], [-math.sin(roll), 0, math.cos(roll)]])
        palms = np.array([p @ Ry for p in palms])
        print(f"[reach] every palm rolled {math.degrees(roll):.0f} deg about its approach axis")
    # HAND=inspire: GraspGenX's inspire_hand description has its hand_base
    # frame with the fingers along -y and the index at -z; IsaacLab's G1
    # Inspire (R_hand_base_link) has the fingers along +x and the index at +z.
    # Measured on both (index proximal: GraspGenX (0.0003, -0.1365, -0.0323),
    # IsaacLab (0.1365, -0.0003, 0.0322)). x_i = -y_g, y_i = -x_g, z_i = -z_g.
    # The mount is the same as the Dex3's: hand_base = wrist + (0.0415, -0.003, 0).
    if os.environ.get("HAND") == "inspire":
        M = np.eye(4); M[:3, :3] = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], float).T
        # The exported grasps are in GraspGen's gripper convention frame (gripper.urdf's `world` link:
        # fingers along +z, fingertip at z 0.15, config.json), NOT in hand_base_link. hand_base sits at
        # world_joint xyz (0.065, -0.01, 0) rpy (1.5708, 2.356194, 0) inside that frame. Without this
        # factor every Inspire grasp was executed with the hand turned ~90 deg (GraspGen's fingertip
        # landed at Isaac palm (0, 0, -0.15) instead of (0.152, 0.06, 0.01), checked against points.json
        # and the measured finger links, 2026-09-27 15:50) -- the two "holds" were scoops by luck.
        from scipy.spatial.transform import Rotation as _Rw
        Twj = np.eye(4); Twj[:3, :3] = _Rw.from_euler("xyz", [1.570796, 2.356194, 0.0]).as_matrix(); Twj[:3, 3] = [0.065, -0.01, 0.0]
        # 2026-09-27 16:30 CORRECTION: grasps_in_cell() already applies the JSON's grasp_to_tool_transform,
        # which IS this world_joint (g1_inspire_arm.yaml), so the palms here are hand_base poses and only M
        # belongs. Applying Twj again (15:50-16:25) turned the palms face-up with the fingertips 10 cm off the
        # floor; the 15:50 note above was wrong about the pipeline, not about the frames. INSPIRE_DOUBLE_MAP=1
        # reproduces the mistake.
        if os.environ.get("INSPIRE_DOUBLE_MAP") == "1":
            M = Twj @ M
        palms = np.array([p_ @ M for p_ in palms])
        print("[reach] Inspire hand: hand_base (grasp_to_tool applied by the loader) -> IsaacLab palm frame")
    # the retarget config's tool frame is the wrist: back off along the palm
    # frame by the fixed palm offset
    P = np.eye(4)
    P[:3, 3] = -WRIST_TO_PALM
    wrists = np.array([p @ P for p in palms])
    return wrists, np.asarray(d["confidence"])


def main():
    clip, grasps_json, cap = sys.argv[1:4]
    out = sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv else None
    # --place X Y Z: not a grasp but a drop -- take the wrist from where the
    # clip leaves it to a point above the crate, hold, and the fingers open
    # (the schedule does the opening). grasps_json and cap are unused then.
    place = ([float(sys.argv[sys.argv.index("--place") + k]) for k in (1, 2, 3)]
             if "--place" in sys.argv else None)
    r, cfg = build()
    jn = list(r.kinematics.joint_names)
    dof, root, quat = _clip_end(clip)
    names = _joint_names_mujoco()
    q = torch.zeros(1, len(jn), dtype=torch.float32, device="cuda")
    for i, n in enumerate(names):
        if n in jn:
            q[0, jn.index(n)] = float(dof[i])
    # cuRobo's floating base is three extra links in a chain, X_ROT then
    # Y_ROT then Z_ROT, so the pelvis rotation it builds is Rx*Ry*Rz --
    # intrinsic XYZ. Fed the usual yaw-pitch-roll decomposition instead, the
    # kneel came out pitched -4.4 deg and rolled -12 where the clip has +9.8
    # and +8.3, and the reach was solved for a body that was not there.
    from scipy.spatial.transform import Rotation as _R
    roll, pitch, yaw = _R.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_euler("XYZ")
    for n, v in (("base_j_x", root[0]), ("base_j_y", root[1]), ("base_j_z", root[2]),
                 ("base_j_xtheta", roll), ("base_j_ytheta", pitch), ("base_j_ztheta", yaw)):
        if n in jn:
            q[0, jn.index(n)] = float(v)
    ks = r.kinematics.compute_kinematics(JointState.from_position(q, joint_names=jn))
    frames = list(ks.tool_frames)
    pick = [frames.index(n) for n in cfg.tool_frames]
    pos0 = ks.tool_poses.position[0, 0, pick].cpu().numpy()      # (L, 3)
    quat0 = ks.tool_poses.quaternion[0, 0, pick].cpu().numpy()
    wi = list(cfg.tool_frames).index("right_wrist_yaw_link")
    print(f"[reach] clip ends: pelvis z {root[2]:.3f}, torso pitch {math.degrees(pitch):+.1f} deg, "
          f"wrist at {np.round(pos0[wi], 3)}")

    def solve(targets, from_here=False, left_targets=None):
        """targets: list of 4x4 wrist poses -> solved joint frames (T, 35), errors.
        left_targets: the same for the left wrist (PIN_JSON: the other hand
        holds the tool's head down while this one takes the handle)."""
        T = len(targets)
        pos = np.repeat(pos0[None], T, axis=0)
        qua = np.repeat(quat0[None], T, axis=0)
        for k, Tw in enumerate(targets):
            pos[k, wi] = Tw[:3, 3]
            qua[k, wi] = mat_to_quat(Tw[:3, :3])
        if left_targets is not None:
            li = list(cfg.tool_frames).index("left_wrist_yaw_link")
            for k, Tw in enumerate(left_targets):
                pos[k, li] = Tw[:3, 3]
                qua[k, li] = mat_to_quat(Tw[:3, :3])
        seq = SequenceGoalToolPose(
            tool_frames=cfg.tool_frames,
            position=torch.tensor(pos, dtype=torch.float32, device="cuda")
            .reshape(T, 1, len(pick), 1, 3).contiguous(),
            quaternion=torch.tensor(qua, dtype=torch.float32, device="cuda")
            .reshape(T, 1, len(pick), 1, 4).contiguous())
        if from_here:
            # Stream the frames through solve_frame with the warm start seeded
            # by the pose the clip ends in, so every frame is the velocity-
            # limited local IK the retargeter uses after its first frame --
            # from the kneel, not from whichever of 64 seeds the global IK
            # liked (it liked standing up: a 259 mm, 218 deg seam).
            from curobo._src.types.tool_pose import GoalToolPose
            r.reset()
            r._prev_solution = q.clone().view(1, -1)
            sols = []
            for k in range(T):
                g = GoalToolPose(tool_frames=cfg.tool_frames,
                                 position=seq.position[k:k + 1].permute(1, 0, 2, 3, 4).contiguous(),
                                 quaternion=seq.quaternion[k:k + 1].permute(1, 0, 2, 3, 4).contiguous())
                sols.append(r.solve_frame(g).joint_state.position.view(-1).cpu().numpy())
            sol = np.stack(sols)
        else:
            res = r.solve_sequence(seq)
            sol = res.joint_state.position.view(T, -1).cpu().numpy()
        ks2 = r.kinematics.compute_kinematics(JointState.from_position(
            torch.tensor(sol, dtype=torch.float32, device="cuda"), joint_names=jn))
        got = ks2.tool_poses.position[:, 0, pick[wi]].cpu().numpy() if ks2.tool_poses.position.dim() == 4 \
            else ks2.tool_poses.position.view(T, -1, 3)[:, pick[wi]].cpu().numpy()
        err = np.linalg.norm(got - pos[:, wi], axis=1)
        return sol, err

    # --crate CRATE.json: the lift of a crate seen on the floor (crate_target.py).
    # Both palms go to the two long faces at half the crate's height, from
    # 8 cm outside, then 2 cm into the faces (the squeeze the PD arms hold),
    # then straight up. Palm +y (its flat side, the thumb's side) faces the
    # crate, fingers (+x) point down; the hands stay open the whole time:
    # a box is carried between two palms, not gripped (PhysHSI, VisualMimic).
    def solve_multi(steps):
        T = len(steps)
        pos = np.repeat(pos0[None], T, axis=0); qua = np.repeat(quat0[None], T, axis=0)
        for t, st in enumerate(steps):
            for k, M in st.items():
                pos[t, k] = M[:3, 3]; qua[t, k] = mat_to_quat(M[:3, :3])
        from curobo._src.types.tool_pose import GoalToolPose
        r.reset(); r._prev_solution = q.clone().view(1, -1)
        sols = []
        for t in range(T):
            g = GoalToolPose(tool_frames=cfg.tool_frames,
                             position=torch.tensor(pos[t], dtype=torch.float32, device="cuda").reshape(1, 1, len(pick), 1, 3).contiguous(),
                             quaternion=torch.tensor(qua[t], dtype=torch.float32, device="cuda").reshape(1, 1, len(pick), 1, 4).contiguous())
            sols.append(r.solve_frame(g).joint_state.position.view(-1).cpu().numpy())
        sol = np.stack(sols)
        ks2 = r.kinematics.compute_kinematics(JointState.from_position(torch.tensor(sol, dtype=torch.float32, device="cuda"), joint_names=jn))
        got = ks2.tool_poses.position
        got = got[:, 0] if got.dim() == 4 else got.view(T, -1, 3)
        errs = []
        for t, st in enumerate(steps):
            errs.append(max(float(np.linalg.norm(got[t, pick[k]].cpu().numpy() - pos[t, k])) for k in st))
        # frames where a hand is far off (the left palm was seen 10 cm low)
        for k in steps[0]:
            e = np.array([np.linalg.norm(got[t, pick[k]].cpu().numpy() - pos[t, k]) for t in range(T)])
            bad = np.where(e > 0.02)[0]
            print(f"[reach]   {cfg.tool_frames[k]}: {len(bad)} frames over 20 mm" + (f", worst frame {int(e.argmax())} at {e.max()*1000:.0f} mm" if len(bad) else ""))
        # per hand, at the last frame that is not a lift: which wrist misses
        t = len(steps) - 1
        gq = ks2.tool_poses.quaternion
        gq = gq[:, 0] if gq.dim() == 4 else gq.view(T, -1, 4)
        from scipy.spatial.transform import Rotation as _R
        for k in steps[t]:
            d = got[t, pick[k]].cpu().numpy() - pos[t, k]
            qg = gq[t, pick[k]].cpu().numpy(); qt = qua[t, k]
            ang = np.degrees((_R.from_quat([qg[1], qg[2], qg[3], qg[0]]).inv() * _R.from_quat([qt[1], qt[2], qt[3], qt[0]])).magnitude())
            print(f"[reach]   {cfg.tool_frames[k]}: last-frame error {np.linalg.norm(d)*1000:5.1f} mm  (dx {d[0]*1000:+.0f} dy {d[1]*1000:+.0f} dz {d[2]*1000:+.0f}), orientation off {ang:5.1f} deg")
        return sol, np.array(errs)

    # --crate-hook CRATE.json: two fingers through each hand slot, curled up
    # behind the bar under the rim, and lift -- form closure, the grip the
    # earlier crate work measured to hold where a wall clamp slipped after
    # 94 mm (humanoid-swarm-sim/common/crate_grip_top.py). Palm frame: fingers
    # (+x) point into the slot along the crate's length, they curl toward +y
    # = up, z = x cross y. The palm stands 7 cm outside the end wall so the
    # last 5 cm of finger are inside; the thumb stays open outside.
    if "--crate-hook" in sys.argv:
        cj = json.load(open(sys.argv[sys.argv.index("--crate-hook") + 1]))
        c = np.array(cj["centre"][:2])
        li = list(cfg.tool_frames).index("left_wrist_yaw_link"); ri = list(cfg.tool_frames).index("right_wrist_yaw_link")
        P = np.eye(4); P[:3, 3] = -WRIST_TO_PALM
        outside = float(os.environ.get("HOOK_OUTSIDE", "0.07")); pre = 0.08; lift = float(os.environ.get("CRATE_LIFT", "0.25"))
        finger_up = 0.016                                     # the fingers sit +1.6 cm along y from the palm centre
        hands = {}
        # HOOK_RIM=1: over the end wall's top edge instead of through the
        # 23 mm slot -- the two wrists land 21 mm off the slots from any kneel
        # tried (0.15..0.25 m back, torso weight 0.1..0.03), and a finger is
        # 20 mm thick. The rim is a line, so 2 cm of error does not matter:
        # the fingers pass 2 cm above it pointing in, then curl DOWN over the
        # edge, and the wall sits in the crook of the fingers for the lift.
        rim = os.environ.get("HOOK_RIM") == "1"
        # HOOK_MODE=pinch: the rim taken between fingers inside and thumb
        # outside, the way a person lifts a crate by its edge. The hand comes
        # straight down from above the wall: fingers (+x) point DOWN inside
        # the crate along the wall, the thumb side (+y) points OUTWARD over the
        # wall, so nothing sweeps the wall on the way in -- measured, the thumb
        # hanging 9 cm under a palm that moved inward is what shoved the crate.
        pinch = os.environ.get("HOOK_MODE") == "pinch"
        for k, name in ((li, "left"), (ri, "right")):
            slot = np.array(cj["slots"][name]); inward = c - slot[:2]; u = np.array([inward[0], inward[1], 0.0]); u /= np.linalg.norm(u)
            if pinch:
                # the left Dex3 is the right one mirrored: its thumb sits on the
                # palm's -y. LEFT_Y_IN=1 turns the left palm's +y toward the crate
                # so that thumb, too, ends up outside the wall (measured: with
                # both palms oriented alike the crate lifted on one side only
                # and fell on its flank).
                y = (u if (name == "left" and os.environ.get("LEFT_Y_IN", "1") == "1") else -u)
                # PINCH_OUTSIDE=1 (Sehoon, 2026-09-28: "왜 안으로 손을 넣냐? 밖으로 손을 넣는게 더 안정적"): the hand
                # stays OUTSIDE the crate -- palm and four fingers on the outer face of the wall, only the thumb
                # reaches over the rim to the inside. Thumb side (+y right / -y left) turns inward, and the palm
                # centre sits PINCH_IN outside the wall instead of inside.
                outside = os.environ.get("PINCH_OUTSIDE", "0") == "1"
                if outside:
                    y = -y
                x = np.array([0.0, 0.0, -1.0]); z = np.cross(x, y)
                T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2] = x, y, z
                insp = os.environ.get("HAND") == "inspire"
                T[:3, 3] = [slot[0], slot[1], cj["top"] + float(os.environ.get("PINCH_ABOVE", "0.115" if insp else "0.03"))]
                # PINCH_ALONG: slide the grip along the wall toward the robot, off the hand-slot hole
                # (the pinch would otherwise close on the 23 mm slot, where there is no wall)
                if os.environ.get("PINCH_ALONG"):
                    tw = np.array([cj["stand"]["x"], cj["stand"]["y"]]) - c; tw /= np.linalg.norm(tw)
                    T[:2, 3] += float(os.environ["PINCH_ALONG"]) * tw
                if insp:
                    # measured on the asset (results/inspire_links_in_palm.json, palm = wrist frame minus the
                    # mount offset): closed, the finger intermediates sit at palm y +0.032 and the thumb distal
                    # at +0.046, 13.5 cm along the fingers; the left hand is the right mirrored in y only. So the
                    # wall belongs 4 cm on the thumb side of the palm centre. Right palm: +y = outward, centre
                    # 4 cm INSIDE the wall. Left palm (+y turned inward, LEFT_Y_IN): its thumb side -y is
                    # outward, centre 4 cm inside as well. Open, the thumb tip is 9.3 cm on its side -- 5 cm
                    # outside the wall on the way down; the fingers (y 0) 4 cm inside.
                    # PINCH_IN_LEFT / PINCH_IN_RIGHT override PINCH_IN per hand: measured on the outside pinch (v4 plan)
                    # the left fingers sat 4.8 cm inside their wall, the right only 2.7 cm, and the right hand held
                    # the crate by one finger (Sehoon, 260928/5지/crate/v10 screenshot)
                    _pin = float(os.environ.get(f"PINCH_IN_{name.upper()}", os.environ.get("PINCH_IN", "0.04")))
                    T[:3, 3] += (-1.0 if outside else 1.0) * _pin * u
                else:
                    # the pinch point sits 4.3 cm from the palm centre on the THUMB
                    # side (+y of the right palm). For the mirrored left palm with
                    # its +y turned inward, that point is 4.3 cm further in -- so
                    # the left palm centre goes the same distance OUTSIDE the wall.
                    # (Measured: right hand on its wall and lifting, left hand
                    # 7 cm inside its wall pinching air, results/crate/pinch4_close_A.png.)
                    sgn = -1.0 if (name == "left" and os.environ.get("LEFT_Y_IN", "1") == "1") else 1.0
                    T[:3, 3] += sgn * (float(os.environ.get("PINCH_IN", "0.01")) + finger_up) * u
                hands[k] = (T @ P, np.array([0.0, 0.0, -1.0]), u)    # the "approach" is straight down; u = inward
                continue
            x = u; y = np.array([0.0, 0.0, -1.0 if rim else 1.0]); z = np.cross(x, y)
            if name == "left" and os.environ.get("HAND") == "inspire":
                y = -y; z = np.cross(x, y)             # the left Inspire curls toward its palm's -y (mirrored in y only)
            T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2] = x, y, z
            if os.environ.get("HAND") == "inspire":
                finger_up = 0.0                         # the Inspire fingers lie in the palm plane (y 0)
            zc = (cj["top"] + float(os.environ.get("HOOK_RIM_ABOVE", "0.05")) + finger_up) if rim else (slot[2] - finger_up)
            T[:3, 3] = [slot[0], slot[1], zc]; T[:3, 3] -= outside * u
            hands[k] = (T @ P, u, u)
        # PINCH_SLIDE (pinch only): come down PINCH_SLIDE m further inside the wall, then slide outward to the
        # grasp so the fingers meet the inner face before the thumb closes -- the plain descent decided by
        # chance whether the fingers landed inside the wall (v1: lift 4 cm) or on the rim top (v4 tester: pried open)
        slide = float(os.environ.get("PINCH_SLIDE", "0")) if pinch else 0.0
        n_over, n_down, n_in, n_out, n_hold, n_lift = 45, 30, 30, (20 if slide else 0), 30, 45
        over_z = cj["top"] + 0.12
        steps = []
        for i in range(n_over + n_down + n_in + n_out + n_hold + n_lift):
            st = {}
            for k, (G, u, uin) in hands.items():
                T0 = np.eye(4); T0[:3, 3] = pos0[k]; T0[:3, :3] = quat_to_mat(quat0[k])
                Gd = G.copy(); Gd[:3, 3] += slide * uin                       # the descent's target: deeper inside
                pre_p = Gd.copy(); pre_p[:3, 3] -= pre * u
                # PINCH_OVER (default 0.20): how high above the grasp the hands travel out over the crate. With the
                # outside grip the right hand's straight path from the kneel pose to its (wider) over point crossed
                # the crate's near corner and turned the crate 18 deg before the descent (260928/5지/crate/v2).
                over = pre_p.copy(); over[2, 3] = over_z if not pinch else Gd[2, 3] + float(os.environ.get("PINCH_OVER", "0.20"))
                if i < n_over:
                    a = (i + 1) / n_over; M = over.copy()
                    # rise first (the first third goes mostly up), then travel out: a bent path, not the straight line
                    au = min(1.0, a * 3.0); M[:3, 3] = T0[:3, 3] + (over[:3, 3] - T0[:3, 3]) * a; M[2, 3] = T0[2, 3] + (over[2, 3] - T0[2, 3]) * max(a, au)
                    if a < 0.34: M[:3, :3] = T0[:3, :3]
                elif i < n_over + n_down:
                    a = (i + 1 - n_over) / n_down; M = pre_p.copy(); M[:3, 3] = over[:3, 3] + (pre_p[:3, 3] - over[:3, 3]) * a
                elif i < n_over + n_down + n_in:
                    a = (i + 1 - n_over - n_down) / n_in; M = Gd.copy(); M[:3, 3] -= pre * (1 - a) * u
                elif i < n_over + n_down + n_in + n_out:
                    a = (i + 1 - n_over - n_down - n_in) / n_out; M = G.copy(); M[:3, 3] = Gd[:3, 3] + (G[:3, 3] - Gd[:3, 3]) * a
                elif i < n_over + n_down + n_in + n_out + n_hold:
                    M = G.copy()
                else:
                    a = (i + 1 - n_over - n_down - n_in - n_out - n_hold) / n_lift; M = G.copy(); M[2, 3] += lift * a
                st[k] = M
            steps.append(st)
        sol, err = solve_multi(steps)
        n_go = n_over + n_down + n_in + n_out
        print(f"[reach] crate hook: {len(steps)} frames, wrist error mean {err.mean()*1000:.1f} mm, max {err.max()*1000:.1f} mm; "
              f"at the slots {err[n_go-5:n_go].mean()*1000:.1f} mm; pelvis z {sol[:, jn.index('base_j_z')].min():.3f}..{sol[:, jn.index('base_j_z')].max():.3f}")
        np.savez(sys.argv[sys.argv.index("--out") + 1], q=sol[None], err=err[None], joint_names=np.array(jn), n_go=n_go,
                 n_lift=n_lift, grasps=np.eye(4)[None], conf=np.array([1.0]), close_from=n_go, lift_from=n_go + n_hold)
        print(f"[reach] wrote {sys.argv[sys.argv.index('--out') + 1]}")
        return

    # --crate-grasps CRATE.json: the crate lifted with GraspGen-X's OWN candidates (grasps_json), one per hand, instead
    # of hand-designed pinch/hook geometry (Sehoon 2026-09-28: "파지점을 찾을 거 아니야? 확률 기반 아님? 더 높은 걸
    # 하는 거지"). Candidates are split into the robot's left/right side of the crate, kept if they sit on the
    # upper part (z > CRATE_GRASP_ZMIN * top), ranked by confidence, and the top PAIRS are solved for both hands
    # (rise first, descend, hold, lift); every pair is written so the physics tester can pick the one that holds.
    # --power-grasp PART_CAP: a POWER grasp of the handle instead of GraspGen-X's fingertip pinch. Sehoon (13:41):
    # "손가락 두세 개로만 잡고 있어. 손바닥에 쥐는 게 아니라 ... 이건 잡는 게 아니라 집는 거잖아" -- every Inspire candidate
    # GraspGen-X proposes for this hand puts the object at the fingertips (its sweep volume sits 11.8-13.5 cm from
    # the hand base) and 0 of 156 hammer candidates have all four fingers on the object; the drill and the hammer
    # both slid out of two-finger pinches during the stand-up. A person lays the palm on the handle and wraps the
    # fingers round it. Here: the handle axis comes from the language part (find_part), the palm faces DOWN
    # (palm +y = -z world), the fingers point horizontally ACROSS the handle (palm +x perpendicular to the axis),
    # the spread (palm z) runs along the handle, and the palm's finger base sits POWER_X along the fingers above the
    # handle centre with the handle POWER_Y below the palm face. Candidates: POWER_FRACS along the handle x two
    # finger directions; approach straight down from 12 cm, hold, lift. Written in the reach_all format.
    if "--power-grasp" in sys.argv:
        from PIL import Image as _Im
        cap_dir = sys.argv[sys.argv.index("--power-grasp") + 1]
        meta = json.load(open(os.path.join(cap_dir, "meta_data.json")))
        seg = np.asarray(_Im.open(os.path.join(cap_dir, "seg.png")), dtype=np.int32); depth = np.load(os.path.join(cap_dir, "depth.npy"))
        sys.path.insert(0, "/home/sehoon/Projects/GraspGenX")
        from graspgenx.utils.scene_loaders import depth_to_camera_xyz, transform_xyz
        xyz = transform_xyz(depth_to_camera_xyz(depth, np.asarray(meta["intrinsics"])), np.asarray(meta["camera_pose"]))
        part = xyz[seg == meta["label_map"].get("obj_part", meta["label_map"]["obj_lang"])]
        whole = xyz[seg == meta["label_map"]["obj_lang"]]
        pc = part.mean(0); hax = np.linalg.svd(part[:, :2] - pc[:2], full_matrices=False)[2][0]; hax3 = np.array([hax[0], hax[1], 0.0])
        t = (part[:, :2] - pc[:2]) @ hax; t0, t1 = float(t.min()), float(t.max())
        if (whole.mean(0)[:2] - pc[:2]) @ hax > 0:                 # +hax runs from the head to the free end
            hax3, t0, t1 = -hax3, -t1, -t0
        top = float(np.percentile(part[:, 2], 90))                  # the handle's upper surface
        radius = float(os.environ.get("POWER_RADIUS", "0.016"))
        # POWER_X / POWER_Y may be comma lists (a grid of finger-base offsets and palm heights). Measured 2026-09-28 on the
        # settled hammer (tester probe + mesh): the handle's underside is 1.5-1.8 cm off the floor, the Inspire fingertip is
        # 1.2-1.5 cm thick, so a finger CAN curl under it -- if the handle sits under the finger base (knuckles at x 0.127),
        # not under the palm heel (x 0.10 put the fingertips on the floor beyond the handle: power_chain, 16/16 LOST).
        pxs = [float(v) for v in str(os.environ.get("POWER_X", "0.10")).split(",")]
        pys = [float(v) for v in str(os.environ.get("POWER_Y", "0.035")).split(",")]
        fracs = [float(v) for v in os.environ.get("POWER_FRACS", "0.35,0.45,0.55,0.65").split(",")]
        wrists, conf = [], []
        for fr, px, py in [(f, x, y) for f in fracs for x in pxs for y in pys]:
            centre = np.array([pc[0], pc[1], top - radius]) + hax3 * (t0 + fr * (t1 - t0))
            for sgn in (1.0, -1.0):
                y_ax = np.array([0.0, 0.0, -1.0])                      # palm face down
                x_ax = sgn * np.array([-hax3[1], hax3[0], 0.0])        # fingers across the handle
                z_ax = np.cross(x_ax, y_ax)
                T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2] = x_ax, y_ax, z_ax
                T[:3, 3] = centre - px * x_ax - py * y_ax               # handle under the finger base, py below the palm face
                P = np.eye(4); P[:3, 3] = -WRIST_TO_PALM
                wrists.append(T @ P); conf.append(1.0 - abs(fr - 0.5))
        wrists = np.array(wrists); conf = np.array(conf)
        print(f"[reach] power grasp: handle axis {np.round(hax3[:2], 2)}, length {t1 - t0:.2f} m, top z {top:.3f}; {len(wrists)} palm-down wraps, palm z {wrists[:, 2, 3].min():.3f}..{wrists[:, 2, 3].max():.3f}")
        n_pre, n_in, n_lift = int(os.environ.get("APPROACH_FRAMES", "60")), int(os.environ.get("GRASP_FRAMES", "60")), 30
        n_go = n_pre + n_in
        T0 = np.eye(4); T0[:3, 3] = pos0[wi]; T0[:3, :3] = quat_to_mat(quat0[wi])
        seqs, errs = [], []
        for k, Tg in enumerate(wrists):
            pre = Tg.copy(); pre[2, 3] += 0.12
            up = Tg.copy(); up[2, 3] += 0.15
            targets = []
            for i in range(n_pre):
                a = (i + 1) / n_pre; M = Tg.copy(); M[:3, 3] = T0[:3, 3] + (pre[:3, 3] - T0[:3, 3]) * a; targets.append(M)
            for i in range(n_in):
                a = (i + 1) / n_in; M = Tg.copy(); M[:3, 3] = pre[:3, 3] + (Tg[:3, 3] - pre[:3, 3]) * a; targets.append(M)
            for i in range(n_lift):
                a = (i + 1) / n_lift; M = Tg.copy(); M[:3, 3] = Tg[:3, 3] + (up[:3, 3] - Tg[:3, 3]) * a; targets.append(M)
            sol, err = solve(targets, from_here=True)
            seqs.append(sol); errs.append(err)
            print(f"[reach] power #{k}: at grasp {err[n_go-1]*1000:5.1f} mm, lift {err[n_go:].mean()*1000:5.1f} mm")
        out = sys.argv[sys.argv.index("--out") + 1]
        np.savez(out, q=np.stack(seqs), err=np.stack(errs), joint_names=np.array(jn), n_go=n_go, n_lift=n_lift, grasps=wrists, conf=conf)
        print(f"[reach] wrote {out}")
        return

    # --assist-wrap PART_CAP: a POWER grasp that is physically possible for a handle lying ON the floor. Measured
    # (fable40/fable42 near captures): the hammer's handle has 0-0.6 cm under it, the drill's 0-1.9 cm -- no finger
    # fits under either, so every palm-down wrap closed its fingers on the floor beside the handle and the tool stayed
    # down (power_chain, 16/16 LOST). A person lifts the free end first. Here the LEFT hand pinches the handle's free
    # end from above (the verified fingertip pinch, palm-down) and raises it ASSIST_LIFT along an arc about the head,
    # which stays on the floor; the RIGHT palm then comes down on the raised, tilted handle at ASSIST_WRAP_FRAC,
    # fingers across it and under it, thumb opposing; the left opens and lifts away; the right lifts. The finger
    # schedule is written per frame ("hands", HAND_NAMES order, left 12 then right 12) because the two hands close
    # and open at different times. Output in the reach_all format with n_go = every frame.
    if "--assist-wrap" in sys.argv:
        from PIL import Image as _Im
        cap_dir = sys.argv[sys.argv.index("--assist-wrap") + 1]
        meta = json.load(open(os.path.join(cap_dir, "meta_data.json")))
        seg = np.asarray(_Im.open(os.path.join(cap_dir, "seg.png")), dtype=np.int32); depth = np.load(os.path.join(cap_dir, "depth.npy"))
        sys.path.insert(0, "/home/sehoon/Projects/GraspGenX")
        from graspgenx.utils.scene_loaders import depth_to_camera_xyz, transform_xyz
        xyz = transform_xyz(depth_to_camera_xyz(depth, np.asarray(meta["intrinsics"])), np.asarray(meta["camera_pose"]))
        part = xyz[seg == meta["label_map"].get("obj_part", meta["label_map"]["obj_lang"])]
        whole = xyz[seg == meta["label_map"]["obj_lang"]]
        pc = part.mean(0); hax = np.linalg.svd(part[:, :2] - pc[:2], full_matrices=False)[2][0]; hax3 = np.array([hax[0], hax[1], 0.0])
        t = (part[:, :2] - pc[:2]) @ hax; t0, t1 = float(t.min()), float(t.max())
        if (whole.mean(0)[:2] - pc[:2]) @ hax > 0:                 # +hax runs from the head to the free end
            hax3, t0, t1 = -hax3, -t1, -t0
        L = t1 - t0
        top = float(np.percentile(part[:, 2], 90)); radius = float(os.environ.get("POWER_RADIUS", "0.016"))
        rest = np.array([pc[0], pc[1], top - radius])                # the handle's centre line at rest
        pivot = rest + hax3 * t0                                      # the head end stays down
        perp = np.array([-hax3[1], hax3[0], 0.0]); up = np.array([0.0, 0.0, 1.0])
        fr_l, fr_r = float(os.environ.get("ASSIST_PINCH_FRAC", "0.88")), float(os.environ.get("ASSIST_WRAP_FRAC", "0.45"))
        h_tip = float(os.environ.get("ASSIST_LIFT", "0.10")); theta = math.asin(min(0.9, h_tip / (fr_l * L)))
        px, py = float(os.environ.get("POWER_X", "0.10")), float(os.environ.get("POWER_Y", "0.03"))
        li = list(cfg.tool_frames).index("left_wrist_yaw_link"); ri = list(cfg.tool_frames).index("right_wrist_yaw_link")
        P = np.eye(4); P[:3, 3] = -WRIST_TO_PALM
        def rot_about(axis, ang):
            axis = axis / np.linalg.norm(axis); K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
            return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K
        tilt_axis = np.cross(hax3, up)                                # rotating hax3 about it raises the free end
        def handle_point(fr, ang):                                    # centre line point at fraction fr with the tip raised by ang
            return pivot + rot_about(tilt_axis, ang) @ (hax3 * fr * L)
        # LEFT: palm-down fingertip pinch at fr_l. The left Inspire curls toward its palm's -y (mirrored in y only),
        # so palm +y points UP and the pinch point is 0.15 along the fingers, 0.06 on the -y side (rank_handle's
        # right-hand [0.15, 0.06, 0] mirrored). Fingers cross the handle horizontally, pointing away from the hand.
        sgn_l = 1.0 if perp @ (handle_point(fr_l, 0) - pos0[li]) > 0 else -1.0
        xl, yl = sgn_l * perp, up.copy(); zl = np.cross(xl, yl)
        Rl0 = np.stack([xl, yl, zl], axis=1); pc_l = np.array([0.15, -0.06, 0.0])
        def T_left(ang):
            Rt = rot_about(tilt_axis, ang) @ Rl0
            T = np.eye(4); T[:3, :3] = Rt; T[:3, 3] = handle_point(fr_l, ang) - Rt @ pc_l
            return T @ P
        # RIGHT: palm on the raised handle at fr_r. Palm +y (its face) looks at the handle from above, perpendicular
        # to the tilted axis; the spread (palm z) runs along the handle; the fingers (+x) cross it, pointing away
        # from the hand; the handle sits px along the fingers and py below the palm face.
        hax_t = rot_about(tilt_axis, theta) @ hax3
        yr = -(up - (up @ hax_t) * hax_t); yr /= np.linalg.norm(yr)
        cr = handle_point(fr_r, theta)
        best = None
        for sz in (1.0, -1.0):
            zr = sz * hax_t; xr = np.cross(yr, zr)
            if best is None or xr @ (cr - pos0[ri]) > best[0]:
                best = (xr @ (cr - pos0[ri]), xr, zr)
        _, xr, zr = best
        Tr = np.eye(4); Tr[:3, 0], Tr[:3, 1], Tr[:3, 2] = xr, yr, zr; Tr[:3, 3] = cr - px * xr - py * yr
        Tr = Tr @ P
        print(f"[reach] assist wrap: handle axis {np.round(hax3[:2], 2)}, length {L:.2f} m, top z {top:.3f}; left pinch at {fr_l:.2f}, "
              f"tip raised {h_tip:.2f} m ({math.degrees(theta):.0f} deg), right palm on the handle at {fr_r:.2f} (z {cr[2]:.3f}, clearance {cr[2]-radius:.3f})")
        n = dict(over=45, downL=60, closeL=30, liftL=45, downR=60, closeR=30, openL=20, upL=30, lift=45)
        T0l = np.eye(4); T0l[:3, 3] = pos0[li]; T0l[:3, :3] = quat_to_mat(quat0[li])
        T0r = np.eye(4); T0r[:3, 3] = pos0[ri]; T0r[:3, :3] = quat_to_mat(quat0[ri])
        Tl_flat, Tl_up = T_left(0.0), T_left(theta)
        over_l = Tl_flat.copy(); over_l[2, 3] += 0.12
        over_r = Tr.copy(); over_r[2, 3] += 0.15
        def lerp(A, B, a):
            M = B.copy(); M[:3, 3] = A[:3, 3] + (B[:3, 3] - A[:3, 3]) * a; return M
        steps, hands_seq, marks = [], [], {}
        _open = np.array([0, 0, 0, 0, 1.308, 0, 0, 0, 0, 0, 0, 0], float)
        _closed = np.array([1.47, 1.47, 1.47, 1.47, 1.308, 0.5, 1.47, 1.47, 1.47, 1.47, 0.8, 1.2], float)
        _th = np.array([False, False, False, False, True, True, False, False, False, False, True, True])
        def ramp(a_th, a_f):
            h = _open.copy(); h[_th] = (1 - a_th) * _open[_th] + a_th * _closed[_th]; h[~_th] = (1 - a_f) * _open[~_th] + a_f * _closed[~_th]; return h
        cur_l, cur_r = T0l, T0r
        for phase, nf in n.items():
            marks[phase] = len(steps)
            for i in range(nf):
                a = (i + 1) / nf
                if phase == "over":
                    Ml = lerp(T0l, over_l, a); Mr = lerp(T0r, over_r, a)
                    if a < 0.34: Ml[:3, :3] = T0l[:3, :3]; Mr[:3, :3] = T0r[:3, :3]
                    hl, hr = _open, _open
                elif phase == "downL":
                    Ml = lerp(over_l, Tl_flat, a); Mr = over_r; hl, hr = _open, _open
                elif phase == "closeL":
                    Ml = Tl_flat; Mr = over_r
                    hl = ramp(min(1.0, a * 2), max(0.0, min(1.0, a * 2 - 1))); hr = _open     # thumb first, the pinch that held
                elif phase == "liftL":
                    Ml = T_left(theta * a); Mr = over_r; hl, hr = _closed, _open
                elif phase == "downR":
                    Ml = Tl_up; Mr = lerp(over_r, Tr, a); hl, hr = _closed, _open
                elif phase == "closeR":
                    Ml = Tl_up; Mr = Tr; hl = _closed; hr = ramp(a, a)
                elif phase == "openL":
                    Ml = Tl_up; Mr = Tr; hl = ramp(1 - a, 1 - a); hr = _closed
                elif phase == "upL":
                    Ml = Tl_up.copy(); Ml[2, 3] += 0.10 * a; Mr = Tr; hl, hr = _open, _closed
                else:
                    Ml = Tl_up.copy(); Ml[2, 3] += 0.10; Mr = Tr.copy(); Mr[2, 3] += 0.15 * a; hl, hr = _open, _closed
                steps.append({li: Ml, ri: Mr}); hands_seq.append(np.concatenate([hl, hr]))
        sol, err = solve_multi(steps)
        T = len(steps)
        print(f"[reach] assist wrap: {T} frames; error at the left pinch {err[marks['closeL']-1]*1000:.1f} mm, at the right wrap {err[marks['closeR']-1]*1000:.1f} mm, lift {err[marks['lift']:].mean()*1000:.1f} mm")
        out = sys.argv[sys.argv.index("--out") + 1]
        np.savez(out, q=sol[None], err=err[None], joint_names=np.array(jn), n_go=T, n_lift=0, grasps=Tr[None], conf=np.array([1.0]),
                 close_from=marks["closeR"], lift_from=marks["lift"], hands=np.array(hands_seq), marks=json.dumps(marks))
        print(f"[reach] wrote {out}")
        return

    if "--crate-grasps" in sys.argv:
        cj = json.load(open(sys.argv[sys.argv.index("--crate-grasps") + 1]))
        c = np.array(cj["centre"][:2]); ax = np.array(cj["long_axis"]); side = np.array([-ax[1], ax[0]])
        left_dir = np.array([-cj["long_axis"][1], cj["long_axis"][0]])            # robot's left when facing +long_axis
        wrists_all, conf_all = grasps_in_cell(grasps_json, cap)
        P2 = np.eye(4); P2[:3, 3] = WRIST_TO_PALM
        palms_all = np.array([w @ P2 for w in wrists_all])
        zmin = float(os.environ.get("CRATE_GRASP_ZMIN", "0.6")) * cj["top"]
        cands = {"left": [], "right": []}
        for k, (Wp, cf) in enumerate(zip(palms_all, conf_all)):
            pinch = (Wp @ np.array([0.15, 0.06, 0.0, 1.0]))[:3]                 # the Inspire pinch point (rank_handle.py)
            if pinch[2] < zmin:
                continue
            s_ = float((pinch[:2] - c) @ left_dir)
            cands["left" if s_ > 0 else "right"].append((float(cf), k))
        for nm in cands:
            cands[nm].sort(reverse=True)
        n_top = int(os.environ.get("CRATE_PAIRS", "4"))
        print(f"[reach] crate grasps: {len(wrists_all)} candidates, {len(cands['left'])} left / {len(cands['right'])} right above z {zmin:.2f}; "
              f"top left {[k for _, k in cands['left'][:n_top]]}, top right {[k for _, k in cands['right'][:n_top]]}")
        li = list(cfg.tool_frames).index("left_wrist_yaw_link"); ri = list(cfg.tool_frames).index("right_wrist_yaw_link")
        lift = float(os.environ.get("CRATE_LIFT", "0.12")); n_over, n_down, n_hold, n_lift = 45, 60, 30, 45
        seqs, errs, pairs = [], [], []
        for (cl, kl) in cands["left"][:n_top]:
            for (cr, kr) in cands["right"][:n_top]:
                hands = {li: wrists_all[kl], ri: wrists_all[kr]}
                steps = []
                for i in range(n_over + n_down + n_hold + n_lift):
                    st = {}
                    for k, G in hands.items():
                        T0 = np.eye(4); T0[:3, 3] = pos0[k]; T0[:3, :3] = quat_to_mat(quat0[k])
                        over = G.copy(); over[2, 3] = G[2, 3] + 0.20
                        if i < n_over:
                            a = (i + 1) / n_over; au = min(1.0, a * 3.0); M = over.copy()
                            M[:3, 3] = T0[:3, 3] + (over[:3, 3] - T0[:3, 3]) * a; M[2, 3] = T0[2, 3] + (over[2, 3] - T0[2, 3]) * max(a, au)
                            if a < 0.34: M[:3, :3] = T0[:3, :3]
                        elif i < n_over + n_down:
                            a = (i + 1 - n_over) / n_down; M = G.copy(); M[:3, 3] = over[:3, 3] + (G[:3, 3] - over[:3, 3]) * a
                        elif i < n_over + n_down + n_hold:
                            M = G.copy()
                        else:
                            a = (i + 1 - n_over - n_down - n_hold) / n_lift; M = G.copy(); M[2, 3] += lift * a
                        st[k] = M
                    steps.append(st)
                sol, err = solve_multi(steps)
                seqs.append(sol); errs.append(err); pairs.append((kl, kr))
                print(f"[reach] pair L#{kl} (conf {cl:.2f}) R#{kr} (conf {cr:.2f}): wrist error mean {err.mean()*1000:.1f} mm, at the grasp {err[n_over+n_down-5:n_over+n_down].mean()*1000:.1f} mm")
        n_go = n_over + n_down
        out = sys.argv[sys.argv.index("--out") + 1]
        np.savez(out, q=np.stack(seqs), err=np.stack(errs), joint_names=np.array(jn), n_go=n_go, n_lift=n_lift,
                 grasps=np.stack([wrists_all[kr] for _, kr in pairs]), conf=np.array([conf_all[kr] for _, kr in pairs]),
                 pairs=np.array(pairs), close_from=n_go, lift_from=n_go + n_hold)
        print(f"[reach] wrote {out}: {len(pairs)} pairs")
        return

    if "--crate" in sys.argv:
        cj = json.load(open(sys.argv[sys.argv.index("--crate") + 1]))
        ax = np.array(cj["long_axis"] + [0.0]); side = np.array([-ax[1], ax[0], 0.0])
        h = float(os.environ.get("CRATE_HAND_Z", "0.55")) * cj["top"]
        squeeze = float(os.environ.get("CRATE_SQUEEZE", "0.02")); pre = 0.08; lift = float(os.environ.get("CRATE_LIFT", "0.25"))
        li = list(cfg.tool_frames).index("left_wrist_yaw_link"); ri = list(cfg.tool_frames).index("right_wrist_yaw_link")
        P = np.eye(4); P[:3, 3] = -WRIST_TO_PALM
        def palm_pose(face, inward):
            # palm frame in the cell: fingers (+x) forward along the crate, flat
            # side (+y) toward the crate, z = x cross y. Fingers pointing down put
            # 12 cm of finger through the floor from a palm at 9 cm (measured:
            # tips at -2.8 cm, the crate flung 90 cm before the palms arrived).
            # The flat of the Dex3 palm faces along the palm frame's z (the
            # fingers are split across z, they curl toward +y): with +y toward
            # the crate the palms faced the floor (snapshot). So z points into
            # the crate and y = z cross x.
            x = np.array([ax[0], ax[1], 0.0]); x /= np.linalg.norm(x)
            z = inward / np.linalg.norm(inward); y = np.cross(z, x)
            T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2] = x, y, z; T[:3, 3] = [face[0], face[1], h]
            return T @ P                                    # wrist target for this palm
        faces = {li: (np.array(cj["grasp_faces"]["left"]), -side if (np.cross(ax, side)[2] > 0) else side),
                 ri: (np.array(cj["grasp_faces"]["right"]), side if (np.cross(ax, side)[2] > 0) else -side)}
        # inward for each hand points from its face toward the crate centre
        c = np.array(cj["centre"])
        for k in faces:
            f, _ = faces[k]; inward = c[:2] - f[:2]; faces[k] = (f, np.array([inward[0], inward[1], 0.0]))
        # The hands hang at the hips, inside the crate's width, and a straight
        # line to the side faces goes through the near end (measured: the
        # 2 kg crate was shoved 60 cm before the palms arrived). So: up and
        # over first -- to a point 8 cm outside each face, a hand above the
        # rim -- then down beside the face, then in, then the squeeze, then up.
        n_over, n_down, n_in, n_sq, n_lift = 45, 30, 25, 30, 45
        over_z = cj["top"] + 0.10
        seq_targets = []
        for i in range(n_over + n_down + n_in + n_sq + n_lift):
            step = {}
            for k, (f, inward) in faces.items():
                u = inward / np.linalg.norm(inward)
                goal = palm_pose(f, u)
                T0 = np.eye(4); T0[:3, 3] = pos0[k]; T0[:3, :3] = quat_to_mat(quat0[k])
                over = goal.copy(); over[:3, 3] -= pre * u; over[2, 3] = over_z + (goal[2, 3] - h)   # wrist offset kept
                pre_p = goal.copy(); pre_p[:3, 3] -= pre * u
                if i < n_over:
                    a = (i + 1) / n_over; M = over.copy(); M[:3, 3] = T0[:3, 3] + (over[:3, 3] - T0[:3, 3]) * a
                    if a < 0.34: M[:3, :3] = T0[:3, :3]
                elif i < n_over + n_down:
                    a = (i + 1 - n_over) / n_down; M = pre_p.copy(); M[:3, 3] = over[:3, 3] + (pre_p[:3, 3] - over[:3, 3]) * a
                elif i < n_over + n_down + n_in:
                    a = (i + 1 - n_over - n_down) / n_in; M = goal.copy(); M[:3, 3] -= pre * (1 - a) * u
                elif i < n_over + n_down + n_in + n_sq:
                    a = (i + 1 - n_over - n_down - n_in) / n_sq; M = goal.copy(); M[:3, 3] += squeeze * a * u
                else:
                    a = (i + 1 - n_over - n_down - n_in - n_sq) / n_lift; M = goal.copy(); M[:3, 3] += squeeze * u; M[2, 3] += lift * a
                step[k] = M
            seq_targets.append(step)
        sol, err = solve_multi(seq_targets)
        n_pre, n_go = n_over + n_down, n_over + n_down + n_in + n_sq
        print(f"[reach] crate: {len(seq_targets)} frames, wrist error mean {err.mean()*1000:.1f} mm, max {err.max()*1000:.1f} mm; "
              f"pelvis z {sol[:, jn.index('base_j_z')].min():.3f}..{sol[:, jn.index('base_j_z')].max():.3f}")
        np.savez(sys.argv[sys.argv.index("--out") + 1], q=sol[None], err=err[None], joint_names=np.array(jn), n_go=n_go,
                 n_lift=n_lift, grasps=np.eye(4)[None], conf=np.array([1.0]), close_from=n_pre + n_in, lift_from=n_go)
        print(f"[reach] wrote {sys.argv[sys.argv.index('--out') + 1]}")
        return

    # --crate-place X Y Z: a two-hand set-down. Both wrists keep the poses the carry leaves them in and move by
    # one common vector until their MIDPOINT is at (X, Y, Z) (Z = the wrist height that puts the crate's bottom
    # PLACE_CLEAR above the desk top: the wrists ride 0.35 m above the crate bottom in the rim pinch), then both
    # descend PLACE_DOWN, then the fingers open (the schedule does the opening from open_from). Needs BIMANUAL=1.
    if "--crate-place" in sys.argv:
        _i = sys.argv.index("--crate-place"); X, Y, Z = [float(sys.argv[_i + k]) for k in (1, 2, 3)]
        li = list(cfg.tool_frames).index("left_wrist_yaw_link"); ri = list(cfg.tool_frames).index("right_wrist_yaw_link")
        T0 = {}
        for k in (li, ri):
            M = np.eye(4); M[:3, 3] = pos0[k]; M[:3, :3] = quat_to_mat(quat0[k]); T0[k] = M
        mid = (T0[li][:3, 3] + T0[ri][:3, 3]) / 2.0
        delta = np.array([X, Y, Z]) - mid
        down = float(os.environ.get("PLACE_DOWN", "0.05"))
        n_go, n_down, n_hold = 90, 30, 45
        steps = []
        for i in range(n_go + n_down + n_hold):
            st = {}
            for k in (li, ri):
                M = T0[k].copy()
                if i < n_go:
                    M[:3, 3] = T0[k][:3, 3] + delta * (i + 1) / n_go
                elif i < n_go + n_down:
                    M[:3, 3] = T0[k][:3, 3] + delta; M[2, 3] -= down * (i + 1 - n_go) / n_down
                else:
                    M[:3, 3] = T0[k][:3, 3] + delta; M[2, 3] -= down
                st[k] = M
            steps.append(st)
        sol, err = solve_multi(steps)
        print(f"[reach] crate place: {len(steps)} frames, wrists' midpoint {np.round(mid, 3)} -> {np.round([X, Y, Z], 3)} then down {down:.2f}; "
              f"wrist error mean {err.mean()*1000:.1f} mm, max {err.max()*1000:.1f} mm; pelvis z {sol[:, jn.index('base_j_z')].min():.3f}..{sol[:, jn.index('base_j_z')].max():.3f}")
        np.savez(out, q=sol, joint_names=np.array(jn), err=err, close_from=-1, lift_from=-1, open_from=n_go + n_down, grasp=np.eye(4), best=-1)
        print(f"[reach] wrote {out}")
        return

    if place is not None:
        T0 = np.eye(4)
        T0[:3, 3] = pos0[wi]
        T0[:3, :3] = quat_to_mat(quat0[wi])
        Tp = T0.copy()
        Tp[:3, 3] = place
        n_go, n_hold = 90, 45
        targets = []
        for i in range(n_go):
            a = (i + 1) / n_go
            M = T0.copy()
            M[:3, 3] = T0[:3, 3] + (Tp[:3, 3] - T0[:3, 3]) * a
            targets.append(M)
        targets += [Tp.copy() for _ in range(n_hold)]
        sol, err = solve(targets, from_here=True)
        print(f"[reach] place: {len(targets)} frames to {np.round(place, 3)}, wrist error "
              f"mean {err.mean()*1000:.1f} mm, max {err.max()*1000:.1f} mm, at the crate "
              f"{err[n_go:].mean()*1000:.1f} mm; pelvis z {sol[:, jn.index('base_j_z')].min():.3f}"
              f"..{sol[:, jn.index('base_j_z')].max():.3f}")
        np.savez(out, q=sol, joint_names=np.array(jn), err=err, close_from=-1,
                 lift_from=-1, open_from=n_go, grasp=Tp, best=-1)
        print(f"[reach] wrote {out}")
        return

    wrists, conf = grasps_in_cell(grasps_json, cap)
    print(f"[reach] {len(wrists)} grasps, wrist targets z "
          f"{wrists[:, 2, 3].min():.3f}..{wrists[:, 2, 3].max():.3f}")


    # --all-out FILE: a short kneel -> grasp -> lift sequence for every
    # candidate, streamed from the kneel pose, for a physics test of each
    # grasp in the cell (GraspGenX validates its grasps by replaying them
    # under physics too; this does the same with the body that reaches).
    if "--all-out" in sys.argv:
        allf = sys.argv[sys.argv.index("--all-out") + 1]
        T0 = np.eye(4)
        T0[:3, 3] = pos0[wi]
        T0[:3, :3] = quat_to_mat(quat0[wi])
        seqs, errs = [], []
        # The approach the way plan_grasp does it: to a pre-grasp 10 cm back
        # along the approach axis (the palm's +y here, robots/g1_right_arm.yaml
        # grasp_approach_axis: y), then straight in along that axis. A line
        # from wherever the wrist is to the grasp pose comes in diagonally
        # and the hand hits the object on the way -- measured, the clamp was
        # swept 31 cm before the fingers closed.
        # APPROACH_FRAMES / GRASP_FRAMES: GraspGenX's own task player stretches the free-space approach and the
        # final straight run-in to 2 s each (end2end/tasks.py: "0.7 s is fast enough that the approach can bump /
        # tip the object before the fingers close"); ours were 1.0 s and 0.67 s.
        n_pre, n_in, n_lift = int(os.environ.get("APPROACH_FRAMES", "60")), int(os.environ.get("GRASP_FRAMES", "60")), 30
        # GRASP_SLIDE (m, default 0): the descent lands the fingertips GRASP_SLIDE beyond the handle (palm -y,
        # away from the thumb), then the hand slides +y until the finger pads sit against the handle, and only
        # then the close. A descent aimed at the grasp itself lands the pads on the handle's edge and whether it
        # shoves the hammer away or not is a coin flip (2026-09-27: 5/40 held, 1 of 2 reruns, renders lost).
        g_slide = float(os.environ.get("GRASP_SLIDE", "0"))
        n_slide = 15 if g_slide else 0
        n_go = n_pre + n_in + n_slide
        # The open fingers must not be under the floor at the grasp pose.
        # GraspGenX's scene filter samples the floor too sparsely to see it,
        # and a grasp that put the fingertips 2 cm into the floor pushed the
        # fingers straight and launched the clamp when they closed. The
        # fingertips' places in the palm frame are read off the hand once
        # (results/fable3/dex3_tips_in_palm.json); a grasp whose lowest tip
        # would be below the support is backed off along its own approach
        # axis until the tip clears it by 5 mm.
        tips = None
        tj = os.path.join(os.path.dirname(cap.rstrip("/")), "dex3_tips_in_palm.json")
        if not os.path.exists(tj):
            tj = "/home/sehoon/Documents/GitHub/g1-factory-tidy/results/fable3/dex3_tips_in_palm.json"
        if os.environ.get("HAND") == "inspire":
            tj = "/home/sehoon/Documents/GitHub/g1-factory-tidy/results/fable3/inspire_tips_in_palm.json"
        floor_z = float(os.environ.get("SUPPORT_Z", "0.0"))
        # FLOOR_CLEAR: how far above the support the lowest open fingertip
        # must be. 5 mm left the tips at -3 mm after the IK error, pressed
        # into the floor, and a 1.4 Nm finger could not close against the
        # floor's friction (velocity-mode close moved the index 0.16 rad in
        # 6 s). GraspGenX's own filter rejects any grasp whose gripper mesh
        # is within 2 cm of the observed scene (collision_filter.py,
        # collision_threshold=0.02); the floor is too sparse in its 8192
        # sampled points to be seen there, so the same 2 cm is applied here
        # against the floor plane.
        # TIP_FLESH: the tips in dex3_tips_in_palm.json are link ORIGINS. The
        # thumb's flesh reaches about 4 cm past its thumb_2 origin: with that
        # origin 3.7 cm above the floor the thumb never closed (stuck at its
        # open 0.72 on every palm-down candidate, mu 10 or 1, rolled or not);
        # with it 5.7 cm up the thumb closed fully. So the clearance is
        # measured from 4 cm beyond the origins.
        floor_clear = float(os.environ.get("FLOOR_CLEAR", "0.005")) + float(os.environ.get("TIP_FLESH", "0.04"))
        # APPROACH_DEEPER: GraspGenX's g1_dex3_right description copies the
        # fingertip (7 cm along the approach) from the unitree_g1 family hand
        # (end2end/write_g1_dex3_config.py: "grasps made against it stop
        # short and shove the object instead of closing on it"). This hand's
        # fingers and thumb meet 4.3 cm from the palm (dex3_tips_in_palm.json,
        # closed), so a grasp lands with its closed pinch point ON the object
        # surface -- measured on the lying hammer: 0.2 mm outside the head
        # for the top-down candidates -- and the fingers touch and push. The
        # grasp is taken 2.7 cm further along its own approach so the object
        # sits inside the fingers' sweep (1.6..5.4 cm from the palm).
        deeper = float(os.environ.get("APPROACH_DEEPER", "0"))
        if deeper:
            for k in range(len(wrists)):
                wrists[k][:3, 3] += deeper * wrists[k][:3, 1]
            print(f"[reach] every grasp taken {deeper*1000:.0f} mm deeper along its approach")
        if os.path.exists(tj):
            tips = np.array(list(json.load(open(tj))["open"].values()))     # (3, 3) in palm frame
            P2 = np.eye(4)
            P2[:3, 3] = WRIST_TO_PALM
            for k in range(len(wrists)):
                palm = wrists[k] @ P2
                tz = (palm[:3, :3] @ tips.T).T[:, 2] + palm[2, 3]
                deficit = floor_z + floor_clear - tz.min()
                ay = wrists[k][:3, 1]                    # approach, into the object
                if deficit > 0 and ay[2] < -0.2:
                    back = deficit / (-ay[2])
                    wrists[k][:3, 3] -= back * ay
                    print(f"[reach] grasp #{k:2d}: open fingertip {tz.min():+.3f} -> backed off "
                          f"{back*1000:.0f} mm along the approach")
                elif deficit > 0:
                    # A side approach (the hand comes in level, fingers hanging
                    # to the floor): backing off along the approach does not
                    # raise the tips. Measured on the lying hammer, #21/#2/#5
                    # had the model tips at -0.001..+0.006 with the approach
                    # axis level, and the sim put the lowest tip at -0.003.
                    # Lift the whole hand straight up by the deficit instead.
                    wrists[k][2, 3] += deficit
                    print(f"[reach] grasp #{k:2d}: open fingertip {tz.min():+.3f} -> raised "
                          f"{deficit*1000:.0f} mm (level approach)")
        # DESCEND (default 1): a grasp whose approach axis is level comes in
        # from 10 cm straight above instead of 10 cm back along the palm --
        # back along the palm is through the object when the object lies on
        # the floor (measured: the hammer was kicked 21 cm and stood on its
        # head before the fingers closed, results/fable6/handle/snap_54_*).
        # Coming down from above is how a person takes a thing off the floor.
        descend = os.environ.get("DESCEND", "1") == "1"
        # PIN_JSON: the left palm, flat (its flat side is the palm frame's z),
        # comes down on the head's top and presses 1.5 cm into it for the
        # whole approach and close; it lets go (rises 10 cm) as the right
        # hand lifts. Needs BIMANUAL=1 so the left wrist is a work frame.
        pin_seq = None
        if os.environ.get("PIN_JSON"):
            pj = json.load(open(os.environ["PIN_JSON"]))
            pt = np.array(pj["point"]); ax_h = np.array(pj["handle_axis"] + [0.0]); ax_h /= np.linalg.norm(ax_h)
            zf = np.array([0.0, 0.0, -1.0]); xf = ax_h; yf = np.cross(zf, xf)
            Tpin = np.eye(4); Tpin[:3, 0], Tpin[:3, 1], Tpin[:3, 2] = xf, yf, zf
            press = float(os.environ.get("PIN_PRESS", "0.015"))
            Tpin[:3, 3] = [pt[0], pt[1], pt[2] + 0.02 - press]             # palm 2 cm thick, pressed in
            Ppin = np.eye(4); Ppin[:3, 3] = -WRIST_TO_PALM
            Wpin = Tpin @ Ppin
            li0 = list(cfg.tool_frames).index("left_wrist_yaw_link")
            TL0 = np.eye(4); TL0[:3, 3] = pos0[li0]; TL0[:3, :3] = quat_to_mat(quat0[li0])
            up = Wpin.copy(); up[2, 3] += 0.12
            pin_seq = []
            for i in range(n_pre):                                         # come down on the head while the right hand goes to its pre-grasp
                a = (i + 1) / n_pre; M = Wpin.copy(); M[:3, 3] = TL0[:3, 3] + (up[:3, 3] - TL0[:3, 3]) * min(1.0, a * 1.5)
                if a > 0.67: M[:3, 3] = up[:3, 3] + (Wpin[:3, 3] - up[:3, 3]) * (a - 0.67) / 0.33
                if a < 0.34: M[:3, :3] = TL0[:3, :3]
                pin_seq.append(M)
            pin_seq += [Wpin.copy() for _ in range(n_in)]
            for i in range(n_lift):                                        # let go as the lift starts
                a = (i + 1) / n_lift; M = Wpin.copy(); M[:3, 3] = Wpin[:3, 3] + (up[:3, 3] - Wpin[:3, 3]) * min(1.0, a * 3)
                pin_seq.append(M)
            print(f"[reach] pin: left palm on the head at {np.round(pt, 3)} (top), pressed {press*1000:.0f} mm")
        # APPROACH_AXIS=x: come in along the FINGERS (the palm frame's +x) from APPROACH_BACK m behind the
        # grasp. The Inspire grasps on the floor hold the palm pitched ~50 deg with the fingers pointing
        # down-and-away; a vertical descent (DESCEND) then drags the open fingers' undersides across the
        # handle in the last 10 cm and shoves the hammer 9.5 cm before the close (5지/hammer/v7, tester
        # 2026-09-27 15:23). Along the fingers, the fingertips lead into the gap beside the handle.
        approach_axis = os.environ.get("APPROACH_AXIS", "z" if descend else "y")
        back = float(os.environ.get("APPROACH_BACK", "0.12"))
        for k, Tg0 in enumerate(wrists):
            Tg = Tg0.copy()
            if g_slide:
                Tg[:3, 3] = Tg0[:3, 3] - g_slide * Tg0[:3, 1]          # the descent's target, beyond the handle
            pre = Tg.copy()
            if approach_axis == "x":
                pre[:3, 3] = Tg[:3, 3] - back * Tg[:3, 0]
                if pre[2, 3] < Tg[2, 3] + 0.03:                       # never start lower than 3 cm above the grasp
                    pre[2, 3] = Tg[2, 3] + 0.03
            elif descend and Tg[2, 1] > -0.5:
                pre[:3, 3] = Tg[:3, 3] + np.array([0.0, 0.0, 0.10])
            else:
                pre[:3, 3] = Tg[:3, 3] - 0.10 * Tg[:3, 1]
            up = Tg.copy()
            up[:3, 3] = Tg[:3, 3] + np.array([0.0, 0.0, 0.15])
            targets = []
            for i in range(n_pre):
                a = (i + 1) / n_pre
                M = Tg.copy()
                M[:3, 3] = T0[:3, 3] + (pre[:3, 3] - T0[:3, 3]) * a
                targets.append(M)
            for i in range(n_in):
                a = (i + 1) / n_in
                M = Tg.copy()
                M[:3, 3] = pre[:3, 3] + (Tg[:3, 3] - pre[:3, 3]) * a
                targets.append(M)
            for i in range(n_slide):                                   # slide the pads against the handle
                a = (i + 1) / n_slide
                M = Tg0.copy()
                M[:3, 3] = Tg[:3, 3] + (Tg0[:3, 3] - Tg[:3, 3]) * a
                targets.append(M)
            up = Tg0.copy()
            up[:3, 3] = Tg0[:3, 3] + np.array([0.0, 0.0, 0.15])
            for i in range(n_lift):
                a = (i + 1) / n_lift
                M = Tg0.copy()
                M[:3, 3] = Tg0[:3, 3] + (up[:3, 3] - Tg0[:3, 3]) * a
                targets.append(M)
            sol, err = solve(targets, from_here=True, left_targets=pin_seq)
            seqs.append(sol)
            errs.append(err)
            print(f"[reach] grasp #{k:2d} conf {conf[k]:.3f}: at grasp {err[n_go-1]*1000:5.1f} mm, "
                  f"lift {err[n_go:].mean()*1000:5.1f} mm")
        np.savez(allf, q=np.stack(seqs), err=np.stack(errs), joint_names=np.array(jn),
                 n_go=n_go, n_lift=n_lift, grasps=wrists, conf=conf)
        print(f"[reach] wrote {allf}")
        return

    # one frame per grasp: which of them can the body reach from here
    sol, err = solve(list(wrists))
    order = np.argsort(err)
    for k in order[:6]:
        print(f"[reach]   grasp #{k:2d} conf {conf[k]:.3f}  wrist error {err[k]*1000:6.1f} mm  "
              f"pelvis z {sol[k, jn.index('base_j_z')]:.3f}")
    best = int(order[0])
    print(f"[reach] best grasp #{best}: {err[best]*1000:.1f} mm")
    if out is None:
        return
    # The reach as a sequence, starting from where the wrist IS: the clip's
    # own end pose. A sequence that opens 8 cm back from the grasp asked the
    # warm start to jump there in one frame and it landed 81 mm off and
    # stayed in that basin. So: current wrist -> pre-grasp (3 cm back along
    # the approach, the tool's +z points away from the object) -> grasp ->
    # hold while the fingers close -> lift 15 cm.
    Tg = wrists[best]
    T0 = np.eye(4)
    T0[:3, 3] = pos0[wi]
    T0[:3, :3] = quat_to_mat(quat0[wi])
    back = Tg.copy()
    back[:3, 3] = Tg[:3, 3] + 0.03 * Tg[:3, 2]
    up = Tg.copy()
    up[:3, 3] = Tg[:3, 3] + np.array([0.0, 0.0, 0.15])
    n_hold, n_in, n_close, n_lift = 60, 30, 30, 45

    def _lerp(A, B, n):
        out = []
        for i in range(n):
            a = (i + 1) / n
            M = B.copy()
            M[:3, 3] = A[:3, 3] + (B[:3, 3] - A[:3, 3]) * a
            out.append(M)
        return out
    targets = (_lerp(T0, back, n_hold) + _lerp(back, Tg, n_in)
               + [Tg.copy() for _ in range(n_close)] + _lerp(Tg, up, n_lift))
    for k, t in enumerate(targets):
        if k < n_hold:
            # orientation: slerp-free blend by keeping the grasp's frame after
            # the first third; the solver has a rotation weight of 0.067 so a
            # step in orientation early costs it little
            t[:3, :3] = T0[:3, :3] if k < n_hold // 3 else Tg[:3, :3]
        else:
            t[:3, :3] = Tg[:3, :3]
    sol, err = solve(targets, from_here=True)
    print(f"[reach] sequence {len(targets)} frames: wrist error mean {err.mean()*1000:.1f} mm, "
          f"max {err.max()*1000:.1f} mm; pelvis z {sol[:, jn.index('base_j_z')].min():.3f}"
          f"..{sol[:, jn.index('base_j_z')].max():.3f}")
    b = np.cumsum([0, n_hold, n_in, n_close, n_lift])
    for nm, a, c in zip(("to pre-grasp", "approach", "at grasp", "lift"), b[:-1], b[1:]):
        print(f"[reach]   {nm:15s} frames {a:3d}..{c-1:3d}  error mean {err[a:c].mean()*1000:5.1f}"
              f"  max {err[a:c].max()*1000:5.1f} mm")
    np.savez(out, q=sol, joint_names=np.array(jn), err=err,
             close_from=n_hold + n_in, lift_from=n_hold + n_in + n_close,
             grasp=Tg, best=best)
    print(f"[reach] wrote {out}")


if __name__ == "__main__":
    main()
