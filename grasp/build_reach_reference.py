"""Walk + kneel clip, then the whole-body reach cuRobo solved, as one SONIC reference.

    python grasp/build_reach_reference.py WALK.pkl REACH.npz OUT_NAME [--hold N]

Writes results/motion/OUT_NAME.pkl (motion_lib layout, 30 fps) and
results/motion/OUT_NAME_hands.npy: (frames, 14) finger targets in GR00T's
G1_HAND_JOINTS order, open until the reach's close frame, closed after --
the same schedule shape the evaluator's SONIC_HAND_SCHEDULE hook reads.
"""
import os
import sys

import joblib
import numpy as np
from scipy.spatial.transform import Rotation as R

SWARM = "/home/sehoon/Documents/GitHub/humanoid-swarm-sim"
sys.path.insert(0, os.path.join(SWARM, "common"))
from gen_planner_motion import qpos_to_motion_lib  # noqa: E402

FPS = 30
# Dex3-1 closed, as the grasp plan ramps them (end2end/tasks.py close_vals);
# the same values grasp/run_wbc_pick.sh read off the plan's last frame.
HAND_OPEN = [0, 0, 0, 0, 0, 0, 0, 0.4, 0.0, 0.4, 0.0, 0.0, 0.7243, 0.0]
HAND_CLOSED = [0, 0, 0, 0, 0, 0, 0, 1.5708, 1.7453, 1.5708, 1.7453, 0.0, -1.0472, -1.5]
HAND_NAMES = [f"{s}_hand_{j}_joint" for s in ("left", "right")
              for j in ("index_0", "index_1", "middle_0", "middle_1", "thumb_0", "thumb_1", "thumb_2")]
PALM_LINK = {"left": "left_hand_palm_link", "right": "right_hand_palm_link"}
HAND_KEEP = ("right_hand", "right_wrist")
if os.environ.get("HAND") == "inspire":
    # IsaacLab's G1 + Inspire (local Isaac asset, variants right_hand/left_hand = Inspire): 12 joints a
    # hand. Open/closed from GraspGenX's inspire_hand config (thumb yaw 1.308 held, pitch 0 -> 0.5 at
    # this model's limit, fingers 0 -> 1.47); the intermediate joints follow their proximal (the real
    # hand couples them), the thumb's intermediate/distal follow its pitch. Only the right hand closes.
    _fingers = ("index", "middle", "ring", "pinky")
    HAND_NAMES = ([f"L_{f}_proximal_joint" for f in _fingers] + ["L_thumb_proximal_yaw_joint", "L_thumb_proximal_pitch_joint"]
                  + [f"L_{f}_intermediate_joint" for f in _fingers] + ["L_thumb_intermediate_joint", "L_thumb_distal_joint"]
                  + [f"R_{f}_proximal_joint" for f in _fingers] + ["R_thumb_proximal_yaw_joint", "R_thumb_proximal_pitch_joint"]
                  + [f"R_{f}_intermediate_joint" for f in _fingers] + ["R_thumb_intermediate_joint", "R_thumb_distal_joint"])
    _open_one = [0, 0, 0, 0, 1.308, 0, 0, 0, 0, 0, 0, 0]
    _closed_one = [1.47, 1.47, 1.47, 1.47, 1.308, 0.5, 1.47, 1.47, 1.47, 1.47, 0.8, 1.2]
    HAND_OPEN = _open_one + _open_one
    HAND_CLOSED = _open_one + _closed_one
    PALM_LINK = {"left": "left_wrist_yaw_link", "right": "right_wrist_yaw_link"}   # the official asset merges the hand base into the wrist link
    HAND_KEEP = ("right_hand", "right_wrist", "R_")
    if os.environ.get("BOTH_HANDS") == "1":
        # a two-hand grip (the crate's rim pinch): the left closes too, and it collides -- every earlier two-hand
        # crate run kept only the right hand's colliders and closed only the right hand, so the left "pinched air"
        HAND_CLOSED = _closed_one + _closed_one
        HAND_KEEP = ("right_hand", "right_wrist", "R_", "left_hand", "left_wrist", "L_")
BOTH_HANDS = os.environ.get("BOTH_HANDS") == "1"
# the official IsaacLab G1 + Inspire asset (Nucleus 5.1), downloaded to the repo: 39 MB base with the meshes
INSPIRE_USD = "/home/sehoon/Documents/GitHub/g1-factory-tidy/assets/g1_inspire/g1_29dof_inspire_hand.usd"


def stiffen_mimic(stage, root="/World/G1"):
    """HAND=inspire: the official asset couples each intermediate/distal joint to its proximal with a PhysX mimic
    joint (gearing -1 / -1.6 / -2.4) at naturalFrequency 25 Hz, dampingRatio 0.005 -- a spring so soft that the first
    contact folds the passive segment back to its -0.34 limit (measured in every tester/render close: proximal 1.47,
    intermediate -0.34, so only the proximal segment ever curled round the handle). Free in the air the coupling
    tracks (finger_mimic_test: 0.26/0.35, 0.51/0.60). The real RH56 four-bar is rigid: stiffen the coupling here,
    after Articulation(cfg) and before sim.reset(). MIMIC_FREQ / MIMIC_DAMPING override; MIMIC_FREQ=0 leaves it."""
    if os.environ.get("HAND") != "inspire":
        return 0
    freq = float(os.environ.get("MIMIC_FREQ", "200"))
    if freq <= 0:
        return 0
    ratio = float(os.environ.get("MIMIC_DAMPING", "1.0"))
    from pxr import Usd  # noqa: E402  (only inside Isaac)
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        for a in prim.GetAttributes():
            nm = a.GetName()
            if nm.startswith("physxMimicJoint:") and nm.endswith(":naturalFrequency"):
                a.Set(freq); n += 1
            elif nm.startswith("physxMimicJoint:") and nm.endswith(":dampingRatio"):
                a.Set(ratio)
    print(f"[hand] inspire mimic joints stiffened: {n} at {freq:.0f} Hz, damping ratio {ratio}")
    return n


def robot_cfg(base_cfg, sim_utils, ImplicitActuatorCfg):
    """The articulation for the chosen hand: the stock Dex3 config, or the local Isaac G1 asset with
    both hands switched to Inspire and NVIDIA's own soft finger drives (unitree.py G1_INSPIRE_FTP_CFG)."""
    if os.environ.get("HAND") != "inspire":
        return base_cfg
    cfg = base_cfg.copy()
    cfg.spawn = cfg.spawn.replace(usd_path=INSPIRE_USD, variants={"Physics": "PhysX", "Sensor": "None", "Robot": "Robot"})
    # only the proximal joints and the thumb's yaw/pitch are driven; the intermediate and distal joints
    # are PhysX mimic joints in this asset (measured: driven as well, they ran to their -0.34 limit while
    # the proximals closed to 1.1, so the fingertips flared out instead of curling round the handle)
    expr = [".*_proximal_joint", ".*_thumb_proximal_(yaw|pitch)_joint"]
    if SOFT_MIMIC:
        # SOFT_MIMIC (default on): the intermediate/distal joints are driven too, at a target that soft_mimic()
        # rewrites every substep from the MEASURED proximal angle with the asset's gear ratios -- a rigid four-bar,
        # which the PhysX mimic joint at any frequency the 1 kHz step tolerates is not (200 Hz: folds to -0.34 on
        # contact; 1000 Hz: 0.6-0.7 of the proximal).
        expr += [".*_intermediate_joint", ".*_thumb_distal_joint"]
    cfg.actuators["hands"] = ImplicitActuatorCfg(joint_names_expr=expr,
                                                 effort_limit=30.0, velocity_limit=10.0, stiffness=10.0, damping=0.2, armature=0.001)
    return cfg


SOFT_MIMIC = os.environ.get("HAND") == "inspire" and os.environ.get("SOFT_MIMIC", "1") == "1"
# (mimic joint, its reference, gearing) as the official asset authors them (physxMimicJoint:rotZ, gearing negated)
_MIMIC = [(f"{s}_{f}_intermediate_joint", f"{s}_{f}_proximal_joint", 1.0) for s in ("L", "R") for f in ("index", "middle", "ring", "pinky")]
_MIMIC += [(f"{s}_thumb_intermediate_joint", f"{s}_thumb_proximal_pitch_joint", 1.6) for s in ("L", "R")]
_MIMIC += [(f"{s}_thumb_distal_joint", f"{s}_thumb_proximal_pitch_joint", 2.4) for s in ("L", "R")]
_mimic_ids = None


def soft_mimic(robot, tgt):
    """Rewrite the intermediate/distal targets in tgt (1, n_joints) from the measured proximal angles. Call after
    robot.update() and before set_joint_position_target(), every substep. No-op unless SOFT_MIMIC."""
    global _mimic_ids
    if not SOFT_MIMIC:
        return tgt
    if _mimic_ids is None:
        _mimic_ids = [(robot.find_joints([a])[0][0], robot.find_joints([b])[0][0], r) for a, b, r in _MIMIC]
        print(f"[hand] software four-bar on {len(_mimic_ids)} inspire joints (SOFT_MIMIC)")
    q = robot.data.joint_pos[0]
    for a, b, r in _mimic_ids:
        tgt[0, a] = r * float(q[b])
    return tgt
# THUMB0: the right thumb's abduction, 0 in GraspGenX's description (copied from
# the unitree_g1 family). With 0 the thumb hangs 5.4 cm (+4 cm of flesh) under
# the palm and every palm-down pinch on a floor-level handle jammed it; swung
# sideways it can oppose the index from beside the object instead of from
# underneath (a person's key pinch on a stick on the floor).
if os.environ.get("HOOK"):
    # the hook grip: index and middle curl behind the crate's slot bar, both
    # hands, thumbs stay open outside the wall
    HAND_OPEN = [0.4, 0.0, 0.4, 0.0, 0.0, 0.7243, 0.0] + HAND_OPEN[7:]
    HAND_CLOSED = [1.5708, 1.7453, 1.5708, 1.7453, 0.0, 0.7243, 0.0, 1.5708, 1.7453, 1.5708, 1.7453, 0.0, 0.7243, 0.0]
if os.environ.get("THUMB0"):
    HAND_OPEN[11] = HAND_CLOSED[11] = float(os.environ["THUMB0"])


def mujoco_names():
    cwd = os.getcwd()
    os.chdir("/home/sehoon/Projects/GR00T-WholeBodyControl")
    try:
        from foot_height import load_urdf
        return load_urdf()[1]
    finally:
        os.chdir(cwd)


def main():
    walk_pkl, reach_npz, name = sys.argv[1:4]
    hold = int(sys.argv[sys.argv.index("--hold") + 1]) if "--hold" in sys.argv else 30
    w = list(joblib.load(walk_pkl).values())[0]
    dof_w = np.asarray(w["dof"], np.float32)
    root_w = np.asarray(w["root_trans_offset"], np.float32)
    quat_w = np.asarray(w["root_rot"], np.float32)                  # xyzw
    qpos_w = np.concatenate([root_w, quat_w[:, [3, 0, 1, 2]], dof_w], axis=1)

    d = np.load(reach_npz)
    q, jn = d["q"], [str(n) for n in d["joint_names"]]
    names = mujoco_names()
    col = [jn.index(n) for n in names]
    dof_r = q[:, col].astype(np.float32)
    base = q[:, [jn.index(f"base_j_{a}") for a in ("x", "y", "z")]]
    rpy = q[:, [jn.index(f"base_j_{a}") for a in ("xtheta", "ytheta", "ztheta")]]
    quat_r = R.from_euler("XYZ", rpy).as_quat()      # xyzw; cuRobo's base chain is Rx*Ry*Rz
    qpos_r = np.concatenate([base, quat_r[:, [3, 0, 1, 2]], dof_r], axis=1).astype(np.float32)

    # The solved reach starts from the clip's last pose, so the two meet; the
    # base still differs by what the solver moved it, which the tracker sees
    # as one step -- print it so it is a number, not a surprise.
    jump = np.abs(qpos_r[0] - qpos_w[-1])
    print(f"[ref] seam: base moves {jump[:3].max()*1000:.1f} mm, joints max "
          f"{np.degrees(jump[7:].max()):.1f} deg at {names[int(jump[7:].argmax())]}")
    # Cross-fade into the reach over its first BLEND frames, root by lerp,
    # rotation by slerp, joints by lerp -- the way walk_clip joins two plans
    # (planner_onnx.md, "Animation Blending"). The reach's own first frames
    # are a hold at the wrist's current pose, so nothing is lost to the fade;
    # without it the base stepped 37 mm and the shoulder 31 deg in one frame,
    # which is the jolt at 40 s that kicked the box.
    BLEND = int(os.environ.get("SEAM_BLEND", "0"))   # the solver is velocity-limited now; a joint-space fade swept the hand through the object
    for i in range(BLEND):
        a = (i + 1) / (BLEND + 1)
        qpos_r[i, 0:3] = (1 - a) * qpos_w[-1, 0:3] + a * qpos_r[i, 0:3]
        qpos_r[i, 7:] = (1 - a) * qpos_w[-1, 7:] + a * qpos_r[i, 7:]
        qa, qb = qpos_w[-1, 3:7], qpos_r[i, 3:7]
        if np.dot(qa, qb) < 0:
            qb = -qb
        q = (1 - a) * qa + a * qb
        qpos_r[i, 3:7] = q / np.linalg.norm(q)
    qpos = np.concatenate([qpos_w, qpos_r, np.repeat(qpos_r[-1:], hold, axis=0)], axis=0)
    out = os.path.join("results", "motion", f"{name}.pkl")
    joblib.dump({name: qpos_to_motion_lib(qpos, FPS)}, out, compress=True)
    print(f"[ref] {len(qpos_w)} walk+kneel + {len(qpos_r)} reach + {hold} hold "
          f"= {len(qpos)} frames at {FPS} fps -> {out}")
    print(f"[ref] pelvis z: kneel {qpos_w[-1, 2]:.3f}, reach {qpos_r[:, 2].min():.3f}"
          f"..{qpos_r[:, 2].max():.3f}")

    close_at = len(qpos_w) + int(d["close_from"])
    hands = np.tile(np.array(HAND_OPEN, np.float32), (len(qpos), 1))
    ramp = 20
    for k in range(ramp):
        a = (k + 1) / ramp
        if os.environ.get("CLOSE_ORDER", "together") == "thumb_first":
            # the close that held the hammer's handle (results/fable6/handle
            # #54): the thumb first over the first half of the ramp, the
            # fingers over the second -- the thumb closing after the fingers
            # jammed on every palm-down grasp (DIAGNOSIS.md, 02:40)
            th = np.array([("thumb" in n and (BOTH_HANDS or n.startswith(("right", "R_")))) for n in HAND_NAMES], bool)   # the right thumb (both with BOTH_HANDS), any hand
            a_t = min(1.0, (k + 1) / (ramp // 2)); a_f = min(1.0, max(0.0, (k + 1 - ramp // 2) / (ramp // 2)))
            h = np.array(HAND_OPEN, np.float32)
            h[th] = (1 - a_t) * np.array(HAND_OPEN)[th] + a_t * np.array(HAND_CLOSED)[th]
            h[~th] = (1 - a_f) * np.array(HAND_OPEN)[~th] + a_f * np.array(HAND_CLOSED)[~th]
            hands[close_at + k] = h
        else:
            hands[close_at + k] = (1 - a) * np.array(HAND_OPEN) + a * np.array(HAND_CLOSED)
    hands[close_at + ramp:] = HAND_CLOSED
    if os.environ.get("NO_CLOSE"):               # a crate is carried between open palms
        hands[:] = HAND_OPEN
    hp = os.path.join("results", "motion", f"{name}_hands.npy")
    np.save(hp, hands)
    print(f"[ref] hands close at frame {close_at} ({close_at/FPS:.1f} s), lift from "
          f"{len(qpos_w) + int(d['lift_from'])} -> {hp}")


if __name__ == "__main__":
    main()
