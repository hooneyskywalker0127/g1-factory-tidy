# Walk the G1 to the stand the pick is planned from, and stop there facing it.
#
# Everything the pick does is unchanged. The only thing missing was getting the
# robot to the spot: until now it was spawned in front of the table already.
#
# The motion comes from GEAR-SONIC's kinematic planner ONNX, and the loop that
# drives it is the one the GR00T-WholeBodyControl repo drives it with:
#   motionbricks/scripts/interactive_demo_g1.py   the tick loop
#   motionbricks/.../demo/full_agent.py           context, replan gate
#   motionbricks/.../demo/controllers.py          a controller per tick
#   docs/source/references/planner_onnx.md        the tensor contract
# The planner's loader and the motion_lib writer are humanoid-swarm-sim's
# gen_planner_motion.py, reused as reach_clip.py reuses its neighbours.
#
# Two things had to come from the official loop rather than from
# gen_planner_motion.py, which rolls the planner out open-loop:
#
# 1. It steers. gen_planner_motion.py pins movement_direction and
#    facing_direction to +X for a whole segment, so "walk for N seconds" goes
#    in a straight line and stops wherever that lands. Both are world-frame
#    vectors and the reference replans whenever they change, so steering is
#    recomputing them toward the goal on every replan.
#
# 2. It commits only 8 frames of each plan. The planner answers with up to 64
#    frames -- two seconds, about two metres of walking -- and playing all of
#    them before planning again makes the robot stride past the goal, get
#    aimed back, and oscillate there forever. The official loop consumes one
#    frame per tick and replans every CONTROLLER_DT (8 frames), so the robot
#    is always walking toward where the goal is now.
#
# 3. It cross-fades one plan into the next. Two plans made 8 frames apart do
#    not agree exactly on where the robot is, and butting them together leaves
#    a step at every seam -- 30 of them in a four second walk, which is what
#    makes a tracked walk judder. The reference blends over 8 frames
#    (planner_onnx.md, "Animation Blending"): joint and root positions
#    linearly, root rotation by slerp.
#
# 4. It can follow a path rather than a point. Steering straight at the goal
#    walks through whatever is in between -- on this cell that is the table,
#    for about three quarters of a second. cuRobo already solved a base path
#    that avoids it, because the table is in the collision world it planned
#    against, so the walk follows those waypoints instead of a bearing. The
#    planner takes them directly: has_specific_target with
#    specific_target_positions / specific_target_headings, four at a time
#    (planner_onnx.md, "Advanced Inputs").
#
#   python grasp/walk_clip.py OUT_DIR NAME START_X START_Y START_YAW \
#                                      GOAL_X GOAL_Y GOAL_YAW
#   python grasp/walk_clip.py OUT_DIR NAME --path results/g1_graspgen.json
import json
import math
import os
import sys

import joblib
import numpy as np

SWARM = "/home/sehoon/Documents/GitHub/humanoid-swarm-sim"
sys.path[:0] = [os.path.join(SWARM, "common")]

from gen_planner_motion import (  # noqa: E402
    STAND_ROOT_Z, _inputs, find_planner, qpos_to_motion_lib, standing_qpos,
)
import onnxruntime as ort  # noqa: E402

FPS = 30                # the planner's own output rate (planner_onnx.md)
WALK, IDLE = 2, 0       # planner modes
ARRIVE_M = 0.12         # close enough to stop walking
SETTLE_S = 2.0          # idle at the goal so the tracker ends standing still
MAX_S = 40.0            # a walk that has not arrived by now is not going to

# The numbers below are the deployment stack's own, not tuning of ours:
#   motionbricks/.../demo/controllers.py   _CONTROLLER_DT = 8 / FPS
#   motionbricks/.../demo/full_agent.py    NUM_FRAMES_PER_TOKEN = 4,
#                                          DEFAULT_PRED_OFFSETS  = 4
# The planner returns up to 64 frames -- two seconds, about two metres at
# walking speed -- but the official loop consumes only REPLAN frames of that
# and then plans again from where it has got to (interactive_demo_g1.py calls
# get_next_frame() one frame per tick and generate_new_frames() every tick,
# which no-ops until CONTROLLER_DT has elapsed). Committing a whole planner
# output instead is what makes a robot stride past its goal and turn round.
REPLAN = 8
BLEND = 8               # planner_onnx.md, "Animation Blending"
CTX_FRAMES = 4
CTX_OFFSET = 4 - 4      # -NUM_FRAMES_PER_TOKEN + DEFAULT_PRED_OFFSETS


def _ctx_at(x, y, yaw_deg):
    """4-frame context of the robot standing still at a pose."""
    q = standing_qpos()
    q[0], q[1] = x, y
    half = math.radians(yaw_deg) / 2.0
    q[3], q[6] = math.cos(half), math.sin(half)     # qw, qz (wxyz, yaw only)
    return np.tile(q, (1, 4, 1)).astype(np.float32)


def _slerp(q0, q1, t):
    """Shortest-arc quaternion interpolation, wxyz, as the reference blends."""
    d = float(np.dot(q0, q1))
    if d < 0.0:
        q1, d = -q1, -d
    if d > 0.9995:
        out = q0 + t * (q1 - q0)
        return out / np.linalg.norm(out)
    th = math.acos(max(-1.0, min(1.0, d)))
    s0 = math.sin((1.0 - t) * th) / math.sin(th)
    s1 = math.sin(t * th) / math.sin(th)
    return s0 * q0 + s1 * q1


def _blend(old, new):
    """Cross-fade `new` onto `old` over len(new) frames, in qpos layout:
    [root_xyz(3), root_quat_wxyz(4), dof(29)]."""
    n = len(new)
    out = new.copy()
    for i in range(n):
        w = (i + 1) / (n + 1)
        out[i, 0:3] = (1.0 - w) * old[i, 0:3] + w * new[i, 0:3]
        out[i, 7:] = (1.0 - w) * old[i, 7:] + w * new[i, 7:]
        out[i, 3:7] = _slerp(old[i, 3:7], new[i, 3:7], w)
    return out


def _waypoints(path, here, n=4, step=0.25):
    """The next n points along `path`, starting from whatever is nearest to
    `here` and spaced about `step` metres apart."""
    d = np.linalg.norm(path[:, :2] - here[None, :2], axis=1)
    i = int(np.argmin(d))
    out, last = [], path[i]
    j = i
    while len(out) < n and j < len(path):
        if np.linalg.norm(path[j, :2] - last[:2]) >= step or j == len(path) - 1:
            out.append(path[j])
            last = path[j]
        j += 1
    while len(out) < n:
        out.append(path[-1])
    return np.asarray(out[:n])


def walk_to(sess, start, goal, seed=0, path=None, look_at=None):
    """Roll the planner from `start` to `goal`, steering as the official
    deployment loop does: consume REPLAN frames of a plan, then plan again
    from there with the directions recomputed for where the robot now is."""
    sx, sy, syaw = start
    gx, gy, gyaw = goal
    goal_xy = np.array([gx, gy], np.float32)
    goal_face = np.array([[math.cos(math.radians(gyaw)),
                           math.sin(math.radians(gyaw)), 0.0]], np.float32)

    plan = None
    prev = None                     # (previous plan, where playback left off)
    ctx = _ctx_at(sx, sy, syaw)
    out, arrived_at = [], None

    while len(out) / FPS <= MAX_S:
        root = (np.asarray(out[-1][:2], np.float32) if out
                else np.array([sx, sy], np.float32))
        to_goal = goal_xy - root
        dist = float(np.linalg.norm(to_goal))

        if arrived_at is None and dist <= ARRIVE_M:
            arrived_at = len(out)
        if arrived_at is not None and (len(out) - arrived_at) / FPS >= SETTLE_S:
            break

        if arrived_at is None:
            mode = WALK
            move = np.array([[to_goal[0], to_goal[1], 0.0]], np.float32) / max(dist, 1e-6)
            # Keep your eyes on the thing you are going to pick up. The planner
            # takes movement_direction and facing_direction as separate
            # world-frame vectors (planner_onnx.md), so where the robot walks
            # and where it looks are two different questions: it walks at the
            # stand the plan chose and looks at the object the whole way, which
            # is also what leaves it facing the object when it arrives. Without
            # an object to look at it faces the way it is walking and only
            # turns onto the pick's heading over the last metre.
            if look_at is not None:
                to_obj = look_at - root
                face = np.array([[to_obj[0], to_obj[1], 0.0]], np.float32)
                face /= max(float(np.linalg.norm(to_obj)), 1e-6)
            else:
                face = goal_face if dist < 1.0 else move
        else:
            # Standing at the goal. Idle is a static mode: the reference only
            # replans it on a mode/facing/height change, which arriving is.
            mode = IDLE
            move = np.array([[1e-6, 0.0, 0.0]], np.float32)
            face = goal_face

        inp = _inputs(mode, seed, -1.0)
        inp["movement_direction"] = move
        inp["facing_direction"] = face
        if path is not None and arrived_at is None:
            # Follow cuRobo's collision-free base path. Four waypoints is one
            # token's worth, which is what the model takes.
            wp = _waypoints(path, root)
            inp["has_specific_target"] = np.array([[1]], np.int64)
            inp["specific_target_positions"] = np.stack(
                [[w[0], w[1], STAND_ROOT_Z] for w in wp])[None].astype(np.float32)
            inp["specific_target_headings"] = np.array(
                [[float(w[2]) for w in wp]], np.float32)
        got, npf = sess.run(None, {"context_mujoco_qpos": ctx, **inp})
        n = int(npf.reshape(-1)[0])
        plan = got[0, :n].astype(np.float32)
        if plan.shape[0] < CTX_FRAMES + 1:
            break

        # The model blends the first 4 frames of every call with the context it
        # was given, so they repeat what has already been played; keep them
        # only on the very first call, when there is nothing to repeat.
        body = plan if not out else plan[CTX_FRAMES:]
        take = body[:REPLAN]
        if len(take) == 0:
            break

        # Cross-fade the seam. The old plan has frames for this stretch too --
        # it was cut short, not finished -- so blend across them rather than
        # cutting straight to the new one.
        if prev is not None:
            p_plan, p_idx = prev
            nb = min(BLEND, len(take), max(0, len(p_plan) - p_idx))
            if nb > 0:
                take = take.copy()
                take[:nb] = _blend(p_plan[p_idx:p_idx + nb], take[:nb])
        out.extend(take)

        i = len(plan) - len(body) + len(take)      # where playback now sits
        prev = (plan, i)
        idx = [max(0, min(i + CTX_OFFSET + k, len(plan) - 1)) for k in range(CTX_FRAMES)]
        ctx = plan[idx][None].astype(np.float32)

    qpos = np.asarray(out, np.float32) if out else np.zeros((0, 36), np.float32)
    return qpos, arrived_at is not None


def main():
    out_dir, name = sys.argv[1], sys.argv[2]
    path = None
    if "--stand" in sys.argv:
        # Where to walk to, decided by vision rather than typed here.
        # grasp/where_to_stand.py runs cuRobo's IK on the grasps GraspGen
        # produced from the far capture, with the floating base free, and the
        # answer comes back as a place to stand in the cell's own frame. The
        # start is the robot's own odometry.
        d = json.load(open(sys.argv[sys.argv.index("--stand") + 1]))
        st = d["stand"]
        start = tuple(float(v) for v in sys.argv[3:6])
        goal = (float(st["x"]), float(st["y"]), float(st["yaw_deg"]))
        look_at = None
        # Hand the planner the stand as a target of its own rather than
        # steering it there by bearing. planner_onnx.md's "Advanced Inputs":
        # has_specific_target with specific_target_positions /
        # specific_target_headings. Steering by bearing commits eight frames
        # at a time and walks past -- measured 0.361 m past this goal, which
        # the arrival glide would then have to slide across.
        path = np.array([[goal[0], goal[1], math.radians(goal[2])]],
                        dtype=np.float32)
        print(f"[walk] goal from vision: {np.round(goal, 3)} "
              f"(grasp #{d.get('grasp_index')}, conf {d.get('confidence'):.3f}, "
              f"{d.get('reachable')}/{d.get('total')} grasps reachable)")
    elif "--path" in sys.argv:
        d = json.load(open(sys.argv[sys.argv.index("--path") + 1]))
        path = np.asarray(d["base_path"], dtype=np.float32)
        ow = d.get("object_world")
        look_at = np.asarray(ow[:2], np.float32) if ow else None
        start = (float(path[0][0]), float(path[0][1]), math.degrees(path[0][2]))
        goal = (float(path[-1][0]), float(path[-1][1]), math.degrees(path[-1][2]))
        print(f"[walk] following {len(path)} cuRobo base waypoints")
    else:
        look_at = None
        start = tuple(float(v) for v in sys.argv[3:6])
        goal = tuple(float(v) for v in sys.argv[6:9])

    sess = ort.InferenceSession(find_planner(), providers=["CPUExecutionProvider"])
    qpos, arrived = walk_to(sess, start, goal, path=path,
                            look_at=look_at)
    if len(qpos) == 0:
        raise SystemExit("[walk] planner returned nothing")

    end_xy = qpos[-1, :2]
    err = float(np.linalg.norm(end_xy - np.array(goal[:2])))
    yaw = math.degrees(2.0 * math.atan2(qpos[-1, 6], qpos[-1, 3]))
    yaw = (yaw + 180.0) % 360.0 - 180.0
    print(f"[walk] {len(qpos)} frames ({len(qpos)/FPS:.1f}s @ {FPS}fps)")
    print(f"[walk] start {np.round(start, 3)} -> end "
          f"({end_xy[0]:.3f}, {end_xy[1]:.3f}, {yaw:.1f}deg)")
    print(f"[walk] goal  {np.round(goal, 3)}  position error {err:.3f} m  "
          f"heading error {abs((yaw - goal[2] + 180) % 360 - 180):.1f} deg")
    print(f"[walk] root height {qpos[:, 2].min():.3f}..{qpos[:, 2].max():.3f} m "
          f"(standing {STAND_ROOT_Z})")
    if not arrived:
        print(f"[walk] DID NOT ARRIVE within {MAX_S:.0f}s -- clip not written")
        raise SystemExit(1)

    os.makedirs(out_dir, exist_ok=True)
    dst = os.path.join(out_dir, f"{name}.pkl")
    joblib.dump({name: qpos_to_motion_lib(qpos, FPS)}, dst, compress=True)
    print(f"[walk] wrote {dst}")

    # The pose the arm is actually in when the walk ends, by joint name.
    # cuRobo plans from a start state; handed the config's pre-grasp pose it
    # plans from a pose the robot is not in, and the difference has to be
    # crossed somehow -- which is the hand teleporting. Given this instead,
    # the planner makes the whole motion itself, from where the arm is to the
    # grasp, and there is nothing left to interpolate.
    import os as _os
    _cwd = _os.getcwd()
    _os.chdir("/home/sehoon/Projects/GR00T-WholeBodyControl")
    try:
        from foot_height import load_urdf
        joint_names = load_urdf()[1]
    finally:
        _os.chdir(_cwd)
    end = {n: float(qpos[-1, 7 + i]) for i, n in enumerate(joint_names)}
    side = os.path.join(out_dir, f"{name}_end.json")
    json.dump(end, open(side, "w"), indent=1)
    print(f"[walk] wrote {side} ({len(end)} joints at the walk's last frame)")


if __name__ == "__main__":
    main()
