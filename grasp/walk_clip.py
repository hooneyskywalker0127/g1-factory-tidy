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
SQUAT = 4               # planner_onnx.md: mode 4 takes a height, 0.4..0.8 m
ARRIVE_M = float(__import__("os").environ.get("ARRIVE_M", "0.12"))
# How straight the robot has to be pointing before the clip is allowed to end.
YAW_TOL_DEG = float(__import__("os").environ.get("YAW_TOL_DEG", "1.5"))
# How close is close enough to stop walking. The default 0.12 leaves up to
# 120 mm for play_in_cell to slide the body across with the legs still, which
# is the lurch in the video -- measured, the frame-to-frame change over those
# fifteen slide frames peaks at sixteen times the median.
SETTLE_S = 2.0          # idle at the goal so the tracker ends standing still
# Squatting needs longer than standing still does: the planner ramps toward a
# commanded height rather than jumping to it, and two seconds got 13 mm of a
# 39 mm change.
SQUAT_S = 12.0
STEADY_N = 45          # frames of pelvis height that have to agree
STEADY_M = 0.002       # ...to within this, before the squat counts as landed
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


def walk_to(sess, start, goal, seed=0, path=None, look_at=None,
            squat_to=None, ctx0=None):
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
    # ctx0: the four qpos frames the robot is actually in, when the walk
    # does not start from standing still -- after a kneel and a lift, say.
    # Handed its real context the planner makes the transition itself, the
    # way gen_planner_motion.py chains modes; handed a standing context it
    # would be told the robot is already up.
    ctx = _ctx_at(sx, sy, syaw) if ctx0 is None else np.asarray(ctx0, np.float32)[None]
    out, arrived_at = [], None

    while len(out) / FPS <= MAX_S:
        root = (np.asarray(out[-1][:2], np.float32) if out
                else np.array([sx, sy], np.float32))
        to_goal = goal_xy - root
        dist = float(np.linalg.norm(to_goal))

        if arrived_at is None and dist <= ARRIVE_M:
            arrived_at = len(out)
        hold_s = SQUAT_S if squat_to else SETTLE_S
        held = 0 if arrived_at is None else (len(out) - arrived_at) / FPS
        # Arriving is a position AND a heading. The hold already names the
        # goal heading through specific_target_headings, but the exit only
        # watched the pelvis height, so the clip ended while the robot was
        # still turning onto it -- measured, -89.26 deg against the -93.91 the
        # pick wants. play_in_cell then spent that 4.66 degrees in fifteen
        # frames with the feet planted, which is the leap: frames 710-712 of
        # the delivered video carried 9.8x the median frame difference, the
        # largest in the whole clip.
        #
        # Wait for the heading too. Nothing is slid afterwards because there
        # is nothing left to slide.
        _yaw_err = 180.0
        if out:
            _q = out[-1][3:7]
            _y = math.degrees(math.atan2(
                2.0 * (_q[0] * _q[3] + _q[1] * _q[2]),
                1.0 - 2.0 * (_q[2] ** 2 + _q[3] ** 2)))
            _yaw_err = abs((_y - goal[2] + 180.0) % 360.0 - 180.0)
        _aimed = _yaw_err <= YAW_TOL_DEG
        # --hold-max S: a ceiling on the hold whatever the heading and the
        # pelvis are doing. A kneel breathes, so the 2 mm settle test never
        # passes, and it holds its heading a few degrees off, so the aim
        # test never passes either; without a ceiling the clip ran to MAX_S
        # and the robot knelt still for thirty seconds.
        if (arrived_at is not None and "--hold-max" in sys.argv
                and held >= float(sys.argv[sys.argv.index("--hold-max") + 1])):
            break
        if arrived_at is not None and held >= hold_s and _aimed:
            # With a height commanded, twelve seconds is a floor and not the
            # end. The planner ramps toward the height and from some starts it
            # is still swinging when the clock runs out -- measured, one clip
            # ended 0.7665 with 13.4 mm of travel across its last two seconds
            # while another from the same code ended 0.7525 with 0.6 mm. What
            # the pick needs is not a duration, it is a pelvis that has
            # stopped moving, so wait for the planner's own output to settle
            # and let MAX_S be the thing that gives up.
            if squat_to is None:
                break
            # --hold-max S: the settle test asks the pelvis to sit still to
            # 2 mm, which a kneel never does (it breathes), so the clip ran
            # to MAX_S and the robot knelt for thirty seconds. Give the hold
            # a ceiling of its own.
            recent = np.asarray([o[2] for o in out[-STEADY_N:]], np.float32)
            if len(recent) >= STEADY_N and \
                    float(recent.max() - recent.min()) <= STEADY_M:
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
            #
            # With a height asked for, stand at THAT height instead. The plan
            # the arm will run was made for a torso at a particular height,
            # and a walk does not end at the height the robot spawns at --
            # 0.7888 against 0.750 on this clip. Those 39 mm put every one of
            # the 71 grasps out of reach: cuRobo's IK got within 6.2 mm of the
            # best of them against a 5 mm tolerance, so it returned nothing.
            # planner_onnx.md gives mode 4 a height between 0.4 and 0.8 m for
            # exactly this, so the robot squats the difference rather than the
            # plan being bent to meet it.
            # --hold-mode picks which of the planner's height-aware modes the
            # robot settles into at the goal: 4 squat (0.4-0.8 m), 5 kneel on
            # both knees, 6 kneel on one knee (0.2-0.4 m) -- planner_onnx.md's
            # own table. Kneeling is how the body gets a hand to the floor.
            mode = (int(sys.argv[sys.argv.index("--hold-mode") + 1])
                    if "--hold-mode" in sys.argv else SQUAT) if squat_to else IDLE
            move = np.array([[1e-6, 0.0, 0.0]], np.float32)
            face = goal_face

        inp = _inputs(mode, seed, squat_to if mode not in (WALK, IDLE) else -1.0)
        inp["movement_direction"] = move
        inp["facing_direction"] = face
        if arrived_at is not None and "--hold-target" in sys.argv:
            # Standing still means naming the place, not asking for almost no
            # movement. planner_onnx.md: below 1e-5 the model "falls back to
            # using the facing_direction with a small scaling factor", so a
            # movement_direction of 1e-6 is a slow shove along the way the
            # robot is looking, and over a squat hold it walks the robot off
            # its own stand -- measured 42 to 190 mm, which is exactly what
            # play_in_cell then has to slide back.
            inp["has_specific_target"] = np.array([[1]], np.int64)
            inp["specific_target_positions"] = np.stack(
                [[goal_xy[0], goal_xy[1], STAND_ROOT_Z]] * 4)[None].astype(np.float32)
            inp["specific_target_headings"] = np.array(
                [[math.radians(goal[2])] * 4], np.float32)
        elif path is not None and arrived_at is None:
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


def cut_calm(frames, fps=30, window=15):
    """End the clip on the calmest frame, not on whichever one came last.

    The squat hold is not a static pose. planner_onnx.md gives mode 4 a height
    and the model "searches the reference clip's keyframes and selects the one
    whose root height is closest", and it keeps doing that: measured over the
    hold of one clip, the knees swung 9.84 and 6.38 degrees and the leg joints
    averaged 0.151 rad/s with peaks at 0.584. That is the wobble visible in
    the walk, and it is also the pose --legs-from hands to the pick.

    Cutting is not authoring motion -- every frame is still the planner's.
    Pick the one where the legs are moving least over a short window.
    """
    import numpy as _np
    a = _np.asarray(frames, _np.float64)
    leg = a[:, 7:19]                       # qpos: 7 root dims, then the legs
    sp = _np.abs(_np.diff(leg, axis=0)).max(axis=1) * fps
    n = len(sp)
    if n <= window * 2:
        return frames
    start = int(n * 0.6)
    score = _np.array([sp[max(0, i - window):i + 1].mean()
                       for i in range(start, n)])
    best = start + int(_np.argmin(score))
    print(f"[walk] cutting at the calmest frame {best} of {n} "
          f"({best / fps:.2f}s): legs {score.min():.3f} rad/s over "
          f"{window} frames, against {sp[-window:].mean():.3f} at the end")
    return frames[:best + 1]


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
        if d.get("base_path"):
            # cuRobo's own collision-free route, planned against the desk it
            # saw. Given only the destination the walk goes straight at it and
            # the body clips the desk on the way -- measured, 23 of the 88
            # moving frames inside its footprint, closest 0.086 m.
            path = np.asarray(d["base_path"], dtype=np.float32)
            print(f"[walk] following {len(path)} collision-free waypoints "
                  f"from cuRobo")
        else:
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

    if "--goal-at" in sys.argv:
        # Walk all the way to the pose the pick will actually run from.
        #
        # play_in_cell.py replays the pick with the robot at its own STAND, and
        # whatever distance the walk stops short of that is closed by sliding
        # the body there over fifteen frames with the legs still. Measured on
        # the delivered clips that slide is 70 to 248 mm, and in the video it
        # is the lurch at 8.4 s: frame-to-frame change peaks at sixteen times
        # the median right across those fifteen frames. Walking the last
        # stretch on its own legs costs nothing and there is nothing left to
        # slide.
        _a = sys.argv[sys.argv.index("--goal-at") + 1]
        if _a.endswith(".json"):
            _d = json.load(open(_a))["stand"]
            goal = (float(_d["x"]), float(_d["y"]), float(_d["yaw_deg"]))
        else:
            _i = sys.argv.index("--goal-at")
            goal = (float(sys.argv[_i + 1]), float(sys.argv[_i + 2]),
                    float(sys.argv[_i + 3]))
        _g = np.array([[goal[0], goal[1], math.radians(goal[2])]], np.float32)
        path = _g if path is None else np.concatenate([path, _g])
        print(f"[walk] walking through to {np.round(goal, 3)} -- the pose the "
              f"pick runs from, so nothing has to be slid afterwards")

    sess = ort.InferenceSession(find_planner(), providers=["CPUExecutionProvider"])
    squat_to = (float(sys.argv[sys.argv.index("--squat-to") + 1])
                if "--squat-to" in sys.argv else None)

    # Looking around before walking. The robot is told what to pick up in
    # words and has to find it first; each look is a head-camera frame at a
    # heading, and between looks it turns on the spot. The turn is the
    # planner's own in-place mode: planner_onnx.md, a movement_direction
    # below 1e-5 "falls back to the facing_direction" and the robot turns to
    # face it. --look-yaws lists the headings it looked from, first to last;
    # the walk then starts from the last one.
    looks = []
    if "--look-yaws" in sys.argv:
        yaws = [float(v) for v in
                sys.argv[sys.argv.index("--look-yaws") + 1].split(",")]
        sx, sy = start[0], start[1]
        for ya, yb in zip(yaws[:-1], yaws[1:]):
            turn, _ = walk_to(sess, (sx, sy, ya), (sx, sy, yb))
            looks.append(turn)
            print(f"[walk] turned in place {ya:.0f} -> {yb:.0f} deg to look "
                  f"again: {len(turn)} frames")
        start = (sx, sy, yaws[-1])

    # --from-clip CLIP.pkl: start where that clip ends, in the pose it ends in.
    ctx0 = None
    if "--from-clip" in sys.argv:
        _c = list(joblib.load(sys.argv[sys.argv.index("--from-clip") + 1]).values())[0]
        _d, _r, _q = (np.asarray(_c["dof"])[-4:], np.asarray(_c["root_trans_offset"])[-4:],
                      np.asarray(_c["root_rot"])[-4:])
        ctx0 = np.concatenate([_r, _q[:, [3, 0, 1, 2]], _d], axis=1)
        start = (float(_r[-1, 0]), float(_r[-1, 1]),
                 math.degrees(2.0 * math.atan2(float(_q[-1, 2]), float(_q[-1, 3]))))
        print(f"[walk] starting from the end of {os.path.basename(sys.argv[sys.argv.index('--from-clip') + 1])}: "
              f"{np.round(start, 3)}, pelvis {float(_r[-1, 2]):.3f}")
    qpos, arrived = walk_to(sess, start, goal, path=path,
                            look_at=look_at, squat_to=squat_to, ctx0=ctx0)
    if looks:
        qpos = np.concatenate(looks + [qpos])
    if len(qpos) == 0:
        raise SystemExit("[walk] planner returned nothing")

    if squat_to is not None and arrived and "--cut-at-height" in sys.argv:
        # End the clip where the pelvis is at the height that was asked for.
        #
        # Across three starts the hold swings 0.6, 14.9 and 21.2 mm and the
        # clip ends 2.5, 7.5 and 22.2 mm above the commanded height -- the miss
        # is the swing, and which side it lands on is down to where the cut
        # falls. All three holds pass through the right height on the way
        # (72%, 44%, 32% of their frames); the third simply ends outside it and
        # the pick cannot be planned from there.
        #
        # Stop on the pelvis rather than the clock. The target is the height
        # already commanded, and the arrival is found the way walk_to finds
        # it -- inside ARRIVE_M of the goal -- but measured on the finished
        # clip, because walk_to returns whether it arrived and not when.
        gxy = np.asarray(goal[:2], np.float32)
        near = np.nonzero(np.linalg.norm(qpos[:, :2] - gxy, axis=1) <= ARRIVE_M)[0]
        if len(near):
            a = int(near[0])
            # --cut-to names the height to stop at, when it differs from the
            # one commanded. They are not the same question: the command is
            # what the policy is asked to track, and the cut is which moment of
            # the resulting swing the pick is planned from. The arm's usable
            # band was measured by sweeping pelvis height against plan_grasp
            # (0.750 to 0.760 here), and the middle of that band is a better
            # place to stop than its edge -- cutting at 0.750 itself landed on
            # 0.7499 and would not plan.
            cut_to = (float(sys.argv[sys.argv.index("--cut-to") + 1])
                      if "--cut-to" in sys.argv else squat_to)
            k = a + int(np.argmin(np.abs(qpos[a:, 2] - cut_to)))
            if k + 1 < len(qpos):
                print(f"[walk] arrived at frame {a}; cut at {k} of {len(qpos)}: "
                      f"pelvis {float(qpos[k, 2]):.4f} against the "
                      f"{squat_to:.3f} asked for (last was "
                      f"{float(qpos[-1, 2]):.4f})")
                qpos = qpos[:k + 1]

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
