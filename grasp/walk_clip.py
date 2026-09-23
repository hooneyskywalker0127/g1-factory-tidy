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
#   python grasp/walk_clip.py OUT_DIR NAME START_X START_Y START_YAW \
#                                      GOAL_X GOAL_Y GOAL_YAW
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
CTX_FRAMES = 4
CTX_OFFSET = 4 - 4      # -NUM_FRAMES_PER_TOKEN + DEFAULT_PRED_OFFSETS


def _ctx_at(x, y, yaw_deg):
    """4-frame context of the robot standing still at a pose."""
    q = standing_qpos()
    q[0], q[1] = x, y
    half = math.radians(yaw_deg) / 2.0
    q[3], q[6] = math.cos(half), math.sin(half)     # qw, qz (wxyz, yaw only)
    return np.tile(q, (1, 4, 1)).astype(np.float32)


def walk_to(sess, start, goal, seed=0):
    """Roll the planner from `start` to `goal`, steering as the official
    deployment loop does: consume REPLAN frames of a plan, then plan again
    from there with the directions recomputed for where the robot now is."""
    sx, sy, syaw = start
    gx, gy, gyaw = goal
    goal_xy = np.array([gx, gy], np.float32)
    goal_face = np.array([[math.cos(math.radians(gyaw)),
                           math.sin(math.radians(gyaw)), 0.0]], np.float32)

    plan = None
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
            # Face the way you are walking while there is ground to cover, so
            # the robot does not sidestep the whole way in; the last stretch
            # turns onto the heading the pick needs.
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
        out.extend(take)

        i = len(plan) - len(body) + len(take)      # where playback now sits
        idx = [max(0, min(i + CTX_OFFSET + k, len(plan) - 1)) for k in range(CTX_FRAMES)]
        ctx = plan[idx][None].astype(np.float32)

    qpos = np.asarray(out, np.float32) if out else np.zeros((0, 36), np.float32)
    return qpos, arrived_at is not None


def main():
    out_dir, name = sys.argv[1], sys.argv[2]
    start = tuple(float(v) for v in sys.argv[3:6])
    goal = tuple(float(v) for v in sys.argv[6:9])

    sess = ort.InferenceSession(find_planner(), providers=["CPUExecutionProvider"])
    qpos, arrived = walk_to(sess, start, goal)
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


if __name__ == "__main__":
    main()
