#!/bin/bash
# One floor object, from the spawn to the crate, the way the hammer was done in fable/v1..v15 (2026-09-30):
# every stage that moves the body is executed under physics first (SONIC legs on a floating base, --video, the
# same flags as the final render), the reached state is dumped, and the next stage is planned from that
# measured state (cuRobo: plan from the measured static state). Nothing is welded or teleported.
#
#   RUN=fable42 OBJ=drill VDIR=/home/sehoon/Desktop/참고/영상보관/g1-factory-tidy/10/261001/5지/drill/fable/v1 \
#   bash grasp/floor_to_crate_chain_opus.sh
#
# Stages (each skipped when its product exists, so a later version can resume; FROM=<n> forces from stage n):
#   1 walk1 to the vision stand (stand_kneel.json from floor_object_chain.sh step 3) -> measure
#   2 standing approach legs, each planned to a goal extended x2 past the kneel stand (SONIC walks ~half of a planner
#     walk; a 0.5 m leg is not executed at all), until within 0.25 m of the stand, then
#     the short walk + two-knee kneel (planner mode 5), the goal corrected by the measured kneel error (<= 2 rounds,
#     as v1 passes B3/B4/K5) -> measure -> kneel_from_state
#   3 whole-body reach for every grasp from the measured kneel and object pose, rank_handle order, the candidates
#     under 10 mm replayed under physics (test_grasps_in_isaac, chain step 7), first HELD -> close / hold 100 / lift x4 -> c0
#   4 run c0 -> measure the pose just before the rise (v8)
#   5 rise clip from that pose (walk_clip --rise-first 2), rise_reference RISE_SLOW 2 -> c0r -> measure standing
#   6 carry pose (planner objectCarrying arm, waist 0.1; v11), carry walk with the goal extended x2.0 along
#     robot->stand (SONIC executes ~40% with the waist on the policy; v14) -> measure the walk end
#   7 place in GraspGenX PickAndDropInBinTask order (lift, hold, move above the crate; v15) solved from the
#     measured arrival, the wrist target offset so the OBJECT (not the wrist) is over the crate centre, then
#     the release (settle 30 / open 10 / hold 45) -> final render into VDIR
# Sources for every number are in the hammer notes (09/260930/5지/hammer/fable/vN/note.txt) and docs/DIAGNOSIS_opus.md.
set -uo pipefail
RUN=${RUN:-fable42}; OBJ=${OBJ:-drill}; TAG=${TAG:-${OBJ}1}
VDIR=${VDIR:?video folder}; FROM=${FROM:-1}
R=/home/sehoon/Documents/GitHub/g1-factory-tidy; F=$R/results/$RUN; M=$R/results/motion
P=/home/sehoon/miniconda3/envs/env_isaaclab/bin/python; G=/home/sehoon/miniconda3/envs/graspgenx/bin/python
CRATE=${CRATE:-$R/results/fable7b/place_target.json}
CAM="--cam-eye -0.6 -4.4 2.3 --look-at -0.6 0.0 0.4"
mkdir -p "$VDIR/evidence"; cd $R || exit 1
say(){ echo "$(date +%H:%M:%S) $*" | tee -a "$VDIR/evidence/chain.log"; }
# --- the v8/v14/v15 environment (hammer v1_env.sh + the later knobs); sources in the notes
export HAND=inspire TIDY_NO_FLOOR_CARTON=1 TIDY_CRATE_ON_DESK="-1.530,-0.946,0.764,0" \
       OMP_NUM_THREADS=1 PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
export FIX_ROOT=0 SONIC_ACTUATORS=1 SONIC_LEGS_ONLY=1 PD_BODY=1 OBJECT_NO_SLEEP=1 SETTLE_IDLE=400 TEST_VERBOSE=1
export SOLVER_IT=8 SOLVER_VIT=4 OBJ_SOLVER_IT=16 OBJ_SOLVER_VIT=1 OBJ_MAX_DEPEN_VEL=5.0 OBJECT_MU=1.0
export MIMIC_URDF_RATIO=1 SONIC_LOWER_N=12 HOLD_AFTER_CLOSE=100
export RETARGET_CFG=unitree_g1_29dof_retarget_floor.yml
# fingertip floor clearance for the reach (reach_from_pose_opus: FLOOR_CLEAR + TIP_FLESH; defaults 0.005 + 0.04 were measured on the
# Dex3): floor_object_chain.sh:14 runs the chain with FLOOR_CLEAR=0 TIP_FLESH=0.03. At 4.5 cm the drill's grasps (object top at 6 cm)
# were backed off until the fingers closed on its top surface (drill v1/v2: 17/17 LOST, dz 0.000)
export FLOOR_CLEAR=${FLOOR_CLEAR:-0} TIP_FLESH=${TIP_FLESH:-0}
nframes(){ $P -c "import joblib; print(len(list(joblib.load('$1').values())[0]['dof']))"; }
# open-hand targets for the walk/kneel passes: play_in_cell needs --hands (line 667: without it the hand's body
# collisions stay on and the articulation exploded to 10 m in drill v1's first run); the hammer passes used v1_hands_open.npy
[ -f $M/${TAG}_hands_open.npy ] || $P -c "import numpy as np; h=np.load('$M/v1_hands_open.npy'); np.save('$M/${TAG}_hands_open.npy', np.tile(h[:1], (4000, 1)))"
# run play_in_cell with a stall watchdog: the renders stop with the CPU spinning (v8 f75, v12, v13, v15 f1590);
# the only sign is the log not growing. Judge by the log's mtime (>150 s silent = stalled), kill by pid, rerun.
run_isaac(){ # run_isaac LOG MP4 <play_in_cell args...>   (env already exported by the caller)
  local log=$1 mp4=$2; shift 2; local try
  for try in 1 2 3; do
    timeout 9000 $P -u grasp/play_in_cell_opus.py "$@" --video "$mp4" > "$log" 2>&1 &
    local pid=$!; echo $pid > "$VDIR/evidence/isaac.pid"
    while kill -0 $pid 2>/dev/null; do
      sleep 20
      local age=$(( $(date +%s) - $(stat -c %Y "$log") ))
      if [ $age -gt 150 ]; then
        say "STALL: $(basename $log) silent ${age}s at $(/usr/bin/grep -a '^\[abs \] frame' "$log" | tail -1 | cut -c1-24); killing $pid (try $try)"
        for c in $(pgrep -P $pid); do kill -9 $c 2>/dev/null; done; kill -9 $pid 2>/dev/null; sleep 5
        cp "$log" "${log%.log}_stall$try.log"; break
      fi
    done
    wait $pid 2>/dev/null; local rc=$?
    if [ -f "$mp4" ] && /usr/bin/grep -aq '^\[eval\]' "$log"; then return 0; fi
    say "run $(basename $log) ended rc=$rc without a result; retrying"
  done
  return 1
}
state_xy(){ $P -c "import json;d=json.load(open('$1'));print('%.4f %.4f'%tuple(d['root_pos'][:2]))"; }
frame0_obj(){ /usr/bin/grep -a -m1 '^\[abs \] frame     0' "$1" | sed 's/.*object \[\([^]]*\)\].*/\1/' | awk '{print $1","$2","$3",1,0,0,0"}'; }

# ---------------------------------------------------------------- 1. walk1 -> measure
if [ $FROM -le 1 ] && [ ! -f $F/${TAG}_stateA.json ]; then
  say "stage 1: walk1 to the vision stand $(cat $F/stand_kneel.json | tr -d '\n ' | cut -c1-90)"
  LOOKED=""; for k in 1 2 3; do y=$(echo "90 -30 -150" | cut -d' ' -f$k); LOOKED="${LOOKED:+$LOOKED,}$y"; /usr/bin/grep -aq '^FOUND' $F/look_$k/text.txt && break; done
  OMP_NUM_THREADS=2 timeout 900 $P grasp/walk_clip.py $M ${TAG}_walk1 1.40 0.70 90 --look-yaws "$LOOKED" --stand $F/stand_kneel.json --goal-at $F/stand_kneel.json --hold-target --hold-max 2 2>&1 | /usr/bin/grep -a "frames (\|goal  \|Traceback" | while read -r l; do say "  $l"; done
  CONTACT_FORCE=1 DUMP_STATE=$F/${TAG}_stateA.json OBJ_EVERY=30 run_isaac $F/${TAG}_passA.log $F/${TAG}_passA.mp4 $F/scene.npy --walk $M/${TAG}_walk1.pkl --hands $M/${TAG}_hands_open.npy --walk-only --clip-arms --sonic --no-settle $CAM || { say "STOP stage 1"; exit 1; }
  say "  measured stand $(state_xy $F/${TAG}_stateA.json); object at frame 0: $(frame0_obj $F/${TAG}_passA.log)"
fi
# ---------------------------------------------------------------- 2. approach, then walk2 + kneel, corrected, -> measure
if [ $FROM -le 2 ] && [ ! -f $M/${TAG}_kneel_m.pkl ]; then
  # 2a. standing approach rounds: SONIC executes only ~half of a planner walk and then kneels where it is
  # (drill v1: 0.69 m planned -> 0.31 m short, a goal moved by that error -> 0.55 m short), so walk without the kneel
  # until the measured stand is within 0.25 m of the kneel stand; each round starts from the measured pose
  # (hammer v1: walk1 stopped 0.4 m short and the kneel walk was that short leg)
  CUR=$F/${TAG}_stateA.json; CHAIN=$M/${TAG}_walk1.pkl
  # the goal of each approach leg is extended x APPROACH_K along start->stand (default 2.0): the planner reaches any goal
  # in about the same time (0.69 m -> 112 f, 0.51 m -> 104 f) and SONIC executes about half of a planner walk (GR00T's
  # own loop doubles the requested speed for that reason, full_agent.py:227-228); a 0.5 m leg was not executed at all
  # (drill v1 S1-S3: 0.51 -> 0.55 -> 0.56 m, modes 2 and 1 alike), the x2.0 goal of the hammer's carry walk (v14) was
  for i in 1 2 3 4 5; do
    DIST=$(python3 -c "import json,math;s=json.load(open('$CUR'))['root_pos'];g=json.load(open('$F/stand_kneel.json'))['stand'];print('%.3f'%math.hypot(s[0]-g['x'],s[1]-g['y']))")
    say "stage 2a: measured stand $(state_xy $CUR) is $DIST m from the kneel stand"
    awk "BEGIN{exit !($DIST <= 0.25)}" && break
    if [ ! -f $F/${TAG}_stateS$i.json ]; then
      $P grasp/clip_tools_opus.py from_state $CUR 4 ${TAG}_stateS${i}_from 2>&1 | /usr/bin/grep "\[clip\]" | while read -r l; do say "  $l"; done
      python3 - <<PY | while read -r l; do say "  $l"; done
import json
d = json.load(open("$F/stand_kneel.json")); s = json.load(open("$CUR"))["root_pos"]; k = float("${APPROACH_K:-2.0}")
gx, gy = d["stand"]["x"], d["stand"]["y"]; d["stand"]["x"] = s[0] + k * (gx - s[0]); d["stand"]["y"] = s[1] + k * (gy - s[1])
d["note"] = "approach leg $i: kneel stand extended x%.1f along the measured start -> stand (hammer v14 rule)" % k
json.dump(d, open("$F/${TAG}_stand_S$i.json", "w"), indent=1); print(f"[goal] leg $i: stand {gx:.3f},{gy:.3f} -> planner goal {d['stand']['x']:.3f},{d['stand']['y']:.3f}")
PY
      WALK_MODE=${APPROACH_MODE:-2} OMP_NUM_THREADS=2 timeout 900 $P grasp/walk_clip_mode_opus.py $M ${TAG}_walkS$i 0 0 0 0 0 0 --from-clip $M/${TAG}_stateS${i}_from.pkl --stand $F/${TAG}_stand_S$i.json --goal-at $F/${TAG}_stand_S$i.json --hold-target --hold-max 2 2>&1 | /usr/bin/grep -a "frames (\|goal  \|Traceback" | while read -r l; do say "  $l"; done
      $P grasp/clip_tools_opus.py concat $CHAIN $M/${TAG}_walkS$i.pkl ${TAG}_walkchain$i 2>&1 | /usr/bin/grep "\[clip\]" | while read -r l; do say "  $l"; done
      CONTACT_FORCE=1 DUMP_STATE=$F/${TAG}_stateS$i.json OBJ_EVERY=30 run_isaac $F/${TAG}_passS$i.log $F/${TAG}_passS$i.mp4 $F/scene.npy --walk $M/${TAG}_walkchain$i.pkl --hands $M/${TAG}_hands_open.npy --walk-only --clip-arms --sonic --no-settle $CAM || { say "STOP stage 2a"; exit 1; }
    fi
    CUR=$F/${TAG}_stateS$i.json; CHAIN=$M/${TAG}_walkchain$i.pkl
  done
  echo "$CUR $CHAIN" > $F/${TAG}_approach.txt
  # 2b. the short walk + two-knee kneel from the measured stand, the goal corrected by the measured kneel error (<= 2 rounds)
  $P grasp/clip_tools_opus.py from_state $CUR 4 ${TAG}_stateA_clip 2>&1 | /usr/bin/grep "\[clip\]" | while read -r l; do say "  $l"; done
  cp $F/stand_kneel.json $F/${TAG}_stand_k0.json
  for round in 0 1 2; do
    S=$F/${TAG}_stand_k$round.json
    say "stage 2b round $round: walk2 + two-knee kneel toward $(python3 -c "import json;d=json.load(open('$S'))['stand'];print('%.3f %.3f %.1f'%(d['x'],d['y'],d['yaw_deg']))")"
    WALK_MODE=${APPROACH_MODE:-2} OMP_NUM_THREADS=2 timeout 900 $P grasp/walk_clip_mode_opus.py $M ${TAG}_walk2_k$round 0 0 0 0 0 0 --from-clip $M/${TAG}_stateA_clip.pkl --stand $S --hold-mode 5 --squat-to 0.35 --hold-target --goal-at $S --hold-max 6 2>&1 | /usr/bin/grep -a "frames (\|goal  \|root height\|Traceback" | while read -r l; do say "  $l"; done
    $P grasp/clip_tools_opus.py concat $CHAIN $M/${TAG}_walk2_k$round.pkl ${TAG}_walk12_k$round 2>&1 | /usr/bin/grep "\[clip\]" | while read -r l; do say "  $l"; done
    N=$(nframes $M/${TAG}_walk12_k$round.pkl)
    CONTACT_FORCE=1 SETTLE_AT=$((N-1)) SETTLE_FRAMES=160 DUMP_STATE=$F/${TAG}_stateK_k$round.json OBJ_EVERY=30 run_isaac $F/${TAG}_passK_k$round.log $F/${TAG}_passK_k$round.mp4 $F/scene.npy --walk $M/${TAG}_walk12_k$round.pkl --hands $M/${TAG}_hands_open.npy --walk-only --clip-arms --sonic --no-settle $CAM || { say "STOP stage 2b"; exit 1; }
    ERR=$(python3 - <<PY
import json, math
# the error is measured against the TRUE kneel stand, not the round's (moved) planner goal: drill v1 k1 knelt 0.144 m
# from the stand but 0.322 m from its x2 goal, and the additive rule then aimed 0.5 m past the object
st = json.load(open("$F/${TAG}_stateK_k$round.json")); g = json.load(open("$F/stand_kneel.json"))["stand"]
ex, ey = st["root_pos"][0] - g["x"], st["root_pos"][1] - g["y"]
nxt = json.load(open("$S")); nxt["stand"]["x"] -= ex; nxt["stand"]["y"] -= ey
nxt["note"] = "round $round goal minus the kneel error measured under physics (hammer v1 passes B3/B4)"
json.dump(nxt, open("$F/${TAG}_stand_k$((round+1)).json", "w"), indent=1)
print(f"{math.hypot(ex, ey):.3f} kneel z {st['root_pos'][2]:.3f} object {[round(v,3) for v in st['object_pos']]}")
PY
)
    say "  round $round kneel error $ERR"
    E=$(echo "$ERR" | awk '{print $1}')
    if awk "BEGIN{exit !($E <= 0.15)}"; then break; fi
    [ $round -eq 2 ] && say "  kneel error still $E m after 3 rounds; using the last"
  done
  ln -sf $M/${TAG}_walk12_k$round.pkl $M/${TAG}_walk12.pkl; cp $F/${TAG}_stateK_k$round.json $F/${TAG}_stateK.json; cp $F/${TAG}_passK_k$round.log $F/${TAG}_passK.log
  $P grasp/kneel_from_state_opus.py $M/${TAG}_walk12.pkl $F/${TAG}_stateK.json 160 ${TAG}_kneel_m 2>&1 | /usr/bin/grep "\[kneel\]\|Error" | while read -r l; do say "  $l"; done
fi
# ---------------------------------------------------------------- 3. grasps from the measured kneel -> c0
if [ $FROM -le 3 ] && [ ! -f $M/${TAG}c0.pkl ]; then
  NOW=$(python3 -c "import json; d=json.load(open('$F/${TAG}_stateK.json')); print(','.join('%.5f'%v for v in d['object_pos']+d['object_quat_wxyz']))")
  PLAN=$(frame0_obj $F/${TAG}_passA.log)
  say "stage 3: reach for all grasps from the measured kneel; object now $NOW (planned at $PLAN)"
  [ -f $F/${TAG}_reach_all.npz ] || OBJECT_POSE_NOW=$NOW OBJECT_POSE_PLAN=$PLAN BODY_W=1.0 PELVIS_W=0.1 timeout 3600 $G grasp/reach_from_pose_opus.py $M/${TAG}_kneel_m.pkl $F/grasps_all.json $F/near --all-out $F/${TAG}_reach_all.npz > $F/${TAG}_solve_all.log 2>&1
  /usr/bin/grep -a "grasps moved\|clip ends\|wrote\|Traceback" $F/${TAG}_solve_all.log | cut -c1-140 | while read -r l; do say "  $l"; done
  $G grasp/rank_handle.py $F/near $F/${TAG}_reach_all.npz --out $F/${TAG}_order.txt 2>&1 | /usr/bin/grep -a "\[rank\]" | cut -c1-160 | while read -r l; do say "  $l"; done
  # candidates: rank_handle order, reach error under 10 mm; then the chain's own physics test (floor_object_chain.sh step 7,
  # test_grasps_in_isaac.py: kneel -> grasp, close, lift; GraspGenX validates its grasps by replaying them under physics too).
  # drill v1 skipped the test and took #37 by error alone: the closing fingers pushed the drill 5 cm away (contact 0).
  $P - <<PY
import numpy as np
d = np.load("$F/${TAG}_reach_all.npz"); n_go = int(d["n_go"]); e = d["err"][:, n_go - 1]
order = [int(x) for x in open("$F/${TAG}_order.txt").read().strip().split(",")]
ok = [k for k in order if e[k] < 0.010] or sorted(order[:10], key=lambda i: e[i])[:5]
open("$F/${TAG}_candidates.txt", "w").write(",".join(map(str, ok)))
print(f"[pick] {len(ok)} candidates under 10 mm in handle order: " + ", ".join(f"#{k}({e[k]*1000:.1f})" for k in ok[:8]))
PY
  NC=$(tr ',' '\n' < $F/${TAG}_candidates.txt | wc -l)
  # FIX_ROOT=1: the tester keeps the authored root_joint and its body is kinematic; unfixed, PhysX fails to create the
  # articulation (memory: isaac-runs-must-be-serial). v2's first run crashed here and fell back to #37 again.
  say "stage 3: physics test of $NC candidates ($(cat $F/${TAG}_candidates.txt | cut -c1-60))"
  [ -n "${GRASP_K:-}" ] || FIX_ROOT=1 timeout 5400 $P grasp/test_grasps_in_isaac.py $F/scene.npy $M/${TAG}_kneel_m.pkl $F/${TAG}_reach_all.npz --top $NC --slow --order $F/${TAG}_candidates.txt 2>&1 | /usr/bin/grep -a "\[test\] grasp\|grasps held\|Traceback" | cut -c1-150 > $F/${TAG}_test_grasps.txt
  /usr/bin/grep -a "grasps held\|HELD" $F/${TAG}_test_grasps.txt | head -4 | while read -r l; do say "  $l"; done
  K=$(/usr/bin/grep -a "HELD" $F/${TAG}_test_grasps.txt | head -1 | sed 's/.*grasp #\s*\([0-9]*\).*/\1/')
  if [ -z "$K" ]; then K=$(cut -d, -f1 $F/${TAG}_candidates.txt); say "  no candidate HELD in the test; taking the first, #$K"; else say "  first HELD: #$K"; fi
  # GRASP_K=<id>: take this grasp (chosen by a test run outside the chain, e.g. the tester with TEST_RISE=0)
  [ -n "${GRASP_K:-}" ] && { K=$GRASP_K; say "  GRASP_K given: #$K"; }
  $P - <<PY
import numpy as np
d = np.load("$F/${TAG}_reach_all.npz"); k = $K; q = d["q"][k]; n_go, n_lift = int(d["n_go"]), int(d["n_lift"])
lift = []
for i in range(n_lift * 4):   # the lift at a quarter speed (chain: 3.75 cm/s)
    f = n_go + i / 4; j, a = int(f), f - int(f); j1 = min(j + 1, n_go + n_lift - 1); lift.append((1 - a) * q[j] + a * q[j1])
seq = np.concatenate([q[:n_go], np.repeat(q[n_go-1:n_go], 150, axis=0), np.array(lift)])
np.savez("$F/${TAG}_reach_pick.npz", q=seq, joint_names=d["joint_names"], err=np.zeros(len(seq)), close_from=n_go, lift_from=n_go + 150, grasp=d["grasps"][k], best=k)
PY
  echo $K > $F/${TAG}_grasp.txt
  $P grasp/build_reach_reference_opus.py $M/${TAG}_kneel_m.pkl $F/${TAG}_reach_pick.npz ${TAG}c0 2>&1 | /usr/bin/grep -aE 'frames at 30|hands close|HOLD_AFTER|Traceback|Error' | cut -c1-140 | while read -r l; do say "  $l"; done
fi
KN=$(nframes $M/${TAG}_kneel_m.pkl); C0=$(nframes $M/${TAG}c0.pkl)
export SONIC_LEGS_DIRECT=$((KN-17)),$((C0-71))     # legs PD to the clip at cuRobo gains only while kneeling static (hammer 671,1300 of 688/1371)
# ---------------------------------------------------------------- 4. run c0 -> pose before the rise
if [ $FROM -le 4 ] && [ ! -f $F/${TAG}_stateRise.json ]; then
  say "stage 4: run the pick (c0 $C0 frames, legs direct $SONIC_LEGS_DIRECT) and measure the pose before the rise"
  CONTACT_FORCE=1 DUMP_STATE=$F/${TAG}_stateRise.json DUMP_STATE_AT=$((C0-1)) WALK_MAX_FRAMES=$((C0+1)) OBJ_EVERY=30 run_isaac $F/${TAG}_passRise.log $F/${TAG}_passRise.mp4 $F/scene.npy --walk $M/${TAG}c0.pkl --hands $M/${TAG}c0_hands.npy --walk-only --clip-arms --sonic --no-settle $CAM || { say "STOP stage 4"; exit 1; }
  /usr/bin/grep -a '^\[eval\]' $F/${TAG}_passRise.log | while read -r l; do say "  pick: $l"; done
  LIFTED=$(python3 -c "import json;d=json.load(open('$F/${TAG}_stateRise.json'));print(d['object_pos'][2])")
  REST=$(frame0_obj $F/${TAG}_passA.log | cut -d, -f3)
  if awk "BEGIN{exit !($LIFTED < $REST + 0.05)}"; then say "FALSIFIED: the object is not lifted before the rise (z $LIFTED, rest $REST); the attempt ends here"; PROBE=$F/${TAG}_passRise; STOP=pick; fi
fi
# ---------------------------------------------------------------- 5. rise from the measured pose -> c0r -> standing
if [ -z "${STOP:-}" ] && [ $FROM -le 5 ] && [ ! -f $F/${TAG}_stateStand.json ]; then
  GX=$(python3 -c "import json;d=json.load(open('$CRATE'))['stand'];print('%.3f %.3f %.1f'%(d['x'],d['y'],d['yaw_deg']))")
  $P grasp/clip_tools_opus.py from_state $F/${TAG}_stateRise.json 4 ${TAG}_stateRise_clip 2>&1 | /usr/bin/grep "\[clip\]" | while read -r l; do say "  $l"; done
  OMP_NUM_THREADS=2 timeout 900 $P grasp/walk_clip.py $M ${TAG}_rise 0 0 0 0 0 0 --from-clip $M/${TAG}_stateRise_clip.pkl --rise-first 2 --goal-at $GX 2>&1 | /usr/bin/grep -a "root height\|frames (\|Traceback" | while read -r l; do say "  $l"; done
  RISE_SLOW=2 $P grasp/rise_reference_opus.py $M/${TAG}c0.pkl $M/${TAG}c0_hands.npy $M/${TAG}_rise.pkl ${TAG}c0r 2>&1 | /usr/bin/grep -a "\[rise\]\|Traceback" | while read -r l; do say "  $l"; done
  C0R=$(nframes $M/${TAG}c0r.pkl)
  say "stage 5: run the rise (c0r $C0R frames) and measure standing"
  CONTACT_FORCE=1 DUMP_STATE=$F/${TAG}_stateStand.json DUMP_STATE_AT=$((C0R-1)) OBJ_EVERY=30 run_isaac $F/${TAG}_passStand.log $F/${TAG}_passStand.mp4 $F/scene.npy --walk $M/${TAG}c0r.pkl --hands $M/${TAG}c0r_hands.npy --walk-only --clip-arms --sonic --no-settle $CAM || { say "STOP stage 5"; exit 1; }
  /usr/bin/grep -a '^\[eval\]' $F/${TAG}_passStand.log | while read -r l; do say "  rise: $l"; done
  HZ=$(python3 -c "import json;d=json.load(open('$F/${TAG}_stateStand.json'));print('%.3f %.3f'%(d['object_pos'][2], d['root_pos'][2]))")
  if awk "BEGIN{exit !($(echo $HZ | cut -d' ' -f1) < 0.5 || $(echo $HZ | cut -d' ' -f2) < 0.6)}"; then say "FALSIFIED: not standing with the object (object z, pelvis z = $HZ)"; PROBE=$F/${TAG}_passStand; STOP=rise; fi
fi
# ---------------------------------------------------------------- 6. carry pose + carry walk -> walk end
if [ -z "${STOP:-}" ] && [ $FROM -le 6 ] && [ ! -f $F/${TAG}_stateCrate.json ]; then
  $P grasp/carry_pose_opus.py $M/${TAG}c0r.pkl $M/${TAG}c0r_hands.npy ${TAG}a 60 30 2>&1 | /usr/bin/grep "\[carry-pose\]\|Traceback" | cut -c1-200 | while read -r l; do say "  $l"; done
  $P grasp/clip_tools_opus.py from_state $F/${TAG}_stateStand.json 4 ${TAG}_stand_clip 2>&1 | /usr/bin/grep "\[clip\]" | while read -r l; do say "  $l"; done
  python3 - <<PY
import json
d = json.load(open("$CRATE")); st = json.load(open("$F/${TAG}_stateStand.json")); sx, sy = st["root_pos"][:2]
gx, gy = d["stand"]["x"], d["stand"]["y"]; k = 2.0    # SONIC executed ~40% of the planned displacement with the waist on the policy (hammer v13/v14)
d["stand"]["x"] = sx + k * (gx - sx); d["stand"]["y"] = sy + k * (gy - sy)
d["note"] = "crate stand extended x2.0 along the robot->stand direction (hammer v14)"
json.dump(d, open("$F/${TAG}_stand_ext.json", "w"), indent=1); print(f"[goal] stand {gx:.3f},{gy:.3f} -> extended {d['stand']['x']:.3f},{d['stand']['y']:.3f}")
PY
  OMP_NUM_THREADS=2 timeout 900 $P grasp/walk_clip.py $M ${TAG}_walk 0 0 0 0 0 0 --from-clip $M/${TAG}_stand_clip.pkl --stand $F/${TAG}_stand_ext.json --goal-at $F/${TAG}_stand_ext.json --hold-target --hold-max 2 2>&1 | /usr/bin/grep -a "frames (\|goal  \|Traceback" | while read -r l; do say "  $l"; done
  # a kinematic placeholder place from the planned walk end, only so the reference can be built (the real one comes from the measured arrival)
  DROP=$(python3 -c "import json;d=json.load(open('$CRATE'));print('%.3f %.3f %.3f'%(d['drop'][0],d['drop'][1],d['rim_z']+0.30))")
  BODY_W=0.1 timeout 1200 $G grasp/reach_from_pose_opus.py $M/${TAG}_walk.pkl none none --place $DROP --out $F/${TAG}_place0.npz > $F/${TAG}_solve_place0.log 2>&1
  $P grasp/carry_place_reference_opus.py $M/${TAG}a.pkl $M/${TAG}a_hands.npy $M/${TAG}_walk.pkl $F/${TAG}_place0.npz ${TAG}c1 2>&1 | /usr/bin/grep -a "\[carry\]\|Traceback" | cut -c1-160 | while read -r l; do say "  $l"; done
  A=$(nframes $M/${TAG}a.pkl); W=$(nframes $M/${TAG}_walk.pkl); WEND=$((A+W-1)); echo $WEND > $F/${TAG}_wend.txt
  say "stage 6: run the carry walk (waist on the policy from $A) and measure the arrival at frame $WEND"
  SONIC_WAIST_POLICY_FROM=$A CONTACT_FORCE=1 DUMP_STATE=$F/${TAG}_stateCrate.json DUMP_STATE_AT=$WEND WALK_MAX_FRAMES=$((WEND+2)) OBJ_EVERY=30 run_isaac $F/${TAG}_passWalk.log $F/${TAG}_passWalk.mp4 $F/scene.npy --walk $M/${TAG}c1.pkl --hands $M/${TAG}c1_hands.npy --walk-only --clip-arms --sonic --no-settle $CAM || { say "STOP stage 6"; exit 1; }
  python3 - <<PY | while read -r l; do say "  $l"; done
import json, math
d = json.load(open("$CRATE")); st = json.load(open("$F/${TAG}_stateCrate.json"))
print(f"arrival root {[round(v,3) for v in st['root_pos']]} err to stand {math.hypot(st['root_pos'][0]-d['stand']['x'], st['root_pos'][1]-d['stand']['y']):.3f} m, object {[round(v,3) for v in st['object_pos']]}")
PY
  OZ=$(python3 -c "import json;d=json.load(open('$F/${TAG}_stateCrate.json'));print(d['object_pos'][2])")
  if awk "BEGIN{exit !($OZ < 0.5)}"; then say "FALSIFIED: the object was lost during the carry walk (z $OZ)"; PROBE=$F/${TAG}_passWalk; STOP=carry; fi
fi
# ---------------------------------------------------------------- 7. place from the measured arrival -> final render
if [ -z "${STOP:-}" ] && [ $FROM -le 7 ]; then
  A=$(nframes $M/${TAG}a.pkl); WEND=$(cat $F/${TAG}_wend.txt)
  $P grasp/clip_tools_opus.py from_state $F/${TAG}_stateCrate.json 1 ${TAG}_state 2>&1 | /usr/bin/grep "\[clip\]" | while read -r l; do say "  $l"; done
  TARGET=$(python3 - <<PY
import json, re
d = json.load(open("$CRATE")); st = json.load(open("$F/${TAG}_stateCrate.json"))
# the wrist target is offset so the OBJECT centre ends over the crate centre (v15 left the hammer head on the rim):
# palm - object from the [abs] line nearest the dump frame
best = None
for l in open("$F/${TAG}_passWalk.log", errors="ignore"):
    m = re.match(r"\[abs \] frame\s+(\d+): object \[([^\]]*)\] palm \[([^\]]*)\]", l)
    if m and abs(int(m.group(1)) - $WEND) <= 30: best = m
o = [float(v) for v in best.group(2).split()]; p = [float(v) for v in best.group(3).split()]
tx, ty = d["drop"][0] + (p[0] - o[0]), d["drop"][1] + (p[1] - o[1])
print(f"{tx:.3f} {ty:.3f} {d['rim_z']+0.30:.3f}")
PY
)
  say "stage 7: place (GraspGenX order) from the measured arrival, wrist target $TARGET (object over the crate centre)"
  ORI_W=0.005 PLACE_GRASPGEN=1 BODY_W=0.1 timeout 1200 $G grasp/reach_from_pose_opus.py $M/${TAG}_state.pkl none none --place $TARGET --out $F/${TAG}_place.npz > $F/${TAG}_solve_place.log 2>&1
  /usr/bin/grep -a "\[reach\] place\|Traceback" $F/${TAG}_solve_place.log | cut -c1-200 | while read -r l; do say "  $l"; done
  $P grasp/clip_tools_opus.py cut $M/${TAG}c1.pkl 0 $((WEND+1)) 0 ${TAG}_base 2>&1 | /usr/bin/grep "\[clip\]" | while read -r l; do say "  $l"; done
  $P -c "import numpy as np; h=np.load('$M/${TAG}c1_hands.npy'); np.save('$M/${TAG}_base_hands.npy', h[:$((WEND+1))])"
  $P grasp/clip_tools_opus.py cut $M/${TAG}_base.pkl $WEND $((WEND+1)) 0 ${TAG}_dummy > /dev/null 2>&1
  PLACE_BLEND=30 $P grasp/carry_place_reference_opus.py $M/${TAG}_base.pkl $M/${TAG}_base_hands.npy $M/${TAG}_dummy.pkl $F/${TAG}_place.npz ${TAG}c2 2>&1 | /usr/bin/grep -a "\[carry\]\|Traceback" | cut -c1-160 | while read -r l; do say "  $l"; done
  say "stage 7: final render ($(nframes $M/${TAG}c2.pkl) frames)"
  SONIC_WAIST_POLICY_FROM=$A CONTACT_FORCE=1 OBJ_EVERY=5 run_isaac $F/${TAG}c2.log $F/${TAG}c2.mp4 $F/scene.npy --walk $M/${TAG}c2.pkl --hands $M/${TAG}c2_hands.npy --walk-only --clip-arms --sonic --no-settle $CAM || { say "STOP stage 7"; exit 1; }
  /usr/bin/grep -a '^\[eval\]' $F/${TAG}c2.log | while read -r l; do say "  final: $l"; done
  python3 - <<PY | while read -r l; do say "  $l"; done
import re, json
d = json.load(open("$CRATE")); last = None
for l in open("$F/${TAG}c2.log", errors="ignore"):
    m = re.match(r"\[abs \] frame\s+(\d+): object \[([^\]]*)\]", l)
    if m: last = [float(v) for v in m.group(2).split()]
inside = abs(last[0]-d["centre"][0]) < d["extent"][0]/2 and abs(last[1]-d["centre"][1]) < d["extent"][1]/2 and last[2] < d["rim_z"]
print(f"object end {[round(v,3) for v in last]}; crate centre {[round(v,3) for v in d['centre']]} rim {d['rim_z']:.3f} -> {'IN THE CRATE' if inside else 'not inside'}")
PY
  PROBE=$F/${TAG}c2
fi
# ---------------------------------------------------------------- deliver whatever was rendered last
for s in "" _head _wrist; do [ -f ${PROBE}$s.mp4 ] && cp ${PROBE}$s.mp4 "$VDIR/${OBJ}_$(basename $VDIR)$s.mp4"; done
cp $F/${TAG}_*.log $F/${TAG}_*.json $F/${TAG}_order.txt $F/${TAG}_grasp.txt "$VDIR/evidence/" 2>/dev/null
[ -f ${PROBE}.mp4 ] && ffmpeg -loglevel error -y -sseof -1 -i ${PROBE}.mp4 -frames:v 1 -update 1 "$VDIR/evidence/room_view_end.png"
say "LANDED $(basename $VDIR) mp4=$(ls "$VDIR"/*.mp4 2>/dev/null | wc -l) stop=${STOP:-none}"
say "CHAIN DONE"
