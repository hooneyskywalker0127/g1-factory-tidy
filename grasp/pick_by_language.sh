#!/usr/bin/env bash
# "Pick up the box." -- the whole chain from those words to the box in hand.
#
#   1. look    the head camera takes a frame; C-RADIO (through cuRobo's own
#              feature-mapping example) grounds the words in it. Nothing found:
#              turn on the spot with GR00T's planner and look again.
#   2. grasps  GraspGenX proposes grasps on the object those words picked.
#   3. stand   cuRobo IK with the base locked: can the arm reach from here?
#              No -> with the base free: where does it have to stand, and
#              plan_cspace: how to get there without walking through the desk.
#   4. walk    GR00T's kinematic planner walks the route, turning first
#              through every heading it looked from.
#   5. look    at the stand, look again; ground the words again; GraspGenX +
#              cuRobo plan the pick from what is seen now.
#   6. pick    replay in the cell, three views, joined on one clock.
#
# Every stage writes under results/fable and is skipped when its output is
# already there, so a run can be resumed.
#
#   bash grasp/pick_by_language.sh ["box" ["table"]]
set -uo pipefail
QUERY="${1:-box}"
ON="${2:-table}"
R=/home/sehoon/Documents/GitHub/g1-factory-tidy
GGX=/home/sehoon/Projects/GraspGenX
F=${FABLE_DIR:-$R/results/fable}
DEST=${FABLE_DEST:-"/home/sehoon/Desktop/참고/영상보관/g1-factory-tidy/09/260924/fable"}
LOG=$F/run.log
START_X=1.40; START_Y=0.70
LOOK_YAWS=(90 -30 -150)          # where it looks, in order, until it finds it
SCENE="-1.30 -0.60 -90"          # where the desk has always been (the cell)
CUT_Z=0.755                      # pelvis height the arm plan is made for (hold_search.txt)
PLAN=${TIDY_PLAN:-$R/results/g1_graspgen.json} # the desk's and box's shape, nothing about the stand
mkdir -p "$F" "$DEST"
source /home/sehoon/miniconda3/etc/profile.d/conda.sh
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True   # the GPU is shared
cd "$R"
say() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"; }
say "===== '$QUERY' on '$ON' -- start ($START_X, $START_Y)"

# ---- 1. look around until the words land on something --------------------
FOUND=""; LOOKED=()
for k in "${!LOOK_YAWS[@]}"; do
  yaw=${LOOK_YAWS[$k]}; n=$((k + 1)); cap=$F/look_$n
  LOOKED+=("$yaw")
  if [ ! -f "$cap/meta_data.json" ]; then
    conda activate env_isaaclab
    python grasp/capture_rgbd.py $START_X $START_Y $yaw --plan "$PLAN" \
        --plan-stand $START_X $START_Y $yaw --scene-stand $SCENE --out "$cap" \
        > "$F/look_$n.log" 2>&1
  fi
  if [ ! -f "$cap/text.txt" ]; then
    conda activate graspgenx
    python grasp/find_by_text.py "$cap" "$QUERY" --on "$ON" --scale 1 \
        2>&1 | grep -a "^\[text\]\|^FOUND\|^NOT FOUND\|Traceback\|Error" \
        | tee "$cap/text.txt"
  fi
  say "look $n (yaw $yaw): $(grep -a "^FOUND\|^NOT" "$cap/text.txt" | tail -1)"
  if grep -aq "^FOUND" "$cap/text.txt"; then FOUND=$cap; FOUND_YAW=$yaw; break; fi
done
[ -n "$FOUND" ] || { say "STOP: '$QUERY' on '$ON' was not seen from any heading"; exit 1; }

# ---- 2. grasps on what the words picked, from across the room ------------
if [ ! -f "$F/far_grasps.json" ]; then
  conda activate graspgenx; cd "$GGX"; mkdir -p "$F/run_far"
  PYOPENGL_PLATFORM=egl PYGLET_HEADLESS=true \
  python end2end/e2e_grasp_demo.py \
      --robot_config end2end/robots/g1_right_arm.yaml --env_config end2end/envs/g1_reach_test.yaml \
      --mesh_file assets/sample_data/hope_objects/GranolaBars.obj \
      --capture_dir "$FOUND" --capture_object obj_lang \
      --task pick_and_lift --playback_mode kinematic --no-viser \
      --num_grasps 200 --topk 80 --grasp_threshold 0.7 --planner graspmoe \
      --moe_outlier_threshold 0.03 --max_plan_attempts 1 --seed 0 \
      --export-grasps "$F/far_grasps.json" > "$F/run_far/run.log" 2>&1
  cd "$R"
fi
[ -f "$F/far_grasps.json" ] || { say "STOP: GraspGenX proposed nothing on the far view"; exit 1; }
say "far grasps: $(python3 -c "import json;print(len(json.load(open('$F/far_grasps.json'))['grasps']))") candidates"

# ---- 3. reach from here? no -> where to stand, and how to get there ------
# where_to_stand.py can also test the stand with the legs in the stance the
# walk ends in (--stance WALK_end.json --pelvis-z Z, commit df33522). Against
# the desk modelled as a solid block that leaves 178 of 1264 stands, all of
# them with the object 67 degrees or more off the body's front, and the arm
# cannot close from there. The block is the wrong desk: the real one is a top
# on four legs and the shins go under it, which is what the pick scene now
# collides against (plan_scene.py). So the stand is chosen for reach and
# facing, and the legs are left the room they actually have.
STANCE=""
if [ ! -f "$F/stand.json" ]; then
  conda activate graspgenx
  python grasp/where_to_stand.py "$F/far_grasps.json" "$FOUND" "$F/stand.json" \
      --from $START_X $START_Y $FOUND_YAW --path "$PLAN" $STANCE 2>&1 \
      | grep -a "^\[stand\]\|Traceback\|Error" | tee "$F/stand.txt"
fi
grep -a "from where it is now\|has to walk\|not required\|with the\|STAND AT\|base path" "$F/stand.txt" | while read -r l; do say "$l"; done
[ -f "$F/stand.json" ] || { say "STOP: no stand"; exit 1; }
read -r SX SY SYAW <<< "$(python3 -c "
import json;d=json.load(open('$F/stand.json'))['stand']
print(f\"{d['x']:.4f} {d['y']:.4f} {d['yaw_deg']:.4f}\")")"

# ---- 4. walk there with GR00T's planner, turning through the looks first --
if [ ! -f "$R/results/motion/$(basename "$F").pkl" ]; then
  conda activate env_isaaclab
  python grasp/walk_clip.py results/motion $(basename "$F") $START_X $START_Y ${LOOKED[0]} \
      --look-yaws "$(IFS=,; echo "${LOOKED[*]}")" \
      --stand "$F/stand.json" --squat-to 0.750 --hold-target --goal-at "$F/stand.json" \
      --cut-at-height --cut-to $CUT_Z \
      2>&1 | grep -a "^\[walk\]\|Traceback\|Error" | tee "$F/walk.txt"
fi
grep -a "turned in place\|position error\|frames (" "$F/walk.txt" | while read -r l; do say "$l"; done
[ -f "$R/results/motion/$(basename "$F").pkl" ] || { say "STOP: the walk did not arrive"; exit 1; }

# ---- 5. at the stand, look again and plan the pick from what is seen -----
if [ ! -f "$F/near/meta_data.json" ]; then
  conda activate env_isaaclab
  python grasp/capture_rgbd.py --stand-from-walk results/motion/$(basename "$F").pkl --back 0.9 \
      --pelvis-from-walk results/motion/$(basename "$F").pkl \
      --plan "$PLAN" --plan-stand $SX $SY $SYAW --scene-stand $SCENE \
      --out "$F/near" > "$F/near.log" 2>&1
  grep -a "^\[cap\] looking\|pelvis z" "$F/near.log" | while read -r l; do say "$l"; done
fi
if [ ! -f "$F/near/text.txt" ]; then
  conda activate graspgenx
  python grasp/find_by_text.py "$F/near" "$QUERY" --on "$ON" --scale 1 \
      2>&1 | grep -a "^\[text\]\|^FOUND\|^NOT FOUND\|Traceback\|Error" | tee "$F/near/text.txt"
fi
say "near look: $(grep -a "^FOUND\|^NOT" "$F/near/text.txt" | tail -1)"
grep -aq "^FOUND" "$F/near/text.txt" || { say "STOP: lost it at the stand"; exit 1; }
if [ ! -f "$F/run/trajectory.json" ]; then
  conda activate graspgenx; cd "$GGX"; mkdir -p "$F/run"
  PYOPENGL_PLATFORM=egl PYGLET_HEADLESS=true \
  python end2end/e2e_grasp_demo.py \
      --robot_config end2end/robots/g1_right_arm.yaml --env_config end2end/envs/g1_reach_test.yaml \
      --mesh_file assets/sample_data/hope_objects/GranolaBars.obj \
      --capture_dir "$F/near" --capture_object obj_lang \
      --task pick_and_lift --playback_mode kinematic --no-viser \
      --num_grasps 200 --topk 80 --grasp_threshold 0.7 --planner graspmoe \
      --num-ik-seeds 128 --max_plan_attempts 20 --hold_after_close_frames 150 --seed 0 \
      --export-trajectory "$F/run/trajectory.json" > "$F/run/run.log" 2>&1
  cd "$R"
fi
say "plan: $(grep -aE "plan_grasp succeeded|All plan_grasp strategies failed" "$F/run/run.log" | sed 's/.*- e2e - [A-Z]* - //' | tail -1)"
[ -f "$F/run/trajectory.json" ] || { say "STOP: no reachable grasp from the stand"; exit 1; }
conda activate env_isaaclab
[ -f "$F/g1_fable.npy" ] || python grasp/traj_from_graspgen.py "$F/run/trajectory.json" "$F/g1_fable" 2>&1 | tee -a "$LOG"

# ---- 6. replay in the cell: the pick, the walk, three views each ---------
if [ ! -f "$F/pick.mp4" ]; then
  python grasp/play_in_cell.py "$F/g1_fable.npy" --pick-stand $SX $SY $SYAW \
      --legs-from results/motion/$(basename "$F").pkl --video "$F/pick.mp4" > "$F/pick.log" 2>&1
fi
say "pick: $(grep -a "\[eval\]" "$F/pick.log" | tail -1)"
if [ ! -f "$F/walk.mp4" ]; then
  python grasp/play_in_cell.py "$F/g1_fable.npy" --pick-stand $SX $SY $SYAW \
      --walk results/motion/$(basename "$F").pkl --walk-only --video "$F/walk.mp4" > "$F/walk.log" 2>&1
fi
grep -aE "stopped at|settling|walk-only" "$F/walk.log" | tail -3 | while read -r l; do say "$l"; done
for v in "" _head _wrist; do
  printf "file '%s'\nfile '%s'\n" "$F/walk$v.mp4" "$F/pick$v.mp4" > "$F/cc$v.txt"
  ffmpeg -y -loglevel error -f concat -safe 0 -i "$F/cc$v.txt" -c copy "$F/fable$v.mp4"
  cp "$F/fable$v.mp4" "$DEST/fable$v.mp4"
  say "$DEST/fable$v.mp4  $(ffprobe -v error -count_frames -select_streams v:0 -show_entries stream=nb_read_frames -of csv=p=0 "$DEST/fable$v.mp4") frames"
done
say "DONE"
