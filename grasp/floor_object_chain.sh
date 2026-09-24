#!/bin/bash
# One floor object, start to render. Usage:
#   RUN=fable14 QUERY="paint can" MESH=painttin_flat.obj OBJ=painttin bash grasp/floor_object_chain.sh
# RUN is results/<RUN>/ holding scene.json (object mesh + desk) and scene.npy; MESH is the
# lying mesh in that folder; OBJ names the videos. Videos go to 영상보관/09/<date>/<RUN>/.
RUN=${RUN:-fable14}; QUERY=${QUERY:-"paint can"}; MESH=${MESH:-painttin_flat.obj}; OBJ=${OBJ:-painttin}
# the object at the right hand -> walk + kneel (capped hold) -> look again ->
# grasps (both planners) -> whole-body reach for every candidate -> physics test.
set -uo pipefail
R=/home/sehoon/Documents/GitHub/g1-factory-tidy; F=$R/results/$RUN; GGX=/home/sehoon/Projects/GraspGenX
export CLOSE_MODE=velocity CLOSE_KD=8 TIDY_NO_FLOOR_CARTON=1 TIDY_CRATE_ON_DESK="-1.530,-0.946,0.764,0" PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True OMP_NUM_THREADS=1
Q=$QUERY
source ~/miniconda3/etc/profile.d/conda.sh; cd $R
say() { echo "[$(date +%H:%M:%S)] $*"; }
# 1. looks
FOUND=""; LOOKED=()
for k in 1 2 3; do
  yaw=$(echo "90 -30 -150" | cut -d' ' -f$k); cap=$F/look_$k; LOOKED+=("$yaw")
  conda activate env_isaaclab
  [ -f $cap/meta_data.json ] || python grasp/capture_rgbd.py 1.40 0.70 $yaw --plan $F/scene.json --plan-stand 1.40 0.70 $yaw --scene-stand -1.30 -0.60 -90 --out $cap > $cap.log 2>&1
  conda activate graspgenx
  python grasp/find_by_text.py $cap "$Q" --on floor --scale 1 2>&1 | grep -a "^\[text\]\|^FOUND\|^NOT\|Traceback" > $cap/text.txt
  say "look $k ($yaw): $(grep -a '^FOUND\|^NOT' $cap/text.txt | tail -1)"
  grep -aq "^FOUND" $cap/text.txt && { FOUND=$cap; FOUND_YAW=$yaw; break; }
done
[ -n "$FOUND" ] || { say "STOP: not found"; exit 1; }
# 2. far grasps + must-walk check
cd $GGX; mkdir -p $F/run_far
PYOPENGL_PLATFORM=egl PYGLET_HEADLESS=true python end2end/e2e_grasp_demo.py --robot_config end2end/robots/g1_right_arm.yaml --env_config end2end/envs/g1_reach_test.yaml --mesh_file $F/$MESH --capture_dir $FOUND --capture_object obj_lang --task pick_and_lift --playback_mode kinematic --no-viser --num_grasps 200 --topk 80 --grasp_threshold 0.7 --planner graspmoe --moe_outlier_threshold 0.03 --max_plan_attempts 1 --seed 0 --export-grasps $F/far_grasps.json > $F/run_far/run.log 2>&1
cd $R; say "far grasps: $(python3 -c "import json;print(len(json.load(open('$F/far_grasps.json'))['grasps']))" 2>/dev/null || echo none)"
python grasp/where_to_stand.py $F/far_grasps.json $FOUND $F/stand_standing.json --from 1.40 0.70 $FOUND_YAW 2>&1 | grep -a "from where it is now\|has to walk\|nothing is reachable" | while read -r l; do say "$l"; done
# 3. stand: object 0.30 m ahead and 0.18 m to the right of the body, facing the object's direction
python3 - <<PY
import json, math, numpy as np
c = np.array(json.load(open("$FOUND/meta_data.json"))["language"]["centre"][:2]); start = np.array([1.40, 0.70])
f = (c - start) / np.linalg.norm(c - start); r = np.array([f[1], -f[0]])
st = c - 0.30 * f - 0.18 * r; yaw = math.degrees(math.atan2(f[1], f[0]))
json.dump({"stand": {"x": float(st[0]), "y": float(st[1]), "yaw_deg": yaw}, "base_path": None, "grasp_index": -1, "confidence": 0.0, "reachable": 0, "total": 0,
           "note": "object 0.30 m ahead, 0.18 m right of the body, from the words' own centre"}, open("$F/stand_kneel.json", "w"), indent=1)
print(f"[stand] kneel stand {np.round(st,3)} yaw {yaw:.1f}, object centre {np.round(c,3)}")
PY
# 4. walk + kneel, hold capped
conda activate env_isaaclab
rm -f results/motion/${RUN}k.pkl
python grasp/walk_clip.py results/motion ${RUN}k 1.40 0.70 ${LOOKED[0]} --look-yaws "$(IFS=,; echo "${LOOKED[*]}")" --stand $F/stand_kneel.json --hold-mode 6 --squat-to 0.35 --hold-target --goal-at $F/stand_kneel.json --hold-max 6 2>&1 | grep -a "frames (\|position error\|root height\|Traceback" | while read -r l; do say "$l"; done
# 5. look again from the kneel, 0.9 m back
read -r SX SY SYAW <<< "$(python3 -c "
import json;d=json.load(open('$F/stand_kneel.json'))['stand']; print(f\"{d['x']:.4f} {d['y']:.4f} {d['yaw_deg']:.4f}\")")"
rm -rf $F/near
python grasp/capture_rgbd.py --stand-from-walk results/motion/${RUN}k.pkl --back 0.9 --pelvis-from-walk results/motion/${RUN}k.pkl --plan $F/scene.json --plan-stand $SX $SY $SYAW --scene-stand -1.30 -0.60 -90 --out $F/near > $F/near.log 2>&1
conda activate graspgenx
python grasp/find_by_text.py $F/near "$Q" --on floor --scale 1 2>&1 | grep -a "^\[text\]\|^FOUND\|^NOT\|Traceback" > $F/near/text.txt
say "near look: $(grep -a '^FOUND\|^NOT' $F/near/text.txt | tail -1)"
grep -aq "^FOUND" $F/near/text.txt || { say "STOP: lost at the kneel"; exit 1; }
# 6. grasps, both planners
cd $GGX
for pl in graspmoe diffusion; do mkdir -p $F/run_$pl
  PYOPENGL_PLATFORM=egl PYGLET_HEADLESS=true python end2end/e2e_grasp_demo.py --robot_config end2end/robots/g1_right_arm.yaml --env_config end2end/envs/g1_reach_test.yaml --mesh_file $F/$MESH --capture_dir $F/near --capture_object obj_lang --task pick_and_lift --playback_mode kinematic --no-viser --num_grasps 400 --topk 120 --grasp_threshold 0.5 --planner $pl --max_plan_attempts 1 --seed 0 --export-grasps $F/grasps_$pl.json > $F/run_$pl/run.log 2>&1
  cd $R; say "$pl: $(grep -aE 'GraspGen returned' $F/run_$pl/run.log | sed 's/.*INFO - //' | tail -1)"; cd $GGX
done
cd $R
python3 - <<PY
import json
a = json.load(open("$F/grasps_graspmoe.json")); b = json.load(open("$F/grasps_diffusion.json"))
a["grasps"] += b["grasps"]; a["confidence"] += b["confidence"]; json.dump(a, open("$F/grasps_all.json", "w"))
print(f"[grasps] merged {len(a['grasps'])}")
PY
# 7. whole-body reach for every candidate, then physics
BODY_W=0.1 RETARGET_CFG=unitree_g1_29dof_retarget_floor.yml python grasp/reach_from_pose.py results/motion/${RUN}k.pkl $F/grasps_all.json $F/near --all-out $F/reach_all.npz 2>&1 | grep -a "clip ends\|wrote\|Traceback" | while read -r l; do say "$l"; done
conda activate env_isaaclab
python grasp/test_grasps_in_isaac.py $F/scene.npy results/motion/${RUN}k.pkl $F/reach_all.npz --top 40 --slow 2>&1 | grep -a "\[test\]\|Traceback" | cut -c1-140 > $F/test_grasps.txt
grep -a "HELD\|grasps held" $F/test_grasps.txt | while read -r l; do say "$l"; done
say "CHAIN DONE"
# --- render the attempt, held or not, so it can be looked at
K=$(grep -a "HELD" $F/test_grasps.txt | head -1 | sed 's/.*grasp #\s*\([0-9]*\).*/\1/')
[ -n "$K" ] || K=$(python3 -c "import numpy as np; d=np.load('$F/reach_all.npz'); print(int(np.argmin(d['err'][:, int(d['n_go'])-1])))")
say "rendering grasp #$K ($( [ -n "$(grep -a HELD $F/test_grasps.txt)" ] && echo held || echo best error, not held))"
python3 - <<PY
import numpy as np
d = np.load("$F/reach_all.npz"); k = $K; q = d["q"][k]; n_go, n_lift = int(d["n_go"]), int(d["n_lift"])
lift = []
for i in range(n_lift * 4):
    f = n_go + i / 4; j, a = int(f), f - int(f); j1 = min(j + 1, n_go + n_lift - 1); lift.append((1 - a) * q[j] + a * q[j1])
seq = np.concatenate([q[:n_go], np.repeat(q[n_go-1:n_go], 150, axis=0), np.array(lift)])
np.savez("$F/reach_pick.npz", q=seq, joint_names=d["joint_names"], err=np.zeros(len(seq)), close_from=n_go, lift_from=n_go + 150, grasp=d["grasps"][k], best=k)
PY
python grasp/build_reach_reference.py results/motion/${RUN}k.pkl $F/reach_pick.npz ${RUN}p 2>&1 | grep "\[ref\]" | while read -r l; do say "$l"; done
python grasp/play_in_cell.py $F/scene.npy --walk results/motion/${RUN}p.pkl --walk-only --clip-arms --hands results/motion/${RUN}p_hands.npy --no-settle --cam-eye -1.8 1.0 1.0 --video $F/pick.mp4 > $F/pick.log 2>&1
grep -a "\[eval\]" $F/pick.log | while read -r l; do say "render: $l"; done
D="/home/sehoon/Desktop/참고/영상보관/g1-factory-tidy/09/260924/$RUN"; mkdir -p $D/evidence
cp $F/pick.mp4 $D/${OBJ}_pick.mp4; cp $F/pick_head.mp4 $D/${OBJ}_pick_head.mp4; cp $F/pick_wrist.mp4 $D/${OBJ}_pick_wrist.mp4
cp $F/near/obj_lang_overlay.png $D/evidence/near_found.png; cp $F/look_3/obj_lang_overlay.png $D/evidence/look_3_found.png 2>/dev/null; cp $F/test_grasps.txt $D/evidence/ 
say "RENDER DONE -> $D"
