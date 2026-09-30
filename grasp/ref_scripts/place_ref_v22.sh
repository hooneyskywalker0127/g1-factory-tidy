#!/bin/bash
# Build the carry + place reference for a freshly built pick reference ($REF), then render it
# with play_in_cell_opus.py at the gains already exported, and deliver as the next vN.
# Same steps as place_any.sh lines 9-14, but the render is the _opus one so HAND_KD /
# SOLVER_IT / FINGER_COACD reach the sim (Fable's play_in_cell.py reads none of them).
R=/home/sehoon/Documents/GitHub/g1-factory-tidy; cd $R
S=/tmp/claude-1000/-home-sehoon-Documents-GitHub-g1-factory-tidy/dc54b632-3298-4596-93ea-d9ca60407d34/scratchpad
RUN=${RUN:?}; REF=${REF:?}; OBJ=${OBJ:?}; F=$R/results/$RUN
say() { echo "[$(date +%H:%M:%S)] $*"; }
source ~/miniconda3/etc/profile.d/conda.sh
conda activate env_isaaclab
(cd /home/sehoon/Projects/GR00T-WholeBodyControl && python $R/grasp/walk_clip.py $R/results/motion ${REF}carry 0 0 0 \
   --from-clip $R/results/motion/${REF}.pkl --stand $R/results/fable7b/place_target.json --hold-max 2 --rise-first 1.5 \
   2>&1 | grep -a "position error\|frames (" | while read -r l; do say "$l"; done)
conda activate graspgenx
DROP=$(python3 -c "import json; d=json.load(open('$R/results/fable7b/place_target.json'))['drop']; print(' '.join(f'{v:.3f}' for v in d))")
BODY_W=0.1 python grasp/reach_from_pose.py results/motion/${REF}carry.pkl $F/grasps_all.json $F/near --place $DROP \
   --out $F/place_${REF}.npz 2>&1 | grep -a "\[reach\] place\|Traceback" | while read -r l; do say "$l"; done
conda activate env_isaaclab
python grasp/build_place_reference.py results/motion/${REF}.pkl results/motion/${REF}_hands.npy \
   results/motion/${REF}carry.pkl $F/place_${REF}.npz ${REF}place 2>&1 | grep -a "\[place\]\|Traceback" | while read -r l; do say "$l"; done
[ -f results/motion/${REF}place.pkl ] || { say "no place reference -> STOP"; exit 1; }
say "$OBJ render start (kp $HAND_KP kd $HAND_KD effort $HAND_EFFORT)"
OBJ_EVERY=${OBJ_EVERY:-10} python grasp/play_in_cell_opus.py $F/scene.npy --walk results/motion/${REF}place.pkl \
   --walk-only --clip-arms --hands results/motion/${REF}place_hands.npy --no-settle \
   --cam-eye -0.2 -3.4 1.9 --look-at -0.9 -0.5 0.7 --video $F/g.mp4 > $F/g.log 2>&1
grep -a "\[hand\] squeeze\|\[eval\]\|\[obj \] frame  *\(600\|700\|800\|900\)" $F/g.log | while read -r l; do say "$OBJ: $l"; done
next_v() { local o="$1"; mkdir -p "$o"; local n=1; while [ -d "$o/v$n" ]; do n=$((n+1)); done; echo "$o/v$n"; }
D=$(next_v "/home/sehoon/Desktop/참고/영상보관/g1-factory-tidy/09/260928/5지/$OBJ"); mkdir -p $D/evidence
{ echo "$WHY"; echo; echo "손 게인: kp $HAND_KP kd $HAND_KD effort $HAND_EFFORT armature $HAND_ARMATURE, 솔버 $SOLVER_IT/$SOLVER_VIT, FINGER_COACD=$FINGER_COACD.";
  grep -a "\[hand\] squeeze\|\[hand\] finger colliders" $F/g.log; grep -a "\[solver\]" $F/g.log;
  grep -a "\[obj \] frame  *\(600\|700\|800\|900\)" $F/g.log; grep -a "\[eval\]" $F/g.log | tail -1; } > $D/note.txt
cp $F/g.mp4 $D/${OBJ}_to_crate.mp4
[ -f $F/g_head.mp4 ] && cp $F/g_head.mp4 $D/${OBJ}_to_crate_head.mp4
[ -f $F/g_wrist.mp4 ] && cp $F/g_wrist.mp4 $D/${OBJ}_to_crate_wrist.mp4
ffmpeg -loglevel error -y -ss 25 -i $F/g.mp4 -frames:v 1 $D/evidence/room_view_25s.png; cp $F/g.log $D/evidence/render.log
cp $F/test_palm_v22.txt $D/evidence/ 2>/dev/null
say "$OBJ DONE -> $D"
