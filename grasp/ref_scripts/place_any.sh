#!/bin/bash
# after the #77 render: carry the hammer to the desk crate and drop it, with the Inspire hand
R=/home/sehoon/Documents/GitHub/g1-factory-tidy; cd $R; S=/tmp/claude-1000/-home-sehoon-Documents-GitHub-g1-factory-tidy/dc54b632-3298-4596-93ea-d9ca60407d34/scratchpad; F=$R/results/$RUN; RUN=${RUN:?}; REF=${REF:?}; OBJ=${OBJ:?}
export HAND=inspire FIX_ROOT=1 TIDY_NO_FLOOR_CARTON=1 TIDY_CRATE_ON_DESK="-1.530,-0.946,0.764,0" OMP_NUM_THREADS=1 PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True ARM_KP_SCALE=4 CLOSE_ORDER=thumb_first CLOSE_MODE=position
source ~/miniconda3/etc/profile.d/conda.sh
say() { echo "[$(date +%H:%M:%S)] $*"; }
while pgrep -f "e2e_grasp_dem[o]|reach_from_pos[e]" > /dev/null; do sleep 15; done
conda activate env_isaaclab
(cd /home/sehoon/Projects/GR00T-WholeBodyControl && python $R/grasp/walk_clip.py $R/results/motion ${REF}carry 0 0 0 --from-clip $R/results/motion/${REF}.pkl --stand $R/results/fable7b/place_target.json --hold-max 2 --rise-first 1.5 2>&1 | grep -a "position error\|frames (" | while read -r l; do say "$l"; done)
conda activate graspgenx
DROP=$(python3 -c "import json; d=json.load(open('$R/results/fable7b/place_target.json'))['drop']; print(' '.join(f'{v:.3f}' for v in d))")
BODY_W=0.1 python grasp/reach_from_pose.py results/motion/${REF}carry.pkl $F/grasps_all.json $F/near --place $DROP --out $F/place.npz 2>&1 | grep -a "\[reach\] place\|Traceback" | while read -r l; do say "$l"; done
conda activate env_isaaclab
python grasp/build_place_reference.py results/motion/${REF}.pkl results/motion/${REF}_hands.npy results/motion/${REF}carry.pkl $F/place.npz ${REF}place 2>&1 | grep -a "\[place\]\|Traceback" | while read -r l; do say "$l"; done
LOOK=$(python3 -c "import json; c=json.load(open('$R/results/fable7b/place_target.json'))['centre']; print(f'{c[0]:.3f} {c[1]:.3f} 0.9')")
OBJ_EVERY=100 python grasp/play_in_cell.py $F/scene.npy --walk results/motion/${REF}place.pkl --walk-only --clip-arms --hands results/motion/${REF}place_hands.npy --no-settle --cam-eye ${CAM:--0.2 -3.4 1.9} --look-at ${LOOK:--0.2 0.0 0.4} --video $F/place.mp4 > $F/place.log 2>&1
grep -a "\[eval\]\|\[obj \] frame  *\(700\|900\|1000\)" $F/place.log | while read -r l; do say "render: $l"; done
next_v() { local o="$1"; mkdir -p "$o"; local n=1; while [ -d "$o/v$n" ]; do n=$((n+1)); done; echo "$o/v$n"; }
D=$(next_v "/home/sehoon/Desktop/참고/영상보관/g1-factory-tidy/09/260928/5지/$OBJ"); mkdir -p $D/evidence
echo "$NOTE" > $D/note.txt; echo "render: $(grep -a "\[eval\]" $F/place.log | tail -1)" >> $D/note.txt
cp $F/place.mp4 $D/${OBJ}_to_crate.mp4; cp $F/place_head.mp4 $D/${OBJ}_to_crate_head.mp4; cp $F/place_wrist.mp4 $D/${OBJ}_to_crate_wrist.mp4; ffmpeg -loglevel error -y -ss 30 -i $F/place.mp4 -frames:v 1 $D/evidence/room_view_check.png

say "PLACE DONE -> $D"
