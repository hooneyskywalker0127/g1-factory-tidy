#!/bin/bash
# After the pick: stand up and walk to the crate the words found, look at it
# again from there, put the object in, open the hand. Everything about the
# crate comes from the pictures (place_target.py), nothing is typed.
#   place_chain.sh F PICK_NAME QUERY_OBJ
set -uo pipefail
R=/home/sehoon/Documents/GitHub/g1-factory-tidy; F=$1; PICK=$2; QOBJ=$3
export TIDY_NO_FLOOR_CARTON=1 TIDY_CRATE_ON_DESK="-1.530,-0.946,0.764,0" PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True OMP_NUM_THREADS=1
source ~/miniconda3/etc/profile.d/conda.sh; cd $R
say() { echo "[$(date +%H:%M:%S)] $*"; }
# 1. the crate, from the kneel look: a first estimate, good enough to walk to
conda activate graspgenx
python grasp/find_by_text.py $F/near crate --on table --scale 1 --label obj_crate 2>&1 | grep -a "^FOUND\|^NOT" | while read -r l; do say "crate (kneel look): $l"; done
OBJ=$(python3 -c "import json;c=json.load(open('$F/near/meta_data.json'))['language_labels']['obj_lang']['centre'];print(f'{c[0]:.3f} {c[1]:.3f}')")
python grasp/place_target.py $F/near --label obj_crate --from $OBJ $F/place1.json | while read -r l; do say "$l"; done
# 2. stand up from the kneel holding the object and walk to that stand
conda activate env_isaaclab
rm -f results/motion/${PICK}w.pkl
python grasp/walk_clip.py results/motion ${PICK}w 0 0 0 --from-clip results/motion/$PICK.pkl --stand $F/place1.json --hold-target --goal-at $F/place1.json --squat-to 0.75 --hold-max 3 2>&1 | grep -a "starting from\|frames (\|position error\|Traceback" | while read -r l; do say "$l"; done
# 3. look at the crate again, standing in front of it, from a step back
rm -rf $F/near_crate
python grasp/capture_rgbd.py --stand-from-walk results/motion/${PICK}w.pkl --back 0.6 --pelvis-from-walk results/motion/${PICK}w.pkl --plan $F/scene.json --plan-stand $(python3 -c "import json;d=json.load(open('$F/place1.json'))['stand'];print(f\"{d['x']:.4f} {d['y']:.4f} {d['yaw_deg']:.4f}\")") --scene-stand -1.30 -0.60 -90 --out $F/near_crate > $F/near_crate.log 2>&1
conda activate graspgenx
python grasp/find_by_text.py $F/near_crate crate --on table --scale 1 --label obj_crate 2>&1 | grep -a "^FOUND\|^NOT" | while read -r l; do say "crate (standing look): $l"; done
python grasp/place_target.py $F/near_crate --label obj_crate --from $(python3 -c "import json;d=json.load(open('$F/place1.json'))['stand'];print(f\"{d['x']:.3f} {d['y']:.3f}\")") $F/place2.json | while read -r l; do say "$l"; done
# 4. the reach above the crate, whole body, from the walk's end pose
DROP=$(python3 -c "import json;d=json.load(open('$F/place2.json'))['drop'];print(f'{d[0]:.3f} {d[1]:.3f} {d[2]:.3f}')")
BODY_W=0.3 RETARGET_CFG=unitree_g1_29dof_retarget_floor.yml python grasp/reach_from_pose.py results/motion/${PICK}w.pkl none none --place $DROP --out $F/reach_place.npz 2>&1 | grep -a "^\[reach\] place\|Traceback" | while read -r l; do say "$l"; done
# 5. one reference: pick + carry + place; fingers open above the crate
conda activate env_isaaclab
python grasp/build_place_reference.py results/motion/$PICK.pkl results/motion/${PICK}_hands.npy results/motion/${PICK}w.pkl $F/reach_place.npz ${PICK}full 2>&1 | grep "\[place\]\|Traceback" | while read -r l; do say "$l"; done
python grasp/play_in_cell.py $F/scene.npy --walk results/motion/${PICK}full.pkl --walk-only --clip-arms --hands results/motion/${PICK}full_hands.npy --no-settle --no-video 2>&1 | grep -a "\[obj \] frame\|\[eval\]\|Traceback" | tail -8 | while read -r l; do say "$l"; done
say "PLACE DONE"
