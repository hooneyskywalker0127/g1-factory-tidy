#!/usr/bin/env bash
set -euo pipefail
cd /home/sehoon/Documents/GitHub/g1-factory-tidy
source /home/sehoon/miniconda3/etc/profile.d/conda.sh
conda activate env_isaaclab
DEST="/home/sehoon/Desktop/참고/영상보관/g1-factory-tidy/09/260918"
mkdir -p "$DEST" results
python grasp/play_in_cell.py results/g1_graspgen.npy \
  --video results/g1_cell_graspgen.mp4 2>&1 | tee results/play_in_cell.log
cp results/g1_cell_graspgen.mp4 "$DEST/"
echo "copied to $DEST/g1_cell_graspgen.mp4"
