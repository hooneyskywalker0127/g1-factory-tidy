#!/usr/bin/env bash
# Kill leftover Isaac processes from THIS repo's scripts, nothing else.
#
# play_in_cell.py and capture_rgbd.py end with os._exit(0), which sometimes
# leaves the GPU context behind. Anything else on the GPU (someone else's
# training run) is left alone.
for pat in "grasp/play_in_cell.py" "grasp/capture_rgbd.py"; do
  # the [] keeps this script's own command line from matching
  pids=$(pgrep -f "python .*${pat/\//[/]}" 2>/dev/null || true)
  for p in $pids; do
    [ "$p" = "$$" ] && continue
    kill -9 "$p" 2>/dev/null && echo "[kill] $p ($pat)"
  done
done
sleep 1
