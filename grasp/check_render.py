"""Look at what was rendered, before claiming anything about it.

Two failures have shipped in this project because the numbers in the log said
nothing about them: a leap at the seam, which the eval's dz cannot see, and a
robot thrashing on the floor, which a HELD or LOST verdict does not mention
either. Both are visible in the frames. So read the frames.

  leap  -- a single frame that differs from its neighbour far more than the
           clip's own median. The delivered set peaked at 9.8x on frames
           710-712, which is the fifteen-frame slide onto the stand.
  fall  -- the robot's pelvis leaving the height it should be standing at,
           read from the run's own log rather than guessed from pixels.

Usage: check_render.py VIDEO [LOG]
"""
import glob
import os
import re
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image

LEAP_X = 4.0          # times the median frame difference
FALL_M = 0.10         # metres below the standing height


def leap(video):
    d = tempfile.mkdtemp(prefix="checkframes_")
    subprocess.run(["ffmpeg", "-v", "error", "-i", video, "-vf",
                    "scale=240:-1", "-q:v", "4", os.path.join(d, "%05d.jpg")],
                   check=True)
    fs = sorted(glob.glob(os.path.join(d, "*.jpg")))
    a = np.stack([np.asarray(Image.open(f).convert("L"), float) for f in fs])
    diff = np.abs(np.diff(a, axis=0)).mean(axis=(1, 2))
    med = float(np.median(diff))
    worst = int(np.argmax(diff))
    ratio = diff[worst] / med
    print(f"[check] {len(fs)} frames, median frame difference {med:.3f}")
    print(f"[check] worst frame {worst + 1} at {(worst + 1) / 30:.2f}s, "
          f"{ratio:.1f}x median")
    spikes = [(i + 1, diff[i] / med) for i in np.argsort(diff)[-8:][::-1]
              if diff[i] > LEAP_X * med]
    if spikes:
        print(f"[check] LEAP: {len(spikes)} frames above {LEAP_X:.0f}x --",
              ", ".join(f"{f} ({r:.1f}x)" for f, r in spikes[:5]))
    else:
        print(f"[check] no leap: nothing above {LEAP_X:.0f}x median")
    for f in fs:
        os.remove(f)
    os.rmdir(d)
    return ratio


def fall(log):
    """Pelvis height over the run, from whatever the run printed."""
    txt = open(log, "rb").read().decode("utf-8", "replace")
    zs = [float(m) for m in re.findall(r"root height ([0-9.]+)", txt)]
    zs += [float(m) for m in re.findall(r"root z ([0-9.]+)", txt)]
    if not zs:
        print("[check] no pelvis height in the log -- cannot judge a fall")
        return None
    lo, hi = min(zs), max(zs)
    print(f"[check] pelvis height {lo:.3f}..{hi:.3f} m")
    if hi - lo > FALL_M:
        print(f"[check] FALL: pelvis dropped {(hi - lo) * 1000:.0f} mm")
    return lo


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    leap(sys.argv[1])
    if len(sys.argv) > 2 and os.path.exists(sys.argv[2]):
        fall(sys.argv[2])
