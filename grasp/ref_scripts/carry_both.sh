#!/bin/bash
# The two picks Sehoon confirmed by render (hammer #138 = fable40v2, drill #57 = fable42p) carried to the desk
# crate and released: pick -> stand up in place (2x slower) -> walk -> open above the crate found by language.
# Camera: front view eye, look-at midway between the tool and the crate so the whole path is in frame.
S=/tmp/claude-1000/-home-sehoon-Documents-GitHub-g1-factory-tidy/dc54b632-3298-4596-93ea-d9ca60407d34/scratchpad
say() { echo "[$(date +%H:%M:%S)] $*"; }
export CAM="-0.2 -3.4 1.9" LOOK="-0.9 -0.5 0.7" HAND_KP=40 OBJECT_NO_SLEEP=1
RUN=fable40 REF=fable40v2 OBJ=hammer \
NOTE="(5지) CARRY: hammer #138 (the pick Sehoon confirmed HELD in v3/v6) carried to the desk crate -- pick, stand up in place 2x slower, walk to the crate found by language, open the hand above it. Right arm held at the lift's last pose through the stand-up and the walk (build_place_reference), finger stiffness 40, object never sleeps." \
  bash $S/place_any.sh 2>&1 | while read -r l; do say "HAMMER $l"; done
RUN=fable42 REF=fable42p OBJ=drill \
NOTE="(5지) CARRY: drill #57 (the pick Sehoon confirmed HELD in v3/v5) carried to the desk crate -- pick, stand up in place 2x slower, walk to the crate found by language, open the hand above it. Right arm held at the lift's last pose through the stand-up and the walk (build_place_reference), finger stiffness 40, object never sleeps." \
  bash $S/place_any.sh 2>&1 | while read -r l; do say "DRILL $l"; done
say "CARRY BOTH DONE"
