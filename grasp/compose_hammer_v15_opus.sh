#!/bin/bash
# One video of the hammer run (fable/v15, 2026-10-01): the sentence, what the head camera saw, the handle found
# from the kneel, then the run itself -- room view over the two on-board cameras -- with the stage times read
# off the render log (grasp/compose_fable.sh shape; the 260930 run's frames: kneel 375, close 980, lift 1305,
# standing 1415, carry walk 1570, arrived 1670, place lift 1745, release 2133/2150, end 2185 @30 fps).
set -e
R=/home/sehoon/Documents/GitHub/g1-factory-tidy/results/fable40
D="/home/sehoon/Desktop/참고/영상보관/g1-factory-tidy/09/260930/5지/hammer/fable/v15"
C=$R/compose_v15; mkdir -p "$C"
F="/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FR="/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
V="-r 30 -c:v libx264 -pix_fmt yuv420p -crf 20"
sub() {  # text from to y [size]
  echo "drawtext=fontfile=$F:text='$1':x=(w-text_w)/2:y=$4:fontsize=${5:-34}:fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=14:enable='between(t,$2,$3)'"
}
still() {  # image seconds out filter
  ffmpeg -y -loglevel error -loop 1 -t "$2" -i "$1" -f lavfi -t "$2" -i anullsrc \
    -filter_complex "[0:v]scale=1920:1080:force_original_aspect_ratio=decrease,pad=1920:1080:(ow-iw)/2:(oh-ih)/2:black,$4[out]" \
    -map "[out]" $V "$3"
}
# --- 0. the sentence
ffmpeg -y -loglevel error -f lavfi -t 3.5 -i color=c=black:s=1920x1080:r=30 \
  -vf "drawtext=fontfile=$F:text='Pick up the hammer on the floor and put it in the crate.':x=(w-text_w)/2:y=(h-text_h)/2-60:fontsize=50:fontcolor=white,\
drawtext=fontfile=$FR:text='One sentence. No object labels, no coordinates, no pelvis weld, no teleport.':x=(w-text_w)/2:y=(h-text_h)/2+50:fontsize=34:fontcolor=gray" \
  $V "$C/p0.mp4"
# --- 1. what it saw: three looks from the head camera, the words found on the third
still "$R/look_1/rgb.png" 2.8 "$C/p1a.mp4" "$(sub '1  Head camera (RealSense D435i), first look - no hammer' 0 2.8 60 30)"
still "$R/look_2/rgb.png" 2.2 "$C/p1b.mp4" "$(sub '1  Turned in place - second look' 0 2.2 60 30)"
still "$R/look_3/obj_lang_overlay.png" 3.5 "$C/p1c.mp4" "$(sub '1  Third look - FOUND: the pixels C-RADIO scores for the word hammer' 0 3.5 60 30)"
still "$R/near/obj_part_overlay.png" 3.5 "$C/p1d.mp4" "$(sub '2  Closer look from the kneel - the handle, found by language' 0 3.5 60 30)"
# --- 2. what GraspGen-X and cuRobo made of the close look (cloud + grasps | planned wrist path), drawn by plan_views_hammer_opus.py
ffmpeg -y -loglevel error -i "$R/hammer_views_cloud.mp4" -i "$R/hammer_views_traj.mp4" \
  -filter_complex "\
[0:v]scale=940:-2[a];[1:v]scale=940:-2[b];\
[a][b]hstack=inputs=2,pad=1920:1080:(ow-iw)/2:(oh-ih)/2:black[v];\
[v]$(sub '2  GraspGen-X: 156 grasp candidates on the points the words picked (Inspire hand)' 0 5 60 32),\
$(sub '2  cuRobo whole-body reach from the measured kneel - the handle grasp it committed to' 5 10 60 32)[out]" \
  -map "[out]" $V "$C/p2.mp4"
# --- 3. the run: room view over head + wrist cameras; times from the render log (frames / 30)
ffmpeg -y -loglevel error -i "$D/hammer_v15_c0.mp4" -i "$D/hammer_v15_c0_head.mp4" -i "$D/hammer_v15_c0_wrist.mp4" \
  -filter_complex "\
[0:v]scale=1152:648[top];\
[1:v]scale=768:432[hl];[2:v]scale=768:432[hr];\
[hl][hr]hstack=inputs=2[bot];\
[top]pad=1920:648:(ow-iw)/2:0:black[topp];\
[bot]pad=1920:432:(ow-iw)/2:0:black[botp];\
[topp][botp]vstack=inputs=2[v];\
[v]$(sub '3  GR00T SONIC walks the planner clip - floating base, goal re-aimed from where it stopped' 0 12.5 36),\
$(sub '4  Kneels on both knees - cuRobo plans the reach from the pose it actually reached' 12.5 32.7 36),\
$(sub '5  The Inspire hand closes on the handle (GraspGenX: close, hold, lift)' 32.7 43.5 36),\
$(sub '6  Lifts the hammer' 43.5 47.2 36),\
$(sub '7  Stands up holding it - the rise planned from the measured kneel' 47.2 52.3 36),\
$(sub '8  Carries it to the crate - walk goal x2, SONIC executes about half' 52.3 55.7 36),\
$(sub '9  Place in GraspGenX order - lift, hold, move over the crate' 55.7 71.0 36),\
$(sub '10  Release - into the crate' 71.0 72.9 36),\
$(sub 'Head camera (left)   -   wrist camera (right)' 0 72.9 1030)[out]" \
  -map "[out]" $V "$C/p3.mp4"
# --- 4. end card
ffmpeg -y -loglevel error -f lavfi -t 4 -i color=c=black:s=1920x1080:r=30 \
  -vf "drawtext=fontfile=$F:text='Simulation only (Isaac Lab). Unitree G1 + Inspire RH56.':x=(w-text_w)/2:y=(h-text_h)/2-70:fontsize=44:fontcolor=white,\
drawtext=fontfile=$FR:text='GR00T-WholeBodyControl (SONIC)  -  GraspGenX  -  cuRobo  -  C-RADIO':x=(w-text_w)/2:y=(h-text_h)/2+10:fontsize=34:fontcolor=gray,\
drawtext=fontfile=$FR:text='github.com/hooneyskywalker0127/g1-factory-tidy':x=(w-text_w)/2:y=(h-text_h)/2+70:fontsize=30:fontcolor=gray" \
  $V "$C/p4.mp4"
printf "file '%s'\n" "$C/p0.mp4" "$C/p1a.mp4" "$C/p1b.mp4" "$C/p1c.mp4" "$C/p1d.mp4" "$C/p2.mp4" "$C/p3.mp4" "$C/p4.mp4" > "$C/cc.txt"
ffmpeg -y -loglevel error -f concat -safe 0 -i "$C/cc.txt" -c copy "$R/hammer_v15_youtube.mp4"
cp "$R/hammer_v15_youtube.mp4" "$D/hammer_v15_youtube.mp4"
ffprobe -v error -count_frames -select_streams v:0 -show_entries stream=nb_read_frames,width,height -of csv=p=0 "$D/hammer_v15_youtube.mp4"
