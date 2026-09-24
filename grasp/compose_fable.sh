#!/bin/bash
# One video that explains the language-to-pick pipeline, in the order it
# happens. Same shape as the 260924 g1_pipeline.mp4: what the robot was told,
# what it saw, what GraspGenX and cuRobo made of it, then the run itself over
# its two on-board cameras. Frame times come from the run's own logs.
set -e
R=/home/sehoon/Documents/GitHub/g1-factory-tidy/results/fable
C=$R/compose; mkdir -p "$C"
D="/home/sehoon/Desktop/참고/영상보관/g1-factory-tidy/09/260924/fable"
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
  -vf "drawtext=fontfile=$F:text='Pick up the box on the table.':x=(w-text_w)/2:y=(h-text_h)/2-60:fontsize=72:fontcolor=white,\
drawtext=fontfile=$FR:text='One sentence. No object labels, no coordinates, no heading typed in.':x=(w-text_w)/2:y=(h-text_h)/2+50:fontsize=34:fontcolor=gray" \
  $V "$C/p0.mp4"

# --- 1. looking for it
still "$R/look_1/rgb.png" 2.8 "$C/p1a.mp4" \
  "$(sub '1  The head camera looks. C-RADIO scores every patch for box, table, floor, wall, shelf' 0 9 40),$(sub 'A box, but not resting on a table  -  NOT FOUND, turn and look again' 0 9 990)"
still "$R/look_2/rgb.png" 2.2 "$C/p1b.mp4" \
  "$(sub '1  The head camera looks. C-RADIO scores every patch for box, table, floor, wall, shelf' 0 9 40),$(sub 'Nothing that looks like a box  -  NOT FOUND, turn again' 0 9 990)"
still "$R/look_3/obj_lang_overlay.png" 3.5 "$C/p1c.mp4" \
  "$(sub '1  FOUND  -  the box whose underside sits on the table top (gap +0.01 m)' 0 9 40),$(sub 'The crate on the rack (+0.36 m) and the box on the floor (-0.69 m) are rejected by geometry' 0 9 990)"

# --- 2. what GraspGenX and cuRobo made of the far look (cloud | arm plan)
ffmpeg -y -loglevel error -i "$R/fable_cloud.mp4" -i "$R/fable_traj.mp4" \
  -filter_complex "\
[0:v]trim=end_frame=300,setpts=PTS-STARTPTS,scale=940:-2[a];\
[1:v]trim=end_frame=300,setpts=PTS-STARTPTS,scale=940:-2[b];\
[a][b]hstack=inputs=2,pad=1920:1080:(ow-iw)/2:(oh-ih)/2:black[v];\
[v]$(sub '2  GraspGenX proposes 80 grasps on the points the words picked, 3 m away' 0 3.3 60 30),\
$(sub '3  cuRobo IK, base locked - 0 of 80 reachable from here, so it has to walk' 3.3 6.6 60 30),\
$(sub '3  cuRobo IK, base free - 1264 stands, keep the one facing the box' 6.6 10 60 30),\
$(sub 'plan_cspace routes it round the desk' 6.6 10 110 30)[out]" \
  -map "[out]" $V "$C/p2.mp4"

# --- 3. the run: room view over head + wrist cameras
# walk clip 760 frames: turns 0-4.3 s, walk to arrival 13.0 s, hold/squat to
# 24.8 s, settle to 25.3 s; then the 830-frame pick: fingers close 12.0-12.6 s
# in, held 150 frames, lift from 20.3 s in.
ffmpeg -y -loglevel error -i "$R/fable.mp4" -i "$R/fable_head.mp4" -i "$R/fable_wrist.mp4" \
  -loop 1 -i "$R/near/obj_lang_overlay.png" \
  -filter_complex "\
[0:v]scale=1144:648[top];\
[1:v]scale=762:432[hl];[2:v]scale=762:432[hr];\
[hl][hr]hstack=inputs=2[bot];\
[top]pad=1920:648:(ow-iw)/2:0:black[topp];\
[bot]pad=1920:432:(ow-iw)/2:0:black[botp];\
[topp][botp]vstack=inputs=2[v0];\
[3:v]scale=372:-1[pip];\
[v0][pip]overlay=x=1544:y=120:enable='between(t,14,24.5)':shortest=1[v];\
[v]$(sub '4  Looking around - GR00T planner turns the robot in place between looks' 0 4.3 40),\
$(sub '5  GR00T planner walks cuRobo route to the stand' 4.3 13.0 40),\
$(sub '6  Arrived - squats to the plan height, looks again, grounds the words again' 13.0 20.0 40),\
$(sub '6  GraspGenX 51 grasps from the new look, cuRobo plans approach, grasp and lift' 20.0 25.33 40),\
drawtext=fontfile=$F:text='Second look - FOUND again':x=1544+(372-text_w)/2:y=340:fontsize=26:fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=10:enable='between(t,14,24.5)',\
$(sub '7  cuRobo arm plan - approach' 25.33 37.3 40),\
$(sub '8  The hand closes on the box' 37.3 45.6 40),\
$(sub '9  And lifts it off the desk' 45.6 53.1 40),\
$(sub 'Head camera (left)   -   wrist camera (right)' 0 53.1 1030)[out]" \
  -map "[out]" $V "$C/p3.mp4"

printf "file '%s'\n" "$C/p0.mp4" "$C/p1a.mp4" "$C/p1b.mp4" "$C/p1c.mp4" "$C/p2.mp4" "$C/p3.mp4" > "$C/cc.txt"
ffmpeg -y -loglevel error -f concat -safe 0 -i "$C/cc.txt" -c copy "$R/fable_pipeline.mp4"
cp "$R/fable_pipeline.mp4" "$D/fable_pipeline.mp4"
ffprobe -v error -count_frames -select_streams v:0 -show_entries stream=nb_read_frames,width,height -of csv=p=0 "$D/fable_pipeline.mp4"
