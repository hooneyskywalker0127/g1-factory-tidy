# 진단: "박스 집어" 한마디에서 파지까지, 무엇이 있고 무엇이 없는가

2026-09-24 오후, 새 세션에서 저장소와 세 원본 저장소(GR00T-WholeBodyControl,
cuRobo, GraspGenX)를 다시 읽고 적은 것. 기준 영상은
`영상보관/09/260924/보관/fixed*.mp4`(커밋 53bdcc5 시점).

## 기준 영상이 실제로 하는 일

| 단계 | 하는 일 | 원본 저장소의 기능인가 |
|---|---|---|
| 물체 찾기 | 시뮬레이터의 정답 세그 라벨 `obj_plan`을 이름으로 지정 | 아니오. 언어도 탐색도 없음 |
| 파지 후보 | GraspGenX `e2e_grasp_demo.py --export-grasps` | 예 |
| 설 자리 | cuRobo IK, 부유 베이스(`extra_links`) + `plan_cspace` 경로 | 예. 다만 `PLAN_REACH=0.35` 같은 순위 규칙은 이 저장소 것 |
| 걷기 | GR00T 플래너 ONNX로 만든 클립을 **루트·관절을 매 스텝 써서 재생** | 반. 플래너는 GR00T 것, 추적 정책(SONIC)은 안 돎 |
| 파지 | GraspGenX+cuRobo 팔 7관절 계획을 **골반 고정** 상태로 재생 | 계획은 예. 실행은 실기가 못 하는 방식 |
| 이음매 | 걷기 끝 자세를 15프레임 슬라이드로 설 자리에 맞춤 | 아니오. 이전 세션이 밤새 쫓던 "도약" |

그리고 실행 스크립트(`vision_run.sh`)는 `where_to_stand.py`를 부르지 않고
이전 실행이 남긴 `stand_far3b.json`을 그대로 읽었다. "비전이 설 자리를
정한다"는 문장은 맞지만, 기준 영상은 그 판단을 다시 하지 않았다.

## 원본 저장소와 어긋난 곳

1. **SONIC이 걷지 않는다.** GR00T의 플래너는 참조 동작을 만들 뿐이고, 몸을
   움직이는 것은 추적 정책이다(README: "SONIC uses motion tracking as a
   scalable training task"). 이전 세션은 ONNX 인코더/디코더를 손으로
   이식했고(`grasp/sonic_control.py`) 세 번 실패했다. 관측 정규화 등 학습
   설정을 재현하지 못한 것으로 보인다(관찰 기록 #16115). 공식 경로는
   `gear_sonic/eval_agent_trl_drive.py` + `sonic_release/last.pt`이고, 09-23에
   같은 셀에서 걷기 클립(`walk_in`)을 추적해 영상까지 낸 기록이 있다
   (`walk_render.log`, Terminated 0/265). 체크포인트는 건드리지 않는다.
2. **골반을 용접한다.** decoupled_wbc는 `navigate_cmd`와
   `target_upper_body_pose`를 한 호출로 받아 걷기와 서기를 명령 크기로
   고른다(`g1_decoupled_whole_body_policy.py`). 이전 세션의 `--wbc --drive`
   시도는 실패 영상만 남겼다.
3. **팔만 계획한다.** 팔 7관절 계획은 골반 높이 0.750~0.760 m에서만 풀리고
   (`hold_search.txt`), 걷기가 끝나는 높이는 플래너의 스쿼트 진동에 따라
   0.750~0.777 사이 어디든 된다. 그래서 클립을 높이로 잘라 맞춘다
   (`--cut-at-height`). cuRobo의 답은 전신 계획이다:
   `unitree_g1_29dof_retarget.yml` + `MotionRetargeter`(마지막 커밋 65ffa88이
   하네스까지 확인했고 아직 파지에는 안 붙었다).
4. **설 자리 IK의 높이가 실제와 다르다.** `where_to_stand.py`는 GraspGenX의
   팔 전용 로봇 프레임(`robot_base_pose` z=0.98)에서 만든 파지를 셀로 되돌려
   부유 베이스 IK를 푼다. 걷기가 끝나는 골반 높이(0.755)와 IK가 가정하는
   높이는 같은 값이 아니다. 근거리 재계획이 이를 덮어 왔다.

## 오늘 붙인 것 (`grasp/pick_by_language.sh`)

- **언어 → 물체**: `grasp/find_by_text.py`. cuRobo 자체 예제
  `feature_mapping.py`의 `CRadioInference`(C-RADIO v3-B, `clip` 텍스트 어댑터)로
  머리 카메라 프레임을 패치별로 "box / table / floor / wall / shelf / robot arm /
  ceiling"에 대해 채점하고, "on the table"은 기하로 판정한다: 물체 밑면이
  `table` 영역의 윗면 위 -0.05~+0.10 m 안에 있어야 한다. 원거리 프레임에서
  책상 위 상자 +0.01 m, 선반 위 크레이트 +0.36 m, 바닥 상자 -0.65 m로
  갈린다. 바닥 높이는 `floor`로 본 픽셀의 중앙값에서 읽고, 그 위 0.10 m
  아래 픽셀은 받침면이 될 수 없다(책상 다리 밑동이 `table`로 읽혀 바닥
  상자를 통과시킨 것을 막는다). 언어가 찾은 조각(원거리 302 px, 근거리
  207 px)은 받침면 위 점들로 키워 물체 전체(근거리 1045 px, 정답 1109 px)가
  된다. 결과는 `obj_lang`으로 `seg.png`/`label_map`에 써서 GraspGenX의
  `load_realworld_scene`이 그대로 읽는다.
- **탐색**: 시작 자세가 상자를 등지고(+90°) 있으면 `NOT FOUND`, 플래너
  제자리 회전(-30°, 다시 -150°)으로 다시 본다. 회전 클립은
  `walk_clip.py --look-yaws`가 만들고, 걷기 클립 앞에 붙는다.
- **걸어야 하는가**: `where_to_stand.py --from`이 베이스를 지금 자리에
  잠근 IK로 먼저 묻는다. 결과 0/80 → 걷는다. GPU를 나눠 쓰는 상황에서
  IK 솔버 두 개가 같이 못 올라가 OOM이 났고, 첫 솔버를 지운 뒤 두 번째를
  만들도록 고쳤다.

## 다리 지터링 (도착 후, 전체뷰 24~25 s부터)

세훈님이 눈으로 찾은 것. 걷기가 끝나고 파지가 시작되는 순간부터 다리가
떤다. 원인은 Isaac 쪽 액추에이터 모델이다.

측정은 다리 관절이 **렌더 프레임 사이에 움직인 각도**로 했다(PhysX가
돌려주는 관절 속도는 안 움직인 관절에서도 20~26 rad/s가 찍혀 쓸 수 없다 —
SimulationContext가 시작할 때 "noisy velocities"를 경고하는 그 값이다).
830프레임 파지 동안 다리 12관절의 프레임당 최대 변위 평균:

| 조건 | mrad/프레임 | 파지 |
|---|---|---|
| 기본: DC 모터 모델, 책상 = 속 찬 블록 | 384 | HELD |
| 책상 = 상판 슬래브 + 다리 4개 | 98 | HELD |
| 책상 = 삼각형 mesh 그대로 | (속도만 측정) | LOST |
| 골반 2 cm 들어 발을 띄움 | 285 | LOST |
| PhysX external forces every iteration | 113 | LOST |
| 다리 각도를 기본 자세로(걷기 자세 아님) | 0.08 | HELD |
| **implicit PD 액추에이터, 같은 게인** | **0.09** | HELD |

`G1_29DOF_CFG`는 다리를 `DCMotorCfg`로 돌린다. 이 모델은 관절 속도에 따라
토크 한계를 줄이는데, 그 속도가 PhysX의 잡음 섞인 추정치다. 걷기가 끝난
구부린 자세에서는 무릎과 엉덩이에 중력 토크가 크다: 잘못된 속도 샘플 하나가
토크 한계를 무너뜨리면 다리가 떨어지고, 다음 샘플에서 한계가 돌아오면 속도
한계(20 rad/s)로 되튄다. 프레임마다 ±0.1 rad씩 번갈아 튀는 것이 로그에
그대로 보인다. 다리를 곧게 편 기본 자세에서는 부하가 작아 드러나지 않았고,
걷기 없는 옛 파지 영상에 지터가 없던 이유다. 골반을 용접한 파지에서는
PhysX의 implicit PD로 같은 게인을 쓴다(`--dc-legs`로 옛 모델 비교 가능).
SONIC/WBC 경로는 건드리지 않았지만 같은 모델을 쓰므로 같은 문제를 안고 있을
가능성이 크다(df33522가 잰 26.1 rad/s).

책상 충돌체는 별개의 실제 오류였다: 경계상자 블록(0.2375 m³, 실제 mesh는
0.012 m³)이 상판 아래 빈 공간을 채워 정강이가 그 안에 있었다. 이제 상판
슬래브(두께 0.03 m)와 다리 4개, 모두 mesh에서 읽어 세운다. 상판 접촉면은
전과 같은 평면이라 파지는 그대로 든다(삼각형 mesh로 바꾸면 상자를 놓쳤다).

## 이 세션의 실행 결과 (`results/fable`, 영상은 `영상보관/09/260924/fable/`)

| 단계 | 결과 |
|---|---|
| 1st look (+90°, 선반) | `box` 2영역, `table` 위 없음 → NOT FOUND |
| 2nd look (−30°, 빈 쪽) | `box` 0 px → NOT FOUND |
| 3rd look (−150°) | 책상 위 상자 416 px, 선반 크레이트 +0.36 m, 바닥 상자 −0.69 m 제외 |
| 걸어야 하는가 | 베이스 잠근 IK 0/80 → 걷는다 |
| 설 자리 | IK 16시드 × 80파지 = 1264 자리 중 정면 2.6°, 0.327 m: (−1.463, −0.668, −95.0°) |
| 걷기 | 플래너 클립 745프레임, 골반 0.7550에서 컷, 위치 오차 9 mm, 방향 5.3° |
| 재관측 | 언어 → 1107 px (정답 1109) |
| 파지 계획 | GraspGen 51후보 → cuRobo `full (a=10, lift=20)` |
| 실행 | HELD, dz +94 mm, 다리 0.09 mrad/프레임 (implicit PD, 슬래브 책상) |

SONIC 자체 추적: 같은 걷기 클립을 GR00T 공식 평가기
(`eval_agent_trl_drive.py` + `sonic_release/last.pt`)로 셀 안에서 추적한 영상이
`fable/evidence/groot_sonic_tracks_the_walk.mp4`다. 걷는 것은 정책이 할 수
있다. 아직 파지 실행과 한 시뮬레이션에 있지 않을 뿐이다.

## 남은 것

- 걷기와 파지가 아직 두 클립이다. 걷기는 SONIC이 추적해야 하고(공식
  평가기), 파지는 골반 용접 대신 decoupled_wbc 또는 전신 리타깃 위에서
  실행돼야 한다.
- 파지 계획이 골반 높이 1 cm 창에 갇힌 것은 팔만 계획해서다. 전신 계획으로
  옮기면 `--cut-at-height`는 필요 없어진다.

## 바닥 상자 → 크레이트 (2026-09-24 저녁, 진행 중)

책상은 비우고 크레이트를 올리고, 그래놀라 상자를 바닥에 (−1.10, −0.45)에 세웠다.
같은 파이프라인에 문장만 "box on floor". 결과는 `results/fable2/`,
`영상보관/09/260924/fable2/`.

| 단계 | 결과 |
|---|---|
| 언어 | 3번째 look에서 바닥 상자 878 px. 바닥은 `floor` 픽셀 중앙값 (없으면 깊이의 최저 평면), 물체 중앙 높이가 그 위 0.30 m 안이어야 "바닥 위" |
| 서서 닿는가 | 0/256. 앉는 높이와 허리를 풀어도 43 mm 부족 |
| GR00T 무릎꿇기 | 플래너 mode 6 (kneelOneLeg) 0.35 m → 골반 0.437 m |
| 전신 리치 | cuRobo `MotionRetargeter`, 발 고정·골반 자유·오른손목 목표: 파지 프레임 오차 5 mm |
| SONIC 추적 | 공식 평가기가 걷기·무릎꿇기·리치를 추적 (`evidence/groot_sonic_tracks_walk_kneel_reach.mp4`) |
| 파지 | **0 / 154**. GraspMoE 79 + diffusion 75 후보를 전신 리치로 가져가 셀 물리로 검증(`grasp/test_grasps_in_isaac.py`), 든 것 없음 |

파지가 안 되는 이유, 잰 것:
- GraspMoE 79개는 전부 위에서 내려오는 파지(수직 기준 0~9°). 손바닥이 상자 윗면 4 cm 위에 서고 검지·중지 끝이 윗면 3 cm 위라 손가락이 윗면에 걸려 0.36 rad에서 멈춘다.
- 책상에서 들었던 파지는 옆에서 62° 각도로 들어와 손바닥이 상자 중심 5 cm 위(윗면 아래)에 있었다. diffusion 75개 중 23개가 45° 이상의 옆 접근이고, 그중 #46은 엄지(−0.044)와 검지(+0.049)가 얇은 축을 양쪽에서 잡아 닫을 때 상자가 서 있었다. 그러나 들면 놓치고(15 cm/s), 책상처럼 5 s 기다렸다 천천히 들면 기다리는 동안 상자가 넘어진다(39 mm 폭으로 선 상자).
- 즉 남은 문제는 GR00T도 cuRobo도 아니고, 얇고 잘 넘어지는 상자에 대한 Dex3 한 손 파지 자체다. 다음은 GraspGenX 자체 물리 검증(`--playback_mode dynamic`)으로 후보를 먼저 거르고, 상자가 넘어지지 않는 파지(윗면 모서리 핀치보다 깊게 감싸는)를 고르는 일.

리타깃 세 가지 (모두 측정):
1. cuRobo 부유 베이스는 X→Y→Z 회전 체인, intrinsic XYZ. yaw-pitch-roll로 넣으면 몸이 −4.4°/−12°로 기운다.
2. `solve_sequence`의 첫 프레임은 64시드 전역 IK라 현재 자세를 무시한다. 웜스타트를 클립 마지막 자세로 심고 `solve_frame`을 흘리면 이음매 37 mm.
3. 느슨한 링크 가중치는 골반·다리·몸통 0.1, 왼팔 0.1, 오른어깨·팔꿈치 0.02. 한 값(0.005)이면 다리가 젖혀지고 왼팔이 올라간다.

## 바닥의 공구 → 크레이트 (2026-09-24 밤, 진행 중)

상자 대신 공장에 있을 법한 물체. Dexonomy DGN_5k mesh를 실물 크기로 맞춰 바닥에 두고,
문장은 "hammer on floor" 등. 파이프라인은 같다(look → 언어 → 원거리 파지 → 설 자리 →
걷기+무릎꿇기 → 재관측 → 파지 후보 → 전신 리치 → 셀 물리 검증 → 렌더).

| 시도 | 물체 | 결과 | 잰 원인 |
|---|---|---|---|
| fable3 | 클램프(눕힘) | 0/N | 손끝이 바닥 아래 2 cm — 닫는 순간 클램프 38 cm 날아감 |
| fable4 | 병(세움) | 0/40 | 먼저 닿은 손가락이 병을 밀어 넘어뜨림. 엄지 먼저/나중 순서 바꿔도 같음 |
| fable5 | 망치(세움, 머리 아래) | 0/40, 팔 PD 0/20 | 32 cm 높이로 선 망치. 닫으면 넘어짐(원점 −0.138 m). 손목 자세 오차는 0.5~1.4°라 리치 문제가 아님 |
| fable6 | 망치(눕힘) | 아래 | 눕히면 3번째 look에서 832 px로 찾고, GraspGen 29후보(충돌 필터 뒤: graspmoe 19 + diffusion 10) |

세 저장소 + IsaacLab에서 NVIDIA가 손가락을 어떻게 닫는지 찾은 것:

1. **GraspGenX는 Dex3를 속도 제어로 닫는다.** `end2end/robots/g1_right_arm.yaml`
   `gripper_control_mode: velocity`, 관절당 ±0.25 rad/s, thumb_0은 0. 구현은
   `dynamic_playback.py:641-661` — 강성 0, 감쇠만(kd 800), `JointTargetMode.VELOCITY`.
   주석 그대로: "position mode snaps the fingers to the closed angles and they bat the
   object away." 닫는 20프레임 뒤 150프레임(2.5 s) 팔을 고정한 채 기다리고, 240프레임(4 s)에
   걸쳐 든다. 성공 기준은 `clutter_task.py:64` 물체 z +0.05 m.
2. IsaacLab의 NVIDIA G1 손 설정(`unitree.py:598-611`)도 손가락 강성 10 / 감쇠 0.2 — 팔의
   3000과 달리 일부러 무르게 둔다. 이 저장소의 테스터는 `G1_29DOF_CFG` 기본 강성 20에 실기
   토크 한계 1.4 Nm을 씌워서, 손가락이 닫히는 내내 토크 포화 상태로 속도 한계까지 가속해
   물체에 부딪혔다. 위 0/154, 0/40이 전부 이 방식이었다.
3. GraspGenX 공식 경로는 그리퍼 mesh가 관측 점군 2 cm 안에 오는 파지를 걸러낸다
   (`collision_filter.py`, `collision_threshold=0.02`; 이번엔 120→19). 그러나 점군을 8192점으로
   줄이기 때문에 바닥은 성글어서, 손끝이 바닥을 긁는 옆 파지가 통과한다. 측정: #21/#2/#5의
   손끝 모델 높이 −0.001~+0.006, 셀에서 −0.003. 1.4 Nm으로는 바닥 마찰(μ 3)을 못 이겨 속도
   모드로 6초를 닫아도 검지가 0.16 rad만 움직였다.
4. 바닥 밑 물체를 세워 두면(병, 망치) 어느 파지든 넘어뜨린다. 공장 바닥의 공구는 눕혀
   있다. mesh를 가장 넓은 면으로 눕혀 다시 놓았다(`results/fable6/hammer_flat.obj`).

이 세션에서 바꾼 것: `test_grasps_in_isaac.py`와 `play_in_cell.py`에 `CLOSE_MODE=velocity`
(강성 0, 감쇠 `CLOSE_KD`, `CLOSE_VEL` 0.25 rad/s, thumb_0은 위치 유지) —
IsaacLab에서 `write_joint_stiffness_to_sim(0)` + `write_joint_damping_to_sim(kd)` +
`set_joint_velocity_target`로 정확히 0.25 rad/s가 나오는 것을 손만 띄워 확인했다.
`reach_from_pose.py` `FLOOR_CLEAR` 기본 0.02 m(NVIDIA 필터와 같은 값): 접근축이 아래를
향하면 접근축을 따라 물러나고, 옆에서 오는 파지는 손 전체를 위로 올린다. 테스터의 물체
기준 높이는 1.5 s 안정 뒤 + 후보마다 리치 직전 값으로 바꿨다(0.12 s 뒤 값을 쓰면 옆으로
누운 망치가 아직 구르는 중이라 모든 후보가 −0.016으로 읽혔다).

### 진짜 원인 (2026-09-25 00:10): 손가락이 매 서브스텝 리셋되고 있었다

손만 띄운 실험에서는 속도 드라이브가 정확히 0.25 rad/s인데 테스터에서는 2 s에 0.015 rad
(1/33)였다. 테스터/렌더는 매 물리 서브스텝(33/프레임)마다 몸 관절 29개를
`write_joint_state_to_sim(joint_ids=몸)`으로 쓰는데, IsaacLab은 부분 집합을 써도 **전체**
`joint_pos`/`joint_vel` 버퍼를 PhysX에 밀어 넣는다(`articulation.py:616, 646`). 버퍼는
`robot.update()`를 부른 프레임 시작 값이라, 시뮬레이션되는 손가락이 서브스텝마다 프레임
시작 위치·속도로 되돌아갔다. 위치 모드든 속도 모드든 손가락은 명령의 1/33만 움직였고, 오늘
저녁의 모든 0/N(상자 154, 클램프, 병 40, 망치 40+29+29, 드릴 40, 손전등 36)이 이 상태에서
나온 것이다. 책상 위 상자가 들렸던 이유는 그 경로가 골반을 용접해 관절을 쓰지 않았기
때문이다. 고침: 서브스텝마다 쓰기 전에 `robot.update(dt)`로 버퍼를 현재 값으로 갱신
(`test_grasps_in_isaac.py`, `play_in_cell.py`). 손만 띄운 재현 실험에서 0.015 → 0.495 rad/2 s.

### 버퍼 수정 뒤 (00:10~00:50)에 잰 것

1. **속도 모드(감쇠만)는 여기서 못 쓴다.** 몸을 서브스텝마다 쓰면 PhysX가 시뮬레이션되는
   손가락에 −2.5~−7.7 rad/s의 엉터리 속도를 보고한다(움직이지 않는 관절에). 감쇠 2에 곱하면
   ±15 Nm, 1.4 Nm 한계를 넘어 손가락 토크가 잡음으로 포화된다. 손 감쇠를 0.05로 내리고
   위치 모드로 돌렸다(`HAND_DAMPING`). NVIDIA의 IsaacLab G1 손도 0.2다.
2. **엄지 살은 링크 원점보다 4 cm 더 나간다.** `dex3_tips_in_palm.json`의 손끝은 링크 원점.
   thumb_2 원점이 바닥 위 3.7 cm이면 엄지가 전혀 안 닫히고(0.72 그대로, 마찰 10/1, 90° 회전
   무관), 5.7 cm이면 완전히 닫힌다. 바닥 여유는 원점 + 4 cm(`TIP_FLESH`)에서 잰다. 그러면
   손바닥은 바닥 위 ≥10.5 cm, 집는 지점은 ≥5~6 cm 높이 — 지름 4~5 cm 원통(손전등·약병·양초)
   은 이 손으로 바닥에서 손바닥-아래 파지가 안 된다. 단면 ≥7 cm짜리(캔 6.6 cm 눕힘, 머그)로
   간다.
3. `APPROACH_DEEPER` 기본값은 0으로 되돌렸다. 2.7 cm 깊이 넣기는 1/33 버그 상태에서 내린
   결론이었고, 손가락이 제대로 움직이자 손바닥·근위지가 물체 위에 올라앉았다.
4. 손전등 #12(눕힌 원통, 손바닥 아래) 사진: 손바닥은 정확히 위에 왔고 손가락은 완전히
   닫히는데 엄지가 바닥에 박혀 안 닫혔다(`results/fable7b/snap_12_grasp_A.png`).

### 01:00~01:45

- `PD_BODY`(기본 켬): 책상 상자가 들렸던 방식대로 관절은 전부 PD로 두고 루트만 프레임당
  한 번 놓는다. 서브스텝마다 관절을 쓰는 방식이 손가락에 잡음 속도와 접촉 충격을 줬다.
- "바닥을 뚫고 떨어짐"(−0.146~−0.281)은 뚫린 게 아니라 **원점 측정의 착시**였다. 눕힌 mesh의
  원점을 바닥 위 16 cm에 뒀기 때문에 캔이 손가락에 채여 구르거나 넘어지면 원점이 10 cm 넘게
  내려간다. CCD와 바닥 슬래브를 넣어도 그대로였고, 측정을 mesh 중심(centroid)으로 바꾸니
  사라졌다. "held +0.26"도 같은 착시.
- 캔(6.6 cm) 40후보: 중심 기준 0/40, 최대 +0.042(#54). 손바닥은 바닥 위 ≥10.4 cm여야 엄지
  살이 바닥을 안 파므로 집는 지점이 ≥5~6 cm — 6.6 cm 캔은 윗면만 스친다. 통조림(7.3 cm)
  다음 페인트 통(9 cm)을 돌린다.

## 02:30 상태 정리 (아침에 볼 것)

**영상**: `영상보관/g1-factory-tidy/09/260925/<물체>/` — bottle, clamp, drill, flashlight(가는 것 +
fat11cm), hammer, hammer_standing, pill_bottle, can, tin_can, paint_tin. 각 폴더 `evidence/`에
언어로 찾은 장면, 파지 검증 로그, 손 사진.

**되는 것**: 문장 → C-RADIO로 물체·크레이트 찾기 → 걸어야 하는지 판단 → GR00T 플래너로
걷기+무릎꿇기 → 재관측 → GraspGenX 후보 → cuRobo 전신 리치(손목 오차 3~10 mm) → SONIC 참조
→ 3뷰 렌더. 크레이트 위 놓기 지점·설 자리도 관측에서 뽑힌다(`place_target.py`, fable7b
look_3: 크레이트 866 px, rim z 0.926).

**안 되는 것**: Dex3 한 손으로 바닥 물체를 쥐는 것. 10종 × 30~40후보 전부 0.

**오늘 밤 잡은 버그 3개(전부 시뮬레이터 쪽, 이제 고쳐짐)**:
1. 서브스텝 부분 관절 쓰기가 전체 버퍼를 밀어 손가락이 1/33 속도로 움직임 → `robot.update()`
   매 서브스텝 (`grasp/finger_drive_check.py`로 재현).
2. 몸을 매 서브스텝 쓰면 PhysX가 손가락에 엉터리 속도(−7 rad/s)를 보고 → 감쇠가 토크를
   포화 → 손 감쇠 0.05, 위치 모드, `PD_BODY`(관절은 PD, 루트만 프레임당 한 번).
3. 원점이 mesh 중심에서 11.5 cm 떨어져 있어 물체가 구르면 "바닥을 뚫음/들림"으로 읽힘 →
   중심(centroid) 측정.

**남은 진짜 문제(측정)**: 손바닥을 아래로 한 파지에서 thumb_1이 열린 값 0.72에서 안
움직인다. 물체(손전등 3.7 / 캔 6.6 / 통조림 7.3 / 페인트 통 9 cm), 마찰(10/1), 90° 회전,
높이(손바닥 6~15 cm)에 무관. 손가락은 다 닫힌다. 손가락이 아래로 향한 파지에서는 엄지가
정상으로 닫힌다(#3, #6: −1.05/−1.5). 엄지 살은 thumb_2 원점보다 ~4 cm 더 내려간다(원점
3.7 cm에서 막힘, 5.7 cm에서 닫힘). 즉 엄지가 물체 위쪽에서 비스듬히 눌러 바닥/물체에
박히는 기하다. 다음에 볼 것: (a) GraspGenX의 Dex3 열린 자세(thumb_0=0)를 바꿔 엄지를 옆으로
벌린 상태에서 파지 생성, (b) GraspGenX 자체 물리 검증(`--playback_mode dynamic`, 손가락 토크
1000)이 같은 후보를 드는지 — 든다면 토크 한계 1.4 Nm이 문제, (c) 엄지 thumb_1을 별도로
먼저 닫아 물체를 손가락 쪽으로 밀어 넣는 순서.

`grasp/floor_object_chain.sh`가 물체 하나의 전체 체인(look→걷기→파지→검증→렌더)이다.

(02:40) 엄지 먼저 닫기(`CLOSE_ORDER=thumb_first`, 페인트 통 손바닥-아래 7후보): #62/#30/#17
에서 엄지가 −1.05/−1.5까지 닫혔다 — 즉 엄지는 손가락이 먼저 물체를 누르면 막히고, 먼저
가면 닫힌다. 그래도 물체는 안 움직였다(0/7). 물체가 엄지와 손가락 사이에 있지 않다.

## 손잡이를 잡기 (2026-09-25 밤, 영상은 `260926/<물체>/`)

세훈님 지적: 영상마다 손이 가장 두꺼운 부분(망치 머리)을 잡으러 간다. 사람은 손잡이를 잡는다.

- **조사(GraspGenX 소스)**: 파지 생성에 영역·부위·언어 조건은 전혀 없다. 모델 입력은 물체
  점군뿐(`grasp_server.py:416`). 판별기는 점수만 낸다. 따라서 "어디를 잡을지"의 유일한
  손잡이는 **어떤 점을 물체로 주느냐**다. 물체 라벨이 손잡이 픽셀만 덮으면 나머지(머리)는
  씬 점군으로 들어가 충돌 필터의 장애물이 된다(`scene_loaders.py:191`). OBB 가지에
  `dense-topandside`(옆 접근)가 구현돼 있으나 CLI `choices`에 빠져 있어 한 단어 추가했다.
  Dexonomy는 같은 Dex3 손으로 망치·펜치·드릴에 유형별(1_Large_Diameter=감싸기) 파지를
  합성해 두었다(`output/tools_unitree_g1/grasp_data/...`, 손바닥 자세 + 7관절).
- **손잡이 인식(로봇이 판단, 좌표 없음)**: `grasp/find_part.py`. 문장으로 찾은 물체 픽셀
  안에서 C-RADIO로 "handle" 대 "hammer head"를 채점해 씨앗을 얻고(147 px), 물체를 바닥 위
  점으로 완성한 뒤 긴 축을 따라 단면 폭을 재서 **씨앗 쪽의 가는 구간**을 손잡이로 삼는다
  (망치 789 px, 클램프 680, 드릴 901; `evidence/handle_found_by_language.png`). 이 픽셀만
  `obj_part`로 써서 GraspGenX에 준다 → 후보 94개가 전부 손잡이 위에 놓인다.
- **접근**: 옆에서 오는 파지의 사전 자세를 손바닥 축 10 cm 뒤 대신 **10 cm 위**로 바꿨다
  (`DESCEND`). 손바닥 축을 따라 들어오면 손이 망치를 21 cm 밀어 머리로 세워 버렸다
  (`evidence/grasp54_kicked_hammer.png`). #54가 "들린" 것은 그 우연이었고 재검증에서 진다.
- **결과**: 손잡이 후보 94개 × (엄지 먼저, 위치 모드, PD 몸통, 위에서 내려오기): 1/60(#55,
  +51 mm)이었으나 재검증 3회 중 0. Dexonomy 감싸기 파지 6개: 손끝이 손잡이 밑으로 들어가는
  자세라 바닥에서는 성립하지 않음(0/6). 원인은 여전히 엄지 기하: 손바닥 아래 5.4 cm + 살
  4 cm에 매달린 엄지가 바닥 높이 손잡이(중심 1.5 cm)를 집으려면 바닥을 뚫어야 한다.
  지금 엄지 벌림(thumb_0 ±0.7)으로 옆에서 집는 형태를 검증 중.
- 픽 → 운반 → 크레이트 놓기 전체 참조(1027프레임)와 렌더는 됐다
  (`hammer_handle_to_crate*.mp4`; 걷기·크레이트 위 손목 오차 5 mm). 망치가 손에 안 남아
  있어 놓기는 빈손이지만, 파지만 되면 이 경로로 바로 크레이트까지 간다.

### Dexonomy 탁상 합성 (23:00~)

- Dexonomy는 같은 Dex3 손(`assets/hand/unitree_g1`)에 유형 템플릿 3개(1_Large_Diameter,
  3_Medium_Wrap, 6_Prismatic_4_Finger)로 파지를 합성한다. 저장소에 있던 결과는 전부
  `floating`(바닥 없음)이라 손가락이 손잡이 밑으로 들어가 바닥에서는 못 쓴다(6/6 실패).
- 바닥 평면을 넣은 `tabletop` scene_cfg를 만들었다(`assets/object/TOOLS/scene_cfg/<obj>/tabletop/
  scale014_p{0,1}.npy`: 물체는 `info/tabletop_pose.json`의 안정 자세, `table: {type: plane}`).
  코드 변경 없이 init 단계가 평면 반공간 필터(손 골격 z ≥ 2 cm)를 적용해 4물체×2자세에서
  400개 init을 만들었다(GPU 5 s). MuJoCo 정련(op=grasp)은 전부 거절했고 워커 로그가 안 나와
  이유를 못 봤다(`file_util.py`의 safe_wrapper에 stderr 출력 한 줄 추가). 그래서 init(템플릿
  자세 + 바닥 필터)을 후보로 가져와 셀 물리로 판정한다(`grasp/dexonomy_grasps.py --tabletop`).
  망치(hammer_005, 0.14 배 = 25 cm, 안정 자세 p1) 50개, 리치 오차 2~4 mm. 검증 진행 중.
- 클램프 손잡이 파지 #32(+79 mm)는 재검증 2회 모두 실패, PD 렌더에서도 안 들림 → 우연.
  망치 손잡이 #54/#55도 같은 패턴. 지금까지 "들림"으로 나온 것은 전부 재현이 안 된다.

## 26일 아침에 볼 것 (2026-09-25 23:40 기준)

**영상** `영상보관/g1-factory-tidy/09/260926/<물체>/`: hammer(손잡이 파지 #54 픽, 픽→운반→
크레이트 전체, 26일 체인, Dexonomy 감싸기), clamp(손잡이 #32 + PD 렌더), drill, flashlight,
paint_tin. `evidence/`에 언어로 찾은 손잡이(분홍), 검증 로그, 손 사진.

**된 것**
- 로봇이 "손잡이"를 스스로 찾는다: 문장으로 찾은 물체 안에서 C-RADIO "handle" 씨앗 + 긴 축을
  따라 가는 구간(망치 789 px, 클램프 671, 드릴 901). 좌표는 어디에도 없다.
- GraspGenX 후보가 전부 손잡이 위에 놓인다(영상에서 손이 손잡이로 간다).
- 픽 → 일어서서 걷기 → 크레이트 위 손목(오차 5 mm) → 손 펴기, 1027프레임 전체 참조와 렌더.

**안 된 것** 손잡이를 실제로 쥐어 드는 것. 손잡이 후보(GraspGenX 94 + Dexonomy 50 + 26일
체인 4물체 × 40)에서 "들림"이 4번 나왔지만(#54 +79, #55 +51, 클램프 #32 +79, 엄지벌림 #22)
재검증에서 전부 실패 — 접근 중 물체를 쳐서 우연히 걸린 것들.

**측정으로 좁힌 원인** 이 손(Dex3)은 엄지가 손바닥 아래 5.4 cm(+살 4 cm)에 매달려 있어,
바닥에 누운 지름 3 cm 손잡이(중심 1.5 cm)를 손바닥-아래로 집으려면 엄지가 바닥을 뚫어야
한다. 손가락-아래 자세는 집는 점이 손바닥 7 cm 아래라 손끝이 바닥 3.5 cm 밑으로 가야 한다.
어느 쪽이든 물체가 엄지와 손가락 사이에 못 들어가고, 손가락은 허공에서 완전히 닫힌다
(모든 실패의 공통 로그: 손가락 1.57/1.75 완전 닫힘, 엄지 0.72 그대로 또는 완전 닫힘).
책상 위 상자가 들렸던 건 상자 옆면이 손 높이에 있어서다.

**다음에 시도할 순서**
1. 손잡이가 바닥에서 떠 있는 자세로 놓기: 망치를 머리의 넓은 면이 아니라 **좁은 면**으로
   눕히면 손잡이가 3~4 cm 뜬다. 공장 바닥에서 자연스러운 자세다. (mesh 회전만 바꾸면 됨:
   `find_part.py`·체인은 그대로.)
2. Dexonomy `op=grasp`가 tabletop init을 전부 거절한 이유 확인(워커 로그를 파일로): 그 정련이
   통과한 파지는 MuJoCo에서 바닥과 함께 검증된 것이라 가장 믿을 만하다.
3. GraspGenX `--moe_obb_density dense-topandside`(옆 접근, CLI 열어 둠)로 손잡이 후보 재생성.
4. 손목 카메라 재관측(세훈님 제안): 무릎 꿇은 뒤 손목 카메라로 손잡이를 다시 보면 점군이
   촘촘해져 GraspGenX 후보의 질이 오른다. `capture_rgbd.py`에 손목 마운트 추가 필요.

## 2026-09-26: 조사, 손잡이 재시도, 크레이트 두 손 들기 (영상 `260927/`)

**조사(웹, 논문·깃)** 요지 — `docs/`에 별도 정리 없이 여기 요약:
- 부위 지향 파지의 공개 시스템(LERF-TOGO, ThinkGrasp, GraspGPT, RAM, MOKA/RoboPoint, OpenAD)은
  전부 **일반 파지 샘플러 위의 필터**다. 우리 구조(C-RADIO 관련도 + GraspGenX 후보 + 손잡이
  픽셀만 물체로)가 그 방식과 같다. OpenAD는 3D 점군에서 "grasp" 어포던스(망치 손잡이 라벨 학습)를
  직접 내니 언어 씨앗이 흔들릴 때 대안.
- 지지면(탁상)을 포함한 다지 손 파지 합성: Dexonomy(우리 Dex3 모델 포함, 탁상 모드), BODex,
  DexGraspNet 2.0, Get a Grip. Dex3 + GraspGenX 조합의 선례(g1-aprilcube-demo)에서도 얇고 긴
  부위 통과율은 9~17%로 낮다.
- **바닥에 평평히 놓인 얇은 물체를 3지 손으로 집는 손 수준 전략은 공개된 것이 없다.** 휴머노이드
  바닥 픽업 데모(AMO, TWIST2, HOMIE, FALCON, VIRAL)는 전부 상자·캔 등 큰 물체다. 권장 전략:
  ① 머리 옆 떠 있는 손잡이 구간, ② 손바닥 세운 옆 핀치, ③ 밀어서 모서리에 걸치기(ExDex).
- 크레이트: PhysHSI·VisualMimic·OmniContact 모두 손가락 없이 **양 손바닥 마찰**로 든다.
- 물리 설정: TGS, 접촉 오프셋 1~2 mm, rest 0, 마찰 결합 max, 회전 마찰 반경, 실물 질량,
  손가락 볼록분해 메시. 여기서 접촉 오프셋·마찰 결합·질량(OBJECT_MASS)을 반영했다.

**손잡이 재시도** (`260927/hammer`, `clamp` evidence)
- 후보 위치를 재보니 GraspGenX 후보는 손잡이 위에 있었다(머리에서 50~85%). 영상에서 손이 손잡이로
  "안 가는" 것은 바닥 여유 규칙이 손을 9~10 cm 높이에 띄워 손잡이 위 6~8 cm 허공에 멈추기 때문.
- 손끝이 바닥에 닿도록 낮춘 검증(TIP_FLESH 0.03, 바닥 0): 망치 0/50, 클램프 2/50이었으나 재검증
  0/4. 사람 순서(머리 근처 떠 있는 구간 우선, `grasp/rank_handle.py`) + 0.5 kg: 1/30이었으나
  +0.935 m "들림" = 튕겨 나감. 재현되는 파지는 여전히 없다.

**크레이트 두 손 들기** (`260927/crate`)
- 언어로 "crate on floor"를 찾고(`--grow-radius 0.5`로 0.6 m 물체 전체를 키움), 점군에서 긴 축·
  크기·설 자리·양 손바닥 목표를 뽑는다(`grasp/crate_target.py`, 좌표 없음). GR00T 플래너로 걷기 +
  스쿼트(mode 4, 골반 0.54 m). cuRobo 리타깃터에 두 손목을 동시에 목표로(`BIMANUAL=1 --crate`):
  위로 넘겨 옆면 8 cm 밖 → 옆면 → 2 cm 조임 → 25 cm 들기. 손은 편 채(`NO_CLOSE`).
- 세 번의 시도: ① 손이 크레이트 가까운 끝을 뚫고 지나가 60 cm 밀어냄 → 위로 넘기는 경로,
  ② 손가락이 아래를 향해 바닥에 3 cm 박혀 크레이트를 90 cm 날림 → 손가락 앞으로,
  ③ 손목 오차 18~34 mm, 사진에서 두 손이 옆면이 아니라 **테두리 높이 위 허공**에 떠 있고 크레이트는
  17 cm 밀렸다(`evidence/two_palms_at_grasp.png`). 스쿼트 자세에서 양손을 0.33 m 앞 ±0.2 m 옆
  9 cm 높이에 놓는 IK가 안 풀린 것으로 보인다(어깨 폭 0.3 m, 팔 0.55 m). 다음: 더 낮은 스쿼트
  (0.40 m) 또는 무릎꿇기(mode 5)에서 리치, 손바닥 목표를 옆면 위쪽(테두리 바로 아래)으로.

### 2026-09-26 밤: 크레이트 걸어 올리기(hook), 27일 큐

- 크레이트의 손잡이 구멍은 **긴 옆면 가운데**(사진 `260927/crate/evidence/slot_hook_knee_in_crate.png`),
  0.40 m 간격. 로봇이 짧은 끝에 서면 양손이 ±0.20 m만 벌리면 되어 도달이 된다(무릎꿇기 0.50 m
  뒤, 몸통 가중치 0.03: 슬롯에서 손목 오차 3~4 mm, 방향 3~8°). 무릎꿇기를 0.15~0.25 m 뒤에서
  하면 무릎이 크레이트 안으로 들어간다(사진).
- 23 mm 슬롯에 손가락(두께 ~20 mm)을 넣는 것은 손 모델 오차로 벽을 밀어 실패. **테두리 위로 넘겨
  안쪽으로 감아 거는(rim hook)** 방식은 손가락이 테두리를 **실제로 걸어 크레이트를 들어올렸다**
  (`evidence/rim_hook_long_faces_at_grasp.png`: 크레이트가 손에 매달려 기울어짐). 그러나 양손이
  비대칭으로 걸려 크레이트가 뒤집혀 떨어졌다(`..._after_lift_flipped.png`). 다음: 넘긴 뒤 안쪽
  이동을 3 cm로 줄여 크레이트가 바닥에 있는 채로 양손을 먼저 감고(close), 그다음 들기.
- 27일 큐(손잡이 순위 + 손끝 바닥 접촉 + 0.5 kg): 망치 0/40, 클램프 0/40, 드릴 0/37, 손전등 1/36
  (#22, 재검증 중), 페인트 통 진행 중. 영상 전부 `260927/<물체>/`.
- rim hook v2(넘기는 높이 8 cm, 손바닥 10 cm 밖): 접근 추적(`test_rim2.txt`)에서 크레이트가
  **접근 44~59프레임(내려오는 단계)** 에 움직인다. 그때 왼손바닥이 z 0.15~0.16(계획 0.268)에 있다
  — 왼손이 계획보다 10 cm 낮게 내려온다. 손목 목표 오차는 마지막 프레임에서만 재고 있었으므로
  중간 프레임의 왼손 오차(혹은 왼손 손바닥 프레임의 거울 대칭 문제)를 다음에 먼저 확인할 것:
  `solve_multi`의 per-frame per-hand 오차 출력, 왼손은 `WRIST_TO_PALM`의 y 부호와 손바닥 축이
  오른손의 거울이다.

### 2026-09-26 밤 늦게: 크레이트 테두리 집기 — 한 손이 든다

- 접근 중 크레이트가 끌린 원인: `PD_BODY`에서 팔 PD 강성이 낮아 뻗은 팔이 10 cm 처짐(IK는 24 mm
  이내). `ARM_KP_SCALE=4`로 팔 강성을 올리자 접근 단계에서 크레이트가 정지.
- 걸기(hook)에서 남은 문제는 엄지: 손가락이 아래로 감기는 자세면 엄지도 손바닥 아래로 매달려 벽을
  민다. 그래서 **위에서 수직으로 내려와 손가락은 벽 안쪽, 엄지는 벽 바깥에 두고 테두리를 집는**
  파지(`HOOK_MODE=pinch`)로 바꿨다. 손바닥 중심은 벽 안쪽 4.5 cm(엄지와 손가락이 만나는 점이
  벽에 오도록), 테두리 위 5 cm. 두 손목 오차 1.4~2.0 mm.
- 결과(`260927/crate/v4`): **오른손이 테두리를 집어 2 kg 크레이트를 공중에 들었다**
  (`evidence/at_lift.png`). 왼손은 안 잡혀 크레이트가 한 손에 매달려 기울었다. 왼손은 오른손의
  거울이라 손바닥 y 방향을 뒤집어 넣었는데(`LEFT_Y_IN`), 그 가정이 틀렸을 수 있어 반대 방향으로
  재검증 중. 양손이 잡히면 → 수평으로 들기 → 일어서서 책상까지 걷기(기존 carry 경로, 양팔 고정)
  → 책상 위 내려놓기(`--place` 양손 버전).

## 손 바꾸기 검토 (2026-09-26 23:50, 세훈님 허용: "삼지 안되면 오지로 해도 돼")

파지 오픈소스가 실제로 지원하는 손:
- GraspGenX `x_grippers/`: **inspire_hand**(points.json·tsdf·pointnet 표현까지 온보딩된 정식 손),
  unitree_g1(다른 리비전 Dex3), g1_dex3_right(우리가 만든 것 — 모델용 표현이 더미 0). 즉 Dex3 후보는
  모델이 손 모양을 모른 채 낸 것이고, Inspire 후보는 모델이 손을 아는 상태에서 낸다.
- Dexonomy `assets/hand/`: unitree_g1(Dex3), shadow, allegro, leap, mano. Inspire 없음.
- IsaacLab: `G1_INSPIRE_FTP_CFG` = G1 29자유도 + Inspire 5지 (`g1_29dof_inspire_hand.usd`, 손가락
  강성 10/감쇠 0.2), NVIDIA 자체 pick-place 태스크가 쓰는 조합. `/home/sehoon/Desktop/참고/할일/26/06/
  260623/G1/configuration/`에 Inspire 손 usda도 있다.

**결론**: 손을 바꾼다면 **Inspire 5지**가 맞다 — 파지 생성기(GraspGenX)와 시뮬레이터(IsaacLab) 둘 다
정식 지원. SONIC 추적은 Dex3 43자유도 모델에 묶여 있으므로 픽 구간은 지금처럼 키네마틱 재생.

**포팅 순서(다음 세션)**:
1. `end2end/robots/g1_inspire_arm.yaml`: g1_right_arm.yaml 복사, `graspgen.gripper_name: inspire_hand`,
   `grasp_to_tool_transform`은 inspire의 `world_joint`(gripper.urdf)에서, tool_frame은 손목.
2. 테스터·렌더에 `HAND=inspire`: `G1_INSPIRE_FTP_CFG`, 손 관절 이름(thumb_proximal_yaw/pitch,
   index/middle/ring/pinky_proximal + 종속 관절), 열림/닫힘 값은 GraspGenX inspire config.json
   (열림 thumb_yaw 1.308, 나머지 0; 닫힘 pitch 0.6, 손가락 1.47).
3. `reach_from_pose.py`의 `WRIST_TO_PALM`을 Inspire 장착 오프셋(USD의 wrist_yaw→hand base)으로.
4. 같은 체인으로 망치 손잡이부터. 5지는 손잡이를 감싸 쥘 수 있고(엄지가 옆에서 대립), 손끝이 바닥에
   닿아도 나머지 손가락으로 든다.

진행 중(자동): 망치 pin-and-grasp(왼손이 머리 누름) → 크레이트 왼손 반대 방향 → 펜치·드라이버 2종·
다른 망치 체인. 12분마다 자동 점검.
- (00:02 점검) 망치 pin-and-grasp(왼손이 머리를 15 mm 눌러 고정, 오른손 손잡이 25후보): 0/25.
  크레이트 왼손 비거울(`LEFT_Y_IN=0`): 0 — v4의 거울 방향(한 손으로 들림)이 맞았다. 왼손 문제는 방향이
  아니라 위치/타이밍: 다음은 왼손만 사진으로 확인(close 순간 왼손 스냅샷) 후 왼손 PINCH_IN 조정.
  펜치 체인 검증 중(후보 240, 손잡이 축 위 222).
- (00:19 점검) 크레이트 pinch6 리치가 10분째 멈춤(hammer2 체인의 GraspGen과 동시 실행) → PID로 죽이고, GPU에 e2e/리치가 없을 때만 돌도록 대기 후 재실행.
- (00:26 점검) 크레이트 pinch6(왼손 손바닥을 벽 밖 4.5 cm): 왼손이 내려오며 크레이트를 오른손 쪽으로
  4 cm 밀어 오른손 집기까지 놓침(0). 왼손 Dex3의 거울 기하(엄지 −y, 손가락 감기는 방향)를 사진으로
  확정하지 않고는 더 못 간다 → 다음: 왼손만 단독으로 벽에 대고 close 스냅샷. 십자 드라이버: 손잡이
  씨앗 55 px뿐이라 GraspGen 후보 0 → 체인 크래시. 부위가 200 px 미만이면 물체 전체로 대체하도록 수정.
- (00:38 점검) hammer2 체인이 검증 단계에서 bash 문법 오류로 죽음 — 실행 중인 체인 스크립트를 편집한 탓(bash는 스크립트를 읽어 가며 실행). 재실행을 큐 끝에 추가. 교훈: 돌고 있는 스크립트는 편집하지 말고 복사본을 고칠 것.

## 2026-09-27 01:30 큐 종료 — 상태 정리

**영상** `영상보관/g1-factory-tidy/09/260927/<부품>/vN/` (각 폴더 `note.txt`, `evidence/`):
clamp v1, crate v1~v4, drill v1, flashlight v1, flat_screwdriver v1, hammer v1, hammer2 v1, paint_tin v1,
pliers v1, screwdriver v2(v1은 씨앗 55 px로 크래시). 파지 검증 결과는 전부 0(망치 0/40, 클램프 0/40,
드릴 0/37, 손전등 1/36→재검증 0, 펜치 0/40, 십자 0/19, 일자 0/33, hammer2 0/40, 망치 pin-and-grasp
0/25, 망치 90° 회전 0/30×2). 크레이트는 v4에서 **오른손 테두리 집기로 2 kg 크레이트가 공중에 들렸다**
(한 손, 기울어짐); 왼손은 거울 기하가 확정되지 않아 아직 못 잡는다.

**밤새 잡은 운영 문제**: 동시에 cuRobo 리치 둘(또는 리치+GraspGen)을 돌리면 멈춤 → 한 번에 하나;
돌고 있는 bash 스크립트 편집 금지; 자기 명령줄과 겹치는 패턴으로 pkill 금지(pid 파일).

**다음 세션 우선순위**
1. 크레이트 왼손: 왼쪽 벽 옆에 카메라를 두고 왼손만 close 스냅샷 → 왼손 Dex3의 엄지 위치(−y?)와
   감기는 방향을 확정 → 양손 집기 → 수평 들기 렌더(v5) → 일어서서 책상까지 운반 → 책상 위 내려놓기.
2. 손 교체(Inspire 5지): 위 "손 바꾸기 검토"의 포팅 4단계. GraspGenX가 정식으로 아는 손이라 후보의
   질이 다르고, 5지는 손잡이를 감싸 쥘 수 있다.
3. 그 뒤 같은 체인으로 망치·클램프·드릴·펜치·드라이버 재시도(260928/<부품>/v1).

## 5지(Inspire) 전환 (2026-09-27 10:00~, 영상 `260927/5지/<부품>/vN/`)

- Nucleus의 `g1_29dof_inspire_hand.usd`는 다운로드가 멈춰서, 로컬 Isaac G1 에셋
  (`Desktop/참고/할일/26/06/260623/G1/g1.usda`, 변형 right_hand/left_hand=Inspire, Physics=PhysX)을 쓴다.
  손 관절 12개/손(proximal 4 + thumb yaw/pitch + intermediate 4 + thumb intermediate/distal), 손 마운트는
  Dex3와 같은 손목+(0.0415, −0.003, 0). 손가락은 손 베이스 +x로 뻗고 엄지는 +y/+z 쪽.
- GraspGenX `inspire_hand` 설명의 hand_base 프레임은 손가락이 −y, 검지가 −z라서 IsaacLab 프레임과
  고정 회전(x_i=−y_g, y_i=−x_g, z_i=−z_g)으로 잇는다(`reach_from_pose.py`, HAND=inspire). GraspGen 손끝
  [0,0,0.15]는 IsaacLab 프레임에서 (0.15, 0.06, 0) — 5지 파워 그립의 중심.
- 로봇 yaml `end2end/robots/g1_inspire_arm.yaml`(gripper_name inspire_hand, grasp_to_tool은 world_joint에서).
- 열림/닫힘: GraspGenX config(엄지 yaw 1.308 고정, pitch 0→0.5, 손가락 0→1.47), 종속 관절은 근위 관절을
  따라간다. 손가락 드라이브는 NVIDIA G1_INSPIRE_FTP_CFG의 강성 10/감쇠 0.2/토크 30.
- 테스터·렌더·순위·체인 전부 `HAND=inspire` 분기. 첫 실행: 망치 손잡이(`5지/hammer/v1`).
- (10:34 점검) 5지 스냅샷 실행이 Dex3 전용 verbose 출력(index_1 키)에서 죽고 Isaac 프로세스 2개가 좀비로 남음 → PID로 정리, 출력 손 무관하게 수정, 재실행.
- (10:49 점검) 5지 망치 v1 렌더가 27분째(스냅샷과 GPU 경합, 로그 정지) → PID로 죽이고 스냅샷이 끝난 뒤 단독으로 다시 렌더하도록 큐.
- (11:00 점검) 5지 #77 스냅샷 테스터가 25분째(들기 단계에서 정지) → PID로 종료; 파지·닫힘 사진 4장은 확보됨. 이어서 v1 렌더가 단독으로 시작.
- (11:20) 로컬 G1 에셋의 Inspire 손은 스냅샷에서 **보이지 않았다**(시각 메시 미해결). 공식 IsaacLab
  G1+Inspire USD(`Assets/Isaac/5.1/Isaac/IsaacLab/Robots/Unitree/G1/g1_29dof_inspire_hand.usd` + configuration
  4파일, base 39 MB)를 `assets/g1_inspire/`에 받았다. 관절 이름·오프셋은 로컬 에셋과 동일. Sensor 변형은
  원격 참조로 멈추므로 "None". `fix_root_link=False`로는 관절 생성이 실패(root_joint 불일치 + 종속 관절
  mimic 오류) → `FIX_ROOT=1`로 검증 중.
- (11:27 점검) 좀비 Isaac 프로세스 3개(RAM 15 GB)를 PID로 정리. 공식 Inspire 에셋에는 R_hand_base_link 바디가 없어(이름 다름) 테스터가 죽음 → 바디 이름 확인 후 수정.
- (11:45) 5지 #77: 근위 관절은 1.1 rad까지 닫혔지만 중간·말단(mimic) 관절을 같이 구동하니 −0.34 한계로 튀어 손끝이 바깥으로 벌어짐 → 근위+엄지 yaw/pitch만 구동하도록 수정, 재검증.
- (11:48) **5지 #77 HELD: 들기 후 +109 mm** (망치 0.5 kg, 손잡이 35% 지점, 근위 관절만 구동). #155 LOST.
  3회 재검증 + 영상(`5지/hammer/v2`) 진행 중.
- (12:24) 5지 #77 렌더가 34분째 무출력 → 종료. 렌더 진행률을 보기 위해 python -u로 짧게 진단 실행.
- (12:33) 5지 렌더가 멈춘 원인: play_in_cell이 플랜의 Dex3 손 관절 이름을 조회하다 예외 → Isaac이 종료되지 않고 좀비로 남음(34분). 없는 관절은 건너뛰도록 수정 후 렌더·운반·공구 큐 재실행.
- (13:25) 렌더가 첫 프레임 전에 조용히 끝난 원인: 플랜의 Dex3 손 관절이 Inspire에 없어 `SystemExit`(Isaac은 종료 후 좀비로 남아 '멈춤'처럼 보임). --clip-arms일 때는 무시하도록 수정, 렌더→운반→공구 재실행.
- (13:30) 5지 렌더 실패 원인 3: 참조 빌더의 엄지-먼저 마스크가 Dex3 14관절 고정 → 24관절 Inspire에서 IndexError, 손 스케줄 파일이 안 만들어짐. HAND_NAMES 기반으로 수정.

## 2026-09-27 13:50 — Inspire #77: tester HELD, render LOST; the difference was the replay, not the grasp

- The render (5지/hammer/v2, v3) finally produced mp4s, but the hammer was flipped at the close instead of lifted
  (`[eval] dz -0.26`: the mesh origin sits 13 cm above the centroid, so a flipped hammer reads as -0.26).
- Checked what differs from the tester run that held (+109 mm):
  - the plan is identical: `fable40p.pkl` frame 433..583 equals `reach_all.npz` q[77][n_go-1] (joints, base xyz, base rpy, 0.0 error);
    the render's wrist joints at the hold `[-0.31 0.73 -0.66]` equal the tester's #77 line.
  - OBJECT_MASS is not implemented anywhere (plan_scene fixes 0.2 kg); the chain's export is a no-op. Same in both.
  - the close ramp: tester `--slow` 30 frames, reference builder 20 frames; the low-gain fingers take ~100 frames either way.
  - **the replay differs**: the tester's PD_BODY path writes the root once a frame and never writes a joint
    (`put()`), the render's PD_BODY path still wrote all body joints once a frame (`write_joint_state_to_sim(tgt_q[:, _body_ids])`),
    snapping the arm onto the plan instead of letting the PD track it, which is exactly the "garbage velocities / contact kicks"
    mode DIAGNOSIS already blamed once. Fixed: under PD_BODY=1 the render writes no joints (play_in_cell.py, the `_hands` block).
- Also: the render's hammer rolled 1.6 cm / 2.9 cm during frames 0-100 while the tester's settle moved 0 m, 0 deg — watch this on v4.
- v4 render queued (insp77_v4.sh); clamp/drill/pliers/screwdriver/hammer2 Inspire chains running behind it.

## 2026-09-27 14:30 — why the tester held #77 and the render did not: the passive fingertips

- Object pose: identical in both after all. The tester's "settle: moved 0" was a stale read (`box.update(0.0)` does not
  refresh; `update(dt)` does), the hammer tips 8.7 deg from its placed pose in both runs (placed on the head's edge with
  the centroid outside the support) and lands at the same centroid (-0.294, 0.095, 0.028). Render eval numbers are the
  mesh ORIGIN (13 cm above the centroid), which is why a flipped hammer reads dz -0.26.
- Palm path: tester grasp palm (-0.223, 0.031, 0.215), render frame 500 (-0.218, 0.027, 0.206). Same to 1 cm.
- Difference: the Inspire asset couples intermediate/distal joints to the proximals with PhysX MIMIC joints at
  25 Hz / damping 0.005. Free in the air they track (finger_mimic_test.py); under contact they fold to the -0.34
  limit (every close: proximal 1.47, intermediate -0.34). After 12 s of walking with swinging arms the render's
  fingertips sat CURLED (+0.3..0.46, thumb +0.74) at the approach, the tester's hung straight (-0.1..-0.3):
  shorter fingers, no nudge on the handle, no hold.
- Fix: `stiffen_mimic()` (build_reach_reference.py) sets naturalFrequency 200 Hz / dampingRatio 1.0 on the spawned
  prims, called after Articulation(cfg) in the tester and the render. Tester with it: #77 HELD +0.132 (fingertips still
  fold under contact, but the free-space state now matches). #155/#78 LOST as before. v5 render running.
- Scratch experiment harness note: a scene-only Isaac script whose RigidObject is read with update(0.0) shows nothing
  moving; always read with update(physics_dt).

## 2026-09-27 14:55 — the fingertips were the grasp: software four-bar (SOFT_MIMIC)

- PhysX mimic joints cannot be made rigid at a 1 kHz step: 200 Hz folds on contact (-0.34), 1000 Hz tracks 0.6-0.7 of
  the proximal and #77 then LOSES (the "hold" at 25/200 Hz was the hyperextended straight fingers scooping the handle).
- `soft_mimic()` (build_reach_reference.py): the intermediate/distal joints are in the actuator group and their targets
  are rewritten every substep from the MEASURED proximal angle x gear (1.0, thumb 1.6/2.4, the asset's own ratios) --
  the RH56's rigid linkage. Tester #77: intermediates 0.69-0.83 for proximals 0.84-0.93, box +0.031 at close,
  lift +0.178 m HELD (was +0.109 with floppy tips). Render v5 (200 Hz, no four-bar): LOST, hammer turned 25 deg.
  Render v6 with the four-bar queued; clamp/drill re-tests queued behind the tool queue (queue_retest.sh).
- 15:05 v6 (four-bar) render: the fingers wrapped the handle and lifted it 2.5 cm, then it slipped during the lift.
  Wrist view at 13.4 s: the OPEN THUMB lands on the hammer head during the descent (#77 grips the handle right next
  to the head) and tilts the hammer 16 deg before the close -- the tester's hold and the render's loss differ only by
  that chaotic nudge. Next: top-40 with the four-bar (test_all4bar.txt), prefer a held candidate farther from the head,
  and re-verify each held one 3x before rendering.
- 15:10 Approach diagnostic (play_in_cell OBJ_EVERY=2 + nearest-link print, no video): after the seam the right
  thumb's distance to the hammer went 14.7 -> 32.8 -> 10.7 -> 26.0 -> 7.7 cm within 20 frames -- the arm RINGS at
  ~3 Hz, +-10 cm, for a second after the reference steps the shoulder 30 deg in one frame (kneel pose -> reach's
  pre-grasp), and the swinging hand is what hit the hammer at approach frame ~30. SEAM_BLEND=15 (default now in
  build_reach_reference.py and a 15-frame kneel->q[0] lerp in the tester): the distance now falls monotonically
  27 -> 9 cm and the hammer is untouched until the fingers arrive. v7 render running.
- Crate insp1 (Inspire two-hand rim pinch, both hands closing/colliding for the first time): LOST, fingers stopped
  at 0.4 rad, crate not lifted; palms exactly where planned (right wrist 3.5 cm inside its wall, thumb 4.8 cm
  outside at rim height). Waiting for the video before changing geometry.
- 15:25 With the blend (no ringing) #77/#78 LOSE in the tester and the hammer is shoved 9.5 cm between approach
  frames 29 and 44 -- the last 10 cm of the VERTICAL descent. v7 wrist view: the open fingers' undersides land on
  the handle next to the head. The 7/40 "held" without the blend were scoops that the ringing arm happened to make.
  The Inspire floor grasps hold the palm pitched ~50 deg (fingers down-and-away), so a vertical descent drags the
  fingers across the handle. New: APPROACH_AXIS=x (reach_from_pose.py) comes in along the fingers from 12 cm back,
  fingertips leading into the gap beside the handle. Re-reach + top-16 test queued (reach40_x.sh -> test_x.txt).
- Rule for my own shell: the GPU guards match `reach_from_pos[e]|e2e_grasp_dem[o]`; a Bash command of mine that
  contains those script names literally keeps every guard waiting while it runs.

## 2026-09-27 15:50 — ROOT CAUSE for the Inspire grasps: the exported grasp frame is not hand_base

- GraspGenX exports `grasps_world` in GraspGen's gripper convention frame = gripper.urdf's `world` link
  (config.json: fingers along +z, fingertip at (0, 0, 0.15)). hand_base_link sits inside it at world_joint
  xyz (0.065, -0.01, 0) rpy (1.5708, 2.356194, 0). reach_from_pose.py applied only the hand_base -> Isaac axis
  swap, so every Inspire grasp was executed with the hand turned by that world_joint rotation. Check: GraspGen's
  points.json mapped into the Isaac palm frame spans x -0.03..0.05, z -0.16..0.02 (fingers along -z) with the
  fingertip at (0, 0, -0.15); with world_joint included it spans x 0..0.22, y -0.03..0.12, fingertip at
  (0.152, 0.06, 0.01) -- the measured Isaac hand. The "holds" (#77, #78, #121, #79) were scoops by a rotated hand.
- Fixed in reach_from_pose.py (INSPIRE_OLD_MAP=1 restores the old behaviour). Re-reach + top-40 test + render
  queued for hammer, clamp, drill, pliers (queue_rereach.sh); the tester-only clamp/drill retest queue cancelled.
  hammer2's chain (next in the tool queue) picks the fix up automatically; flat_screwdriver rendered with the old map.
- 16:05 With the corrected frame the GraspGen approach axis is straight down and the fingers/thumb side point 45 deg
  down (a wrap from above), but the open fingertips then sit below the floor, and TIP_FLESH=0.03 raised every grasp
  until the pinch point was ~0.15 m up (handle at 0.03). Only 25/156 stayed "on the axis". The hammer handle is
  2-3 cm off the floor near the head, so the fingertips must be allowed down to the floor: re-reach queue restarted
  with TIP_FLESH=0.005 (rereach_render.sh). (A pkill on the script's name killed my own shell once more: exit 144.)
- 16:20 CRATE v4 (5지/crate/v4): the render lifts the 2 kg crate 8 cm, level, with both Inspire hands pinching the
  long walls' rims (thumb outside 2 cm under the top, four fingers inside; finger stiffness 40, both hands closing and
  colliding). The tester of the same plan lost it (right fingers pried back), and the crate follows only 8 of the
  25 cm the wrists rise -- marginal. v5 = 1 cm higher. Next after a repeatable lift: stand + carry to the desk
  (two-arm carry) and set down.

## 2026-09-27 16:30 — CORRECTION of the 15:50 "root cause": the frame was right all along

- `grasps_in_cell()` multiplies every exported grasp by the JSON's `grasp_to_tool_transform`, and for
  g1_inspire_arm.yaml that transform IS the gripper URDF's world_joint. So the palms handed to the Inspire branch
  were already hand_base poses and the single axis swap M was correct. My 15:50 check compared "G@M" with
  "G@Twj@M" on the raw GraspGen points and forgot the loader's factor. With the double transform the palms faced
  up and the open fingertips sat 10 cm above the floor (test: 0/40, tips z +0.107). Reverted (INSPIRE_DOUBLE_MAP=1
  reproduces it). What stands from this afternoon: PD_BODY must not write joints, SOFT_MIMIC four-bar, SEAM_BLEND,
  the descent lands the fingers on the handle -> fingertip margin 5 mm instead of 3 cm (re-reach queue restarted).
- 16:40 Hammer, correct frame, TIP_FLESH 0.005 (fingertips down to the floor beside the handle), four-bar, seam
  blend: 5/40 HELD in the tester -- #65 (+193 mm), #64 (+138), #96 (+142), #123 (+133), #95 (+79). #65 rendering
  (5지/hammer/v8); repeatability run on 65/64/96/123 in parallel; the carry-to-crate chain (place65.sh) queued on
  the same reference.
- Crate v5 (1 cm higher): render lifts 9 cm level, tester lost -- same split as v4. v6 adds a 3 cm outward slide.
- 16:52 Crate v6 (3 cm outward slide): render lifts 6.9 cm level, tester lost. v4/v5/v6 renders all lift 7-9 cm and
  slip as the wrists rise 25 cm; v7 = finger stiffness 100, palms 4.5 cm inside. Hammer #65: first tester run HELD
  +193 mm, rerun LOST -- the floor pinch is still a coin flip; 64/96/123 reruns pending.
- 17:05 Repeatability of the 5/40: #65 and #64 LOST on rerun (the first "held" runs were flukes). #65 render (v8)
  LOST, carry render (v9) walked to the crate empty-handed (walk + crate reach fine, 3.3 mm). Crate v7 (stiffness
  100, 4.5 cm inside, slide): tester LOST, render pending. Queued: hammer with GRASP_SLIDE=0.03 (fingertips land 3 cm
  beyond the handle, slide back against it, then close) after the clamp re-reach.
- 17:16 Crate v7 (stiffness 100, 4.5 cm inside): render 2.6 cm -- worse. Not a squeeze problem. v8 = v5 geometry
  (best, 9 cm) with the slide and a 12 cm half-speed lift to test whether the crate follows a slow lift.
- 17:25 CRATE LIFT BUG: in v4-v7 the crate followed the hands with NO slip (right wrist stayed 0.32 m above the crate
  throughout), the hands simply rose only 8-9 cm: the crate solution holds 30 frames at the grasp (lift_from = n_go
  + 30) and the replay (crate script + tester) started its 4x-slowed lift at n_go, so only 15 of 45 lift frames
  played. Fixed in both. v8 stopped; v9 = v5 geometry + slide + full lift.
- 17:25 Hammer with GRASP_SLIDE=0.03: 3/40 HELD in the tester (#93 +105, #95 +119, #0 +102 mm); #93 rendering
  (5지/hammer/v10); repeatability run on 93/95/0 x2. Crate v9 (full lift): tester lost, render pending.
