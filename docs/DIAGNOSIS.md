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

## 남은 것

- 걷기와 파지가 아직 두 클립이다. 걷기는 SONIC이 추적해야 하고(공식
  평가기), 파지는 골반 용접 대신 decoupled_wbc 또는 전신 리타깃 위에서
  실행돼야 한다.
- 파지 계획이 골반 높이 1 cm 창에 갇힌 것은 팔만 계획해서다. 전신 계획으로
  옮기면 `--cut-at-height`는 필요 없어진다.
