# 파지가 깨지는 자리 (260922)

## 측정

Isaac 재생 830프레임 (30 fps). 물체는 `/World/GraspTarget`의 월드 좌표를 직접 읽음.

| 구간 | 프레임 | 물체 이동 |
|---|---|---|
| go_to_pre_grasp_pose | 0–120 | 2~5 mm (노이즈) |
| hold_at_pre_grasp | 120–180 | 2~5 mm |
| go_from_pre_grasp_to_grasp_pose | 180–300 | 2~5 mm |
| hold_at_grasp | 300–360 | 2.5 mm |
| close_fingers | 360–380 | 2.5 mm → 93.6 mm, z 0.826 → 0.783 |
| 이후 전부 | 380–830 | 93.4 mm 고정 |

접근은 문제가 없다. 손가락을 닫는 20프레임(0.67초)에서 물체가 튕겨 나가고,
그 뒤로는 빈 손을 들어 올린다.

## 원인

`end2end/tasks.py`의 close_fingers는 `_ramp(open_vals, close_vals, n_close)`,
즉 손가락 관절을 open에서 close까지 선형으로 보간한 위치 명령이다.
`close_vals`는 그리퍼 설정의 닫힘 각도이고, 이 손에서는 관절 한계와 같다.

측정한 명령값 (frame 360 → 380):

| 관절 | 360 | 380 | URDF 한계 |
|---|---|---|---|
| thumb_1 | 0.724 | −1.047 | −1.047 |
| thumb_2 | 0.0 | −1.500 | −1.571 |
| index_0 | 0.400 | 1.571 | 1.571 |
| index_1 | 0.0 | 1.745 | 1.745 |
| middle_0 | 0.400 | 1.571 | 1.571 |
| middle_1 | 0.0 | 1.745 | 1.745 |

물체가 있든 없든 손가락을 한계까지 접으라는 명령이다. 접촉에서 멈추게 하는 건
명령이 아니라 시뮬레이터 쪽이어야 하는데, 지금 Isaac의 손 액추에이터는
IsaacLab `G1_29DOF_CFG`의 `"hands"` 항목을 그대로 쓰고 있다.

```
"hands": ImplicitActuatorCfg(effort_limit=300, velocity_limit=100,
                             stiffness=20, damping=2, armature=0.001)
```

Unitree 자신의 URDF(`g1_29dof_with_hand_rev_1_0.urdf`)에서 Dex3-1 손가락 관절은
전부 `effort="1.4" velocity="12"`다. 즉 시뮬의 손가락이 실물의 214배 토크를
낼 수 있다. 위치 오차 0.6 rad × stiffness 20 = 12 N·m가 한계 없이 그대로 나가고,
상자는 튕긴다.

Newton(`--playback_mode dynamic`)에서는 이게 덜 보였다. 그 경로는 손가락을
속도 제어로 돌리고 접촉에서 멈춘 실제 각도를 trajectory.json에 기록하기 때문이다.
kinematic으로 바꾸면 계획된 램프가 그대로 나온다. 엔진을 바꿔 가릴 문제가 아니라
Isaac 쪽 액추에이터가 실물과 달랐던 것이다.

## 참고한 곳

- [DexGraspBench](https://github.com/JYChen18/DexGraspBench) (BODex, ICRA 2025).
  MuJoCo 파지 평가 벤치마크. 한 파지를 `approach_qpos → pregrasp_qpos →
  grasp_qpos → squeeze_qpos → lift_qpos` 다섯 자세로 정의하고
  `src/task/eval_func/tabletop_arm.py`가 그 순서대로 보간해 재생한다.
  코드 주석에 "pre → grasp → squeeze는 단계별 선형이어야 하고, 하나로 합치면
  성능이 크게 떨어진다"고 적혀 있다. 우리 파이프라인에는 squeeze가 없고
  close_fingers가 그 자리를 관절 한계로 대신하고 있다.
  평가 설정: `obj_mass: 0.1` kg, `miu_coef: [0.6, 0.02]`, 손 kp는 1에서 5로 올림.
  성공 판정은 `trans_thre: 0.05` m, `angle_thre: 15`도.
- [Dexonomy](https://github.com/JYChen18/Dexonomy) (RSS 2025). 같은 저자.
  이 저장소가 이미 초기 조사에 쓴 것. approach / grasp / squeeze 세 자세를 내놓고,
  MuJoCo 검증에서 접촉점마다 필요한 힘을 구해 transposed-Jacobian으로 관절 토크를
  만든다. 위치 명령으로 밀지 않는다.
- [DexGraspNet 2.0](https://github.com/PKU-EPIC/DexGraspNet2) (CoRL 2024).
  IsaacGym 기반, 어질러진 장면에서의 생성형 파지.

## 다음에 볼 것

1. Isaac 손 액추에이터 effort_limit을 URDF 값 1.4 N·m으로. 값의 출처는 Unitree URDF.
2. squeeze 자세. close_vals를 관절 한계 대신 물체 표면 기준으로 잡는 자리.

## 탐색 — 비전 보고 행동하는 쪽 (260922)

지금 우리 구조는 한 장 보고 끝이다. RGB-D 한 프레임 → 파지 자세 → cuRobo 경로 →
눈 감고 재생. 상자가 밀려도 고칠 방법이 없다. 아래는 그 자리를 닫힌 루프로
바꾼 연구들이고, 전부 매 스텝 관측에서 관절 목표를 내놓는다.

### 우리 스택(Isaac Lab)에 가장 가까운 것

- IsaacLab 2.3이 `isaaclab_tasks/manager_based/manipulation/dexsuite`를 들고 온다.
  Kuka+Allegro로 물체를 집어 드는 RL 과제. 손가락 끝마다 `ContactSensorCfg`를 달고
  접촉력을 관측에 넣으며, 주석에 "contact force in finger tips is under 20N
  normally"라고 적고 clip을 ±20 N으로 건다. 행동은
  `RelativeJointPositionActionCfg(scale=0.1)` — 절대 위치가 아니라 현재 각도에서의
  증분이다. 우리처럼 한계까지 가는 절대 위치 램프가 아니다.
- [DextrAH-G](https://arxiv.org/abs/2407.02274) (CoRL 2024, NVIDIA/Stanford).
  깊이 영상 → 행동. geometric fabric 위에서 RL로 특권 정책을 학습하고 깊이 입력으로
  증류. 실기 파지 89%. NVIDIA가 GR00T-Dexterity 워크플로로 Isaac Gym에서 Isaac Lab으로
  옮겼다고 밝혔다. 후속이 [DextrAH-RGB](https://arxiv.org/abs/2412.01791).
  우리와 시뮬레이터가 같다는 게 가장 큰 장점.

### 손가락을 어떻게 닫느냐에 직접 답하는 것

- [DexGraspBench](https://github.com/JYChen18/DexGraspBench) (BODex, ICRA 2025).
  파지 하나를 approach → pregrasp → grasp → squeeze → lift 다섯 자세로 정의.
  `obj_mass 0.1` kg, `miu_coef [0.6, 0.02]`, 손 kp 5.
- [RobustDexGrasp](https://github.com/zdchan/RobustDexGrasp) (CoRL 2025).
  단일 시점 점구름에서 500종 이상을 잡는다. 교사 정책은 물체 점구름 + 접촉/충격량까지
  보는 시각-촉각 정책이고, 학생은 단일 시점 점구름과 잡음 섞인 관절값만 본다.
  둘 다 저수준 PD 컨트롤러가 받을 관절 목표를 낸다. 시뮬 97.0%, 실기 94.6%.
  RaiSim + Allegro/UR5라 포팅 비용은 크다.
- [ClutterDexGrasp](https://arxiv.org/abs/2506.14317) (CoRL 2025). 어질러진 장면에서의
  닫힌 루프 파지. 교사(특권 상태) → 학생(부분 점구름, DP3) 증류, 실세계 시연 0건.

### 언어까지 붙이는 쪽

- [DexGraspVLA](https://dexgraspvla.github.io/). VLM을 상위 계획자로, 하위 컨트롤러는
  모방학습으로 학습한 닫힌 루프 정책. 미지 장면 90%+.
- [Sim-to-Real RL for Vision-Based Dexterous Manipulation on Humanoids](https://arxiv.org/abs/2502.20396)
  (CoRL 2025, Lin 외). 휴머노이드 양손. 자동 real-to-sim 튜닝, 접촉·물체 목표 기반 보상,
  분할정복 증류. 우리 로봇 형태와 가장 비슷하다.
- [UniDexGrasp++](https://github.com/PKU-EPIC/UniDexGrasp2) (ICCV 2023).
  상태 기반 교사 → 시각 기반 학생 증류의 원형. 시각 정책 85.4%/78.2%.

### 공통점

전부 교사(특권 정보) → 학생(실제 센서) 증류이고, 학생의 출력은 매 스텝의 관절 목표다.
그리고 전부 학습 전에 손 액추에이터를 실물에 맞춰 놓는다. IsaacLab의 Allegro는
`effort_limit_sim=0.5`, `stiffness=3.0`, `damping=0.1`이다. 우리가 쓰던
`G1_29DOF_CFG`의 `"hands"`는 `effort_limit=300`, `stiffness=20`이다. 그 설정은
보행 학습용이고 파지용이 아니다. 어떤 정책을 붙이든 이걸 먼저 맞춰야 한다.

## 닿지를 못한다 (260922)

비전이 만든 후보 36개를 `--force_grasp_idx`로 하나씩 계획해 봤다.
신뢰도 상위 5개(0.964 / 0.963 / 0.962 / 0.956 / 0.956)가 전부
`Goalset planning returned None`이다. 파지가 나쁜 게 아니라 팔이 거기까지 못 간다.
원래 실행된 #23이 신뢰도 0.873이었던 것도, cuRobo가 goalset에서 닿는 것 중 고른
결과였기 때문이다.

### 얼마나 좁은가

`map/measure_reach.py`로 URDF에서 직접 쟀다. 골반 고정, 오른손바닥 위치.

| 손바닥 높이 (골반 기준) | 팔 7관절만 (지금) | + 허리 3관절 |
|---|---|---|
| +0.15 | 0.388 | 0.617 |
| +0.20 | 0.386 | 0.625 |
| +0.25 | 0.403 | 0.579 |
| +0.30 | 0.406 | 0.589 |
| +0.35 | 0.415 | 0.590 |
| 최저 높이 | −0.106 | −0.213 |
| 좌우 폭 (+0.20) | −0.526 ~ +0.257 | −0.612 ~ +0.525 |

허리만 풀어도 앞으로 1.6배, 오른쪽으로 두 배, 손이 0.107 m 더 내려간다.
지금 cuRobo 설정(`end2end/curobo_assets/g1_right_arm.yml`)은 `base_link: torso_link`에
오른팔 7관절뿐이라 허리가 체인에 없다.

주의: 높이를 torso 기준으로 재면 허리를 풀어도 값이 안 변한다. 허리 관절이
골반과 torso 사이에 있어서 torso와 팔이 함께 돌기 때문이다. 골반 기준으로 재야 한다.

## GR00T-WholeBodyControl에서 가져올 것

`/home/sehoon/Projects/GR00T-WholeBodyControl`. IsaacLab 2.3.2 기반이라 버전이 같다.
두 덩어리로 나뉜다.

- `gear_sonic` (SONIC) — IsaacLab에서 전신 추종 정책을 학습·평가
- `decoupled_wbc` — 배포·원격조작 스택. MuJoCo에서 돈다 (`control/envs/g1/sim/base_sim.py`)

우리에게 필요한 조합은 `control/policy/g1_decoupled_whole_body_policy.py`의
`G1DecoupledWholeBodyPolicy`다. 상체 정책과 하체 RL 정책을 합치고 목표를 이렇게 받는다.

- `target_upper_body_pose` — 상체 관절값. `InterpolationPolicy`가 관절 공간에서 보간하고
  `upper_body_max_joint_speed`로 속도를 제한한다. 우리 팔 궤적이 그대로 들어간다
- `navigate_cmd` — 하체 이동 속도
- `base_height_command` — 기저 높이. 바닥 물체용 스쿼트가 이것

### MuJoCo로 옮기지 않아도 된다

하체 정책이 ONNX로 들어 있다.
`decoupled_wbc/sim2mujoco/resources/robots/g1/policy/GR00T-WholeBodyControl-Balance.onnx`
(그리고 `-Walk.onnx`). 규격은 같은 폴더의 `g1_gear_wbc.yaml`과
`scripts/run_mujoco_gear_wbc.py`에 전부 적혀 있다.

- 행동 15개 = 다리 12 + 허리 3. 위 표의 허리가 여기 포함된다
- 관측 516 = 86 × 6프레임.
  `[0:7]` 명령, `[7:10]` 기저 각속도 × 0.5, `[10:13]` 기저 프레임 중력 방향,
  `[13:42]` 관절 위치 × 1.0, `[42:71]` 관절 속도 × 0.05, `[71:86]` 직전 행동.
  관절 29개 전부를 본다 — 팔이 뻗는 것을 보고 다리로 보상한다
- 명령 7 = `[0:3]` 이동 vx,vy,wz (scale 2.0/2.0/0.5), `[3]` 기저 높이(기본 0.74),
  `[4:7]` 몸통 rpy
- `action_scale: 0.25`, `kps/kds`, `default_angles` 모두 yaml에 있음
- 제어 주파수 `simulation_dt 0.005 × control_decimation 4` = 50 Hz

IsaacLab에서 같은 86차원을 만들어 onnxruntime으로 추론하고, 15개 행동을 주어진
PD 게인으로 하체에 적용하면 된다. 셀·테이블·촬영·재생을 MuJoCo로 옮길 필요가 없다.
