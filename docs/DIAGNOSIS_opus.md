# 진단 (Opus 이어쓰기)

페이블(Fable 5.1)이 쓴 `docs/DIAGNOSIS.md`는 **보존 대상**이라 손대지 않는다. 2026-09-28 17:00 이후 Opus가 쓰는
진단은 모두 이 파일에 이어 쓴다. 보존 규칙과 위치는 `docs/HANDOVER_MODEL.md` 참고.

## 2026-09-28 17:00 — 모델이 Fable → Opus로 바뀜; 페이블 산출물 보존, 운반으로 이동

- **세훈님: 페이블이 만든 코드와 진단 파일은 건드리지 말고 보존하고, 복사해서 그 사본에 작업하라.** 커밋되지 않은 채
  남아 있던 페이블의 마지막 작업 트리(`grasp/reach_from_pose.py` +121줄, `grasp/test_grasps_in_isaac.py` +33줄: 실제
  기립 재생 `TEST_RISE_PKL`, `--regrasp-wrap`, `--assist-wrap`)를 그대로 커밋(`162c21d`)하고 태그
  `fable-260928-preserved`를 달았다. 이후 Opus가 바꾸는 것은 사본에만 한다: `grasp/*_opus.py`, 이 파일.
- **세훈님: 되는지 안 되는지 사실 모른다 — 랜더로 확인을 못 했다. 페이블이 한참 공부하고 시험은 안 해봤을 수 있다.**
  맞는 지적이다. 14:00~17:00 사이 납품된 영상은 hammer/v8 하나뿐이고, 그 사이 파워 그립 변형(팜다운 감싸기, 양손 보조,
  GraspGen-X 팜 주석, 그리드 24개, 스쿱, 공중 재파지)은 전부 테스터에서만 LOST로 끝나 영상이 없다.
- **세훈님: 드는 것까지는 확인했고, 그 다음은 크레이트로 옮기는 것.** 그래서 파지 반복은 중단한다. 세훈님이 영상으로
  HELD를 확인한 두 파지 — hammer #138(`fable40v2`, v3·v6), drill #57(`fable42p`, v3·v5) — 를 그대로 써서 크레이트까지
  운반을 렌더한다(`carry_both.sh` → `place_any.sh`: 집기 → 제자리에서 2배 느리게 기립 → 언어로 찾은 데스크 크레이트까지
  보행 → 크레이트 위에서 손 열기). 운반 카메라는 정면 eye(-0.2, -3.4, 1.9)에 look-at을 공구와 크레이트 중간
  (-0.9, -0.5, 0.7)으로 두어 경로 전체가 프레임에 들어오게 했다.
- 해머 운반 클립: 크레이트까지 보행 184프레임(위치 오차 0.052 m, 방향 1.1°), 놓는 손목 오차 평균 3.6 mm·최대 7.5 mm,
  기립 86프레임을 2배로 늘려 172프레임, 전체 1184프레임, 37.0 s에 손가락 열림.
- 진단용으로 만들어 두고 지금은 쓰지 않는 것: `test_grasps_in_isaac_opus.py`의 `RISE_PROBE` — 기립 5지점에서 물체를
  **손바닥 자신의 좌표계**로 찍어 어느 방향으로 빠지는지(손가락 축 방향 = 손끝에서 미끄러짐, 손바닥 법선 방향 = 케이지
  밖으로 떨어짐) 구분한다. 세훈님이 운반을 먼저 보자고 해서 큐에서 내렸다.

## 2026-09-28 17:16 — 해머 운반 v9: 기립 시작 1.7초 안에 놓침 (영상 있음)

- `260928/5지/hammer/v9` — 집기 → 기립 → 보행 → 크레이트 위에서 손 열기, 1184프레임. **LOST.**
  `[eval] end pos [-0.2524 0.0231 0.1533] dxy 0.0547 m dz -0.0087 m`.
- 물체 궤적(`results/fable40/place.log`)이 시각을 정확히 준다: 프레임 700에서 z 0.2516 m(+0.10 m, 들려 있음),
  프레임 800에서 z 0.1541 m에 **속도 0**(이미 바닥에 정지). 늘린 기립은 프레임 749에서 시작하므로 해머는 기립
  **첫 1.7초 안에** 손을 떠났다. 0.10 m 낙하는 4프레임이므로 이탈 시점은 프레임 780 근처.
- 프레임 800 이후 해머는 가만히 누워 있고 로봇은 크레이트까지 걸어가 그 위에서 손을 편다. **보행과 놓기 자체는
  맞다** — 옮길 것이 손에 없을 뿐이다. 즉 지금 막힌 것은 운반 경로가 아니라 기립 하나다.
- 접근 중 물체가 바닥에서 5.3 cm 밀린다(프레임 0→600, `moved` 0.025 → 0.053 m). 파지 전에 손이 해머를 미는 것.
  치명적이지는 않지만 기록.
- 렌더 카메라(방/머리/손목)로는 손가락 배치가 안 보인다. 방 카메라는 너무 멀고 머리·손목은 너무 가깝다. 그래서
  **측정으로 간다**: `RISE_PROBE`(기립 5지점에서 물체를 손바닥 좌표계로 + 근접 스냅샷 2뷰). 손가락 축 방향으로
  빠지면 손바닥이 돌면서 손끝 집기가 손잡이를 놓치는 것이고, 손바닥 법선 방향이면 기립 가속으로 접촉이 끊긴 것이다.
  전자면 손목이 손의 월드 자세를 유지하도록 풀고, 후자면 기립을 더 늦추거나 팔을 몸쪽으로 당긴다. 드릴 렌더가
  끝나면 자동으로 실행되게 큐에 걸었다(`rise_probe.sh`, carry_both가 죽을 때까지 대기).

## 2026-09-28 17:35 — 그립이 느슨한 이유: 조임 게인이 GraspGenX 자신의 값보다 50배 약했다

세훈님이 hammer/v9 를 보고: "20초 쫌 지나서 해머는 떨어지네 파워그립이라고 좀 꽉지는걸 추가하려고했었는데 페이블이",
"너무 그립이 느슨함", "25초쯤에 그립이 살짝 풀리는것 같은데. 오히려 꽉잡아야하는데".
그리고 방법 지시: **"그런걸 확인할때 해당부분 오픈소스를 봐야해. 그립이지? 그럼 graspgenx 코드를 읽어."**

그대로 NVIDIA GraspGenX 원본을 읽었다 (`/home/sehoon/Projects/GraspGenX`, origin NVlabs/GraspGenX).

| | GraspGenX 원본 | 우리 (v9/v6 렌더) |
|---|---|---|
| finger kp | 2000 (`end2end/dynamic_playback.py:70` FINGER_KP_DEFAULT) | 40 |
| finger kd | 200 (`:71` FINGER_KD_DEFAULT) | 0.4 = 0.2·(kp/10)^0.5 |
| finger effort limit | 200 N·m (`:689` finger_effort_limit) | 30 N·m |
| 제어 모드 | POSITION (`robot_profiles.py` UR10eInspireHandProfile) | position ✓ |
| close 타깃 | 엄지 yaw 1.308 고정 / pitch **0.6** / 네 proximal **1.47** | 1.308 / 0.5(우리 asset 한계) / 1.47 ✓ |
| 핑거팁 마찰 | mu 3.0 | 3.0 ✓ |
| mimic 팔로워 | kp 50 / kd 10 (`MIMIC_KP/KD`) | SOFT_MIMIC 이 매 서브스텝 타깃 재작성 |
| gripper armature | Inspire 프로파일은 설정하지 않음(0). Franka만 0.5 | 0.001 |

즉 **닫는 목표와 마찰은 이미 NVIDIA와 같았고, 조이는 힘만 50배 약했다.** 특히 kd 0.4 는 접촉이 흔들릴 때 손가락이
되밀리는 것을 막지 못한다 — 일어서면서 물체 관성이 손가락을 밀면 되밀려 나가고, 세훈님이 25초쯤 "살짝 풀린다"고 본
것이 그것이다. 힘이 아예 없어서 놓친 게 아니라, **속도 저항이 없어서 접촉이 풀렸다.**

렌더와 테스터는 같은 `robot_cfg()`(build_reach_reference.py:99-103)를 쓰므로 두 경로의 게인은 일치했다.
즉 이번엔 "두 실행의 차이"가 아니라 **양쪽이 같이 약했다**. 확인 방법: `[hand] squeeze` 로그 줄.

변경: 페이블 파일은 건드리지 않고 `grasp/play_in_cell_opus.py`(새 사본)와 `grasp/test_grasps_in_isaac_opus.py` 에
`robot_cfg()` 직후 오버라이드를 넣었다 — `HAND_KD` 가 설정될 때만 동작하므로 없으면 기존과 동일하다.
`HAND_KP=2000 HAND_KD=200 HAND_EFFORT=200`. 파지·클립·카메라·보행·드롭은 v9/v6 과 동일, 손 게인만 바뀐다.

주의(정직하게): 200 N·m 는 실제 Inspire 손가락 토크가 아니다. 실물 URDF 한계는 1.4 N·m 수준이고,
NVIDIA 자신도 `:684-688` 주석에서 "URDF의 effort=20 이 PD 힘을 묶어 손가락이 접촉에 대고 닫지 못하므로 올렸다"고
적어 두었다. 파지 검증 파이프라인의 값을 따르는 것이고, 실물 이식 때는 다시 내려야 한다.

### 해머와 드릴의 이탈 프레임

| | 700프레임 (들고 있음) | 바닥에 정지 | 판정 |
|---|---|---|---|
| hammer v9 (#138, fable40v2) | z 0.2516 | 800프레임 z 0.1541, 속도 0 | LOST |
| drill v6 (#57, fable42p) | z 0.2622 | 900프레임 z 0.1591, 속도 0 | LOST |

둘 다 일어서는 구간에서 놓쳤고, 그 뒤 보행과 손 펴기는 정상. 원인이 같은지는 확인 안 됨 — 각각 렌더해서 각각 판정한다.

## 18:20 그립 — 게인은 이미 같았고, 남은 차이는 접촉 솔버 반복 수

GraspGenX 소스(`/home/sehoon/Projects/GraspGenX/end2end/dynamic_playback.py`)와 항목별 비교:

| 항목 | GraspGenX 검증값 | 우리 v9/v6 | 우리 v10/v7 | 출처 |
|---|---|---|---|---|
| 물체 질량 | 0.2 kg | 0.2 | 0.2 | `:87` |
| 물체 mu / 손끝 mu | 10 / 3 | 10 / 3 | 10 / 3 | `:88-89`, 우리 `plan_scene.py:27-28` |
| 손가락 kp / kd / effort | 2000 / 200 / 200 | 40 / 0.4 / 30 | 2000 / 200 / 200 | `:68-71`, `:689` |
| 접촉 솔버 반복 (pos/vel) | 100 / 50 | 12 / 4 | 12 / 4 | `:91-98` |

그 소스가 두 실패를 각각 이름으로 적어 두었다.
- `:95-97` — 반복이 10이면 "the constraint solver doesn't fully converge and grasps slip during the lift segment". v9/v6 은 일어서는 구간에서 놓쳤다.
- `:84-87` — 마찰이 3.0 이면 "prone to being launched by close-time contact impulse spikes". v10/v7 은 닫는 순간 21 cm 튕겨 나갔다. 다만 우리 물체 마찰은 이미 10 이다.

결론: 질량·마찰·손가락 게인은 이미 그들과 같고, 남은 차이는 솔버 반복 수뿐이다(12/4 vs 100/50).
v11(해머)·v8(드릴)은 이 한 항목만 100/50 으로 올려 렌더한다. `SOLVER_IT`/`SOLVER_VIT` 로 로봇 아티큘레이션과
물체 바디 양쪽에 적용하고, `dump_physics` 로 실제 적용을 확인했다(`solverPositionIterationCount=100`).

### probe_v9 측정은 무효
물체가 rise 전 구간 `[-0.282 0.047 0.153]`(바닥 안착 높이) 고정, 손가락 `q - 목표 = -0.000`(접촉력 0),
손바닥 거리 304 → 560 mm. 이 실행에서는 물체가 손에 들어간 적이 없다(`SETTLE_IDLE=200` + `TEST_RISE_SLOW=2`).
게인 판정 근거로 쓰지 않는다.

## 18:45 손가락 게인 2000/200/200 은 Robotiq 기본값이었다 (v11 LOST 후)

v11(솔버 100/50, kp2000): LOST, dxy 0.617 m. v10(솔버 12/4, 같은 게인)도 같은 650→700
구간에서 튀었다 — **접촉 솔버 반복 수는 원인이 아니었다.**

측정(v11 render.log): 오른손 네 근위 관절이 target 1.47 에 대해
frame 500 q 0.64~0.68 / 600 0.65~0.82 / 650 0.65~1.01 — 캐리 내내 약 0.80 rad 모자란 채 정지.
kp 2000 × 0.80 rad = 1600 N·m 요구, 즉 effort 200 N·m 에 계속 포화.
frame 700 물체 속도 [-0.885 0.894 -0.339] m/s (수평 1.26 m/s) = 낙하가 아니라 사출.
head_700.png: 해머가 손가락에 감기지 않은 채 손바닥 옆으로 밀려나 공중에 있다.

오픈소스 재확인 — 우리가 베껴 온 2000/200/effort200 은 `dynamic_playback.py` 의
FINGER_KP_DEFAULT, 즉 **Robotiq 기본값**이다. GraspGenX 자신은 비-Robotiq 위치제어
그리퍼에 다른 숫자를 쓴다:

- `robot_profiles.py:378-381` (Panda): "Position control with the gains from
  `newton/examples/.../example_robot_panda_hydro.py`: ke=650, kd=100,
  effort_limit=20, armature=0.5."
- `robot_profiles.py:120-123`: "Newton's PD is unstable on low-inertia joints
  without armature; the Newton Panda examples use 0.15–0.5 here."

v11 의 실제 armature 는 **0.001** 이었다(note.txt 의 [hand] squeeze 줄) — 소스가 말하는
0.15~0.5 범위보다 150~500 배 작다.

다음(v12/v9): kp 650 / kd 100 / effort 20 / armature 0.5, 솔버 100/50 유지.

### 18:42 드릴 v8 (해머 v11 과 동일 설정) — LOST, dxy 0.300 m

frame 0~400 정지(z 0.1591), 450 에 moved 0.2325 m vel [-0.219 0.375 -0.383],
500 부터 끝까지 [-0.5673 0.187 0.1591] 정지. **들어올린 적이 없다** — 손가락이 닫히는
400~450 구간에 옆으로 차여 나가 0.30 m 떨어진 바닥에 멈췄고 z 는 처음 높이 그대로다.

해머는 700 프레임에 수평 1.26 m/s 로, 드릴은 닫는 순간에 사출됐다. 두 물체 모두
손가락 드라이브 포화라는 같은 후보를 가리킨다. v12/v9 로 kp650/kd100/effort20/armature0.5 진행.

(기록) 이 렌더 직후 실행 중인 체인 스크립트를 편집해 bash 의 읽기 오프셋이 깨졌고
같은 설정의 드릴 렌더가 한 번 더 시작됐다가 중단됐다. drill/v8/evidence/render.log 가
원본이다. 이후 체인은 실행 중 편집이 불가능하도록 별도 사본(render_gains_v12.sh)을 쓴다.

## 18:54 해머 v12 — 파지가 처음으로 성립했고, 남은 것은 사출이다

kp 650 / kd 100 / effort 20 / armature 0.5, 솔버 100/50. 판정 LOST (dxy 1.001 m).
그러나 실패의 성격이 바뀌었다.

| | v11 (effort 200) | v12 (effort 20) |
|---|---|---|
| 네 근위 관절 q vs target 1.47 | 0.65~1.01 (0.80 rad 부족) | 1.29~1.35 (0.13~0.17 rad 부족) |
| 물체 최고 높이 | 0.2837 (700 에서 사출) | **0.3405 (750)** |
| 손에 머문 구간 | ~0 | **500~770, 약 250 프레임(8 초)** |

즉 effort 를 200 -> 20 으로 내리자 손가락이 물체를 실제로 감쌌고, 처음으로 들어서 옮겼다.

남은 실패는 미끄러짐이 아니다. 프레임 정확 추출(hammer/v12/evidence/acc_765·770·775.png):
765·770 에 해머는 오른손에 들려 있고, 775 에는 이미 1 m 떨어진 바닥에 누워 있다.
5 프레임(0.167 s)에 약 1.0 m = 약 6 m/s. 750 프레임의 기록 속도는 0.72 m/s 였다.
접촉 임펄스 스파이크다.

(방법 메모: `-ss` 로 뽑은 프레임은 키프레임 탐색이라 시점이 어긋난다 — 실제로 room_771 은
해머가 이미 바닥에 있는 것처럼 보였다. `select=eq(n\,N)` 로 다시 뽑아 765/770 이 파지
상태임을 확인했다. 이탈 시점 판단에는 프레임 정확 추출만 쓴다.)

다음(v13/v10): v12 에서 한 번에 바꾼 네 값 중 **armature 만** 0.5 -> 0.001 로 되돌려 분리.
GraspGenX 의 `gripper_armature` 는 Newton 의 **프리즈매틱** 그리퍼 관절용이고 단위가
질량(kg)이다(robot_profiles.py:120-123, Panda 프로파일). 우리 손가락은 회전 관절이라
단위가 kg·m² 이고, 링크 관성은 `g1_right_arm.urdf` 의 right_hand_thumb_0_link 기준
ixx 1.60e-5 kg·m² — 0.5 kg·m² 는 그 약 3만 배다. 물체 로그는 10 프레임마다 남겨
사출 시점을 좁힌다.

### 19:05 드릴 v9 — 같은 개선, 같은 잔여 실패 (재현됨)

kp 650 / kd 100 / effort 20 / armature 0.5. LOST (dxy 0.988 m).

- 손가락: target 1.47 에 대해 q 0.96~1.18 (600~700). v8(effort 200)은 닫는 구간에 물체를
  0.23 m 차 냈고 z 가 한 번도 올라가지 않았다.
- 운반: z 0.1591 -> 0.3105 (700), 31 cm.
- frame 650 에 물체 속도 1.66 m/s — 아직 손에 있고 상승 중인데 임펄스가 들어온다.
- 750~800 사이 이탈, 850~900 사이 다시 0.6 m 이동 = 걸어가는 로봇 발에 차인 것.
  (해머 v12 와 최종 위치가 비슷한 것은 같은 구석으로 차였기 때문이다. 원인 판단에 쓰지 않는다.)

서로 다른 물체·클립에서 같은 신호가 나왔다: effort 20 으로 파지는 성립하고, 남은 것은
캐리 중의 접촉 임펄스다. 이미 큐에 있는 v13/v10(armature 0.5 -> 0.001, 물체 로그 10 프레임)이
그 한 값을 분리한다.

## 19:25 — 해머 v13: armature 0.5 는 사출의 원인이 아니다 (분리 완료), 그리고 임펄스의 출처는 손뿐이다

v13 = v12 에서 armature 만 0.5 -> 0.001. 결과는 개선이 아니라 파지 자체의 붕괴다.

- 손가락이 물체를 통과했다: 프레임 700 에 q [1.47 1.46 1.47 1.47 …] = 목표까지 완전히 닫힘
  (v12 는 같은 구간에서 목표에 0.13~0.17 rad 못 미친 채 멈췄다 = 물체가 막고 있었다).
- 물체는 들리지 않았다: 프레임 430~880 동안 z 0.1498 -> 0.153, 총 이동 0.074 m.
- 프레임 880->890 에 vel 14.3 m/s 로 사출, 이후 xy 속도가 고정된 채 z 만 자유낙하 -> 바닥 관통, dz -392.8 m.
  → armature 0.5 유지. effort 20 N·m 를 관성 0.001 kg·m² 관절에 걸면 접촉력으로 멈출 수 없다
  (GraspGenX robot_profiles.py:120-123 의 경고 그대로).

**정정 — "걸어가는 발에 차였다"는 틀렸다.** 두 렌더 모두 로그에
`[walk] body does not collide with the cell; hand links kept: 15` 가 있다. 손 링크 15개 외에는
콜라이더가 없으므로 다리·발은 물체를 건드릴 수 없다. 드릴 v9 의 850~900 이동도, 해머 v13 의
885 프레임 사출(evidence/acc_885.png 에서 해머가 오른손/다리 위치에서 관통 상태)도 모두 손이 만든 것이다.
**물체가 받는 임펄스의 유일한 출처는 손이다.**

**정정 2 — 해머 v12 는 850~900 에 다시 차인 것이 아니다.** v12 의 물체는 프레임 800 에 이미
최종 위치 [-1.0771 0.6931 0.1487] 에 있었고 그 뒤로 움직이지 않았다(850, 900 동일). 즉 750->800
구간의 1.07 m 이동이 사건 전부다. 드릴 v9 은 달랐다(800 에 [-0.61 -0.2333], 900 에 [-1.0571 0.6882]).
두 물체의 최종 위치가 비슷한 것은 경로가 같아서가 아니다.

### 다음 변경(v14/v11): 물체의 maxDepenetrationVelocity = 5 m/s

- IsaacLab 의 파지 태스크 설정은 예외 없이 잡히는 물체에 `max_depenetration_velocity=5.0` 을 준다:
  `manipulation/lift/config/franka/joint_pos_env_cfg.py:58`, `direct/factory/factory_tasks_cfg.py:141`,
  `manipulation/deploy/gear_assembly/gear_assembly_env_cfg.py:61` 등. (인핸드 재배치 태스크만 1000.)
- 우리는 준 적이 없다: v12 의 `[phys] /World/GraspTarget … ` 덤프에 그 속성이 없다.
- v12 의 실패는 손가락이 아직 닫힌 상태에서 0.2 kg 물체가 약 6 m/s 로 떠난 것이고, 이는 관통 복구가
  만드는 속도의 모양이다. 유일한 임펄스 출처가 손이라는 위 사실과 맞는다.
- 구현은 `grasp/play_in_cell_opus.py` 의 `OBJ_MAX_DEPEN_VEL`(미설정이면 속성을 쓰지 않아 기존과 동일).
  물체 로그는 10 프레임마다 남겨 사출 시점을 10 프레임 안으로 좁힌다.

## 20:05 — 손가락 콜라이더가 전부 볼록껍질이었다 (해머 v14 / 드릴 v11)

두 렌더 모두 LOST. 해머 dxy 0.8714 m, 드릴 dxy 0.9881 m (dz -0.0029 m).
maxDepenetrationVelocity 5 m/s 는 어느 쪽도 지키지 못했다. 해머에서 이탈 속도의 모양은 바뀌었다
(v12 약 6 m/s / 5 프레임 -> v14 약 2.9 m/s / 10 프레임). 이것만으로 원인 여부를 단정하지 않는다.

**측정된 결정적 사실.** 오른손 R_thumb_proximal_pitch 는 목표 0.5 rad 를 프레임 500 부터 받고 있는데,
해머 v14 에서 프레임 480~770 (290 프레임, 약 9.7 s) 동안 q = 0.00 rad 였고, 물체가 손을 떠난 뒤
30 프레임 만에 0.50 에 도달했다. 드릴 v11 은 같은 신호의 약한 형태로 480~760 동안 0.12~0.22 rad 였다.
네 손가락 proximal 도 목표 1.47 에 대해 해머 1.29~1.35 / 드릴 0.89~1.06 에서 멈췄다.
[near] 로 본 물체 무게중심과 가장 가까운 손 링크의 거리는 캐리 내내 6.0~7.4 cm 였다.
즉 엄지는 명령을 받고 자유공간에서는 움직일 수 있는데, 물체를 잡는 동안에만 막혀 있었다.

**원인 (추정 아님, USD 실측).** assets/g1_inspire/g1_29dof_inspire_hand.usd 를 열어 세었더니
손가락 콜라이더 50 개가 전부 physics:approximation = convexHull 이다
(R_thumb_intermediate, R_thumb_distal, R_*_intermediate 포함, 다른 값으로 저작된 것은 없음).
GraspGenX end2end/robot_profiles.py UR10eInspireHandProfile 은 같은 손에 대해
coacd_link_keywords = ("intermediate", "distal") 을 지정하고, 주석에
"The thumb tip / distal and the *_intermediate links have the concavity that matters for object contact."
라고 적는다. 손가락 안쪽의 오목한 면이 물리에서는 통째로 채워져 있었던 것이다.
물체 콜라이더가 볼록껍질이었던 것과 같은 종류의 문제가 접촉의 반대편(손)에 남아 있었다.

**다음 (v15 / v12).** 바꾸는 값은 하나: R_*_intermediate / R_*_distal 링크의 콜라이더를
convexDecomposition 으로 (play_in_cell_opus.py 의 FINGER_COACD, 기본값 없음 -> 끄면 종전과 동일).
USD 가 instanceable 이라 각 링크의 collisions 스코프를 먼저 de-instance 한다
(plan_scene.py:266 이 그 제약을 적어 두었다). 나머지는 v14/v11 그대로:
kp 650 kd 100 effort 20 armature 0.5, solver 100/50, maxDepenetrationVelocity 5.

## 20:15 — 해머 v15: 콜라이더 분해는 작동했고, 그 다음 문제가 드러났다

[eval] dxy 0.2388 m, dz -0.2497 m -> LOST. 그러나 실패의 종류가 바뀌었다.

**의도한 기구는 고쳐졌다.** `FINGER_COACD=intermediate,distal` (6 메시 분해) 후
R_thumb_proximal_pitch 가 프레임 560 에 목표 0.5 rad 에 도달하고 끝까지 유지한다.
네 손가락 proximal 도 1.47 에 도달한다. v14 에서 290 프레임 동안 0.00 에 막혀 있던 엄지가 풀렸다.
손가락 콜라이더의 볼록껍질이 엄지를 막고 있었다는 것이 확인됐다.

**드러난 다음 문제: 닫히는 손가락이 물체를 차 버린다.**
0~480 프레임 해머는 z 0.1499 에 정지. 490~500 첫 접촉.
프레임 510 에 z 0.3005 (10 프레임에 +15 cm), 속도 [0.04 -1.483 -1.395] = 2.04 m/s.
520 에 z -0.0611 (바닥 아래), 540 에 -0.0877 에서 정지.
손가락이 1.47/0.50 에 도달한 것은 560 이므로, 다 닫혔을 때는 이미 빈손이었다.
닫히는 속도(측정): 0 -> 1.47 rad 을 480~560, 80 프레임 = 평균 0.55 rad/s.

**다음 (v16/v13).** GraspGenX 의 close 방식을 그대로 쓴다:
end2end/robots/g1_inspire_arm.yaml 은 이 손에 `gripper_control_mode: velocity` 와
`gripper_close_velocity` thumb_0 = 0.0 / 나머지 +-0.25 rad/s 를 지정하고,
end2end/dynamic_playback.py:71 은 FINGER_KD_DEFAULT = 200.0 이다.
play_in_cell_opus.py:735-741 에 GraspGenX 자신의 이유가 이미 적혀 있다 --
position mode 는 "snaps the fingers to the closed angles and they bat the object away".
v15 의 0.55 rad/s 는 그들의 0.25 rad/s 의 2.2 배다.
바꾸는 것: CLOSE_MODE=velocity CLOSE_VEL=0.25 CLOSE_KD=200 (코드는 이미 구현되어 있다).
정지 상태의 쥐는 힘은 같다: 200 x 0.25 = 50 N.m 요구 -> effort 20 N.m 로 잘리므로
position mode 에서 잘리던 20 N.m 과 동일하다. 달라지는 것은 접근 속도뿐이다.
나머지(FINGER_COACD, kp 650, effort 20, armature 0.5, solver 100/50, depenetration 5)는 v15 그대로.

**정정 (20:20): velocity mode 의 kd 는 800 이다.**
위에 CLOSE_KD=200 이라고 적었던 것은 틀렸다. dynamic_playback.py:71 의 FINGER_KD_DEFAULT = 200.0 은
position mode 의 kd 이고, velocity mode 는 :642-650 에서 `joint_target_ke = 0.0`,
`joint_target_kd = finger_velocity_kd` 기본값 **800.0** 을 쓴다("Match newton_grasp_eval.py's FINGER_KD=800").
v16 은 CLOSE_KD=800 으로 실행한다.

같은 파일을 읽다가 나온 것 하나 더 (다음 단계의 근거): dynamic_playback.py:685-691 주석이
URDF 의 `effort=20` 을 지목하며 "caps the PD force so the fingers can't close against gravity / contact"
라고 쓰고 finger_effort_limit 기본값을 200 으로 올린다. g1_inspire_arm.yaml 은 1000.0 이다.
우리는 EFF=20 으로 돌리고 있다 — 소스가 이름을 들어 지목한 값이다.
v15 에서 손가락이 1.47 까지 닫힌 것은 빈손일 때였으므로, 20 N.m 로 물체를 실제로 쥘 수 있는지는 측정된 바 없다.
한 번에 하나만 바꾸므로 v16 은 effort 20 을 유지하고, 접근을 견디고 쥐는 데서 미끄러지면 v17 에서 200 으로 올린다.

## 20:35 — 정정: 엄지는 풀리지 않았다. 그리고 v16 은 v15 의 복제였다

세 가지를 기록한다. 첫 번째는 내가 20:15 에 쓴 것의 정정이다.

**1) 정정: 콜라이더 분해가 엄지를 풀었다는 것은 틀렸다.**
20:15 에 "엄지가 프레임 560 에 0.5 rad 에 도달하고 유지한다 -> v14 의 290 프레임 블록이 풀렸다"고 썼다.
프레임별 추적을 다시 읽으면 순서가 반대다. 해머 v15 의 엄지 pitch q 는
  460: -0.00, 470: -0.00, 480: -0.00, 490: -0.00, 500: -0.00, 510: -0.00  (목표는 460 부터 계속 0.5)
  520: 0.12, 530: 0.29, 540: 0.46, 550: 0.50
이고 물체는 510 에 튕겨나가 520 에 바닥 아래로 빠졌다. 즉 엄지는 **물체가 없어진 뒤에** 움직였다.
v14(290 프레임 막힘 -> 물체 이탈 후 30 프레임 내 0.5 도달)와 같은 패턴이고, 막힌 기간만 50 프레임으로 짧다.
짧아진 이유는 물체가 더 일찍 떠났기 때문이지, 엄지가 풀렸기 때문이 아니다.
**물체가 손에 있는 동안 엄지 pitch 가 0.00 을 내놓는 현상은 v11/v14/v15 에서 그대로 재현된다.**
콜라이더 볼록껍질은 (측정된 사실로서) 있었고 분해도 적용됐지만, 엄지를 막은 원인은 그것이 아니다.

같은 구간에서 네 손가락 proximal 은 460: 0.03 -> 510: 0.81 로 감긴다. 엄지가 0.00 인 채 네 손가락만
감기므로 물체는 맞은편이 없는 한쪽 그립에서 밀려나간다(510 에 2.04 m/s).

**2) 드릴 v12 는 해머와 실패의 종류가 다르다.** dxy 0.3547 m, dz -0.0029 m -> LOST.
프레임 440 에 속도 2.39 m/s 를 받는데, 그때 네 손가락 proximal 은 아직 0.00 rad 이고 엄지 pitch 만 0.08 이다.
손목의 물체 기준 z 는 430 에 0.04, 440 에 0.01 로 아직 내려오는 중이었다.
즉 드릴은 손이 파지 자세로 접근하는 동안 맞았고, 닫는 속도와 무관하다. (자세한 수치는 drill/v12/note.txt)
드릴의 다음 단계는 파지 자세에서의 손-물체 관통이며, GraspGenX 가 gripper 메시 충돌로 grasp 를 걸러내는
단계를 우리 경로가 건너뛴 자리와 같다. 지금은 기록만 한다.

**3) v16 은 velocity close 를 시험하지 못했다 — 설정이 적용되지 않았다.**
render_gains_v16.sh:9 의 export 블록이 `CLOSE_MODE=position` 을 직접 넣는다. 체인이 export 한
`CLOSE_MODE=velocity` 를 이 줄이 덮어쓴다. 렌더 로그에 `velocity-mode close` 줄이 없고
`[hand] squeeze (GraspGenX position-mode gains): ... effort 20.0` 만 있는 것으로 확인된다.
물체 추적이 v15 와 소수 4 자리까지 동일하다(510 에 z 0.3005, 속도 [0.04 -1.483 -1.395]).
(부수적으로 얻은 것: 같은 설정은 프레임 단위로 재현된다. A/B 비교는 깨끗하다.)
중단된 hammer/v16 폴더(프레임 530 에서 끊긴 로그, [eval] 없음, 잘린 영상)는 시도가 아니므로
영상보관에서 빼서 scratchpad/aborted/hammer_v16_aborted 로 옮겼다. 해머는 v15 까지가 마지막 시도다.
`CLOSE_MODE=${CLOSE_MODE:-position}` 로 고친 사본을 다음 체인에 쓴다.

**다음 한 가지: effort 20 -> 200.**
근거가 되는 숫자는 오픈소스에 이름까지 적혀 있다. dynamic_playback.py:685-691 은 URDF 의 `effort=20` 을
지목하며 "caps the PD force so the fingers **can't close against gravity / contact**" 라고 쓰고
finger_effort_limit 기본값을 200.0 으로 올린다(:689). g1_inspire_arm.yaml 은 1000.0 이다.
우리는 EFF=20 으로 돌려 왔다. 측정된 증상이 그 문장과 같다: 접촉이 없는 네 손가락은 감기고,
물체를 밀어야 하는 엄지 pitch 만 0.00 을 내놓는다.
v16 은 v15 에서 effort 만 20 -> 200 으로 바꾼다. close 방식은 position 그대로 둔다(한 번에 하나).
엄지가 물체를 밀면서 0.5 에 도달하면 원인이 확정된다. 도달하고도 물체가 튕기면 그때 velocity close
(CLOSE_MODE=velocity, CLOSE_VEL=0.25, CLOSE_KD=800)를 시험한다.

---

## v16 결과와, 세훈님이 되돌아가라고 한 지점 (260928 21:00)

**1) v16 은 LOST 다. effort 20 -> 200 은 절반만 맞았다.**
엄지는 풀렸다. 엄지 pitch(목표 0.5)가 450: 0.01 -> 480: 0.41 -> (하중에 0.25 로 눌렸다가) 530: 0.50 으로
올라와 유지된다. v15 의 평평한 -0.00 과 다르다. 2.04 m/s 의 튕김도 사라졌다(최대 0.245 m/s).
그런데도 물체는 한 번도 잡히지 않았다.
- 프레임 560 이후 손가락 12 축의 q 가 target 과 잔차 0 으로 일치한다. 접촉이 없다는 뜻이다(빈손).
- 손목은 물체 기준 z 0.01(560) -> 0.62(840) 로 61 cm 올라가는데 물체는 책상 높이에 정지해 있다.
- 닫는 동안 가장 가까운 손가락 링크 원점이 물체 중심에서 최소 6.4 cm(480), 닫힌 뒤 9.5 cm.
- 프레임 880 에 걸어가는 로봇이 물체를 5.6 m/s 로 찬다. 바닥에 떨어진 건 이것 때문이고, 파지 실패의
  결과이지 원인이 아니다.
게인은 이제 충분하다. 남은 원인은 파지 자세다. (drill v13 도 LOST: dxy 1.4693 m, dz -0.0029 — 들린 적이
없고 1.47 m 밀렸다. v12 와 같은 접근 중 타격이며 닫힘 실패가 아니다.)

**2) 세훈님 지적이 맞다. 성공했던 구간이 있고, 나는 그 뒤로 계속 나빠졌다.**
vN 폴더 전체를 읽어 HELD/LOST 를 세었다.

| 성공(HELD) | 손 설정 |
|---|---|
| hammer v3 (dz +0.1109), v6 (+0.1109, v3 와 동일 클립), v8 (+0.0907) | 기본 actuator: kp 40, kd 0.4, effort 30, solver 12/4 |
| drill v3, v5 (+0.1044) | 〃 |
| crate v1 (+0.0656), v4 (+0.0847) | 〃 |

hammer v9 / drill v6 / crate v5 이후는 전부 LOST 다. v10 부터 내가 손가락 게인을 GraspGenX 의
position-mode 숫자(kp 2000, kd 200, effort 200, armature 0.001)로 올렸다. v10 자신의 note 가
"kp 2000 은 잡는 게 아니라 밀어내는 힘이었다" 로 끝난다. v10~v16 은 이미 답이 나온 방향을 계속 판 것이다.

**3) 그래서 원래 풀어야 했던 문제는 이것이다 — hammer/v9 의 note 에 이미 적혀 있다.**
"the pick holds, the stand-up drops it." v4, v5, drill v4 와 같은 실패다. v9 는 프레임 700 에 물체
z 0.2516(+0.10 m, 들려 있음), 800 에 0.1541(바닥)을 쟀고 상승은 749 에 시작하므로 780 근처에서 빠졌다.
v9 가 지정한 다음 측정(팔 기준 좌표계에서의 물체 위치)은 끝내 실행되지 않았다.

**4) 오픈소스 재독 (GraspGenX end2end) — velocity close 는 들어올리는 내내 유지되는 설계다.**
- `dynamic_playback.py:641-661` velocity 모드: `joint_target_ke = 0`, `joint_target_kd = 800`
  (`newton_grasp_eval.py` 의 FINGER_KD 와 맞춤), mode = VELOCITY. position 모드 주석: "snaps the fingers
  to the closed angles and they bat the object away".
- `dynamic_playback.py:1686 drive_segments` 는 궤적의 **매 프레임** velocity target 을 다시 넣는다.
  판정은 `is_open = abs(target - open_v) < 1e-6` 하나뿐이라, 스케줄이 open 이 아닌 동안에는 접근·닫힘·
  **들어올림·이송** 전 구간에서 손가락이 계속 닫는 쪽으로 구동된다. 즉 쥐는 힘이 각도 오차에 비례하지
  않고, 되밀림을 kd 800 의 댐퍼가 상수로 막는다. 이것이 "일어설 때 풀린다"에 정확히 대응하는 기구다.
- `dynamic_playback.py:685-691` URDF 의 effort=20 을 finger_effort_limit(기본 200, 우리 yaml 은 1000)으로
  덮는다.
- `end2end/robots/g1_inspire_arm.yaml:39-51` `gripper_control_mode: velocity`, thumb_0 은 **0.0**(구동하지
  않음), thumb_1/2 는 -0.25, 네 손가락은 +0.25.

우리 구현은 이 설계에 충실하다. `play_in_cell_opus.py:797-805, 828-829` 는 프레임마다 그리고 substep
마다 velocity target 을 다시 넣고, `test_grasps_in_isaac_opus.py` 의 `put()`(:289)도 SQUEEZING 인 동안
substep 마다 다시 넣으며 `squeeze(True)` 는 꺼지지 않는다. 다만 **우리 기본값 CLOSE_KD 는 8.0 으로
GraspGenX 의 800 보다 100 배 작다.**

스케줄 쪽은 원인이 아님을 쟀다. pick 클립(fable40v2 749 프레임, fable42p 734)은 449/434 프레임부터
끝까지 open 에서 떨어져 있고, place 클립만 1128/1113 에서 놓는다. v9 의 780 프레임 이탈은 쥐라고
명령한 구간 한가운데다.

**5) 지금 도는 것: v9 가 지정한 측정을, 세훈님이 되돌아가라고 한 그 설정에서.**
`scratchpad/rise_ab.sh` — A = hammer v3/v6/v8 의 손 그대로(HAND_KP=40, HAND_KD 미설정 -> kd 0.4/effort 30,
position close), B = A 에서 **오직 한 가지**, GraspGenX 자신의 velocity close(CLOSE_MODE=velocity,
CLOSE_VEL=0.25, CLOSE_KD=800)만 바꾼 것. 확정된 파지(hammer #138, drill #57)와 각자의 carry 클립으로
일어서는 동안 5 지점에서 물체를 **손바닥 자신의 좌표계**에서 읽는다. 손가락 축을 따라 빠지면 손끝
집기가 손잡이를 놓치는 것이고, 손바닥 면에서 멀어지면 상승 가속도에 접촉을 잃는 것이다. 처방이 다르다.

## v9 의 질문이 드디어 측정되기 직전이다 — 그리고 "성공"의 정체가 달랐다 (260928 21:20)

**1) hammer #138 이 HELD 였다는 기록은 수직 들어올리기였다. 전신 일어서기는 그때도 실패한다.**
같은 파지, 같은 물체 위치(`[-0.3 0.05 0.162]`, 테스터·렌더 동일), 같은 링크(PALM_LINK = right_wrist_yaw_link)로
두 기록을 맞춰 보았다.

| | 형식 | 결과 |
|---|---|---|
| v6 `evidence/verify_138.txt` | `box at close +0.024  after lift dz +0.147 m` — 테스터 :590 의 **수직 들어올리기** | HELD |
| 오늘 (기준 게인 그대로) | `slipped in the stand-up (+0.000 of 0.36 m)` — `TEST_RISE_PKL` 의 **전신 일어서기** | LOST |

두 줄은 서로 다른 시험이다. 즉 #138 은 팔로 수직으로 들면 +0.147 m 를 버티고, 캐리 클립의 전신 일어서기
(몸 0.36 m)에서는 물체가 전혀 따라오지 않는다. hammer/v9 가 쓴 "the pick holds, the stand-up drops it" 이
테스터 안에서, **기준 게인(kp 40 / kd 0.4 / effort 30, position close)에서** 재현된다. 게인 문제가 아니다.
(v6 의 cuRobo 4.9 mm 와 오늘의 3.9 mm 는 같은 궤적의 다른 웨이포인트다 — 출력이 `err[k, n_go-1]` 이고
n_go 가 프레임 수에 따라 달라진다. 해가 바뀐 증거가 아니므로 폐기한다. reach_all.npz(12:12)는 v6(13:45)보다
앞서므로 인덱스 138 은 같은 파지다.)

**2) 그런데 내 probe 는 손실의 순간을 볼 수 없었다. 첫 관측이 이미 사후였다.**
`test_grasps_in_isaac_opus.py:537` 의 `_checks = {int(round(len(_seq) * _fr)) - 1 for _fr in (0.001, ...)}` 에서
선두 항 0.001 은 실제 N(150~260)에서 `round(N*0.001) - 1 == -1` 이 되어 **한 번도 발화하지 않았다.**
그래서 가장 이른 관측이 일어서기의 24% 다. 그 시점에 물체는 이미 손바닥 좌표계로 304 mm 밖이고 정지해 있으며,
24/50/75/100% 네 점이 서로 같은 값을 주는 이유가 이것이다(물체 `[-0.282 0.047 0.153]` 고정, tilt 6.8 deg 고정,
손가락 12 축 1.47 에 잔차 0). 손실은 첫 1/4 안에서 끝난다.
`_checks` 를 0/4/8/12/16/20/25/50/75/100% 로 고쳤다(내 사본만). 첫 프레임이 반드시 찍힌다.

**3) 다시 도는 것:** `scratchpad/rise_ab2.sh` — A(기준 게인, position close) / B(A + GraspGenX 자신의
velocity close: kd 800, 0.25 rad/s)를 해머 먼저, 각 arm 마다 일어서기 첫 1/4 을 촘촘히 읽는다.
손가락 축을 따라 빠지는지, 손바닥 면에서 멀어지는지가 여기서 갈린다.

**4) 참고로 확보한 숫자(아직 시험 안 함):** `end2end/tasks.py:170-184` 는 들어올리기를 LIFT_FRAMES 240
(60 fps = **4 초**)로 늘리며 이유를 적는다 — "cuRobo's lift_interpolated_trajectory is only ~41 waypoints,
which at 60 fps plays in 0.7 sec — too fast for the gripper to keep grip on the object under physics."
접근(APPROACH_FRAMES)과 최종 직진(GRASP_FRAMES)도 각각 120 프레임 = 2 초로 늘린다.
우리 일어서기는 플래너 ~1.4 초를 `build_place_reference.py:66-69` 의 RISE_SLOW=2 로 늘린 ~2.8 초다.
RISE_SLOW 는 환경변수이므로 페이블 파일을 고치지 않고 올릴 수 있다. probe 결과가 "상승 가속도에 접촉을
잃는다"로 나오면 이것이 다음 한 가지다.

## "충분히 꽉 쥐었다"를 판정하는 오픈소스는 IsaacLab 안에 있다 (260928 21:40)

세훈님: "드릴 13 보면 이미 잘 잡은 상태같은데 계속 더 조이다가 날아가는듯. 엔비디아 오픈소스 중
tactile 관련 없나. 어느정도면 충분히 꽉쥐었다를 판단할 수 있으면 좋겠다."

### 먼저, 세훈님 판독이 맞고 내 기록이 틀렸다 (측정)
| | 들린 높이 | 사출 순간 | 사출 속도 |
|---|---|---|---|
| drill v13 | z 0.1695 -> 0.3138 (+0.144 m) | frame 740 -> 750 | 4.2 m/s (vel [-3.02 1.08 -2.70]) |
| hammer v14 | z 0.1768 -> 0.3215 (+0.165 m) | frame 770 -> 780, 0.66 m / 10 frame | 약 4 m/s |

두 건 다 GraspGenX 의 성공 기준(end2end/clutter_task.py:64 `LIFT_SUCCESS_DZ = 0.05`) 의 약 3 배를
들어 올린 뒤에 터졌다. 미끄러짐이 아니라 사출이다. drill v13 을 "접근 중 타격, 들리지 않음"이라고
적었던 것은 틀렸고 vN note.txt 두 개에 정정해 넣었다.
들고 있는 동안에도 위치는 고정인데 속도만 튄다(v14 frame 680: 2.007 m/s). 매 스텝 관통 복구
임펄스가 들어갔다 감쇠되는 모양이며, 이것이 쌓이다 마지막에 터진 것으로 보인다(이 인과는 아직 미측정).

### GraspGenX 에는 없다
`grep -rl "tactile|contact_force|net_contact|force_closure|grasp_quality|ContactSensor" --include=*.py`
전체 0 건. 성공 판정은 `LIFT_SUCCESS_DZ = 0.05` 하나뿐(:603, :845)이다. 그래서 언제 그만 조여야
하는지를 판단하는 장치가 설계상 없다 — `dynamic_playback.py:1686` 은 궤적 전 프레임에 손가락 속도
목표를 다시 찍는다.

### IsaacLab 에는 세 갈래가 있고, 우리에게 맞는 것은 DexSuite 다
1. **DexSuite (Kuka + Allegro, 다지 손 파지)** — 우리 문제와 같은 모양.
   `manager_based/manipulation/dexsuite/config/kuka_allegro/dexsuite_kuka_allegro_env_cfg.py:43-56`
   손끝 링크마다 `ContactSensorCfg(prim_path=.../<tip>, filter_prim_paths_expr=["{ENV_REGEX_NS}/Object"])`
   를 달아 **그 손끝이 그 물체에 주는 힘**만 뽑는다. 관측은 `clip=(-20.0, 20.0)` 이고 주석이
   `# contact force in finger tips is under 20N normally` 다.
   판정식은 `mdp/rewards.py:50-71`:
   ```python
   good_contact = (thumb_mag > threshold) & ((index_mag > threshold) | (middle_mag > threshold) | (ring_mag > threshold))
   ```
   호출부 `mdp/rewards.py:111` 의 threshold 는 **1.0 N**. 즉 NVIDIA 의 "제대로 쥐었다" 정의는
   *엄지 1 N 이상 + 마주보는 손가락 중 하나 1 N 이상*(대향 접촉쌍)이고, 정상 범위 상한은 20 N 이다.
2. **Forge (조립, 힘 제어)** — `direct/forge/forge_env.py:95-104` 가
   `root_physx_view.get_link_incoming_joint_force()` 로 6 축 FT 를 읽어 EMA 로 평활하고,
   `:242-243` 에서 `relu(|F| - threshold)` 를 페널티로 준다. 임계는
   `forge_tasks_cfg.py:18 contact_penalty_threshold_range = [5.0, 10.0]` N.
3. **TacSL (진짜 촉각)** — `isaaclab_contrib/sensors/tacsl_sensor/`, GelSight 영상형 촉각 센서
   (`isaaclab_assets/sensors/gelsight.py`, demo `scripts/demos/sensors/tacsl_sensor.py`).
   손끝에 젤시트를 붙여 변형 영상을 렌더하는 방식이라 우리 5지 손에는 자산부터 새로 필요하다.

### 우리 코드에 붙는가
`grasp/play_in_cell_opus.py:84` 는 이미 `from isaaclab.sensors import Camera, CameraCfg` 를 쓴다.
같은 패키지의 `ContactSensorCfg` 를 손끝 링크마다 `filter_prim_paths_expr=["/World/GraspTarget"]`
로 달면 `data.force_matrix_w` 가 나온다. 새 의존성은 없다.
다음 한 걸음은 게인을 또 만지는 것이 아니라 **힘을 계측하는 것**이다: 사출 직전 구간(v14 frame
600~780) 에서 손끝별 접촉력이 실제로 몇 N 인지 찍고, DexSuite 의 1 N / 20 N 과 비교한다.
그 숫자가 나오기 전에는 "과압이 원인"도 아직 가설이다.

## 손끝 힘을 처음으로 쟀다: 최대 55,478 N (260928 22:05, hammer v17)

세훈님 질문("어느정도면 충분히 꽉쥐었다를 판단할수 있음 좋은데")에 대한 답을 추측이 아니라
계측으로 적는다. IsaacLab 의 ContactSensor 를 오른손 링크 전체에 달고 물체 하나로 필터해서
(`play_in_cell_opus.py`, CONTACT_FORCE=1 일 때만 동작) v14 와 물리 설정이 한 글자도 다르지 않은
렌더를 다시 돌렸다.

측정치 (results/fable40/g.log, 프레임 = 60 fps):

| 프레임 | 손끝 힘 합 | 최대 링크 | 상태 |
|---|---|---|---|
| 480-490 | 0 N | - | 접근 중, 접촉 없음 |
| 500 | 56 N | ring_intermediate 56 | 첫 접촉 |
| 510-580 | 465 → 2130 N | thumb_intermediate 751 | 쥠. 물체 z 0.149 → 0.16 |
| 610 / 630 | 7870 / 8187 N | thumb_proximal 8187 | 접촉이 0 N 과 kN 을 오간다 |
| 670-760 | 794 → 13,966 N | thumb_proximal 8000~8900 | 들림 진행 (z 0.26 → 0.34) |
| **770** | **55,478 N** | **ring_intermediate 24,651** | **사출** |
| 780 | 0 N | - | 물체는 0.88 m 날아가 바닥 [-0.557 0.883 0.052] |

비교 기준 (전부 NVIDIA 자신의 코드):
- `IsaacLab .../dexsuite/config/kuka_allegro/..._env_cfg.py:43-56` — 손끝 접촉력 관측을
  **±20 N 로 클립**하며 주석은 "contact force in finger tips is under 20N normally".
- `.../dexsuite/mdp/rewards.py:50-71`, 임계값 `:111` — 파지 성립은 **엄지 > 1.0 N 이고
  반대쪽 손가락 중 하나 > 1.0 N**.

즉 우리 손끝은 정상대역의 **450배**(8,900 N), 사출 순간은 **1,200배**(24,651 N)였다.
0.2 kg 물체에 55 kN 이 걸리면 한 스텝에 수백 m/s 의 속도 변화가 가능하다. 세훈님이
영상에서 "이미 잘 잡은 상태같은데 계속 더 꽈아아앙아악 지다가 날라가는듯"이라고 읽은 것이
숫자로 그대로 나왔다. v13(드릴)·v14(해머)의 4 m/s 사출은 미끄러짐이 아니라 과압이다.

### 왜 그렇게 됐나 — 내가 올린 게인이 NVIDIA 기준의 65배였다

`isaaclab_assets/robots/unitree.py:598-611`, **NVIDIA 가 G1 + Inspire 손에 직접 쓰는 값**:

```python
G1_INSPIRE_FTP_CFG.actuators["hands"] = ImplicitActuatorCfg(
    effort_limit_sim=30.0, velocity_limit_sim=10.0,
    stiffness=10.0, damping=0.2, armature=0.001)
```

이 저장소의 기본값(`build_reach_reference.py:99-102`)도 같은 값이다. 그런데 v13~v17 은
`kp 650 / kd 100 / effort 20 / armature 0.5` 로 돌렸다 — 강성 65배, 감쇠 500배, armature 500배.
`[hand] frame 540` 에서 손가락 q 0.86 vs target 1.47 (0.61 rad 부족) 이므로 PD 는 매 프레임
650 × 0.61 = 396 N·m 를 요구하고 effort 한계 20 N·m 에 **영구 포화**한다. 포화한 PD 는 접촉을
밀어내고, 떨어진 접촉을 다음 스텝에 다시 때린다 — 610/620/630 프레임의 8187 N → 0.24 N → 0 N
진동이 그 모양이다.

### 다음 한 가지 변경 (v18, 큐에 올림)

손 게인을 위 NVIDIA 값으로 되돌리고 힘을 같이 잰다. 다른 것은 v14/v17 과 동일.
성공 판정은 이제 추측이 아니라 DexSuite 기준으로 읽는다: 유지 구간의 손끝 힘이
1 N 이상 20 N 근처인가. 필요한 힘은 계산상 아주 작다 — 물체 0.2 kg, 재질 마찰
staticFriction=10 이므로 필요 파지력은 m·g/(2μ) ≈ 0.1 N 이다. 20 N 은 그 200배다.

### 일어서기 A/B 는 무효였다 (같은 날, rise_ab2)

arm A(position, kp 40/kd 0.4)와 arm B(velocity, kd 800) 둘 다
`slipped in the stand-up (+0.000 of 0.36 m) -> LOST` 로 **완전히 같은 줄**이 나왔다.
그러나 이 테스트는 파지를 재고 있지 않았다:
- 물체 월드 좌표가 rise 0% 부터 100% 까지 `[-0.282 0.047 0.153]` 로 **mm 단위까지 동일**하다.
  손이 0.5 m 움직이는 동안 물체는 1 mm 도 끌려가지 않았다.
- arm A 의 `q - target` 은 전 구간 |0.008| rad 이하 — 손가락이 **아무 저항 없이** 목표까지
  닫혔다. 사이에 물체가 없었다는 뜻이다.

그러므로 "close mode 는 일어서기 실패의 변수가 아니다"라고 결론 낼 수 없다. 이 A/B 는
버리고, 테스터의 일어서기 구간은 rise 0% 에서 접촉력이 DexSuite 기준(엄지 1 N + 반대 1 N)을
넘는지 먼저 확인한 뒤에만 의미가 있다. 그 확인 수단이 이번에 생겼다.

## 테스터의 '일어서기' 판정은 처음부터 빈 손을 재고 있었다 (260928 22:20)

앞 절에서 hammer #138 의 A/B 가 무효라고 적었는데, 다른 물체·다른 파지·다른 시각의 실행에서도
같은 서명이 나온다. 세 실행을 같은 두 줄로 확인했다:

| 실행 | 물체 월드 좌표 (rise 0% → 100%) | 손가락 `q - target` |
|---|---|---|
| probe_v9 (18:05, 게인 올리기 전) | `[-0.282 0.047 0.153]` → 동일 | (해당 없음, 같은 설정) |
| r2_A/B hammer #138 (21:1x) | `[-0.282 0.047 0.153]` → 동일 | 전 구간 ≤ 0.008 rad |
| r2_A drill #57 (21:39) | `[-0.294 0.084 0.159]` → 동일 | 전 구간 ≤ 0.008 rad |

손이 0.5 m 움직이는 동안 물체는 mm 단위로 움직이지 않았고, 손가락은 아무 저항 없이 목표까지
닫혔다. hammer 실행의 파지 시점 출력은 `at grasp: palm - object [0.147 -0.053 0.136]` — 0.206 m 다.

같은 파지를 렌더(`play_in_cell_opus.py`)로 돌리면 반대 결과가 나온다: hammer v17 에서 물체는
0.1996 m 올라갔고 접촉 센서는 kN 을 읽었다. 즉 **렌더는 물체를 잡고, 테스터의 일어서기 단계는
잡지 않는다.**

두 도구가 읽는 모션 파일이 다르다는 것은 확인된 사실이다:
- 테스터: `results/motion/${RUN}k.pkl` + `results/${RUN}/reach_all.npz --only <k>` (rise_ab2.sh:24)
- 렌더:   `results/motion/${REF}place.pkl` + `${REF}place_hands.npy` (render_gains_v17.sh:18)

왜 테스터의 리치 종료 자세가 렌더의 것과 다른지는 여기서 규명하지 않았다. 확정할 수 있는 것은
범위가 정해진 하나뿐이다: **테스터가 낸 `slipped in the stand-up -> LOST` 판정은 파지를 재고 있지
않으므로 근거로 쓸 수 없다.**

이것이 중요한 이유는, v13~v17 에서 손 강성을 NVIDIA 기준의 65배까지 올린 출발점이 18:05 의
probe_v9 — 즉 위 표의 첫 줄 — 이었기 때문이다. "일어설 때 미끄러지니 더 세게 쥐어야 한다"는
전제가 빈 손을 잰 결과에서 나왔다. 앞으로 테스터의 일어서기 판정은 rise 0% 에서 접촉력이
DexSuite 기준(엄지 1 N + 반대쪽 1 N)을 넘는지 확인된 뒤에만 읽는다.

## NVIDIA 값으로 되돌리니 과압이 사라졌다. 그런데 이번엔 놓는다 (260928 22:01, hammer v18)

v17 은 손 게인 kp 650 / kd 100 / armature 0.5 / effort 20 으로 최대 **55,478 N** 을 찍고 물체를
사출했다. v18 은 같은 물리·같은 모션으로 손 게인만 NVIDIA 가 이 로봇·이 손에 직접 쓰는 값
(`isaaclab_assets/robots/unitree.py:598-611`, `G1_INSPIRE_FTP_CFG.actuators["hands"]`:
stiffness 10, damping 0.2, effort_limit_sim 30, armature 0.001)으로 되돌려 다시 측정했다.
렌더 로그가 실제로 쓴 값: `[hand] squeeze ... kp 10.0 kd 0.2 effort 30.0 armature 0.001`.

측정된 차이는 크기뿐이 아니라 **모양**이다.

| | v17 (kp 650) | v18 (kp 10) |
|---|---|---|
| 유지 구간 힘 | 0.24 N ↔ 8,187 N 진동 | 468~494 N 평탄 |
| `dexsuite_good` | f620/f680/f750 세 번 끊김 | f510~f650 **130프레임 연속 유지** |
| 최대 힘 | 55,478 N (f770) | 493.8 N (f570) |
| 결말 | 0.88 m 사출 | f660 해제, 제자리 복귀 |

즉 v17 의 폭주는 게인 문제였고 제거됐다. PD 포화 계산과 일치한다: 정지 오차 0.75 rad 에서
kp 650 은 프레임마다 650×0.75 = 488 N·m 을 요구하는데 effort 한계는 30 N·m 이므로 영구 포화 →
접촉을 붙잡는 대신 두들긴다.

**그러나 v18 은 들어올릴 때 쥐는 힘이 스스로 빠졌다.** f590 481 N → f630 222 N → f650 116 N →
f660 0 N 단조 감소. 물체는 f600 부터 z 0.1483 → 0.1870 (+0.0387 m) 까지 올라갔다가 f660 에
0.1531 로 되돌아가 정지했다. GraspGenX 의 성공 기준 `end2end/clutter_task.py:64
LIFT_SUCCESS_DZ = 0.05` 의 77% 에서 놓친 것이다. 손가락 각도가 원인을 보여준다: f520~590 동안
목표 1.47 에 대해 0.86~0.89 에 막혀 있다가, f600 이후 0.92 → 0.97 → f680 에 전부 1.47 에
도달한다. 물체가 빠져나간 빈 공간을 따라 손가락이 계속 닫혔고, 침투 깊이가 줄면서 접촉력도
같이 줄었다. 사출이 아니라 하중에 밀려난 점진적 미끄러짐이다.

kp 10 이 요구한 토크는 10 × 0.75 = **7.5 N·m**, NVIDIA 자신의 한계 30 N·m 의 25% 다. 남은 75%
를 쓰지 않은 채 놓쳤다. 포화되지 않는 가장 큰 stiffness 는 30 ÷ 0.75 = **40** 이므로 v19 는
kp 40, kd 는 이 저장소 자신의 규칙(`build_reach_reference.py:99-102`,
`damping=0.2*(kp/10)**0.5`)에 따라 0.4, effort 30 / armature 0.001 은 NVIDIA 값 유지로 간다.
(kp 40 은 NVIDIA 의 10 보다 4배다. 권장값이 아니라 "측정된 정지 오차에서 effort 한계를 넘지
않는 최대치"라는 뜻으로만 고른 값이다.)

## `[eval]` 의 dxy 는 발에 차인 것까지 센다 (260928 22:01)

v18 의 판정은 `dxy 2.7568 m dz -0.0088 m -> LOST` 였다. 이 2.76 m 는 파지와 무관하다.
물체는 f660 해제 후 f840 까지 `[-0.263 0.046 0.153]` 에 **속도 0 으로 완전히 정지**해 있었다.
그동안 로봇은 크레이트로 걸어가고, 오른발목이 25.4 cm(f850) → 21.6 → 19.6 → 11.3 cm(f880) 로
물체에 접근한다. f890 에 물체가 `[0.6076 -0.1617 0.4901]`, 속도 `[3.613 -0.724 +z]` 로 튀어
f960 에 2.75 m 밖에 정지한다. 놓친 물체를 지나가면서 발로 찬 것이다.

따라서 `dxy`/`dz` 만으로는 "쥐다가 사출"과 "놓친 뒤 발에 차임"을 구분할 수 없다. 앞으로 파지
판정은 **해제 프레임**(접촉력이 0 이 되는 프레임)과 **최대 상승 높이**(GraspGenX 기준 0.05 m)로
읽는다. v13~v17 의 dxy 들도 같은 오염 가능성이 있으므로, 사출이라고 적은 건들은 힘 트레이스가
있는 v17 외에는 재확인 대상이다.

## kp 40 은 19.1 cm 를 들어올렸고, 스파이크 하나에 놓쳤다 (260928 22:15, hammer v19)

v19 는 손 stiffness 만 10 -> 40 으로 올렸다(kd 0.4, effort 30, armature 0.001 유지).
**v18 의 실패 원인은 실제로 해결됐다.** 물체가 z 0.1483 -> **0.3397** 까지 올라가
f700~f770 동안 그 높이를 유지했다. 최대 상승 **+0.1914 m** 로, GraspGenX 의 성공 기준
`end2end/clutter_task.py:64 LIFT_SUCCESS_DZ = 0.05` 의 3.8배다. v18 은 +0.0387 m 에서
f660 에 놓았다. 하중에 밀려 손가락이 back-drive 되던 문제는 kp 40 에서 사라졌다.

그러나 같은 실행이 v17 의 사출을 되살렸다.

| | v18 (kp 10 / kd 0.2) | v19 (kp 40 / kd 0.4) |
|---|---|---|
| 접촉 초기 f500~550 | 2.4 → 357 → 469 N 단조 상승 | **0.29 ↔ 1,571.78 N 프레임마다 왕복** |
| 유지 구간 | 468~494 N 평탄 | 820~1,110 N + 스파이크 |
| 스파이크 | 없음 | f660 5,726 / f690 6,051 / **f760 24,588 N** |
| 최대 상승 | +0.0387 m | **+0.1914 m** |
| 결말 | f660 해제(놓음) | f760 사출 |

접촉 초기의 0 ↔ 1,572 N 왕복은 힘의 크기가 아니라 **PD 의 진동**이다. 원인이 되는 수는
감쇠 대 강성의 비율이고, 오픈소스 두 곳에 값이 있다.

- GraspGenX `end2end/dynamic_playback.py:70-71` — `FINGER_KP_DEFAULT = 2000.0`,
  `FINGER_KD_DEFAULT = 200.0` → **kd/kp = 0.1**. 같은 파일 `MIMIC_KP = 50.0 / MIMIC_KD = 10.0`
  → 0.2. `end2end/robot_profiles.py` 의 `UR10eInspireHandProfile` 은 `finger_kp`/`finger_kd`
  를 오버라이드하지 않는다(grep 0건) — **같은 Inspire 손이 원본에서 2000/200/effort 200 으로 돈다.**
- IsaacLab `isaaclab_assets/robots/unitree.py:598-611` — stiffness 10 / damping 0.2 → **0.02**.

v19 가 쓴 비율은 `0.4/40 = 0.01` 로 둘 중 어느 쪽보다도 작았다. 이유는 이 저장소의 규칙
`build_reach_reference.py:99-102` 의 `damping = 0.2*(kp/10)**0.5` 가 kd 를 sqrt(kp) 로만
올려 **kd/kp 가 1/sqrt(kp) 로 감소**하기 때문이다(kp 10 → 0.02, kp 40 → 0.01,
kp 650 → 0.0025). v17 이 kp 650 에서 55,478 N 을 찍은 것도 같은 방향이다.

v20 은 kp 40 을 유지한 채 **kd 만 0.4 → 4.0** (GraspGenX 비율 0.1)으로 간다. 두 출처의
비율이 다르므로(0.1 vs 0.02), 측정된 실패가 링잉인 점을 근거로 감쇠가 센 쪽을 먼저 쓴다.
4.0 이 과감쇠로 드러나면(접촉력이 늦게 올라오거나 정지 각도가 0.86 rad 보다 작아지면)
다음은 NVIDIA 비율의 0.8 이다. 이 선택은 측정으로 판별할 예정이며, 아직 근거 없는 추정이다.

`[eval]` 의 `dxy 1.4562 m` 는 또 발에 차인 값이다: f790~f870 물체는 속도 0 으로 정지,
f880 `right_ankle_pitch_link` 10.9 cm, f900 물체가 `[1.0336 -0.2116 0.2041]`. v18 과 같은
패턴이 반복됐으므로, 판정은 계속 해제 프레임과 최대 상승으로 읽는다.

## 게인은 v20 에서 맞았다. 이제 문제는 손끝으로 잡는다는 것 (260928 22:35, hammer v20)

v20 = kp 40 / kd 4.0 / effort 30 / armature 0.001. v19 에서 감쇠만 GraspGenX 원본
비율(`end2end/dynamic_playback.py:70-71` FINGER_KD 200 / FINGER_KP 2000 = 0.1)로 올린 것.

사전에 적어둔 4개 기준 중 3개 통과:
- 접촉 초기 링잉 사라짐. f520~f590 이 991~1005 N 평탄. v19 는 같은 구간에서
  30.13 / 1571.78 / 0.29 / 1122.53 N 으로 플랩했다.
- 2,000 N 초과 스파이크 0회. 전 구간 최대 1,289.58 N. v19 는 5,726 / 6,051 / 24,588 N.
- 상승 +0.0933 m (z 0.1485 -> 0.2418). GraspGenX `clutter_task.py:64` LIFT_SUCCESS_DZ
  0.05 의 1.9 배. `dexsuite_good` 은 f510~f690 180 프레임 연속 True.

들어올리는 동안 미끄러지지도 않았다. palm 이 10 프레임당 1.2 cm 오르는 90 프레임 내내
(물체 z - palm z) 가 -0.031 ~ -0.036 m 로 고정이다. 강체처럼 따라왔다.

그런데 f690 -> f700 한 스텝(0.33 s)에 그냥 떨어졌다. 사출이 아니다. 직전 힘은 695 N,
스파이크는 없었다. **더 이상 게인 문제가 아니다.**

측정된 원인은 파지 위치다. `[abs ] f690` 에서 object x -0.244, palm x -0.139 —
손바닥 원점에서 10.5 cm 바깥이다. 손가락 각도가 순서를 그대로 보여준다(target 전부 1.47):
j3(pinky) 는 f550 에 이미 1.47 로 완전히 닫혀 물체를 한 번도 건드리지 않았고,
j2(ring) 는 0.99 -> 1.04 -> 1.08 -> 1.22 -> 1.39 -> 1.47 (f550~f680) 로 자루를 놓치며
닫혔다. j0/j1(index/middle) 만 0.96~1.00 에서 계속 정지 = 접촉 유지.
엄지 접촉 링크는 R_thumb_intermediate(546.94 N, f640) -> R_thumb_distal(341.06 N, f670)
로 이동했다. 자루가 말단으로 굴러 나간 것이다. `evidence/head_f690.png` 에 자루가
말단 지골을 가로지르고 망치 머리가 손 바깥에 매달린 장면이 찍혀 있다.

`[eval] dxy 0.5713 m` 은 v18/v19 와 똑같이 발에 차인 값이다. 물체는 f720~f880 동안
속도 0 으로 정지해 있다가 f890 에 vel -1.34 m/s 로 날아간다.

### 다음 하나: 손끝 콜라이더 (v21)

GraspGenX `end2end/robot_profiles.py` UR10eInspireHandProfile:

    coacd_link_keywords: Tuple[str, ...] = ("intermediate", "distal")
    # Only the fingertips need CoACD ... The thumb tip / distal and the
    # *_intermediate links have the concavity that matters for object contact.

우리 asset `assets/g1_inspire/g1_29dof_inspire_hand.usd` 은 손가락 콜라이더 50 개가
전부 `physics:approximation = convexHull` 이다. 손가락 안쪽 오목한 굽이가 물리에서
메워진다. v20 에서 실제로 힘을 받은 링크 6 개(R_thumb_intermediate, R_thumb_distal,
R_index/middle/ring/pinky_intermediate)가 GraspGenX 가 분해하라고 지정한 바로 그 6 개다.
원통형 자루가 홈에 앉지 못하고 볼록면 위에 얹혀 말단으로 굴러 나갔다는 측정과 맞는다.
[[object-collider-was-convex-hull]] 에서 망치 자루 자체가 같은 이유로 물리에 없었던 것과
같은 계열의 문제다.

기록: `FINGER_COACD` 는 20:58 `results/fable42` 에서 한 번 켜진 적이 있으나 그때 게인은
kp 650 / kd 100 / effort 200 / armature 0.5 로 사출이 나 졌다(dxy 1.4693). 잡히는 게인
위에서 콜라이더 효과를 본 적은 아직 없다.

v21 = v20 게인 + `FINGER_COACD=intermediate,distal`. 바뀌는 것은 콜라이더 근사 하나.
검증: (a) j2(ring) 가 상승 구간에서 1.47 에 닿지 않는다 (b) (물체 z - palm z) 오프셋이
f700 이후에도 유지된다 (c) f760 에 시작 높이 +0.05 m 이상 (d) 스파이크 0회.

## 파워 그립 후보는 v19 게인으로 버려졌다 (260928 22:45, v21 렌더 중 조사)

`docs/DIAGNOSIS.md:765-786` 에 페이블이 정리해 둔 대로, NVIDIA 가 배포하는
`gripper_descriptions/assets/x_grippers/inspire_hand/config.json` 의 sweep volume 은
손끝에만 달려 있다:

    "fingertip": [0.0, 0.0, 0.15],
    "sweep_volume": { "offset": [0,0,0.135], "extents": [0.08,0.045,0.04],
                      "offset2":[0,0,0.118], "extents2":[0.04,0.045,0.035] }

즉 물체가 놓일 자리를 hand_base_link 기준 z 0.10~0.155 m 로 지정한다(너클은 8.5 cm).
v20 에서 측정한 손바닥-물체 거리 10.5 cm 는 이 사양 안이다. **손끝 파지는 버그가 아니라
NVIDIA 기술서가 요청한 결과다.** 그래서 v20 의 낙하는 파지 생성 쪽을 봐야 하는 문제다.

페이블이 만든 로컬 변형 `inspire_hand_palm`(같은 URDF, sweep box 를 손바닥면~손끝 전체로:
open 5x7.7x10 cm @ z 0.10, half-open 4x7.7x7 cm @ z 0.095)로 파워 그립 후보가 이미
생성돼 디스크에 있다: `results/fable40/grasps_palm.json` (14:58),
`order_palm.txt` / `reach_palm.npz` (15:15), `test_palm.txt` (15:40, 40 후보).

전부 LOST 다. 그런데 어떤 게인으로 졌는지가 문제다. `palm_chain.sh` 는

    HAND_KP=40 OBJECT_NO_SLEEP=1 RISE_SLOW=2 ...

로 돌았고 `HAND_KD` 를 주지 않았다. 그러면 `build_reach_reference.py:99-102` 의
`damping = 0.2*(kp/10)**0.5` 가 적용돼 kd = 0.4 — **v19 와 정확히 같은 kp 40 / kd 0.4** 다.
그 조합은 접촉 초기 0 <-> 1,571.78 N 링잉과 5,726 / 6,051 / 24,588 N 스파이크를
내는 것으로 v19 에서 측정됐다.

또 40 후보 중 28 개는 `stand-up test: object followed +0.000 m` 다. 미끄러진 게 아니라
물체를 건드리지도 못했다. 실제로 움직인 것은 12 개뿐이다.

따라서 "파워 그립은 안 된다" 는 결론은 아직 근거가 없다. 검증된 게인
(kp 40 / kd 4.0, GraspGenX 비율 0.1)으로 다시 돌린 적이 한 번도 없다.

v21(손끝 콜라이더)이 잡지 못하면 v22 는 이것이다: `grasps_palm.json` / `reach_palm.npz`
를 v20 게인으로 재평가. 관련: [[crate-power-grip-later]].

## 엄지 말단 미믹 배수가 실물 URDF 의 3.6 배다 (260928 22:50, 측정)

GraspGenX 가 쓰는 실물 Inspire URDF
(`ext/gripper_descriptions/.../x_grippers/inspire_hand/gripper.urdf`):

    thumb_intermediate_joint  mimic thumb_proximal_pitch_joint  multiplier 1.334  limit 0 ~ 0.8
    thumb_distal_joint        mimic thumb_proximal_pitch_joint  multiplier 0.667  limit 0 ~ 0.4

우리 `build_reach_reference.py:109-110`:

    (thumb_intermediate, thumb_proximal_pitch, 1.6)
    (thumb_distal,       thumb_proximal_pitch, 2.4)

로그의 target 값으로 실제 적용을 확인했다. v20 f710: j5(pitch) 0.24 -> j10 target 0.38
(= 1.6 x 0.24), j11 target 0.56 (= 2.4 x 0.235).

우리 asset 의 관절 한계도 실물과 다르다 (`g1_29dof_inspire_hand.usd`, USD 는 도 단위):

    R_thumb_proximal_pitch  0 ~ 28.648 deg = 0 ~ 0.500 rad   (URDF 0 ~ 0.6)
    R_thumb_intermediate   -9.167 ~ 55.004 = -0.160 ~ 0.960  (URDF 0 ~ 0.8)
    R_thumb_distal        -13.751 ~ 82.506 = -0.240 ~ 1.440  (URDF 0 ~ 0.4)

배수 1.6 / 2.4 는 우리 asset 의 한계(0.96 / 1.44)에 비례해 뽑힌 값이라 그 자체로 틀린
설정은 아니다. 다만 실물 손의 비율은 1.334 / 0.667 이고, 우리 엄지 말단은 실물보다
3.6 배 더 말린다(한계 1.44 rad vs 0.4 rad).

v20 에서 엄지 접촉이 R_thumb_intermediate -> R_thumb_distal 로 옮겨간 것과 방향이 맞는다.
말단이 과하게 말리면 물체를 손아귀 안쪽이 아니라 바깥쪽으로 밀어낸다. 다만 이것이
원인이라는 증거는 아직 없다 — 접촉 링크 이동은 물체가 굴러 나간 결과일 수도 있다.

v21 이 실패하면 v22 후보: `_MIMIC` 배수를 URDF 값 1.334 / 0.667 로. 페이블 파일은 건드리지
않고 `play_in_cell_opus.py` 에서 `build_reach_reference._MIMIC` 을 import 후 치환한다.

## 엄지가 한 번도 굽지 않았다 (hammer v21, 측정)

v21(kp 40 / kd 4.0 / effort 30 / armature 0.001 + CoACD 손끝 6 메시)은 v20 보다 모든 축에서
나았다 — 최고 상승 +0.0933 → +0.1193 m, 유지 180 → 220 프레임, 스파이크 여전히 0 회 —
그러나 같은 방식으로 놓쳤다. `[eval] dxy 0.0211 m dz -0.0090 m -> LOST`(발에 차이지 않은 깨끗한 낙하).

관절각이 원인을 말한다. `R_thumb_proximal_pitch_joint` 는 목표 0.5 rad 에 대해
q 0.03(f510) → -0.00(f530) → 0.00(f750) 으로 **한 번도 굽지 않았다**. 오차 0.5 rad × kp 40 = 20 Nm 를
내내 걸고 있는데 움직이지 않는다 = 네 손가락이 자루를 통해 엄지를 역구동하고 있다는 뜻이다.
엄지 중간/말단도 -0.16 근처에 머물렀다. 엄지는 펴진 기둥이었고, 파지는 네 손가락이 자루를 그 기둥에
눌러붙인 형태였다.

접촉력은 튀지 않고 빠졌다: 엄지 828.78(f520) → 519.79(f700) → 315.88(f740) → 0.00 N(f760), 단조 감소.
그 사이 손가락이 하나씩 접촉을 놓쳤다 — 새끼 j3 1.14(f610) → 1.46(f630), 약지 j2 1.02(f690) → 1.47(f750).
목표 1.47 에 닿았다는 것은 그 손가락이 더는 물체에 눌리지 않는다는 뜻이다. 남은 접촉이 줄다가
일어서기 가속(f760)에서 빠졌다. dexsuite_good 은 f520~f740 내내 True 였다 — 이 기준은 놓치기
직전까지 참이므로, 놓침을 예고하지 못한다.

## 파워 그립 후보는 v19 조건에서 버려졌다 (Fable 테스터가 게인 변수를 읽지 않는다)

`results/fable40/test_palm.txt`(15:40, 0/40 held)가 팜 표기 후보들에 대한 유일한 판정이다.
그 판정을 낸 `grasp/test_grasps_in_isaac.py` 에는 `HAND_KD`, `SOLVER_IT`, `FINGER_COACD` 가
하나도 없다(확인: grep 결과 전무). 손 게인은 `build_reach_reference.robot_cfg()` 의
`damping = 0.2*(kp/10)**0.5` 로 고정되므로 `HAND_KP=40` 은 kd 0.4 를 낳는다 — 비율 0.01,
v19 에서 약지 접촉력이 0↔1,571.78 N 로 진동하고 24,588 N 스파이크를 낸 바로 그 조건이다.
솔버도 12/4 로 고정되어 있는데, GraspGenX 는 `dynamic_playback.py:91-98` 에서 100/50 을 쓰며
"10 회로는 제약 솔버가 수렴하지 않아 들어올리는 구간에서 파지가 미끄러진다"고 적어 두었다.

따라서 "파워 그립은 안 된다"는 결론에는 아직 근거가 없다. 40 개 중 28 개는
`object followed +0.000 m` = 접촉조차 없었다.

v22 = 사본 `grasp/test_grasps_in_isaac_opus.py`(게인 오버라이드 + 솔버 100/50 + CoACD 손끝, 세 곳만 추가)
으로 같은 40 개를 kp 40 / kd 4.0 / effort 30 / armature 0.001 에서 재판정하고, 가장 많이 들어올린
후보를 carry+place 까지 렌더한다.

## 엄지가 막힌 곳: 실물에 없는 뒤로 젖히는 범위

v21 에서 `R_thumb_intermediate_joint` 는 f530 부터 f750 까지 **-0.16 rad** 에 있었다. 이 값은
우리 자산 `assets/g1_inspire/g1_29dof_inspire_hand.usd` 의 이 관절 하한(-9.167 deg = -0.160 rad)과
정확히 같다 — 즉 한계에 박혀 있었다. 같은 구간에 `R_thumb_proximal_pitch_joint` 는 목표 0.5 rad 에
대해 0.00 이었다. 자산은 이 둘을 PhysX 미믹(gearing -1.6)으로 묶고 `stiffen_mimic` 이 200 Hz 로
굳히므로, 중간마디가 하한에 박히면 근위 피치도 0 에 묶인다.

제조사 서술에는 그 음의 범위가 없다. GraspGenX `ext/gripper_descriptions/.../x_grippers/inspire_hand/gripper.urdf`:

| 관절 | 실물 URDF | 우리 USD |
|---|---|---|
| `thumb_proximal_pitch_joint` | 0 ~ 0.6 | 0 ~ 0.500 |
| `thumb_intermediate_joint` | **0** ~ 0.8 (mimic x1.334) | **-0.160** ~ 0.960 (mimic x1.6) |
| `thumb_distal_joint` | **0** ~ 0.4 (mimic x0.667) | **-0.240** ~ 1.440 (mimic x2.4) |

네 손가락은 이 문제가 없다. 실물 `index_intermediate` 하한 -0.045 대 우리 -0.340 으로 우리가 더
넉넉하지만, v21 에서 이들은 0.58~0.84 로 양수에 머물렀다 — 막힌 것은 엄지뿐이다.

v23 = `THUMB_LIMIT_URDF=1`: 엄지 중간/말단의 **하한만** URDF 의 0 으로 올린다(상한·기어비·게인은
v21 그대로). IsaacLab 런타임 `write_joint_position_limit_to_sim` 으로 `sim.reset()` 직후에 쓴다 —
USD 스테이지 편집은 인스턴스 프록시가 읽기 전용이라 조용히 실패할 수 있다.
사전 등록 합격 기준: (a) 유지 구간에 pitch q > 0.1 rad (v21 최대 0.07) (b) intermediate q >= 0 내내
(c) 상승 구간에 근위가 1.47 에 닿는 손가락 없음 (d) 최고 상승 >= +0.1193 m.

아직 원인이라는 증명은 아니다. 중간마디를 -0.16 에 붙들고 있던 힘이 무엇인지는 측정하지 않았다
(구동 토크는 오차 0.16 rad x kp 40 = 6.4 Nm 로 0 쪽으로 밀고 있었는데도 움직이지 않았다).

## 미믹 종동 관절을 주동과 같은 세기로 구동하고 있다 (NVIDIA 는 1:40)

GraspGenX `end2end/dynamic_playback.py`:

```
 70  FINGER_KP_DEFAULT = 2000.0      79  MIMIC_KP = 50.0
 71  FINGER_KD_DEFAULT = 200.0       80  MIMIC_KD = 10.0
664  builder.joint_target_ke[q_off] = finger_kp    # 주동(gripper_drives)
673  builder.joint_target_ke[q_off] = MIMIC_KP     # 종동(mimic_drives)
```

즉 종동은 주동의 **1/40** 세기로만 구동되고, 실제 결합은 `add_constraint_mimic` 의 강체 제약이 맡는다
(575-580 줄 주석: 제약이 없으면 "한쪽은 접촉에 멈추고 다른 쪽은 빈 공간을 지나 닫힌다").

우리는 `build_reach_reference.robot_cfg()` 에서 주동과 종동을 한 액추에이터 그룹에 넣는다:

```
expr = [".*_proximal_joint", ".*_thumb_proximal_(yaw|pitch)_joint"]
if SOFT_MIMIC: expr += [".*_intermediate_joint", ".*_thumb_distal_joint"]
```

하나의 kp 로 전부 구동되므로 비율이 1:1 이다. 네 손가락은 기어비가 1.0 이라 주동과 종동의 목표가
같아 문제가 드러나지 않고, 기어비가 1.6 / 2.4 인 엄지에서만 드러난다 — v21 에서 막힌 것이 엄지뿐인 것과
일치한다. 다만 이것이 중간마디를 -0.16 에 붙들어 둔 힘이라는 측정은 아직 없다.

순서: v23 이 하한 정정(측정된 사실에 직접 대응)을 먼저 시험하고, 실패하면 v24 에서 종동을 별도
액추에이터 그룹으로 분리해 NVIDIA 의 1:40 비율을 준다.

## v23 실험 결과: 하한 정정은 원인이 아니었다 (디스크 폴더 hammer/v22)

폴더 이름 주의. 체인의 `next_v()` 가 렌더 시각에 폴더를 잡으므로, 위에서 "v23" 이라 부른 하한
정정 실험은 디스크에 `영상보관/.../5지/hammer/v22` 로 떨어졌다. 아래 번호는 디스크 기준이다.

한계 변경은 실제로 걸렸다 (`results/fable40/g.log`):

```
[hand] R_thumb_intermediate_joint limits -0.160 .. 0.960 rad -> 0.000 .. 0.960
[hand] R_thumb_distal_joint       limits -0.240 .. 1.440 rad -> 0.000 .. 1.440
```

그런데 더 나빠졌다. `[eval] dxy 0.0892 m  dz -0.2497 m -> LOST` (v21 은 dz -0.0090).
사전 등록 기준 4개 중 통과 0개다.

측정된 엄지 관절각 (`right finger q`, 목표는 pitch 0.5 rad):

```
frame   480   520   560   600   640   680   720   760  |  800
pitch -0.00 -0.00 -0.00  0.00 -0.00  0.02 -0.00  0.00  |  0.49
inter -0.00 -0.00 -0.00  0.00 -0.00  0.00 -0.00 -0.00  |  0.79
dist  -0.00 -0.00 -0.00  0.01 -0.00  0.06  0.00  0.00  |  1.18
```

**마스터가 한 번도 움직이지 않았다.** 그러므로 v21 에서 중간마디를 -0.160 에 붙들고 있던 것은
관절 한계가 아니다 — 한계를 0 으로 열어 주었는데도 사슬 전체가 0 에 그대로 머물렀고, 박히는
자리만 -0.160 에서 0 으로 옮겨갔을 뿐이다. 위에 적어 둔 "하한 정정" 가설은 기각한다.

f800 이 결정적이다. 물체가 f780 에 빠져나간 **직후** 세 관절이 전부 목표까지 간다(0.49 / 0.79 / 1.18).
즉 사슬을 붙들고 있던 것은 물체와의 접촉이고, 접촉이 걸린 링크는 전 구간에서 엄지 패드가 아니라
종동 링크였다:

```
f520 sum  1409 N  max  628 N  R_thumb_intermediate
f600 sum  1448 N  max  703 N  R_thumb_intermediate
f740 sum  1009 N  max  514 N  R_thumb_intermediate
f760 sum 30779 N  max 23366 N R_thumb_distal        <- v21 최대 1855 N
f780 sum     0 N  물체 이탈
```

종동 관절이 마스터와 같은 kp 40 으로 "기어비 x 마스터 q = 0" 을 500~700 N 으로 지켜내면, 엄지는
굽는 사슬이 아니라 곧은 기둥이 된다. 그 기둥을 pitch 로 자루에 밀어 넣으려다 기하학적으로 막힌다.
v21 에서는 기둥이 -0.160 에 눌러앉아 있었고, v22 에서 그 자리를 없애자 두 제어기(PhysX mimic 구속과
kp 40 위치 드라이브)가 경계에서 충돌해 23 kN 이 터졌다. 머리뷰 f780 프레임에서 손가락이 곧게 펴진
채 망치가 바닥에 놓인 것이 보인다 (`hammer/v22/evidence/head_f780.png`).

`dexsuite_good` 은 f760 까지 True 였다. NVIDIA DexSuite 접촉 기준이 낙하를 예측하지 못한 것이
v19~v22 네 번 연속이다.

## 마찰은 한 번도 한계가 아니었다 (마찰 계열 노브 전부 제외)

`play_in_cell_opus.py:441` 의 손 재질은 `friction_combine_mode="max"`, 물체 재질
(`plan_scene.py:116`)은 조합 모드를 설정하지 않아 기본 `"average"` 다. PhysX 조합 우선순위는
average < min < multiply < **max** 이므로 max 가 이겨 접촉 마찰계수는 `max(10, 3) = 10` 이다.
0.2 kg 물체를 들기 위한 접선력은 ~2 N 인데 측정된 수직력은 300~800 N 이었다. 마찰 용량은 필요량의
수천 배였다. `FINGER_MU`/`OBJECT_MU`/조합 모드는 더 이상 후보가 아니다.

참고로 IsaacLab `RigidBodyMaterialCfg` 에는 torsional/rolling 마찰 필드가 없다. NVIDIA 가
`dynamic_playback.py:186-197` 에서 쓰는 `condim 6` / `mu_torsional` / `mu_rolling` 은 Newton 쪽
개념이고 PhysX 재질에 대응물이 없으므로, 그 오버라이드를 그대로 옮길 수는 없다.

## 기어비가 제조사 URDF 와 다르다 (부호가 아니라 크기)

`gripper_descriptions/assets/x_grippers/inspire_hand/gripper_spherical_dof.urdf` 의 mimic 태그와
우리 `build_reach_reference.py:105-109` 를 맞대면:

| 종동 관절 | 제조사 URDF | 우리 에셋 |
|---|---|---|
| thumb_intermediate ← thumb_proximal_pitch | x1.334 | x1.6 |
| thumb_distal ← thumb_proximal_pitch | **x0.667** | **x2.4 (3.6배)** |
| 네 손가락 intermediate ← proximal | x1.06399, offset -0.04545 | x1.0, offset 없음 |

부호는 문제가 아니다. `build_reach_reference.py:107` 의 주석 "(physxMimicJoint:rotZ, gearing
negated)" 는 저장된 값이 이미 양수임을 뜻한다. 크기만 다르다. 다만 v22 에서 마스터가 0 을 떠나지
못한 이상 기어비는 아직 발현되지도 않았으므로, 이것을 지금 고치는 것은 순서가 아니다.

## v24 = 종동 게인만 NVIDIA 비율로 (디스크 폴더는 hammer/v23 이 된다)

`grasp/play_in_cell_opus.py` 에 `MIMIC_SPLIT` 을 넣었다. `HAND=inspire` 이고 `MIMIC_SPLIT=1` 일 때
`.*_intermediate_joint` / `.*_thumb_distal_joint` 를 `hands` 그룹에서 떼어 `hands_mimic` 그룹으로
옮기고, 게인만 GraspGenX 비율로 내린다:

```
종동 kp = 40 x (MIMIC_KP 50 / FINGER_KP 2000) = 40 x 0.025 = 1.0
종동 kd = 4.0 x (MIMIC_KD 10 / FINGER_KD 200) = 4.0 x 0.05  = 0.2
```

`soft_mimic()` 의 서브스텝 목표값 재작성(NVIDIA `_set_joint_targets` 에 해당)과 기어비는 건드리지
않는다. `THUMB_LIMIT_URDF` 는 되돌려 단일 변수로 만든다. 나머지는 v21 과 동일
(kp 40 / kd 4.0 / effort 30 / armature 0.001, solver 100/50, CoACD intermediate+distal).

사전 등록 합격 기준: (a) f520~f760 에 thumb_proximal_pitch q > 0.1 rad (v21 0.07, v22 0.02)
(b) 같은 구간에 thumb_intermediate q > 0.1 rad — 종동이 마스터를 따라 굽는다
(c) 최대 접촉력 < 2000 N (v21 1855, v22 23366) (d) 최고 상승 >= +0.1193 m 이고 dz > -0.05 m.

## 손바닥 접근 후보 재선별 결과: 40개 중 진짜 성공 0개

`test_grasps_in_isaac_opus.py --top 40 --order order_palm.txt` (solver 100/50, kp 40 / kd 4.0):

```
[test] 1/40 grasps held: #46 (589067 mm)
[test] grasp #46 conf 0.994 cuRobo 6.7 mm  box at close +0.036  after lift dz +589.067 m -> HELD
```

`dz +589 m` 는 수치 폭발이지 파지가 아니다. 진짜 유지는 0/40 이고, v19 조건에서 기록한 0/40 과
같다. 손바닥 접근으로 후보를 갈아 끼워도 결과가 같다는 것은, 지금의 병목이 파지 후보 선택이
아니라 손이 닫히는 방식에 있다는 뜻이다 — v24 의 우선순위를 뒷받침한다.

### 기록: 렌더 충돌 (내 실수)

23:37:59 에 v24 체인을 띄울 때, palm_v22 체인이 테스터가 끝난 직후 23:37:00 에 자기 렌더 단계를
이어 돌리고 있다는 것을 확인하지 않았다. 두 렌더가 같은 `results/fable40/g.log` 와 `g.mp4` 를
`>` 로 열어 1분 차이로 겹쳤다. 두 체인을 pid 로 모두 종료하고(980166/979742/904321 과
982806/982784, 전부 사망 확인), 오염된 산출물을 지우고 v24 만 다시 띄웠다. 영상 폴더는 `next_v()`
가 렌더 종료 후에 호출되므로 아무 폴더도 잡히지 않았다.

palm_v22 의 렌더는 다시 띄우지 않는다. 그 체인이 고른 후보 #46 은 `dz +589 m` 로 판정된
수치 폭발이므로, 렌더해도 보여 줄 것이 없다. 선별 결과(진짜 0/40)는 이미 위에 기록했다.

## PhysX mimic 부호 규약 확정 — 부호 가설 기각

NVIDIA 자체 테스트에서 읽었다. `omni.physx.tests` 의 `PhysxMimicJointAPI.py`,
`test_prismatic_simple` / `test_revolute_simple` 는 `gearing = 1.0, offset = 0.0` 으로
mimic 을 걸고 기준 관절에 stiffness 1.0e10 드라이브로 목표를 주입한 뒤 이렇게 검사한다:

```
self.assertAlmostEqual(linkAPos[axis], -linkBPos[axis], delta=posErrTolerance)
self.assertAlmostEqual(linkAAngleDegree, -linkBAngleDegree, delta=posErrTolerance)
```

`_create_simple_prismatic_setup` 은 두 관절을 같은 축·같은 부모(rootLink)에 두고
`localPos0` 를 각 링크의 정지 위치(−1, 0, 0)/(+1, 0, 0)로 잡는다. `_get_joint_pos` 는
`q = (링크 현재 위치 + localPos1) − (부모 위치 + localPos0)` 이므로 `q_A = −0.1` 일 때
어서션은 `q_B = +0.1` 을 요구한다. 세 축 모두 같은 결론이다:

> **q_follower = −gearing × q_reference − offset**

따라서 우리 에셋의 `−1.0 / −1.6 / −2.4` 는 종동을 **+1.0 / +1.6 / +2.4 배로** 끌고,
이는 `soft_mimic` 의 양수 표, 제조사 URDF 의 양수 배율과 **같은 방향**이다.
"USD 구속이 종동을 거꾸로 끌어 음의 하한에 박는다" 는 가설은 기각한다.
틀린 것은 부호가 아니라 크기다.

## MIMIC_SPLIT 은 물리에 닿지 않았다 (디스크 폴더 hammer/v23)

패치는 제대로 잡혔다 (로그 51행): masters kp 40.0 kd 4.0, followers
`['.*_intermediate_joint', '.*_thumb_distal_joint']` kp 1.0 kd 0.2. 그런데 v21 과
v23 의 `[force] frame` 라인을 전부 diff 해도 한 줄도 다르지 않고 관절값·최고
상승·`[eval]` 이 모두 일치한다. **종동 kp 를 40배 내려도 측정 가능한 변화가 없다** =
종동 PD 토크는 파지를 제한하는 요인이 아니다. 왜 완전히 동일한지는 아직 설명하지
못했다. `play_in_cell_opus.py` 245–345 의 이후 액추에이터 블록을 모두 읽었고 전부
`"arms"/"legs"/"feet"` 만 건드리므로 `hands_mimic` 를 덮어쓰는 코드는 없다.

## v23 측정: 손가락은 작동하고 엄지 사슬만 접혀 막힌다

`[eval] dxy 0.0211 m dz −0.0090 m → LOST`. 최고 상승 **+0.1202 m (f720)**,
접촉이 0 N 이 되는 첫 프레임 **f760**.

| | 목표 | f520 | f560 | f640 | f720 | f760 | f800(이탈 후) |
|---|---|---|---|---|---|---|---|
| thumb_proximal_pitch | 0.5 | 0.00 | 0.00 | −0.00 | 0.00 | 0.03 | **0.50** |
| thumb_intermediate | — | −0.15 | −0.16 | −0.16 | −0.13 | −0.07 | **0.80** |
| thumb_distal | — | −0.17 | −0.18 | −0.01 | 0.05 | −0.03 | **1.20** |
| 네 손가락 proximal | 1.47 | 0.96~1.03 | 0.95~1.09 | 0.96~1.47 | 0.97~1.47 | 1.02~1.47 | 1.47 |
| 네 손가락 intermediate | (마스터 추종) | 0.59~0.88 | 0.61~0.84 | 0.59~1.47 | 0.59~1.47 | 0.82~1.47 | 1.47 |

엄지 마스터는 목표 0.5 에 대해 **f520~f760 내내 0.000** 이었고, 해머가 떠난 다음
프레임 f800 에 즉시 목표까지 갔다 — 게인 문제가 아니라 막혀 있었다. f480 시점,
네 손가락이 아직 0.38 rad 일 때 이미 `intermediate −0.16`, `distal −0.24`(둘 다 우리
USD 하한)였다. **엄지 사슬만** 음수로 접힌다(네 손가락 intermediate 는 0.59~0.88 로 정상).
엄지 yaw 도 목표 1.31 에서 1.17 까지 0.14 rad 밀려났다.

접촉 지탱 링크 (f440~f760, 프레임 수 / 최대 N):

```
R_index_intermediate   26 / 465.27      R_thumb_intermediate  23 / 790.16
R_middle_intermediate  25 / 238.34      R_ring_intermediate   20 / 233.95
R_pinky_intermediate    9 / 334.93      R_thumb_proximal       4 / 828.78
R_thumb_distal          5 /  29.57
```

엄지 패드(distal)는 5 프레임 29.57 N 뿐이다. `evidence/head_f720.png` 에서도 손잡이는
네 손가락이 감고 있고 엄지는 그 옆에 곧게 누워 대립하지 않는다.

DexSuite 접촉 기준은 f750 까지 `dexsuite_good=True` 였다 (sum 638.02 N, thumb 304.44 N,
best-opposing 234.95 N). v19~v23 **다섯 번 연속** 낙하를 예측하지 못했다.

## v25 = 엄지 사절링크 비율을 제조사 URDF 로 (디스크 폴더는 hammer/v24 가 된다)

`gripper_descriptions/x_grippers/inspire_hand/gripper_spherical_dof.urdf` 원본:

| 관절 | URDF mult | URDF limit | 우리 USD gearing | 우리 닫힘 명령 |
|---|---|---|---|---|
| thumb_intermediate_joint | **1.334** | 0 .. 0.8 | 1.6 | 0.8 |
| thumb_distal_joint | **0.667** | 0 .. **0.4** | **2.4** | **1.2** |
| finger intermediate | 1.06399 / off −0.04545 | −0.04545 .. 1.56 | 1.0 | 측정 proximal |
| thumb_proximal_pitch (마스터) | — | 0 .. 0.6 | — | 0.5 |

0.5 rad 닫힘이 `thumb_distal` 에 **1.2 rad** 를 명령한다 — 실제 관절 전체 가동범위
0.4 rad 의 3 배이고, URDF 배율의 3.6 배다. `MIMIC_URDF_RATIO=1` 이
`play_in_cell_opus.py` 에서 (1) `build_reach_reference._MIMIC` 의 엄지 두 항목을
1.334 / 0.667 로 바꾸고 (2) USD 의 해당 `physxMimicJoint:*:gearing` 을
−1.334 / −0.667 로 다시 쓴다. Fable 파일은 런타임 몽키패치로만 건드린다.

**한계를 분명히 적는다:** 유지 구간에는 마스터가 0.000 이었으므로 두 배율 모두 종동
목표를 0 으로 만든다. 이 배율 오차가 유지 구간의 직접 원인이라는 증거는 없다.
영향을 주는 곳은 닫기 구간(f440~f480, 마스터 0 → 0.07)과 USD mimic 구속이다.

**다음 후보 (아직 실행 안 함):** 엄지 사슬의 음수 travel 을 URDF 대로 없애기.
URDF 는 `thumb_intermediate 0 .. 0.8`, `thumb_distal 0 .. 0.4` 로 뒤로 접히는 구간이
아예 없는데 우리 USD 는 −0.16 / −0.24 를 허용한다. v22 는 이 중 **intermediate 하나만**
열었고 distal 의 −0.24 는 그대로 뒀다 — 사슬이 여전히 distal 에서 접힐 수 있었으므로
v22 의 실패가 이 가설을 기각하지는 못한다. 다음에는 두 관절을 함께 0 으로 잡는다.

## 마찰 계열과 확인된 오픈소스 수치 (누적)

- 마찰은 한 번도 한계가 아니었다: 손 재질 `friction_combine_mode="max"` 가 물체의
  `"average"` 를 이겨 유효 mu = 10. 300~800 N 수직력에 0.2 kg 물체가 필요로 하는
  접선력 ~2 N. 마찰 노브는 전부 제외.
- GraspGenX `end2end/robot_profiles.py` UR10eInspireHandProfile: 엄지 yaw
  **1.308 고정(open=close)**, 엄지 pitch 0 → **0.6**, 네 손가락 0 → **1.47**.
  우리는 yaw 1.308, 손가락 1.47 로 이미 일치하고 pitch 만 0.5 (마스터 상한은 0.6).

## 손은 0.5 rad/s 로 닫히고 있었다 (v24 에서 측정, 결론)

**결론:** Inspire 손의 12 관절 전부가 PhysX 에서 **0.5 rad/s** 로 제한돼 있다.
`robot_cfg()` 가 주는 `velocity_limit=10.0` 은 시뮬레이터에 도달하지 않는다. 그래서
지금까지의 모든 파지 시도는 손이 다 닫히기 전에 일어서기 구간에 들어갔다.

측정 (`grasp/thumb_free_opus.py`, 자유공간·물체 없음·루트 고정, 20 s/설정):

- `robot.data.joint_velocity_limits` → **0.500 rad/s** (12 관절 전부).
- 실측 `joint_vel` 이 구동 6 관절에서 정확히 +0.50 에 포화. 종동절은
  +0.67 / +0.33 = 0.5 × URDF 배율 1.334 / 0.667 로 마스터의 제한된 속도를 따라간다.
- `SUB=1` 로도 같은 0.50 → 서브스텝 아티팩트가 아니다.

오픈소스가 말하는 이유 (`IsaacLab`):

| 위치 | 내용 |
|---|---|
| `actuators/actuator_pd.py:81-91` | ImplicitActuator 는 `velocity_limit` 을 **쓰지 않는다**: "we continue to not use it … please use `velocity_limit_sim` instead" → `cfg.velocity_limit = None` |
| `assets/articulation/articulation.py:1773` | PhysX 로 가는 것은 `actuator.velocity_limit_sim` 뿐 |
| `actuators/actuator_base_cfg.py:91-97`, `actuator_base.py:183` | `velocity_limit_sim=None` → **USD prim 의 값**(0.5)이 남는다 |

올릴 값도 감이 아니라 제조사 숫자다: GraspGenX
`ext/gripper_descriptions/.../x_grippers/inspire_hand/gripper_spherical_dof.urdf`
의 Inspire 12 관절 전부가 `<limit … velocity="5.0"/>` (131~422 행).

자유공간 A/B (같은 게인·같은 `MIMIC_URDF_RATIO`, `hammer/v24/evidence/free_air_vel_ab.log`):

| | 0.5 rad/s (에셋) | 5.0 rad/s (URDF) |
|---|---|---|
| thumb_proximal_pitch 0.500 도달 | 1.50 s | **0.67 s** |
| index_proximal 1.47 도달 | 2.33 s 후 1.173 — **미완** | **0.67 s** (1.467) |
| thumb_proximal_yaw 1.300 도달 | 2.83 s | **0.57 s** |
| 종동절 | 0.667 / 0.333 정확 | 0.667 / 0.333 정확, 불안정 없음 |

산수로 확인: proximal 전폐 1.47 rad 에 ≥ 2.94 s, 엄지 0.5 rad 에 ≥ 1.0 s. v24 의
파지 창은 f460~f760 = 10 s, 접촉을 밀며 닫는 데 걸린 시간은 f460~f840 = **12.7 s**.

`HAND_VEL` (내 `play_in_cell_opus.py`, `thumb_free_opus.py`)이
`velocity_limit_sim` 을 설정한다. `velocity_limit=None` 을 함께 넘겨야 한다 —
ImplicitActuator 는 두 값이 다르면 `ValueError` 를 던진다(`actuator_pd.py:96-101`).

### v24 판정과 기각한 가설

`[eval] dxy 1.1593 m  dz −0.0090 m → LOST`. 놓친 것이 아니라 **때려낸** 것이다:
f530→f540 에 첫 조임이 해머를 0.33 s 동안 z −0.117 / x −0.108 로 밀어냈고(접촉
1273 N → 0 N), f550~f770 은 바닥에서 끌었고, f770→f780 에 1.42 m 사출됐다. 그
사출의 최고점(+0.5271 m)은 상승 기준 (d) 를 **거짓 합격**시켰다.
유지 구간 접촉은 `R_thumb_proximal` 1117.51 N / `R_thumb_intermediate` 684.73 N 이고
**엄지 패드(distal)는 한 번도 닿지 않았다**. `dexsuite_good=True` 가 25 프레임
떴는데도 놓쳤다 — best-opposing 힘만으로 "충분히 쥐었다"를 판정할 수 없다.

URDF 배율 교정(v24 의 변경)은 효과가 있었다: `thumb_intermediate` 가 v23 의 하한
−0.160 고정에서 −0.160 … **+0.160** 으로 풀렸다. 다만 마스터는 최대 +0.100 (목표
0.500 의 20 %) 에 머물렀다 — 이제 그 이유가 속도 한계로 설명된다.

측정으로 기각한 가설 (추측을 사실로 쓰지 않기 위해):

1. mimic `offset` ≠ 0 → USD 덤프: 엄지 네 관절 모두 **0.0**. 기각.
2. mimic 부호가 종동절을 음으로 끈다 → 자유공간 설정 B(제약 단독): 마스터 0.500 →
   종동 **+0.656 / +0.312**. 부호는 양. 기각.
3. 1/33 서브스텝 적분 문제 → `SUB=1` 도 같은 0.50. 기각.
4. 손바닥/엄지 자기충돌 → `enabled_self_collisions=False`. 기각.
5. 엄지 하한을 URDF 대로 0 으로 잡기(이전 절의 "다음 후보") → v22 가 실제로 한
   변경이고 접촉이 23,366 N 으로 튀었다. 속도 한계가 원인으로 측정된 지금은
   **후보에서 내린다**.

USD 한계는 도(degree)로 저작돼 있다: `thumb_proximal_pitch 0..28.6479 deg = 0..0.500 rad`
(URDF 0..0.6) — 즉 우리가 마스터에 주는 0.5 는 **정확히 하드스톱**이고,
`thumb_proximal_yaw` 상한은 1.300 인데 1.308 을 명령한다.

## 닫기는 끝난 적이 없다 — 멈추는 것은 힘 상한이고, 우리는 그 20 % 만 쓴다 (v25 측정, 결론)

**v25 (HAND_VEL=5.0 만 바꾼 렌더) 판정: `[eval] dz -0.0090 m → LOST`.** 판정식은
`play_in_cell_opus.py:1633`, `dz = 끝 z − 시작 z > +0.05 m` 일 때만 HELD 다. 해머는 출발점
근처 바닥(z 0.153)에 다시 누웠다.

그런데 실패 방식이 처음으로 바뀌었다:

| | v24 (0.5 rad/s) | v25 (5.0 rad/s) |
|---|---|---|
| 상승 | 사출뿐 (f770→f780 에 1.42 m / 0.33 s) | **진짜 상승 f600→f750 z 0.148→0.268 (+0.120 m), f720~f750 평탄, x 는 −0.252→−0.232** |
| 엄지 패드(R_thumb_distal) 접촉 | 0 프레임 | **13 프레임, 최대 683.80 N @f630** |
| T.pitch / T.intermediate / T.distal 최대 | +0.100 / +0.160 / +0.070 | +0.350 / +0.440 / +0.240 |
| 접촉 합 최대 / 단일 링크 최대 | 1425.45 / 1117.51 N | 1975.51 / 880.75 N |

보행 팔 오차는 두 렌더가 동일(2853.9 mrad)이므로 보행·팔은 교란 변수가 아니다.

**놓친 순간은 f760 이고, 팔이 채간 것이 아니다.** palm z 는 0.309→0.315 (+0.006 m) 뿐인데
물체는 0.067 m 떨어지고 −x 로 0.10 m 갔다. `[near]` 6.1 → 13.3 → 21.7 cm, 접촉 737 → 0 N,
root z 0.421→0.428→0.468 (여기서 일어서기 시작). 그리고 **물체가 빠진 다음 프레임 f770 에
손이 목표로 스냅한다**: 0.350/0.440/0.240/1.260/1.430 → 0.500/0.670/0.330/1.300/1.470.
즉 닫기는 시작 후 10 초가 지나도 진행 중이었고 물체가 손을 벌리고 있었다. 737 N · mu 10 이면
0.2 kg 해머에 마찰 여력은 수천 N 이라 마찰 부족은 아니다. (조임이 손잡이를 밀어냈다는 해석은
아직 측정으로 확정되지 않았다 — 가설로 남긴다.)

**닫기를 언제 멈추는지 원본 확인 (`GraspGenX/end2end/dynamic_playback.py`):**

| 행 | 원문 요지 |
|---|---|
| 128-129 | "physics naturally limits how far the close actually goes when there's contact" — 완전 닫기를 명령하고 접촉이 멈춘다 |
| 645 | velocity 모드: "contact equilibrium stops the motion" |
| 685-690 | URDF `effort=20` 이 "caps the PD force so the fingers can't close against contact" → `finger_effort_limit` 기본 **200.0** 으로 올려 "authority to close" 를 준다 |
| 69-71 | `FINGER_KP_DEFAULT 2000.0`, `FINGER_KD_DEFAULT 200.0`, "heavy damping prevents the close from overshooting under contact" |

**명시적 종료 조건도, 촉각 판정도 없다.** 완전 닫기 명령 → effort limit 이 힘의 천장 →
접촉 평형이 정지 → `settle_frames=30` 수동 안정화 → 들어올리기 자체가 시험. v25 가 0.500 을
명령해 0.350 에 멈춘 것은 원본이 의도한 동작 그 자체다. (세훈님의 "꽉 잡았는지" 질문에 대한
오픈소스의 답: 촉각 신호가 아니라 힘 상한 + 접촉 평형 + 들어올리기 시험이다.)

**그래서 우리 문제는 "언제 멈출까"가 아니라 "왜 20 % 에서 멈추나"다.** v25 의 엄지 마스터는
0.350/0.500 에서 멈췄으므로 토크 명령은 `kp·Δq = 40 × 0.150 = 6.0 N·m`, effort limit
30 N·m 의 **20 %** 다. 우리 닫기는 힘 상한이 아니라 스프링이 먼저 소진되어 멈춘다 — 원본
설계와 반대다.

**v26 (큐에 들어감): kp 40 → 200, kd 4.0 → 20.0. effort 30 N·m 과 HAND_VEL 5.0 은 그대로.**
`200 × 0.150 = 30.0 N·m` 으로 같은 오차에서 기존 상한에 정확히 포화한다. **힘의 천장은
바뀌지 않는다** — 손이 그 천장에 도달하는지만 달라진다. kd 는 GraspGenX 의 10:1 비(2000:200)
를 유지했다. 합격 기준: (a) T.pitch ≥ 0.45 (v25 0.350) (b) **f700 이전에** 0.45 도달 = 몸이
움직이기 전에 닫기 완료 (c) 접촉 합 ≤ 4000 N, 단일 ≤ 2000 N — 넘으면 폭주로 보고 이 방향 기각
(d) 엄지 패드 접촉 ≥ 13 프레임 (e) `[eval] dz > +0.05` → HELD.

drill / crate / flat_screwdriver 는 같은 0.5 rad/s 원인으로 LOST 였으므로 해머가 일어서기를
통과한 뒤 함께 재실행한다.

## kp 200 은 기각 — 엄지는 290 프레임 동안 30 N·m 로 밀면서 0.000 에 붙어 있었다 (v26)

**v26 (kp 40→200, kd 4.0→20.0, 그 외 v25 동일) 판정: `[eval] dxy 1.3749 m dz −0.0779 m → LOST`.**
예측과 반대로 **엄지가 덜 닫혔다**: T.pitch 최대 **+0.020** (v25 +0.350).

| | v25 (kp 40) | v26 (kp 200) |
|---|---|---|
| T.pitch / T.int / T.distal 최대 | +0.350 / +0.440 / +0.240 | +0.020 / −0.120 / +0.030 |
| 접촉 합 / 단일 링크 최대 | 1975.51 @f630 / 880.75 | **2948.00 @f530 / 1835.07** |
| 엄지 패드 접촉 | 13 프레임 | 2 프레임 |
| 끝 | 제자리 낙하 (dz −0.0090) | 1.37 m 사출 (dz −0.0779) |

궤적이 원인이다: **f470 에 검지가 한 스텝에 0.110 → 1.030 으로 닫히고, 그 순간부터 f760 까지
290 프레임 동안 T.pitch 가 정확히 0.000, T.int 가 −0.160 에 붙어 있다**(목표 0.500 / 0.000).
`200 × 0.500 = 100 N·m` 명령이 30 N·m 상한에 걸린 채 290 프레임을 밀었는데 관절은 움직이지
않았고, **물체가 사출된 다음 프레임 f770 에 +0.480 으로 풀린다.** kp 200 에서는
`CLOSE_ORDER=thumb_first` 의 순서 효과가 한 제어 스텝 안에 사라진다.

같은 잠김은 이미 `play_in_cell_opus.py:211-214` 에 v21/v22 증상으로 적혀 있었다. 그래서
**MIMIC_SPLIT(종동절 게인 분리)을 함께 켜는 후속안은 버렸다**: `v23` 이 kp 40 에서 그것을
켰고 같은 잠김이었다(T.pitch 최대 +0.030, T.int −0.160 이 f550~f750 유지). MIMIC_SPLIT 이
켜진 렌더는 v23 뿐이므로 v25→v26 은 게인 단일 변수다.

**T.pitch 최대값 전체 순위:** v22 0.060 / v23 0.030 / v24 0.100 / **v25 0.350** / v26 0.020.
엄지를 실제로 움직인 변경은 속도 한계(0.5 → 5.0 rad/s) 하나뿐이다.

## v27: 일어서기 전에 정착시킨다 (GraspGenX settle_frames)

v25 의 `[abs]` 궤적은 클립이 **f720~f750 에 이미 40 프레임 정지**하고(palm z 0.3090, root z
0.4210, 물체 z 0.2680 고정) 그 구간을 해머가 접촉 끊김 없이 버틴다는 것을 보여준다. 일어서기는
f750→f760→f770 에 root z 0.4210 → 0.4280 → 0.4680 으로 시작하고, **놓친 프레임은 정확히 그
첫 프레임 f760** 이며 그때 T.pitch 는 0.350 으로 아직 상승 중(최대값이 f760)이었다. 파지가
완성되기 전에 몸이 움직인다.

원본이 같은 일을 한다: `dynamic_playback.py:1049 settle_frames: int = 30`, `:1310-1312
"Stepping %d trajectory frames + %d settle frames"` — 궤적을 멈춘 상태로 추가 프레임을 돌린 뒤
파지를 판정한다(`:128-129` 접촉 평형은 움직임이 멈춘 뒤 생긴다).

**v27 = v25 게인(kp 40 kd 4.0) + `SETTLE_AT=750 SETTLE_FRAMES=150`.** 클립 인덱스만 remap 하고
진단 프레임 번호는 계속 증가시킨다(`play_in_cell_opus.py:955-965`). 150 프레임의 근거: v25 의
T.pitch 는 f460~f760 300 프레임에 0.350 올랐다 = 평균 0.00117 rad/프레임, 남은 0.150 rad 를 같은
평균으로 **외삽**하면 약 128 프레임(측정값이 아니라 외삽). 기준: (a) T.pitch ≥ 0.45 (b) f900 이전
도달 (c) 접촉 합 ≤ 4000 / 단일 ≤ 2000 (d) 엄지 패드 ≥ 13 프레임 (e) dz > +0.05 → HELD
(f) 정착 구간에 접촉 0 프레임 없음.

## 정정: 그 잠김은 v26 만의 것이 아니다 — 엄지는 어느 시도에서도 홀드 중에 닫힌 적이 없다

앞 절에서 나는 "290 프레임 잠김"을 v26(kp 200)의 실패 원인으로 적었다. **v27 이 도는 중에 같은
창을 v25 와 프레임 단위로 맞춰 보니 v25 도 똑같이 잠겨 있었다.** 10 프레임 간격 측정
(q/target, R_thumb_proximal_pitch = T.pitch, R_thumb_intermediate = T.int):

| frame | v25 T.pitch | v25 T.int | v26 T.pitch | v26 T.int | v27 T.pitch | v27 T.int |
|---|---|---|---|---|---|---|
| f560 | +0.000 | −0.160 | +0.000 | −0.120 | +0.000 | −0.160 |
| f620 | −0.000 | −0.160 | +0.000 | −0.160 | −0.000 | −0.160 |
| f700 | −0.000 | −0.160 | +0.000 | −0.160 | −0.000 | −0.160 |

목표는 내내 T.pitch 0.500 / T.int 0.000 이다. v25 의 "T.pitch 최대 0.350" 은 **물체를 놓친
f760 에서야** 나온 값이므로, 파지 구간의 엄지 상태를 대표하지 않는다. v27 은 f700 까지 v25 와
소수점까지 동일하다 — 정착(settle)은 f750 이후에만 갈라지므로 단일 변수가 맞다.

그래서 **kp 가 아니라 이 잠김 자체가 공통 원인이다.** 소스에 이미 적혀 있던 기구는
`play_in_cell_opus.py:711-713`: T.int 가 USD 하한 −0.160 에 걸리면 **하드 PhysX 미믹이 마스터
pitch 를 0 에 못 박는다**(q_follower = −gearing × q_reference). 제조사 URDF
(`x_grippers/inspire_hand/gripper_spherical_dof.urdf`)는 이 관절에 **역방향 가동이 없다**:
thumb_intermediate 0 ~ 0.8, thumb_distal 0 ~ 0.4, master thumb_proximal_pitch 0 ~ 0.6.
우리 USD 는 −0.160 ~ 0.960 / −0.240 ~ 1.440 으로 저작되어 있다.

**전체 시도의 플래그 감사(렌더 로그에서 직접 읽음):**

| | THUMB_LIMIT_URDF | MIMIC_URDF_RATIO | HAND_VEL | kp/kd/eff | 판정 |
|---|---|---|---|---|---|
| v21 | . | . | 0.5 | 40/4/30 | LOST |
| **v22** | **Y** | **.** | **0.5** | 40/4/30 | LOST (23,366 N) |
| v23 | . | . (+MIMIC_SPLIT) | 0.5 | 40/4/30 | LOST |
| v24 | . | Y | 0.5 | 40/4/30 | LOST |
| v25 | . | Y | **5.0** | 40/4/30 | LOST |
| v26 | . | Y | 5.0 | 200/20/30 | LOST |
| v27 | . | Y | 5.0 | 40/4/30 | 진행 중 (+settle) |

**하한을 올린 렌더는 v22 하나뿐이고, 그때는 기어비가 2.4(URDF 0.667 의 3.6 배)여서 0.5 rad
닫기가 thumb_distal 에 1.2 rad — 실제 가동 전체의 3 배 — 를 명령했고 손 속도는 0.5 rad/s 였다.**
그 두 교란 요인은 v24(기어비)와 v25(속도)에서 각각 고쳤다. 따라서
`THUMB_LIMIT_URDF=1` + `MIMIC_URDF_RATIO=1` + `HAND_VEL=5.0` 조합은 **아직 한 번도 돌지 않았다.**
v27 판정을 기록한 뒤 이것을 v28 의 단일 변경으로 넣는다.

## 정착은 성공했고, 그래서 원인이 특정됐다 — 우리는 접촉을 수백 N 으로 눌러 두고 있었다 (v27)

**v27 판정: `[eval] dxy 0.3881 m dz −0.0079 m → LOST`.** 그런데 이 렌더는 실패로서보다
**분리 실험으로서** 값이 있다.

클립 f750 을 150 프레임(5.0 s) 붙들었더니:

| 정착 구간 f750~f900 | 값 |
|---|---|
| 물체 z | **0.268 고정 (150 프레임 내내)** |
| root z / palm z | 0.421 / 0.310 고정 |
| 접촉 합 / 엄지 패드 | 751 → 774 N / 377 → **387 N**, 끊김 없음 |
| T.pitch / T.int / 검지 | **−0.000 / −0.160 / +1.00** 내내 (목표 0.500 / 0.000 / 1.47) |

**정적으로는 5 초를 버틴다.** 그리고 **내 외삽은 반박됐다**: "엄지가 남은 0.150 rad 를 닫는 데
약 128 프레임 필요"라고 적었지만, 완전 정지 상태로 150 프레임을 더 줬는데 엄지는 0.000 rad 에서
전혀 움직이지 않았다. **엄지는 느린 것이 아니라 388 N 접촉 평형에 막혀 있다.**

놓치는 순간은 시간이 아니라 **클립이 다시 움직이는 프레임**이다:

| | f900 | f910 |
|---|---|---|
| 물체 | [−0.2311 0.0116 0.2680] | [−0.3377 0.2735 0.0391] |
| 물체 속도 | ~0 | **[−0.296 0.863 −0.631] = 0.86 m/s** |
| 접촉 | 774 N (엄지 387 N) | **0 N** |
| 손 q (검지 / T.pitch / T.int) | 1.00 / −0.00 / −0.16 | **1.43 / 0.38 / 0.49** |
| palm z / root z | 0.310 / 0.421 | 0.315 / 0.428 |

**0.007 m 의 몸통 상승이 물체에 0.86 m/s 를 줄 수는 없다.** 수백 N 으로 눌려 있던 접촉이
풀리면서 물체가 사출되고, 자유가 된 12 개 관절이 한꺼번에 0.4 rad 목표로 튀었다. v25 의 f760 과
같은 사건이고, 정착으로 그 시점만 150 프레임 미룰 수 있었다. 덧붙여 f900 의 기하는
물체 [−0.231 0.011 0.268] / palm [−0.139 0.044 0.310] — 해머는 손바닥에서 **9.2 cm** 떨어진
손끝 집기이며 파워 그립이 아니다.

## v28: 힘 상한을 제조사 값으로 — 이 수정은 5지 손에 한 번도 들어간 적이 없다

진단은 이 저장소에 **이미** 적혀 있었다. `play_in_cell_opus.py:267-273`:

> Dex3-1 finger torque, from Unitree's own URDF: every hand joint … is `effort="1.4"`.
> IsaacLab's G1_29DOF_CFG leaves the hands at effort_limit=300, 214x the real actuator,
> **so the closing fingers drive straight through the object and PhysX ejects it instead of
> stalling them on contact.** The trajectory commands the fingers all the way to their joint
> limits …, so **what stops them has to be the actuator, not the command.**

그리고 바로 그 아래가 `if os.environ.get("HAND") == "inspire": pass` 다 — **5지 손은 이 수정을
받은 적이 없다.** Dex3 는 1.4 N·m 로 내렸고, Inspire 는 `HAND_EFFORT` 기본 200 / 우리 실행값
30 으로 돌아왔다. 제조사 URDF(`x_grippers/inspire_hand/gripper_spherical_dof.urdf`)는
**12 개 회전 관절 전부 `effort="10" velocity="5.0"`** 이다(그 외: spherical 1000, prismatic 200 —
손가락 관절이 아니다). GraspGenX 도 같은 말을 한다(`dynamic_playback.py:685-690`): URDF 의
effort 가 PD 힘을 상한하며, **닫기를 멈추는 것은 힘 상한이다.**

**v28 = v27 + `EFF=10`** (정착은 같은 창 비교를 위해 유지). 측정된 환산: EFF=30 에서 엄지 패드
387 N → N·m 당 약 12.9 N, 따라서 EFF=10 은 약 130 N 을 예상한다(측정 기반 외삽).
기준: (a) 정착 구간 엄지 패드 < 200 N (b) 정착 구간 물체 z 변동 < 5 mm (c) f901~f920 물체 속도
< 0.3 m/s (d) 재개 시 관절 점프 < 0.15 rad/10프레임 (e) `dz > +0.05` → HELD.
그래도 사출이 남으면 다음 값은 Dex3 규모의 1.4 N·m(예상 패드 약 18 N)이다.

## v28 결과: 힘 상한은 접촉력을 정하는 변수가 아니었다 — 세 가지 정정

`[eval] end pos [-1.6093  0.3646  0.1535]  dxy 1.3466 m  dz -0.0085 m  -> LOST`

측정된 경과(OBJ_EVERY=10):

| 프레임 | 접촉 합 / 엄지 패드 | 물체 z | 검지 q / T.pitch |
|---|---|---|---|
| f460 | 0 N | 0.1499 | +0.11 / +0.030 |
| f470 | **576.8 / 284.1 N** | 0.1531 | 첫 접촉 |
| f470~f600 | 604~864 N | — | **+0.89 에서 정지**(목표 1.47) / **0.000** |
| f610~f670 | 466~**1322 N** | 0.1526 → **0.2143** (+6.17 cm) | 들어올림 |
| f670 | 168.3 / 89.1 N | 0.2143 | 접촉 붕괴 시작 |
| f680 | **0 N** | **0.1527** (바닥) | +1.39 / **+0.430** |
| f690 | 0 N | 0.1533, 시작점에서 0.1016 m | +1.47 / **+0.500**(목표) |

**정정 1 — 예측이 반증됐다.** v28 절에 적은 "12.9 N/N·m → EFF=10 이면 패드 약 130 N" 은 틀렸다.
EFF=10 의 접촉은 v27(EFF=30, 합 최대 775.8 N)보다 **더 컸다**(합 최대 1322.3 N, f630).
손가락은 힘 상한에 눌려 약해진 것이 아니라 기하적으로 물려 있다. **effort limit 으로 조임을
조절하는 줄은 여기서 끝난다.** Dex3 규모의 1.4 N·m 로 더 내리는 후속안도 같은 이유로 철회한다.

**정정 2 — 엄지를 막은 것은 미믹이 아니라 접촉 반력이다.** v26/v27 에서 "하드 미믹이 마스터를
고정한다"고 적었으나, 같은 `MIMIC_URDF_RATIO=1` 에서 T.pitch 는 접촉이 있는 내내 0.000 이었고
접촉이 사라진 f680 에 +0.430 → f690 +0.500(목표)까지 **10 프레임 안에** 갔다. 미믹 제약은
엄지를 막지 않는다.

**정정 3 — "재개 한 스텝에 0.760 rad 점프"는 명령 불연속이 아니다.** `fable40v2place_hands.npy`
는 클립 468~1108 내내 완전히 평평하다(네 근위 1.47 / 엄지 pitch 0.5, 프레임차 0.0000 rad).
0.760 rad 은 10 프레임 출력 간격의 **실측 q** — 물체가 빠진 뒤 손가락이 빈 공간을 닫은 결과이며
원인이 아니다. (위 f900/f910 표의 0.4 rad 도 같은 성질이다.)

또한 v28 을 처음 "파지가 형성되지 않았다"고 읽은 것은 오독이었다. 정착창의 0 N 은 물체가
f680 에 이미 이탈했기 때문이고, 파지는 형성되어 6.17 cm 까지 들렸다.

### 두 실패는 같은 기제의 양단이다 — position 모드의 상시 오차

손 클립은 네 근위를 관절 한계 1.47, 엄지 pitch 를 0.5 로 닫고 그 값을 캐리 내내 붙잡는다.
물체가 있으면 손가락은 q 0.89~0.95 에서 멈추므로 **위치 오차 0.52~0.58 rad 이 영구히 남고**,
kp 40 에서 요구 토크는 40 × 0.55 ≈ **22 N·m** 다.

| | EFF=30 (v27) | EFF=10 (v28) |
|---|---|---|
| 22 N·m 요구 | 포화 안 됨 → 캐리 내내 누름 | 상한에 포화 |
| 정착창 엄지 패드 | 375~388 N | 0 N (이미 이탈) |
| 결과 | 5 초 정지는 버티고, 눌림이 풀리는 f910 에 **1.11 m/s 사출** | 들어올리는 6 cm 구간에서 **미끄러짐**(f670) |

v11 도 동일했다: kp 2000, 오차 0.8 rad → 요구 1600 N·m 이 effort 200 N·m 에 포화, 수평
1.26 m/s 사출. **조절할 변수는 "얼마로 누르나"가 아니라 "위치 오차로 계속 누른다"는 방식이다.**

### 파지 기하는 원인이 아니다 — 측정으로 후순위로 내렸다

물체 중심을 파지 프레임에서 본 z (손끝은 `inspire_hand/config.json` 의 `fingertip [0,0,0.15]`):

| 파지 | z | conf | 비고 |
|---|---|---|---|
| **#138** | **+13.5 cm** | 0.921 | v9~v28 이 운반하는 파지, 손끝에서 1.5 cm |
| #48 / #36 / #37 / #47 | +11.7 cm | 0.981~0.982 | 같은 fingertip 세트의 최고신뢰 군집, 1.8 cm 더 깊다 |
| #45 | +13.7 cm | 0.996 | palm 재주석 세트 최고(v8 이 사용) |

fingertip 세트 중앙값 +12.5 cm, palm 세트 중앙값 **+13.7 cm**. `inspire_hand_palm/config.json` 의
`_note` 는 이 재주석이 "fingertip pinch 대신 wrap(power) 파지를 요구한다"고 적었지만,
**측정은 그 반대다** — palm 재주석은 파지를 손바닥으로 끌어오지 못했고 신뢰도 순위만 바꿨다.
게다가 파지를 바꾸면 reach → reach reference → walk clip → place reference 를 전부 다시
만들어야 한다. 그래서 닫는 방식을 먼저 고친다.

## v29: GraspGenX 가 실제로 쓰는 닫기 — 속도 모드

`GraspGenX end2end/robots/g1_inspire_arm.yaml`:

```yaml
gripper_control_mode: velocity
gripper_close_velocity:   # thumb_0 0.0, thumb_1/2 -0.25, index_0/1 +0.25, middle_0/1 +0.25
finger_effort_limit: 1000.0
```

`end2end/dynamic_playback.py:645` "contact equilibrium stops the motion", `:128-129` "physics
naturally limits how far the close actually goes when there's contact". **속도 모드에는 붙잡아 둘
위치 오차가 없다.** 접촉이 속도 명령과 평형을 이루면 그 자리에서 멈추고, 상시 압박이 사라진다.

우리 쪽 구현은 이미 배선돼 있다: `play_in_cell_opus.py:915-934`(설정), `:1001-1009`(프레임
전환), `:1033`(substep 재발행) — squeeze 구간에 stiffness 0 / damping `CLOSE_KD`, thumb_0 제외.

**v29 = v27 + `CLOSE_MODE=velocity CLOSE_VEL=0.25 CLOSE_KD=40`** (EFF 는 v27 의 30 으로 복귀).
`CLOSE_KD` 는 감이 아니라 계산이다: 막힌 관절의 토크 = `CLOSE_KD × CLOSE_VEL` = 40 × 0.25 =
**10.0 N·m** = 제조사 URDF 의 12 개 회전 관절 `effort="10"`. 힘 상한이 아니라 실제 구동
토크로 제조사 값을 구현한 것이다(그래서 EFF=30 은 상한으로 걸리지 않는다).

이 조합은 한 번도 돌지 않았다. velocity close 가 나오는 로그는 `fable6/7b/10/12/13/14` 의
`pick.log` 뿐이고 전부 Dex3 시절(9/22~27)이며, `MIMIC_URDF_RATIO=1` · `HAND_VEL=5.0` ·
implicit PD 다리 · convexDecomposition 콜라이더 · 솔버 100/50 이전이고, 5지 Inspire 도
캐리 체인도 아니었다.

기준: (a) 접촉이 f470~f1170 내내 끊기지 않는다 (b) 정착창 엄지 패드 최대 < v27 의 387 N
(c) f901~f920 물체 속도 < 0.3 m/s (d) `dz > +0.05` → HELD.

## v29 결과: 속도 닫기는 맞았고, 남은 것은 회전이다 (접촉이 살아 있는 채 22.7도)

판정 `[eval] end pos [-0.493 0.1505 0.153] dxy 0.2176 m dz -0.0090 m -> LOST`.

닫기 방식 교체는 오픈소스가 설명한 대로 작동했다. 근위 q 가 0.25 rad/s(10프레임당
0.085 rad)로 램프하고, 접촉이 생긴 f560 에 q 0.88 에서 **스스로 멈췄다**. position
모드에서 캐리 내내 남던 위치 오차 0.52~0.58 rad(요구 토크 40 x 0.55 = 22 N·m)이
사라졌다 — GraspGenX `end2end/dynamic_playback.py:645` "contact equilibrium stops the
motion" 그대로다. 손가락이 1.47 을 넘어 관절한계 1.700 까지 간 것은 물체가 빠진 f710
이후이며(f710 0.96 -> f760 1.38), 속도 모드에 위치 피드백이 없기 때문이다.

| frame | sum N | thumb N | best-opp | dex | obj z | prox q0 |
|---|---|---|---|---|---|---|
| 520 | 7.59 | 0.00 | 7.59 | False | 0.1548 | 0.62 |
| 550 | 368.06 | 147.65 | 89.52 | True | 0.1574 | 0.85 |
| 600 | 942.01 | 415.29 | 198.45 | True | 0.1562 | 0.89 |
| 630 | 1651.20 | 317.59 | 775.73 | True | 0.1825 | 0.88 |
| 690 | 481.77 | 240.73 | 103.47 | True | 0.2509 | 0.88 |
| 700 | 532.38 | 243.79 | 146.75 | True | 0.2490 | 0.89 |
| 710 | 0.00 | 0.00 | 0.00 | False | -0.0071 | 0.96 |

들어올리기는 개선됐다: 물체 z 0.1563(f540) -> 0.2509(f690) = **+9.46 cm**, 접촉
150 프레임 연속(f550~f700), `dexsuite_good` 그 150 프레임 전부 True, best-opposing
78~211 N. **DexSuite 판정기는 이 실패를 걸러내지 못한다.**

실패 순서는 회전이 먼저다(앞서 "사출이 아니라 회전"이라고만 쓴 것을 정정한다):
f700 에 물체가 quat [0.98 0.197 -0.032 -0.001] = 22.7 deg(0.396 rad) 회전하는데 접촉은
아직 532 N 이고 y 가 10 프레임에 -4.1 cm 움직인다. f710 에 접촉 0 N, 속도 1.01 m/s.
손목 관절은 매끄러웠고(10프레임당 약 0.02 rad) 팔 z 도 꾸준했다 — 몸의 충격은 없다.

### 회전은 클립이 명령한 것이다

`results/motion/fable40v2place.pkl` 의 `dof[:,22:29]` 직접 측정:

| 구간 | 손목 3축 프레임차 (rad) |
|---|---|
| f480~f560 | 0.0000 / 0.0000 / 0.0000 (완전 정지) |
| f600~f640 | -0.0246 / +0.0453 / +0.0318 |
| f640~f680 | -0.0779 / +0.0505 / +0.0914 |
| f680~f720 | -0.0776 / +0.0403 / +0.1106 |

f440~f760 전체 범위는 어깨 0.396, 팔꿈치 0.264, 손목 0.143/0.180/0.139/0.235 rad.
물체가 빠진 f700 은 재배치가 시작된 f600 에서 100 프레임 뒤이고, 물체 자신의 회전
0.396 rad 은 가장 큰 손목 명령 회전 0.234 rad 의 **1.7 배**다. 추종 지연이 아니라,
손이 명령대로 돌 때 파지가 모멘트를 버티지 못한 것이다.

### 쥐는 힘으로는 막히지 않는다 (숫자)

- 버텨야 하는 중력 모멘트 = 0.2 x 9.81 x 0.05 = **0.098 N·m** (0.05 m = 가장 가까운
  손가락 링크에서 물체 중심까지 측정값 4.9~5.9 cm)
- 물체를 0.396 rad 돌리는 토크 = 1.1e-3 x 2(0.396)/0.333^2 = **0.008 N·m**
- condim=3 점접촉이 접촉선 축으로 버티는 비틀림 강성 = **0**

GraspGenX `dynamic_playback.py:988-993` 은 condim 을 기본값 3 으로 두고 비틀림
마찰(condim=4)을 "a band-aid for the object-spin regression" 이라며 되돌렸다. 그들
기준으로도 회전은 마찰로 막는 것이 아니다. 942 N 을 쥐어도 접촉선 축 강성은 0 이므로,
접촉선 밖의 접촉(손바닥)이 있어야 한다.

### 이번에 닫은 레버 (v30 에서 다시 건드리지 않는다)

| 레버 | 닫은 근거 |
|---|---|
| 힘 상한 | v28: EFF=10 이 접촉을 더 키웠다 (1322 N 대 776 N) |
| 마찰/질량/솔버 | 이미 GraspGenX 검증값 (FINGER_MU 3.0, OBJECT_MU 10.0, 0.2 kg, 100/50) |
| 닫기 게인 | 우리 40 x 0.25 = 10.0 N·m 대 그들 5지 프로필 `ur10e_surge_hand.yaml` 의 50 x 0.25 = 12.5 N·m |
| 구동 지속 | 그들 `:1340-1355` is_open/close_vel 논리가 우리 `_want` 과 동일 |
| 물체 관성 | 직접 측정: hull 대 mesh 관성 비 [1.03, 0.98, 1.03] — 내 관성 가설은 반증됐다 |
| 비틀림 마찰 | 위 `:988-993` 대로 그들이 band-aid 로 되돌렸다 |
| 팔 추종 오차 | end-of-walk arm error 2853.9 mrad 이 v25~v29 바이트 동일 — 변별력 없음 |

`g1_inspire_arm.yaml` 은 `finger_velocity_kd` 를 생략해 기본 800 이 되는 덜 튜닝된
프로필이고, 5지 손의 튜닝된 대응물은 `ur10e_surge_hand.yaml`(kd 50, effort 200)이다.

## v30: 손끝 핀치 대신 손바닥에 앉힌다 (재구축 비용 0)

유일한 변경은 캐리 클립 REF `fable40v2place -> fable40pwplace`(렌더 스크립트 31행 한 줄).
닫기는 v29 그대로. 정착창만 닫기 시작이 f459 -> f520 으로 61 프레임 늦어진 만큼 750 -> 811.

파워 그립 자세는 이미 디스크에 있고 측정값이 남아 있다: `results/fable40/test_pw3.txt` 는
palm z **+0.215** 에 lowest fingertip z **+0.154** (손바닥이 손끝보다 6.1 cm 위 = 손잡이를
위에서 덮는 자세), 닫힘 간격 3.3 mm. 두 캐리 클립은 같은 무릎 자세·같은 위치에서 닫는다
(root [0.083 0.029 0.397] 대 [0.030 0.026 0.423], hip pitch -1.83 동일, yaw 동일) — 같은
장면 같은 망치에 대한 다른 접근이다. 우팔만 다르다: 손목 롤 **-0.417 대 +1.932 rad**.

이 자세가 아직 안 된 이유는 판정이 전부 위치 모드 닫기 시절이라는 것이다.
`test_power.txt` #0 은 "held the lift but slipped in the stand-up",
`test_power2.txt`/`test_pw3.txt` 는 "slipped in the stand-up" 으로 LOST 다 — v27/v28/v29 가
무너뜨린 그 닫기다. 파워 그립 + 속도 닫기 조합은 한 번도 돌지 않았다.

합격 기준: (a) 손목 재배치 구간(f660~f780)에 접촉이 0 N 이 되지 않는다 (b) 그 구간 끝
물체 회전 < 0.2 rad = 손과 함께 도는 것 (c) `[eval] dz > +0.05 m` -> HELD (d) 손바닥
접촉이 있는데도 회전으로 잃으면 기하 줄도 닫히고, 다음 레버는 클립이 명령하는 손목 회전
자체(`build_place_reference` 의 블렌드)다. 쥐는 힘 쪽은 더 보지 않는다.

## v30 결과: 손바닥 파지는 400 프레임 버텼고, 잃은 것은 파지가 아니라 솔버였다

판정 `[eval] end pos [-1.6737 1.0136 -0.0877] dxy 1.6780 m dz -0.2497 m -> LOST`.

핀치(v29) 대 손바닥(v30), 측정값:

| | v29 핀치 | v30 손바닥 |
|---|---|---|
| 손가락 정지 위치 | q 0.88 | q 1.05 (더 깊이 앉았다) |
| 접촉 최대 | 1651 N | 535 N (하중을 나눠 받는다) |
| 물체 유지 | f710 에 이탈 | f560~f960, **400 프레임** |
| 들어올림 | +9.46 cm | **+12.8 cm** |
| 그 구간 회전 증가 | 0.396 rad 후 사출 | 34.5° -> 58.1° (23.6°) |

v29 가 놓친 f710 을 접촉 216 N 으로 통과했다. 쥐는 힘이 아니라 기하였다는 v29 의 결론은
이 지점까지 맞았다. 그러나 HELD 는 아니다.

**f970 은 역학적 이탈이 아니다.** 접촉 3049.36 N, 물체 속도 `[-53.918 -63.575 64.123]` = 105.170 m/s.
0.2 kg 의 105 m/s = 1103 J 이고, 직전 프레임 접촉은 463 N 이었다. 저장될 수 없는 에너지다.
접촉 링크는 f960 과 같은 두 개(`R_thumb_proximal` 229.64->1658.66, `R_middle_intermediate` 177.64->1390.70)
이므로 새 물체와의 충돌도 아니다. 책상은 `[-1.53 -0.946]` top z 0.764, 물체는 `[-0.2758 0.1304 0.2728]`.

가설 두 개를 반증했다:
- "정착 해제에서 팔이 튄다" — 아니다. `play_in_cell_opus.py:961-963` 의 `_idx` 는 811 에서 812 로
  연속 재개한다. 명령 점프가 없다.
- "클립에 불연속이 있다" — 아니다. `fable40pwplace` 의 클립 810~830 프레임차 최대 0.0254 rad,
  `fable40v2place` 는 0.0234 rad. 둘 다 매끄럽다.

남은 사실만: sim f970 = 클립 820, 정착은 sim f961 에 끝났다. 정착 150 프레임 동안 몸은 정지한 채
(z 0.2788->0.2728, 속도 0.009~0.071) 10 N·m 속도 닫기가 계속 구동했다. 원인인지는 단정하지 않는다.

**교란 변수 (v30 은 한 변수 실험이 아니다):** 의도한 변경은 파지 기하 하나였지만 pw 클립은
닫기 자세 대비 명령 손목 회전이 0.085 rad 이고 v2 클립은 0.234 rad 이다(1/3). 400 프레임 유지를
손바닥 접촉만의 성과로 돌릴 수 없다. 또 회전 61.1° 는 캐리가 아니라 닫는 순간(f520)에 나고
f540 에 34.5° 로 되돌아왔다 — 닫는 동작 자체가 손잡이를 굴린 것으로 보이나 프레임을 보지 않았다.

`dexsuite_good` 은 3049 N 폭주 프레임에서도 True 였다. v29 의 150 프레임 회전-이탈에 이어 두 번째로
이 판정기가 실패를 걸러내지 못했다.

## v31: 정착창을 오픈소스 기본값으로 되돌린다 (150 -> 30)

`GraspGenX end2end/dynamic_playback.py:1049` 의 `settle_frames: int = 30`. 우리는 150, 5배다.
정착창을 늘린 이유는 위치 모드 닫기의 누르는 힘(v27 387 N)을 재기 위해서였고, v29 부터의
속도 닫기에는 그 누름이 없다(접촉 평형에서 스스로 멈춘다). 이제 남길 이유가 없다.

한 변수로 갈린다:
- 폭주가 sim f850(= 클립 820)에 그대로 나타나면 원인은 클립 820 프레임이다 -> 그 프레임을 본다.
- 사라지면 원인은 150 프레임 정지 창이다(정지한 몸 + 계속 구동하는 닫기).

나머지는 v30 과 동일(REF=fable40pw, velocity 0.25 rad/s kd 40, KP 40 KD 4.0 EFF 30).
기준: (a) sim f850 근처에 1000 N 초과 프레임이 없다 (b) `[eval] dz > +0.05 m`.

## v31 결과: 폭주는 정착창 길이가 키운 것이고, 잃는 자리는 클립 812~820 그대로다

판정 `[eval] end pos [-0.1704 0.2523 0.153] dxy 0.2403 m dz -0.0090 m -> LOST`.
물체는 바닥 안착 높이로 되돌아왔다 — v30 의 1.7 m 사출과 다르다.

| | v30 (정착 150) | v31 (정착 30) |
|---|---|---|
| 최대 접촉 | 3049.36 N | **510.83 N** (@f790) |
| 최대 속도 | 105.170 m/s | **2.095 m/s** (@f860) |
| 최고 z | +0.3299 (사출) | +0.2869 (+13.9 cm) |
| 놓친 sim 프레임 | f970 | f850 |
| **= 클립 프레임** | **820** | **820** |
| 정착 종료 | sim f961 | sim f841 |

두 실행은 sim f810 까지 수치가 완전히 같다(f660 29.01 N / 1.353 m/s, f800 443.67/244.27/145.35,
z +0.2824, 54.0°). 정착 길이 외에 변경이 없었고 시뮬은 결정적이다. 정착창 150 은 심각도를
6배(접촉) / 50배(속도) 키웠지만, 잃는 클립 프레임은 옮기지 못했다. 둘 다 정착 해제 9 프레임 뒤다.

**내 판정 기준에 결함이 있었다.** "sim f850 에 나타나면 원인은 클립 프레임"이라 미리 적었고
실제로 sim f850 에 나타났다. 그러나 두 실행 모두 클립 811 에서 해제하므로 '해제 전이'와
'클립 812~820' 이 완전히 교란되어 있다. 내가 바꾼 정착 길이는 sim 프레임만 옮기고 클립 프레임은
옮기지 않는다. 이 실행으로 둘을 가릴 수 없다.

**팔은 이 실패의 원인이 아니다** (`fable40pwplace.pkl` 직접 측정): `dof[:,22:29]` 는 클립
805/815/825/835/845/855 에서 바이트 동일 `[-0.433 -0.206 -0.237 +0.375 +1.931 -0.772 -1.580]`,
클립 790~880 우팔 프레임차 0.0000, 손목 3축 811 기준 |Δ| 0.0000. v29 의 "명령 손목 회전이
물체를 굴려 뺀다" 기제는 pw 클립의 이 구간에 존재하지 않는다.

움직이는 것은 일어서기다: root z 클립 800 `+0.4506` -> 880 `+0.7816` (+33.1 cm), 최대 상승률
클립 839->849 = +107.9 mm / 0.333 s = **0.324 m/s**, 무릎 R 2.212 -> 0.349 rad. 단 잃는 지점
(클립 812~820)에서는 root 가 1.4 cm 올랐을 뿐이고 상향 가속 약 0.28 m/s² = 0.2 kg 에 0.056 N
(무게의 2.8 %)이므로 관성 하중으로 설명되지 않는다.

## v32: 중간 정착을 없앤다 — 오픈소스에는 그런 구성물이 없다

숫자가 아니라 코드 위치가 정한다. `GraspGenX end2end/dynamic_playback.py`:
- `:1075` "extra frames to step **after the trajectory ends**"
- `:1310` `total_with_settle = total_traj + settle_frames`
- `:1328` `wp_idx = min(t, total_traj - 1)` — 마지막 웨이포인트로 고정
- `:1380` `phase = "plan" if t < total_traj else "settle"`

그들의 정착은 궤적 끝에 붙는 **꼬리**이고 멈췄다가 다시 이어지는 일이 없다. 재개가 없으므로
해제 전이도 없다. 우리가 v27 에 넣은 중간 정착(클립 811 을 붙잡고 812 로 재개)은 오픈소스에
대응물이 없는 우리 구성물이며, 넣은 이유는 위치 모드 닫기의 누르는 힘(v27 387 N)을 정지한 몸에서
재기 위해서였다. v29 부터의 속도 닫기에는 그 누름이 없다.

`SETTLE_AT=-1 SETTLE_FRAMES=0` (`play_in_cell_opus.py:959-963` 의 `_clip_i` 가 항등이 된다).
기준: (a) sim f812~820 에 접촉 0 N 프레임이 없다 (b) 일어서기(클립 839~849, root 0.324 m/s)를
접촉 유지로 통과한다 (c) `[eval] dz > +0.05 m`.
갈림: (a) 실패면 원인은 클립 812~820 프레임 자체다. 통과하면 중간 해제가 원인이었고 영구히 뺀다.

## v32 결과: 중간 해제는 무죄고, 문제는 닫기보다 먼저 있었다

`[eval] dxy 0.8501 m dz -0.0139 m -> LOST`. 정착창을 완전히 없앴는데도 접촉은
클립 830 에 0 N 이 됐다. 세 번 다 같은 자리다.

| | 정착 | 접촉 0 N 이 되는 sim 프레임 | = 클립 프레임 | 최고 접촉 | 최고 속도 |
|---|---|---|---|---|---|
| v30 | 150 | f970 | 820 | 3049.36 N | 105.170 m/s |
| v31 | 30 | f850 | 820 | 510.83 N | 2.095 m/s |
| v32 | 0 | f830 | 830 | 510.83 N | 3.413 m/s (낙하 후) |

사전 등록한 규칙대로 중간 해제는 제거됐다. 정착창 길이는 심각도만 정했다.
f800/f810 접촉은 세 번 모두 443.67 / 425.46 N 로 소수점까지 같다 — 시뮬레이터는
결정론적이고, 정착창 외에 다른 차이가 없었다는 확인이다.

### 잃는 자리가 아니라 닫는 자리를 봐야 했다
- f510: 손가락 q **0.06 rad**(거의 안 닫힘), 가장 가까운 링크 `R_thumb_distal` **9.0 cm**,
  물체 정지(vel 0.000), 기울기 8.9도
- f520: 물체 누적 이동 0.0290 -> **0.1349 m (+10.6 cm)**, 기울기 **61.1도**, 손가락 q 0.14

손가락이 닫히기 전에 망치가 10.6 cm 밀리고 52도 돌아간다. 이후의 접촉
200~510 N 은 전부 "설계된 자세에서 52도 돌아간 물체"를 쥔 것이다. 접촉 센서가
f520 에 0 N 을 읽는 것은 샘플이 10프레임 간격이라 그 사이 충격이 안 보이는 것이다.

### 원인은 손바닥이 손잡이 위에 없다는 것 — 페이블 자신의 측정에 있었다
`results/fable40/test_pw3.txt` 파지 #3: `at grasp: palm - object [0.165 -0.108 0.184]`
(검산: palm z 0.215 - object z 0.032 = 0.183 = 세 번째 성분, 같은 좌표계).

| 측정 | 값 |
|---|---|
| 손바닥-물체 수평거리 | **0.197 m** (#1 0.199, #0 0.168) |
| 망치 반길이 | 0.160 m |
| 손잡이 축 방향 / 측면 | 16.5 cm (끝을 0.5 cm 지나침) / **10.8 cm** |
| 손잡이 단면 (메시 실측) | 4.2 x 2.5 cm (반폭 2.1 cm) |
| 머리 단면 | 13.6 x 7.4 cm |

닿는 것은 벌어진 엄지 하나뿐이다(yaw 1.30 = GraspGen `config.json` 의 open 값
1.308, 이건 맞다). 잡은 뒤에도 손가락은 멀어진다: f700 `R_index_proximal` 4.2 cm
-> f820 `R_index_intermediate` 10.7 cm, 접촉이 269~476 N 인 동안에도 7~12 cm 밖이다.
엄지 yaw 는 1.06 -> 0.66 으로 단조 감소(밀려서 벌어짐), 엄지 pitch 는 목표 0.5 에
대해 0.07 에서 막혀 있다. 손가락으로 감싼 파지가 아니라 엄지-검지 옆면 집기다.

그리고 테스터는 이미 2026-09-28 에 말했다 — test_pw3.txt 네 후보 **전부**
`stand-up test: body up 0.35 m, object followed -0.000 m`. 후보 집합 자체가
일어서기를 버틴 적이 없다. v27~v32 는 이 집합 위에서 닫기 방식만 바꿔 왔다.

### 오픈소스에 이 손의 기준이 파일로 있다
`x_grippers/inspire_hand/config.json`: close 는 thumb_pitch **0.6**(우리 목표 0.5),
`sweep_volume` extents **[0.08, 0.045, 0.04] @ offset [0,0,0.135]`, `standoff` 0.005.
쥐는 부분은 그 8 x 4.5 x 4 cm 상자에 들어와야 한다 — 손잡이 4.2 x 2.5 cm 는 들어가고
머리는 못 들어간다. 우리 파워 랩 자세(`reach_from_pose.py:517-542`)는 POWER_X/POWER_Y
격자로 해석적으로 만든 것이고 conf 는 `1.0 - |fr - 0.5|`(GraspGen 점수가 아니다).
`e2e_grasp_demo.py:1039 collision_free_grasps` 독스트링: *"The pick_and_lift path never
calls it, so until now this pipeline handed cuRobo raw candidates. ... which is why the
hand could arrive already through the box."* 두 검사 모두 이 자세에 적용된 적이 없다.

## v33: POWER_X 0.10 -> 0.127 (페이블이 실측해 둔 너클 위치)

`reach_from_pose.py:519-520` 주석, 페이블 2026-09-28 실측:
*"the handle sits under the finger base (knuckles at x 0.127), not under the palm heel
(x 0.10 put the fingertips on the floor beyond the handle: power_chain, 16/16 LOST)."*
`pxs` 기본값이 0.10 이고 pw3 는 기본값으로 만들어졌다. 위에서 측정한 "손잡이 옆
10.8 cm, 끝을 0.5 cm 지나침" 이 그 0.10 의 결과다. 변경은 이 한 숫자.

대조군을 같은 실행에 넣는다: `POWER_X=0.10,0.127 POWER_FRACS=0.4,0.45` -> 후보 8개
(0.10 쪽이 pw3 의 알려진 결과를 재현하는 대조군). 게이트는 렌더가 아니라 테스터다 —
`at grasp: palm - object` 수평거리가 0.197 m 에서 줄고(목표: 반길이 0.160 m 안),
`stand-up test: object followed` 가 0.000 m 에서 커지는지 먼저 본다. 통과한 후보만
클립으로 만들어 렌더한다. 닫기 쪽(위치/속도, effort, kp/kd, 정착창, 솔버)은 모두 닫혔다.

## v33·v34 판정 무효 — 테스터가 5지가 아니라 Dex3 손을 들고 돌았다

**결론부터: 기하 문제가 아니었다. 내 두 실행은 `HAND=inspire` 없이 돌아서 손가락이
한 번도 닫히지 않았다.** 그래서 v34 의 "px 는 레버가 아니다" 는 **철회한다** —
손이 열려 있는 실행의 `followed +0.000 m` 에는 정보가 없다.

근거(`results/fable40/test_pw34_only.txt`, 내가 만든 로그):

```
right fingers at close (index0 index1 middle0 middle1 thumb0 thumb1 thumb2):
        [0. 0. 0. 0. -0. 0.72 0.]  of closed [1.57 1.75 1.57 1.75 0. -1.05 -1.5]
right finger q [0. 0. 0. 0. -0. 0.72 0.]  target [0. 0. 0. 0. 0. 1.47 1.47]
```

헤더가 Dex3 이름 7개다. 5지면 한 손 12개(`R_*_proximal/intermediate`)가 찍힌다.

메커니즘(두 파일의 비대칭):
- `reach_from_pose.py:559-561` — power-grasp 분기의 `_open`/`_closed` 는 **HAND 와
  무관하게** 12칸 인스파이어 벡터로 하드코딩되어 있다. 그래서 npz `hands` 는 항상
  24칸으로 나온다(pw3·pw34 마지막 행이 바이트 단위로 같다: `_open_one + _closed_one`).
- `test_grasps_in_isaac.py:146-147` — `hand_ids` 는 `HAND_NAMES` 에서 오고, 그쪽은
  `HAND == "inspire"` 로 갈린다. 미설정이면 Dex3 이름 14개.
- `put(root7, dof29, hands14)` 는 `for k, j in enumerate(hand_ids): tgt[0,j] = hands14[k]`
  로 zip 한다 → 24칸 벡터의 **앞 14칸**만 쓰인다. 왼손 0..6, 오른손 7..13 =
  `[0,0,0,0,0,1.47,1.47]`. 검지·중지는 0(활짝 열림), thumb_1/2 만 1.47.

`docs/DIAGNOSIS.md:492` 가 이미 "테스터·렌더·순위·체인 전부 `HAND=inspire` 분기"
라고 적어 둔 그 변수다. 내가 체인을 새로 쓰면서 빠뜨렸다.

### pw3 재현은 파라미터 역산이 아니라 그 실행의 스크립트로 해야 했다

`scratchpad/power3_chain.sh`(pw3 를 만든 스크립트)와 내 `pw34_chain.sh` 를 맞춰 보면
바뀐 변수가 1개가 아니라 12개다: `HAND=inspire`, `POWER_RADIUS` 0.0115(기본 0.016),
`POWER_SLIDE` 0.05, `POWER_FRACS` 0.4,0.55, `grasps_all.json`(내 것은 `grasps_palm.json`),
`RETARGET_CFG=..._retarget_floor.yml`, `BODY_W=0.1`, 씬(`TIDY_NO_FLOOR_CARTON`,
`TIDY_CRATE_ON_DESK`), 테스터의 `FIX_ROOT ARM_KP_SCALE=4 PD_BODY HAND_KP=40
OBJECT_NO_SLEEP SETTLE_IDLE=400 --slow`.

`POWER_RADIUS` 가 달랐으므로 내 역산값도 틀렸다: `centre_z = top - radius` 이므로
pw3 의 실제 `POWER_Y` 는 0.0445 가 아니라 **0.0400**(스크립트 기본값 `PY:-0.04`).
즉 "pw3 파라미터를 정확히 복원했다" 는 앞 절의 주장도 철회한다.

### 커밋 2e60868 제목 철회

"The hand was never over the handle: 0.197 m from a hammer with a 0.160 m half-length" —
그 0.197 m 는 (a) `for i in range(n_go)` **뒤**에 찍히는 post-lift 표본이고,
(b) 손잡이 축이 world −y 이므로 그 거리의 x 성분 0.130 은 `POWER_X` 설계 오프셋이다.
파지 시점(f165) 값이 아니다.

### 부수 발견: 테스터 배치 실행은 첫 후보만 믿을 수 있다

`test_grasps_in_isaac.py:122,270` — `box0` 는 `sim.reset()` 직후 상태, 즉 **낙하 전
`[-0.3 0.05 0.162]`(13 cm 공중)**. 후보마다 이 상태로 되돌리므로 매번 다시 떨어지고,
그 사이 팔은 다음 후보의 pre-pose 로 블렌딩한다. px 를 깊게 준 v34 배치에서는 팔이
떨어지는 망치를 쳐서 8개 중 6개가 30~90 cm 밖 또는 173° 뒤집힌 상태로 시작했다.
→ 후보별 판정은 `--only k` 를 **프로세스마다 새로** 띄워서만 신뢰한다(1회 ≈ 19 s).

### v35: pw3 의 스크립트 그대로, PX 만

`power3_chain.sh` 를 동결 복사해 env 를 손대지 않고 `PX=0.13,0.147` 만 준다
(0.13 = pw3 자신, 같은 실행 안의 대조군). 게이트는 `right fingers at close` 가
12칸 5지 헤더로 찍히는지 먼저 확인하는 것이고, 그 다음이 `followed`.

## v35 — 손은 닫혔다. 손이 손잡이보다 21 mm 위에서 닫혔고, 그 중 20 mm 는 명령을 안 따라온 팔이다

pw3 의 생성 스크립트(scratchpad/power3_chain.sh) 환경을 그대로 쓰고 `POWER_X` 만 바꾼 8후보.
`HAND=inspire` 가 살아 있으므로 v33·v34 와 달리 판정이 유효하다. 결과 8/8 LOST (followed ±0.000 m).

**결론 먼저**
1. 5지 손은 제대로 닫힌다. 이제 이건 원인이 아니다.
2. 놓친 이유는 높이다. 검지·약지 끝이 손잡이 윗면보다 **+21 mm**, 엄지만 윗면 높이. 감싸지 못했다.
3. 그 높이 오차 31 mm 는 두 개다: **IK 가 10.3 mm 못 내려가고**, **물리가 명령보다 19.7 mm 더 높다.**
4. 물리 오차는 **접촉 전, 빈 공간에서 이미 있다.** 접촉·마찰·조임 이야기가 아니다.
5. 그래서 기하 노브(POWER_X/Y/FRACS)를 더 도는 것은 정당화되지 않는다. 노브가 움직이는 거리(10~20 mm)보다
   추종 오차(30 mm)가 크다.

**어떻게 쟀나 (그동안 못 하던 변환)**
`box-relative` 줄은 `root_pos_w` 를 빼고 찍히는데 `obj_centre()` 는 `root_pos_w + Rm @ _c_off` 라서
둘로는 절대 높이가 안 나온다. 그런데 그 줄에 `right_wrist_yaw_link` 자신이 들어 있다 —
그걸로 root 가 소거된다: `root_z = 0.094 - 0.067 = 0.027`. 후보 #1, close 프레임:

| 링크 | box-rel z | 절대 z | 손잡이 윗면 0.0377 대비 |
|---|---|---|---|
| index_intermediate | +0.032 | 0.059 | **+21 mm** |
| pinky_intermediate | +0.032 | 0.059 | **+21 mm** |
| thumb_distal | +0.008 | 0.035 | −3 mm (윗면 높이) |

**31 mm 의 분해 (cuRobo FK, 후보 #1 wrist z)**

| | 값 | 차 |
|---|---|---|
| 계획 목표 | 0.0632 | — |
| cuRobo 해의 FK | 0.0735 | **+10.3 mm** — IK 가 그만큼 못 내려간다 |
| Isaac 실측 | 0.094 | **+19.7 mm 더** — 명령한 관절값인데 거기 없다 |

계획 목표는 소스대로다: `reach_from_pose.py:534-540` 의 `T[:3,3] = centre - px*x_ax - py*y_ax`,
`y_ax=[0,0,-1]` → palm z = `(top-radius) + POWER_Y` = 0.0262+0.04 = 0.0662, wrist = palm − 0.003.
npz 의 `grasps[1]` 이 `[-0.1085 0.0192 0.0632]` 로 일치한다.

**물리 오차가 접촉 탓이 아니라는 증거**
하강 끝(frame 120, 아직 아무것도 안 닿음): FK `[-0.0663 0.0237 0.0759]` vs Isaac `[-0.098 0.002 0.104]`
= `[+31.7 +21.7 -28.1] mm`. 그리고 x 오차가 자세마다 다르다(#1 +26.7, #3 +47.7 mm) → 고정 좌표계
차이가 아니라 추종 오차다. 손목 관절 자체는 정확히 따라온다(실측 `[-0.5 0.67 -1.58]` vs 명령
`[-0.5 0.666 -1.582]`) — 오차는 손목보다 위에서 생긴다.

**철회 셋**
- v34 노트의 "box0 13 cm 재낙하로 후보가 무효": `SETTLE_IDLE=400` 이면 후보마다의 리셋은 깨끗하고
  물체는 pw3 의 배치 그대로 앉는다. 튕겨나간 망치는 계획된 `sgn +1` 접근이 쓸어낸 것이다(짝수 후보).
- 판정 게이트로 적어둔 "12칸 헤더로 손을 구분한다": 헤더 문구는 손과 무관한 고정 문자열이다.
  구분하는 것은 값의 **개수**다.
- 판정선의 "cuRobo N mm" 를 파지 잔차로 읽던 것: 그 값은 `err[n_go+n_slide-1]`, 즉 마지막 슬라이드
  프레임이다. 실제 close/hold 잔차는 11.6 / 12.3 mm 다. 앞으로 close/hold 에서 읽는다.

**테스터의 보고 버그 둘 (읽을 때 피해간다, 고치지 않는다)**
- `right fingers at close (...)` 의 괄호 안 이름은 손과 무관한 고정 문자열이다.
- `_hq = robot.data.joint_pos[0, hand_ids[7:]]` 의 7 이 하드코딩이라 24관절 Inspire 에서는 17개가
  찍히고 뒤의 `of closed` 와 어긋난다.

**다음 한 가지**
29개 관절의 명령값 대 실제값(+ Isaac 쪽 관절 한계)을 close 자세에서 찍어, 어느 관절이 안 따라오는지
또는 한계에 걸리는지를 숫자로 만든다. Fable 파일은 건드리지 않고 `test_grasps_in_isaac_opus.py` 에
`JOINT_TRACE` 출력만 추가했다(24줄, 명령하는 값은 하나도 바뀌지 않음). 아직 확인 못 한 단서:
허리가 한계 근처로 명령된다(`waist_roll` 0.49~0.51, `waist_pitch` 0.50 rad, G1 한계 ±0.52 부근),
그리고 `ARM_KP_SCALE` 은 `cfg.actuators["arms"]` 만 건드리는데 허리가 그 그룹인지 확인되지 않았다.

## 망치를 놓치는 이유는 허리다. 허리 3관절이 35.5 mm 중 32.8 mm 를 만든다

앞 절에서 "명령한 관절값인데 손목이 거기 없다"고만 적어둔 30 mm 를 끝까지 따라갔다.
결론부터.

1. **골반은 정확하다.** 파지 프레임(195)과 마지막 프레임(254) 둘 다 명령 대 실측
   위치차 0.0 mm, 자세차 0.00 deg. `fix_root_link=True` 여도 `write_root_state_to_sim` 은 먹는다.
2. **기하도 같다.** 프레임 254 에서 cuRobo FK 손목 `[-0.1092 0.0213 0.2159]` vs Isaac
   `[-0.1083 0.0222 0.2153]`, **차이 1.4 mm**. cuRobo URDF 와 Isaac Inspire USD 사이에
   링크 기하 차이는 없다.
3. **같은 로봇이 프레임 195 에서는 35.5 mm 어긋난다.** 자세 의존적이라는 뜻이고,
   원인은 그 자세에서 안 따라오는 관절이다.
4. **그 관절은 허리다.** 프레임 195 의 실측 관절값을 cuRobo FK 에 도로 넣으면:

   | 관절 | 명령 | 실측 | 차 | 이 관절 하나만 바꿨을 때 손목 이동 |
   |---|---|---|---|---|
   | waist_roll  | +0.4899 | +0.4208 | −69.2 mrad | **23.0 mm** |
   | waist_yaw   | +0.3857 | +0.4494 | +63.7 mrad | **19.9 mm** |
   | waist_pitch | +0.5047 | +0.4603 | −44.4 mrad | **10.0 mm** |
   | 다리 6관절 | — | — | 최대 1050 mrad | **0.0 mm** |

   허리 3개만 실측값으로 바꾸면 32.8 mm 가 움직이고 Isaac 실측까지 4.1 mm 남는다.
   12개를 다 바꾸면 4.1 mm. **인과가 닫혔다.**
   다리는 골반이 고정이라 손목에 아무 영향이 없다(1050 mrad 이 0.0 mm). 앞 절에서 다리 오차를
   크게 적어둔 것은 손목과는 무관한 수치다.

5. **허리는 토크 포화다.** IsaacLab `G1_29DOF_CFG` 의 `waist` 그룹은
   `stiffness 5000, damping 5, effort_limit {yaw 88, roll 50, pitch 50}`.
   kp 5000 × 69 mrad = **345 N·m 를 요구하는데 상한이 50 N·m** 다. URDF 는 더 낮은 35 N·m 다.
   즉 강성이 모자란 게 아니라 **토크 상한이 정한다** — 손 조임에서 배운 것과 같은 구조다.
   이 자세에서 상체를 그만큼 비틀어 버티려면 실제 G1 허리 모터 용량을 넘는다.

**정정 (내가 한 시간 전에 한 철회를 다시 철회한다).**
"허리가 한계에 걸린 게 원인"이라는 가설을 앞 절에서 폐기했는데, 그 근거로 쓴 측정이
**프레임 254(들어올린 뒤, 허리에 하중이 거의 없는 자세)** 였다. 내 출력이 `dofs[n_go-1]` 을
읽었고 `n_go = 255` 였다. 파지 자세에서 다시 재니 허리가 그대로 원인이다. 폐기를 취소한다.
다만 기전은 "관절 한계 클램프"가 아니라 **토크 포화**다. 그리고 내가 "한계 밖"이라고 찍은 것은
`soft_joint_pos_limits`(하드 한계 ±0.52 의 0.9배 = ±0.468)였고 PhysX 는 그걸 강제하지 않는다.
실측 0.4208 은 클램프가 아니라 못 버틴 값이다.

**계획은 이미 기다려 준다.** 손목 FK 의 프레임당 이동량은 슬라이드(120~165) 0.8~1.2 mm/frame,
**165~225 는 0.1~0.2 mm/frame(60프레임 정지)**, 리프트(225~) 4~5 mm/frame. 손가락은 165 에서
닫기 시작해 180 에 다 닫힌다. 정지 30프레임째인 195 에서도 35 mm 가 남아 있으므로
단순한 추종 지연이 아니다(정지 끝 224 프레임에서 재측정 중).

**도구.** `test_grasps_in_isaac_opus.py` 의 진단 출력을 `_jtrace(frame)` 함수로 빼서
재생 루프 안의 임의 프레임(`TRACE_FRAME`)과 기존 `n_go-1` 양쪽에서 부를 수 있게 했다.
출력만 하고 명령하는 값은 하나도 바꾸지 않는다. Fable 파일은 건드리지 않았다.

## v36 — 허리는 무른 게 아니라 토크가 없다. 그리고 그래서 계획 한계를 조여도 안 된다

### 결론

1. **허리 3관절은 파지 구간 내내 effort 상한에 붙어 있다.** 재생 프레임 120~224 에서 PhysX 가
   실제로 가한 토크는 `waist_yaw -88.0`, `waist_roll +50.0`, `waist_pitch +50.0` N·m — IsaacLab
   `G1_29DOF_CFG` 의 `effort_limit {yaw 88, roll 50, pitch 50}` 과 **정확히 같은 값**이다.
   같은 구간의 관절 속도는 |vel| < 0.5 rad/s, 즉 움직이지 않는다. 진동이 아니라 포화다.
2. **하중이 사라지는 순간 오차도 사라진다.** 리프트가 시작되는 225 직후, 프레임 230 에서
   토크는 `-3.5 / +21.6 / +2.1` N·m 로 떨어지고 추종 오차는 `+0.7 / -4.3 / -0.4` mrad 가 된다.
   프레임 240·250 도 3 mrad 안쪽이다. 허리는 고장난 게 아니고, 이 자세에서만 토크가 모자란다.
3. **포화된 관절의 실측 각도는 명령이 아니라 중력이 정한다.** 그래서 계획 모델의 허리 한계를
   조이는 것(v36)은 듣지 않았다: 명령을 78 mrad 낮췄더니 실측은 155 mrad 더 나빠졌다
   (0.4208 → 0.2653). 정적 평형이라면 설정값만 바뀔 때 실측이 움직일 수 없다.
4. **v36 은 v35 와 결과가 같다.** 후보 8개 전부 `+0.000 of 0.35 m`, 전부 LOST.
   cuRobo 계획 오차는 3.2~6.9 mm(v35 는 3.3~6.6 mm)로 사실상 동일.
   Isaac 실측 손목–손잡이 거리는 **44.1 mm → 51.2 mm 로 7.1 mm 악화**됐다.

### 내 귀속 계산을 폐기한다

앞 절에서 "허리 3관절이 35.5 mm 중 32.8 mm 를 만든다"고 썼다. 그 계산은 **한 관절씩만 실측값으로
바꿔 넣은 민감도**이고, 민감도로서는 맞다. 그러나 원인으로 쓸 수 없다. 실제로 허리 명령을 줄여
본 결과 손목은 **7.6 mm** 밖에 움직이지 않았고 방향도 반대였다. 관절 오차들은 한 해답에 대한
독립적인 교란이 아니라 전신 추종 실패이고, 계획이 바뀌면 나머지 관절도 같이 바뀐다.
"허리 32.8 mm" 는 개입으로 반증됐다.

같은 트레이스가 보여 준 전신 오차: 후보 1 의 `total |d| 14241.6 mrad / 29관절`,
`right_knee cmd +2.6097 got -0.0428` (−2652 mrad), `left_hip_yaw −2151 mrad`. 골반이 고정이라
다리 오차는 손목을 0.0 mm 움직이지만, 로봇이 명령한 자세를 **전혀** 잡고 있지 않다는 뜻이다.

### 남은 비대칭 하나 — 골반은 공짜다

`FIX_ROOT=1` 에서 루트는 `write_root_state_to_sim` 으로 직접 써진다. 측정한 모든 프레임에서
`root d pos [0. 0. 0.] mm  orientation 0.00 deg` 다. 반면 허리로 만든 자세는 위처럼 상한에 걸린다.
**루트로 사는 리치는 추종 오차가 0 이고, 허리로 사는 리치는 살 수 없다.** 다음 실험은 이 비대칭을
쓴다: 계획 모델의 허리를 ±0.25 / ±0.10 / ±0.05 로 점점 금지하고 cuRobo 자신의 파지 오차만
읽는다(솔브만, Isaac 없음). 상체를 세운 채로도 바닥 손잡이에 닿는지가 이 실험의 이진 답이다.

### 도구

`test_grasps_in_isaac_opus.py` 에 `WAIST_HISTORY` 를 추가했다. 프레임 120 부터 5프레임마다
허리 3관절의 명령·실측·속도·`applied_torque` 를 찍는다. 스냅샷 하나로는 상한에 눌린 관절과
명령 주위로 진동하는 관절을 구분할 수 없어서(waist 는 stiffness 5000 / damping 5, ζ≈0.02)
구간 전체를 읽어야 했다. 출력만 하고 명령값은 바꾸지 않는다. Fable 파일은 건드리지 않았다.
기록: `results/fable40/waisth_pw35.txt`, `waisth_pw36.txt`.

## v37 — 상체를 세우면 허리 두 관절은 풀린다. 그런데 그래도 못 잡는다. 허리는 범인이 아니다

v36의 실패 이후 남은 이진 질문은 하나였다: 바닥 손잡이를 잡는 데 허리가 정말 필요한가.
솔브 전용 스윕이 먼저 답했다 — 무제한 12.3/13.6/15.2 mm, ±0.25 rad 15.3/17.1/18.6,
**±0.10 rad 14.7/18.8/23.9 mm**. 몸통을 5배 덜 숙여도 손잡이에 닿는다. 그래서 v37은
계획 모델의 허리를 ±0.10 rad 로 묶고(`..._floor_w10.yml`) 물리에서 토크를 읽었다.

### 1. 예측대로: 몸통 기울기가 허리 yaw·pitch의 하중이었다

candidate 3, f195 — `yaw cmd +0.0983 got +0.0972 tau +5.3`, `roll cmd +0.0982 got +0.0972
tau −3.8`, `pitch cmd +0.0984 got +0.0986 tau −1.1`. 세 관절 모두 **1 mrad 안**, 토크는 상한
50/88의 1/10. v35에서 셋 다 상한에 눌려 있던 것과 정반대다.

### 2. 예측과 달리: 그 후보도 LOST 했다

허리가 완벽히 추종하고 루트도 0.0 mm 인데 파지에 실패한다. **허리 추종은 병목이 아니었다.**
v34–v36에서 허리를 쫓은 방향을 여기서 접는다.

### 3. 중력은 허리를 포화시키지 않는다 (URDF 질량으로 직접 계산)

`waist_roll` 축 둘레의 중력 모멘트를 URDF의 링크 질량·관성 원점으로 FK 계산했다:
w10 cand0 **+2.95 N·m**, pw35 cand1 **+9.88 N·m**. 상한은 50. 허리 위 질량은 14.94 kg 이고
가장 큰 단일 기여는 `torso_link` +1.94 N·m 뿐이다. 즉 +50 N·m 는 몸무게를 버티는 힘이
아니라 무언가와 싸우는 PD 항이다. 무엇과 싸우는지는 아직 측정하지 못했다 — 추정을 적지 않는다.

### 4. 진짜 실패는 기하다 — 8/8이 전부 위로 빗나간다

계획된 손목 목표 대비 실제 도달(`right_wrist_yaw_link`, 닫는 순간, mm):

| # | dx | dy | **dz** | \|d\| | # | dx | dy | **dz** | \|d\| |
|---|---|---|---|---|---|---|---|---|---|
| 0 | −50.0 | −22.7 | **+86.8** | 102.7 | 4 | −53.3 | −26.0 | **+94.1** | 111.2 |
| 1 | −27.9 | +1.7 | **+35.2** | 44.9 | 5 | −22.5 | −8.2 | **+33.2** | 40.9 |
| 2 | −51.0 | −24.1 | **+90.9** | 106.9 | 6 | −54.0 | −29.5 | **+99.5** | 117.0 |
| 3 | −7.4 | +9.6 | **+17.4** | 21.2 | 7 | −25.2 | +0.0 | **+33.4** | 41.8 |

여덟 개 전부 dz 가 +다. 한 번도 낮게 떨어지지 않는다 — 계통 오차다. 크기는 허리 오차와
맞는다: #0은 `waist_roll` 이 244 mrad 모자라고, 허리~어깨 지렛대 0.4 m × 0.244 rad ≈ 98 mm
로 측정된 +86.8 mm 와 같은 크기다. #3은 허리가 자유롭고 +17.4 mm. **허리는 후보 사이의 차이를
설명하지만, 허리를 다 풀어도 21 mm 가 남는다.**

### 5. 그래서 손가락은 손잡이를 한 번도 물지 못한다

닫힘 후 `index_intermediate` 의 상자 기준 z: `sgn=+1` 네 개(#0 −0.031, #2 −0.036, #4 −0.040,
#6 −0.039)는 손가락이 **바닥을 짚어** j1이 0.91–1.28에서 멈추고, `sgn=−1` 네 개(#1 +0.039,
#3 +0.030, #5 +0.042, #7 +0.034)는 손잡이 **위 3–4 cm 허공**을 1.47까지 완전히 쥔다.

**결론: 손잡이의 잡을 수 있는 높이 띠는 약 20 mm(밑면 바닥에서 15–18 mm, 윗면 z=0.038)인데,
허리를 다 풀어준 최선의 후보조차 실행 오차가 21 mm다. 여유가 0이다.** knob 을 아무리 쓸어도
8/8 이 실패한 이유가 이것이다.

### 철회 — "Isaac 한계 밖" 경고는 원인이 아니다

내 `_jtrace` 가 찍던 `commanded OUTSIDE Isaac's limits` 는 `soft_joint_pos_limits`
(= USD 범위 × `soft_joint_pos_limit_factor` 0.9) 와의 비교다. `articulation.py` 를 읽으면
이 값은 767–768/1662–1664 줄에서 계산·저장될 뿐 위치 목표를 자르는 데 쓰이지 않는다.
내 트레이스가 만든 허수아비이므로 원인 목록에서 뺀다.

### 도구

`scratchpad/rollmoment.py` (URDF 질량으로 허리 축 둘레 중력 모멘트를 링크별 분해),
`scratchpad/whohits.py` (명령 FK로 물체에 가장 가까운 링크 추적). 둘 다 읽기 전용이고
Fable 파일은 건드리지 않았다. 기록: `results/fable40/test_w10.txt`.

## v37 렌더 결과, 그리고 v37 결론의 철회

렌더: `[eval] end pos [-0.23 0.0782 0.1537] dxy 0.0755 m dz -0.0083 m -> LOST`. v36의 dxy
0.0152 m 보다 5배 더 밀었다. 전체뷰는 무릎 꿇은 자세는 정상이고 오른손이 망치 옆 바닥에
손가락을 편 채 놓여 있다. 물체 궤적은 f400 `[-0.281 0.040 0.150]` → f500 `[-0.226 0.067 0.148]`
인데 손가락이 닫히는 프레임은 549다. **내려오는 팔이 손잡이를 61 mm 치고 지나간 뒤에 빈 바닥을
쥐었다.**

### 철회 — "손이 계획한 표적보다 높게 떨어진다"는 실행 오차가 아니다

v37에서 잰 표(achieved − target, 8/8 dz 가 +17.4 … +99.5 mm)는 숫자로는 맞다. 그 차이를
**실행**에 돌린 것이 틀렸다. 후보 3, 손 닫는 프레임에서 실행계를 전부 재 보면:

| 무엇 | 명령 | 실제 | 차 |
|---|---|---|---|
| root | `[0.0998 0.0398 0.3246]` | 동일 | **0.0 mm, 0.00 deg** |
| 허리 3관절 | — | — | **1 mrad** |
| 오른팔 shoulder/elbow/wrist | — | — | **0.1–0.4 mrad**, 토크 0–7 N·m / 상한 300 |

처지지도 않았고 토크가 모자라지도 않았다. 그런데 **cuRobo가 자기 계획을 자기 FK로 풀면
손목 표적보다 17.2 mm 위**이고, **Isaac은 그 FK를 1.4 mm 오차로 재현**한다.

```
표적            [-0.0915  0.0193  0.0632]
cuRobo FK       [-0.0977  0.0296  0.0804]   표적 대비 [ -6.2 +10.4 +17.2] mm
Isaac 측정      [-0.0989  0.0289  0.0806]   표적 대비 [ -7.4  +9.6 +17.4] mm
Isaac − cuRobo  [ -1.2  -0.7  +0.2] mm  = 1.4 mm
```

npz 안의 cuRobo 자기 잔차(`err`)를 닫힘/유지 프레임에서 읽으면 여덟 후보 전부
**14.7 / 20.1 / 15.8 / 21.0 / 15.9 / 23.0 / 16.1 / 23.9 mm**. 손잡이의 잡을 수 있는 띠는
약 20 mm다. **계획이 이미 띠 밖이고, 손은 계획이 놓은 자리로 1.4 mm 안으로 정확히 간다.**

즉 v30–v37에서 게인·effort·close mode·허리 한계·허리 토크·POWER_* 로 갔던 조정은 전부
**잘못된 단계**를 겨눴다. 실패는 실행이 아니라 **솔버 잔차**다. (도구: `scratchpad/fkgap.py`,
`grasp/test_grasps_in_isaac_opus.py` 의 `ARM_HISTORY`.)

## v38 — 골반과 몸통만 풀어서 잔차를 깎는다

Fable의 `build()` docstring이 이 가중치의 양끝을 이미 재 놓았다: 골반을 full weight로 잡으면
손목이 17 mm 벗어나고, **전신을 0.005로 풀면 9 mm까지 들어오지만 다리가 뒤로 빠져 무릎
자세가 한 다리로 서는 자세로 무너진다.** 그래서 docstring이 "마지막 몇 cm"의 공을 돌린 두
링크(손 위로 기우는 골반과 몸통)만 `PELVIS_W=0.005`로 떼어내고 엉덩이·무릎은 `BODY_W=0.1`에
남겼다 (내 사본 `grasp/reach_from_pose_opus.py`, 새 환경변수 하나).

솔버 잔차, 유지 프레임 (mm):

| 후보 | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| v37 `BODY_W=0.1` | 14.7 | 20.1 | 15.8 | 21.0 | 15.9 | 23.0 | 16.1 | 23.9 |
| v38 `PELVIS_W=0.005` | 14.1 | 17.3 | **11.0** | 18.7 | 13.0 | 19.5 | **12.1** | 22.0 |

0.6–4.8 mm 깎였고 최선이 11.0 mm다. 전신 0.005가 냈던 9 mm에는 못 미친다. 궤적을 따라 보면
이유가 보인다: `err` 는 f15–f34 에서 0.7–2 mm 까지 내려갔다가 손이 바닥으로 내려갈수록 다시
올라 11–22 mm 에서 **평평해진다**. 접근 초반의 표적은 닿고, 파지 자세는 못 닿는다. 즉 파지
자세가 이 무릎 자세의 도달 가능 집합 가장자리에 있다. 골반·몸통을 풀어도 골반의 위치는
결국 다리가 정하는데 엉덩이·무릎은 0.1로 잡혀 있다.


## v38 결과, 그리고 진짜 원인: 자세는 한 번도 측정된 적이 없다

v38 (PELVIS_W=0.005)은 잔차를 실제로 깎았다. close 프레임, w10 -> pw38:

| # | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| w10  | 14.7 | 19.8 | 15.8 | 20.4 | 15.1 | 21.5 | 15.9 | 23.9 |
| pw38 | 12.6 | 16.0 | 11.3 | 18.5 | 11.0 | 19.1 | 11.4 | 21.8 |

그런데 8/8 LOST 그대로다. 그래서 테스터 추적을 다시 읽었고, 거기에 답이 있었다.

**물체가 한 번도 안 움직인다.** 후보 2에서 f104부터 f255까지 망치는
`[-0.288 0.093 0.030]`에 고정이고 기울기도 8.9도 그대로다. 미끄러진 게 아니다.
접촉이 아예 없다 — 손가락이 허공에서 닫힌다.

**왜 허공인가: err는 위치 전용이다.** `reach_from_pose.py:248`:

```python
err = np.linalg.norm(got - pos[:, wi], axis=1)
```

`pos`뿐이다. 자세 오차는 이 파이프라인 어디에서도 계산되지 않는다. v30부터 v38까지
내가 깎아온 14.7 -> 11.0 mm는 전부 위치 잔차였고, 자세는 아무도 안 봤다.

측정했다 (`scratchpad/orierr.py`, 계획 자체의 관절로 cuRobo FK, close 프레임):

| # | 위치 잔차 | **자세 오차** | 계획한 손가락 축 -> 실제 |
|---|---|---|---|
| 0 | 12.6 mm | **15.0 deg** | [1 0 0] -> [0.97 0.04 -0.25] |
| 2 | 11.3 mm | **12.3 deg** | [1 0 0] -> [0.98 0.04 -0.21] |
| 4 | 11.0 mm | **12.6 deg** | [1 0 0] -> [0.98 0.03 -0.21] |
| 6 | 11.4 mm | **14.0 deg** | [0.97 0.03 -0.23] |
| 1,3,5,7 | 16.0~21.8 mm | 5.4~7.2 deg | [-1 0 0] -> [-1 -0.05 -0.07] |

손가락이 12~18도 **아래로 처진다**. 손목에서 손끝까지 약 150 mm이므로
150·sin(14°) ≈ **36 mm** 손끝이 내려간다. 위치 잔차 11 mm의 세 배다.
테스터의 box-relative가 독립적으로 같은 말을 한다: close 프레임에서 손가락 링크가
물체보다 24~38 mm 아래, 29~40 mm 뒤.

**원인은 가중치이고, 오픈소스가 정답을 갖고 있다.** `build()`는 모든 링크에
`rpy=[0.067]*3`을 준다 — 위치 1.0 대비 15배 낮다. 이 숫자는 NVIDIA 것이 맞지만
**어디에 쓰는지를 봐야 한다**: `curobo/examples/getting_started/humanoid_retargeting.py:243-248`은
0.067을 **pelvis와 ankle**에 쓴다. 사람 모션 클립을 리타게팅하는 링크, 즉 위치만 따라가고
자세는 흘려보내도 되는 링크다. 같은 파일 455-460의 실제 런타임 빌더는
`pw = 100*t_weight/max` 대 `rw = 10*r_weight/max` — 10:1이다.
그리고 `ToolPoseCriteria.track_position_and_orientation`의 cuRobo 기본값은
`rpy=[1.0, 1.0, 1.0]`, 위치와 동등하다.

우리 손목은 리타게팅 대상이 아니다. **파지 자세**로 보내는 링크이고, 손바닥을 아래로
향한 채 손가락이 손잡이를 가로질러 감싸야 하므로 자세가 곧 파지다. v39는 작업 링크에만
cuRobo 기본값을 돌려준다 (`ORI_W`). 발과 몸통 링크는 0.067 그대로 둔다.

## 렌더 증거가 틀려 있었다 (실험과 별개인 계측 버그)

v38 렌더에서 망치는 `z 0.1478`, 속도 `[-0 -0 -0]`이다. 놓인 z 0.162에서 12 mm 떨어지고
PhysX가 재웠다. 테스터는 `SETTLE_IDLE=400`으로 끝까지 떨어뜨려 `z 0.032`에 앉히고,
인식이 손잡이를 찾은 z 0.026도 그쪽이다. **렌더는 계획이 겨냥한 곳보다 118 mm 위에 떠 있는
망치를 상대로 채점해 왔다.** `room_view_check.png`에 노란 망치가 바닥이 아니라 로봇 허벅지
옆 공중에 떠 있는 게 눈으로 보인다.

판정의 권위는 테스터에 있고 (테스터는 제대로 앉힌다) 테스터도 LOST이므로 결론은 안 바뀐다.
하지만 세훈님은 영상으로 원인을 찾으므로, v39부터 렌더에 `DIAG_IDLE_FRAMES`
(`play_in_cell.py:654`, 이미 있는 훅)를 걸어 물체를 앉힌 뒤 찍는다.

## WRIST_ONLY: 가중치인가 도달 한계인가

작업 링크만 1.0, 발도 1.0, 나머지 11개 몸통 링크를 1e-4로 내리고 풀었다
(발을 풀면 로봇이 통째로 목표로 이동해 버려서 아무것도 말해주지 않는다):

| # | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| w10   | 14.7 | 19.8 | 15.8 | 20.4 | 15.1 | 21.5 | 15.9 | 23.9 |
| pw38  | 12.6 | 16.0 | 11.3 | 18.5 | 11.0 | 19.1 | 11.4 | 21.8 |
| wonly |  4.3 | 10.5 |  4.3 | 12.5 |  5.2 | 12.6 |  4.1 | 14.9 |

둘 다 맞다. 몸통 가중치가 잔차의 상당 부분을 만들고 (11.0 -> 4.1 mm), 동시에 짝수 후보는
4~5 mm까지 내려가는데 홀수 후보는 10~15 mm에서 멈춘다 — 홀수(앞에서 접근) 쪽은 이 무릎
자세의 도달 한계가 남아 있다. 다만 자세 오차 36 mm가 위치 잔차 11 mm보다 크므로 v39는
자세를 먼저 고친다. 몸통 가중치는 그 다음이다.

## v39: 자세는 고쳐졌다. 그리고 내가 프레임을 잘못 읽었다 (철회)

ORI_W=1.0 은 주장한 대로 동작했다. close 프레임(f165)에서 손가락 축
`[1,0,0]` → `[1.00 0.01 -0.02]`, **12–18° → 0.6–1.9°**. 이건 유지한다.

그런데 직전 커밋에서 내가 쓴 "망치는 1 mm도 움직이지 않는다, 손가락은 허공을
쥔다"는 **틀렸다. 철회한다.** 그 숫자를 f254 에서 읽었다.

테스터는 손을 **두 번** 닫는다. 한 번은 per-frame `hands` 배열로 f165(정확),
한 번은 f254 에서 `HAND_CLOSED` 로 — 들어올린 자세의 꼭대기에서. 원인은
`reach_from_pose.py:639` 의 파워그립 savez 가 `n_go=T`(255) 를 쓰고,
테스터는 `close_from` 을 **한 번도 읽지 않는다**는 것이다(`grep close_from
test_grasps_in_isaac.py` → 0건). 그래서 `box-relative` 트레이스와 판정줄의
"cuRobo X mm" 는 둘 다 f254, 즉 손목이 손잡이보다 **145 mm 위**에 있는
자세에서 측정된 값이다. `err` 도 고정 파지점이 아니라 **프레임별 목표**까지의
거리라서(`reach_from_pose.py:248`, `pos[:, wi]`), `err[254]=4.6 mm` 는
"들어올린 자세를 4.6 mm 로 따라간다"는 뜻이다. 판정줄의 `+0.000` 역시
f254 의 `_bp0` 에서 재므로 **일어서기 구간만** 말한다. 좋아 보였던 숫자들은
파지에 대한 숫자가 아니었다.

렌더는 처음부터 옳았다: `build_reach_reference.py:209` 가 `close_from` 을
써서 549 = 384+165 에서 닫는다. 두 단계가 손 닫는 시점에 대해 한 번도
일치한 적이 없었고, 틀린 쪽은 판정 권위를 가진 테스터였다.

## 올바른 프레임에서 읽은 실제 트레이스 (v39, 8/8)

테스터가 스스로 출력하는 구간별 줄을 제 프레임에서 읽으면 실패는 **한 가지가
아니라 두 가지**이고, 접근 방향으로 정확히 갈린다.

| | 짝수 후보 (sgn=+1, 뒤에서) | 홀수 후보 (sgn=−1, 앞에서) |
|---|---|---|
| f60→f120 (하강) | tilt 6.8° → **19.5 / 28.1 / 35.3°** | tilt 6.8° → 7.0–9.0° |
| f120→f165 (슬라이드) | y **+28…+36 mm**, z **+6…+19 mm** | ≤ 3 mm |
| close 시점 물체 | 이미 밀려나 있음 | 제자리 |
| at-grasp 잔차 | 13.5–18.9 mm | 18.1–23.2 mm |
| 결과 | 파지 전에 쳐서 날린다 | 아예 닿지 않는다 |

짝수는 **접촉이 없어서가 아니라 너무 일찍 접촉해서** 실패한다. 후보 0 은
f104→f119 사이, 손목이 아직 손잡이 위 31 mm 에 있을 때 망치를 친다
(tilt 6.8→19.5°, y +8 mm, z +6 mm). 후보 4 는 f255 에 tilt 94.4°,
z 0.13 — 튕겨 날아간다.

## 원인은 이미 오픈소스에 측정되어 적혀 있었다

`reach_from_pose.py:553-554`:

> POWER_LAND (m): where the hand lands along the fingers relative to the wrap
> pose (default -POWER_SLIDE: short of it). Measured on scoop #3: **landing
> 5 cm short put the hooked fingertips ON the handle; +0.03 puts them on the
> floor beyond it.**

v39 는 `POWER_SLIDE=0.05` 에 `POWER_LAND` 를 **설정하지 않았다** → `land_off
= -0.05`, 즉 "5 cm short", 손가락이 손잡이 위에 내려앉는다고 이미 측정되어
있는 그 값이다. 내 체인 스크립트 전체를 훑으면 `POWER_LAND` 를 쓴 것은
`scoop_one.sh` 하나뿐이고 (`SLIDE=0.03 LAND=0.03 CLOSE=thumb_first`),
pw33 부터 v39 까지 전부 문서화된 실패값으로 돌았다. 프레임 단위로 측정한
타격 시점이 그 주석과 일치한다. v40 은 `POWER_LAND=0.03` 한 가지만 바꾼다.

## 기록만 하고 쫓지 않는 것: 왼다리가 명령된 무릎자세를 못 따라간다

f254 에서 `left_ankle_pitch 789 / left_hip_yaw 765 / left_hip_pitch 528 /
left_knee 407 mrad`, 29관절 합 3182 mrad, 그리고 `right_ankle_pitch,
right_ankle_roll, right_wrist_roll, right_wrist_yaw` 4개는 Isaac 한계
**밖으로** 명령된다. 실재성 문제로 남긴다. 손목 위치는 옮기지 못한다 —
테스터가 매 프레임 root 를 직접 써넣으므로 손목은 root + 허리 + 오른팔만으로
결정되고, 왼다리는 그 체인에 들어오지 않는다. cuRobo FK 와 Isaac 이
1.4 mm 로 일치한 이유가 이것이다.

## 한 파이프라인 안의 물체 자세 세 개

`scene.npy` / 테스터 배치 `[-0.300 0.050 0.162]` → 물리 안착
`[-0.295 0.096 0.032]` (Δ = [+5 +46 −130] mm) → 파지 생성기가 쓴 손잡이는
`top z 0.038`, 즉 **안착된** 높이. z 는 솔버가 이미 안착값을 쓰므로 빗맞힘의
원인이 아니고, y 46 mm 는 210 mm 손잡이의 축방향이라 관용된다. 그러나
`scene.npy` 의 z 0.162 는 렌더 증거를 118 mm 틀리게 만든 그 낡은 배치와
같은 값이다. 결론에 쓰지 않고 남겨둔다.

## 렌더의 "떠 있는 망치" 는 없었다 — 두 단계가 서로 다른 점을 측정한다

내가 v38 부터 증거 문제로 취급해 온 것, 즉 "렌더의 망치는 z 0.15 에서 공중에 잠들어
있고 테스터는 z 0.032 로 앉힌다 → 모든 영상이 118 mm 위의 물체를 상대로 채점됐다"
는 **틀렸다. 철회한다.** 두 숫자는 같은 물리 상태다.

근거는 추측이 아니라 페이블이 그 자리에 적어 둔 주석이다.
`test_grasps_in_isaac.py:90-93`:

```
# The rigid body's origin is the mesh origin, which for the lying tools sits
# 16 cm above the bottom of the mesh -- a can knocked over by the fingers
# moves that origin 10-15 cm down without going anywhere. What is measured
# below is the mesh centroid, carried with the body's pose.
_c_off = ... trimesh.load(meta["object"]["mesh"]).centroid
```

측정값:
- `hammer_flat.obj` 로컬 바운드 z −0.160 .. −0.0863, 로컬 무게중심 z **−0.1229**.
  메시 원점은 바닥면보다 160 mm 위에 있다.
- `plan_scene.py:124` 가 배치 때 직접 찍는다: `target at [-0.3 0.05 0.162] bottom z 0.002`.
  즉 원점 z 0.162 는 **망치가 바닥에 놓인 상태**로 설계된 값이다. 낡은 좌표가 아니다.
- 렌더 `play_in_cell.py:659` 는 `root_pos_w` — **원점**을 찍는다 → 0.1499.
- 테스터는 `obj_centre()` — **원점 + R·무게중심**을 찍는다 → 0.031.
- 0.1499 − 0.1229·cos(6.8°) ≈ 0.027. 테스터의 0.031 과 수 mm 안에서 일치한다.

따라서:
1. 렌더의 망치는 6 프레임 만에 12 mm 떨어져 바닥에 앉았고 속도 0 이 된 것이다.
   `vel [0. -0. 0.]` 은 콜라이더 위에서 쉬는 신호가 맞았다 — 그 콜라이더가 **바닥**이다.
2. `DIAG_IDLE_FRAMES` 는 고장나지 않은 것을 고치려 한 것이다. 실험에는 무해하므로
   v40 에서도 그대로 두지만, 증거 수정으로서는 의미가 없다.
3. `[eval] end pos [-0.1404 0.0064 0.1546] dxy 0.1655 m dz -0.0074 m` 는
   "떠 있는 망치를 쳐냈다"가 아니라 **"바닥의 망치를 165 mm 옆으로 쳐냈고 들어올리지
   못했다"** 는 뜻이다. 테스터가 프레임 단위로 본 타격·전복과 정확히 같은 이야기다.
   두 단계는 갈라지지 않았다. 일치한다.

남는 교훈은 v40 의 근거를 더 강하게 만든다: 렌더와 테스터가 독립적으로 같은 결론을
낸다 — 손이 파지 전에 망치를 때린다. `POWER_LAND` 가 그 착지점을 정하는 유일한 노브다.

## v40: `POWER_LAND=0.03` — 타격이 사라졌고, 실패가 접근 문제에서 그립 문제로 바뀌었다

오픈소스 주석이 측정해 둔 그대로 작동했다. `results/fable40/test_pl40.txt`, 프레임별.

| | v39 (land_off −0.05) | v40 (land_off +0.03) |
|---|---|---|
| 하강+슬라이드 최대 기울기 | 19.5 / 28.1 / 35.3° (짝수 후보) | **7.0 – 9.1°** (8/8 전부) |
| 슬라이드 중 물체 y 이동 | +28 … +36 mm | **≤ 5 mm** |
| 파지 전 타격 | 있음 (후보 0 은 f104~f119) | **없음** |
| solve 단계 파지 잔차 | 13.5 – 23.2 mm | 13.7 – 23.9 mm (변화 없음) |
| 방향 오차 (f165) | 0.6 – 1.9° | 0.7 – 1.8° (유지) |

그리고 처음으로 **물체가 들린다**:

```
후보 0:  f165 슬라이드 끝  object z 0.030 -> 0.041,  y 0.094 (그대로)
         f164 .. f224 (60프레임) 동안 [-0.307 0.091 0.041] 로 유지
         f239 에 z 0.032 로 떨어짐 — 그 사이 오른손 palm z 0.05 -> 0.14
후보 1:  f194 에 z 0.031 -> 0.048, f224 까지 유지, 리프트에서 떨어짐
후보 2, 3: 한 번도 닿지 않음 (물체 0.5 mm 이내로 정지)
후보 4:  홀드 구간에서 30 mm 옆으로 밀림
```

즉 손이 손잡이 **위에서 닫히고 11–16 mm 들어올려 60프레임 유지**한 뒤, 15 cm 리프트가
시작되는 순간 놓친다. v39 까지의 실패(접근 중 타격)와는 다른 실패다.
8/8 LOST 는 여전하지만, LOST 의 이유가 바뀌었다.

### v41 = `POWER_CLOSE=thumb_first` (한 가지)

같은 문장의 나머지 절반이다. `reach_from_pose.py:555`:
"POWER_CLOSE=thumb_first blocks the near side before the fingers scoop."
구현은 `reach_from_pose.py:579`:

```python
af, at = (max(0,min(1,a*2-1)), min(1,a*2)) if thumb_first else (min(1,a*2), max(0,min(1,a*2-1)))
h[_fing] = _hookq[_fing] + (_closed[_fing]-_hookq[_fing])*af   # 손가락
h[~_fing] = _hookq[~_fing] + (_closed[~_fing]-_hookq[~_fing])*at  # 엄지
```

`thumb_first` 면 엄지가 `n_close` 전반에, 손가락이 후반에 닫힌다. 기본값
`fingers_first` 는 그 반대이므로 **먼 쪽이 막히기 전에 손가락이 손잡이를 쓸고 지나간다.**
`POWER_LAND` 를 설정한 유일한 스크립트 `scoop_one.sh` 는 `CLOSE=thumb_first` 도 함께
썼다. v40 은 기본값으로 돌았다. 측정되어 작동한 레시피의 나머지 절반이다.

`POWER_SLIDE` 는 0.05 로 둔다 (scoop #3 은 0.03). 한 번에 하나만 바꾼다.

## v41: 닫는 순서는 아무것도 바꾸지 않았다 — 손가락 사이에 아무것도 없다

`POWER_CLOSE=thumb_first` 는 물리를 전혀 바꾸지 않았다.  변경이 실제로 적용됐는지는
npz 를 직접 비교해 확인했다: `q` 와 `err` 은 비트 단위로 동일하고, `hands` 만
f165~f193 (닫기 구간) 에서 다르며 엄지·손가락 램프가 `reach_from_pose.py:579` 대로
뒤바뀌어 있다.  그런데 c0 의 궤적은 f224 까지 v40 c0 과 동일하고 f239 에서만 갈라진다.

애초에 물체는 **f165, 즉 닫기 구간이 시작되기 전에** 이미 z 0.041 이다.  11 mm 상승은
손가락이 닫아서가 아니라 hook + slide 가 만든 것이다.

그래서 닫기 구간을 직접 재봤다 (`test_grasps_in_isaac_probe.py` + `FINGER_TRACE`, 읽기 전용).
대조군은 망치를 한 번도 들지 못하는 후보 2 (물체 z 0.0304 고정 — 손에 아무것도 없다):

| 프레임 | 후보 0 (망치를 11 mm 들어 올린다) | 후보 2 (대조군, 빈 손) |
|---|---|---|
| f195 | q 0.964, 명령 1.47, **506.3** mrad 부족 | q 0.964, 명령 1.47, **506.0** mrad 부족 |
| f210 | q 1.211, 258.8 | q 1.212, 259.7 |
| f224 | q 1.442, 27.7 | q 1.443, 27.2 |
| f239 | q 1.470, 8.0 | q 1.470, 8.0 |

두 열이 1 mrad 안에서 같다.  결론 두 개가 같이 나온다:

1. f195 의 506 mrad 부족은 **접촉 저항이 아니라 `HAND_KP=40` 위치 드라이브의 지연**이다.
   빈 손도 똑같이 부족하다.  (이 숫자를 접촉으로 읽었다면 그립 세기를 올렸을 것이다.)
2. 그 따름정리가 진단이다.  **망치를 쥐고 있는 후보의 손가락도 1.470 rad — 완전한
   주먹 — 까지 닫힌다.**  지름 23 mm 의 손잡이가 손가락 사이에 있었다면 불가능하다.
   손가락 사이에는 아무것도 없다.

손잡이는 스쿱에 실려 **손가락 등에 올라앉아** 있다.  hook 0.7 rad 에서 손끝은 손잡이
아래에 있고 (그게 11 mm 상승), 0.7 -> 1.47 로 더 감으면 손끝은 손잡이 아래에서 빠져나가며
뒤로 말린다.  맞은편에서 가둘 면이 없다: `POWER_Y=0.04` 이므로 손바닥면은 손잡이 축보다
40 mm 위, 반지름 0.0115 인 손잡이 윗면보다 **28.5 mm 위 허공**이다.

> `reach_from_pose.py:527-539` — `centre` = 손잡이 중심선, `T[:3,3] = centre - px*x_ax - py*y_ax`,
> `y_ax = [0,0,-1]` 이므로 `-py*y_ax` 는 `+py in z`.  `WRIST_TO_PALM = [0.0415,-0.003,0]` 의
> y 성분이 0 이므로 T 원점은 손바닥면 위에 있다 — **py 는 손바닥면부터 손잡이 축까지의 거리 그 자체**.

렌더 판정 (순변위):  v39 0.1655 -> v40 0.0821 -> v41 0.0055 m, 전부 LOST.  줄어드는 것은
망치를 밀어내지 않게 됐다는 뜻이고, 들어올림의 척도는 아니다.  파워 그립의 판정은
렌더 `[eval]` 과 테스터의 **프레임별** 트레이스뿐이다 (테스터 머리글은 f254 측정, a22b42a).

-> **v42 = `POWER_Y=0.02,0.03`** (0.04 에서 낮춤; 손잡이 윗면보다 8.5 / 18.5 mm 위 —
감싸기 안쪽이면서 solve 잔차 14~24 mm 는 남긴다).  `POWER_X` 는 0.13 만 남긴다: px 0.13 인
후보 0,1 은 v40/v41 에서 망치를 들었고 px 0.147 인 후보 2,3 은 두 판 모두 한 번도 닿지 않았다.
후보 수는 8 로 동일하다.

성공 신호는 하나다: 닫기 구간에 손가락이 1.47 에 도달하지 **못하고** 대조군 곡선에서
벗어나는 것.  그것이 손잡이가 손 안에 있다는 유일한 증거다.

---

## v42 의 판정과, 실행 오차 하나만 남기고 닫은 후보들

v42(`POWER_Y=0.02,0.03`)는 8후보 전부 LOST 다.  `cuRobo 4.9~18.8 mm`, stand-up 변위
`-0.000` 6건 / 후보1 `-0.099` / 렌더 `[eval] dxy 0.2068 dz -0.1051`.  손바닥면을 손잡이
윗면 8.5 / 18.5 mm 위까지 내려도 손 안에 손잡이가 남지 않는다.

그래서 자세 탐색을 멈추고 **실행 오차** 자체를 측정했다.  cuRobo 가 파지점에서 5~18 mm
를 보고하는데 Isaac 의 손목은 ~80 mm 떨어진 곳에 있다.  이 오차는 `right_shoulder_roll`
하나가 지고 있고, 낮은 리치 구간에서 자기 명령에서 **184 mrad** 떨어진 채 드라이브가
300 Nm 에 클램프된다.  8개 버전을 지배한 것은 파지 자세가 아니라 이것이다.

### 계측기부터 버렸다 (결론: F 와 tau 와 vel 은 쓸 수 없다)

- `applied_torque` 는 해석식이다.  `ImplicitActuator.compute()` =
  `stiffness*error_pos + damping*error_vel + joint_efforts` 를 클립한 값
  (`isaaclab/actuators/actuator_pd.py`).  `tau -300.0/300` 은
  `12000 x 0.184 > 300` 을 되풀이한 것뿐이고, **"300 Nm 하중" 판독은 순환논증이었다 — 철회한다.**
- `velocity_limit` 은 implicit actuator 에서 폐기된다 ("we continue to not use it").
  `velocity_limit=100` 은 무효이고 `effort_limit` 만 `effort_limit_sim` 으로 PhysX 에 닿는다.
- `vel` 은 f195 에서 +1.50 rad/s 를 보고하는데 위치 미분은 -0.013 rad/s 다 — 부호가
  반대이고 60배 차이.  이 저장소가 이미 기록한 PhysX 속도 쓰레기값이다.
  따라서 **"링잉 배제"도 철회한다**: 그 판정을 낸 실행은 awk 열이 잘못되어
  `shoulder_roll` 대신 `wrist_roll` 을 찍었고, 같은 프레임·같은 kp 의 깨끗한
  `d +181.5` 와 모순된다.
- `F`(incoming joint force) 는 하중 측정치가 아니다.  1100 N 은 `wrist_yaw` 에서 끝나고
  손가락에 1.4 N 도 닿지 않는다 — 끝이 1.4 N 인 체인은 1100 N 을 전달하지 못한다.
  게다가 원위 질량이 줄어드는데 F 는 바깥으로 단조증가한다
  (1032.8 -> 1047.9 -> 1061.0 -> 1075.3 -> 1089.1 -> 1091.3 -> 1103.8).
  자기일관적인 것은 `T` 뿐이고, 그것이 보여주는 것은
  `shoulder_pitch T 303.8` 인데 자기 구동은 `-48.6/300` 뿐 (즉 ~300 Nm 가 축 밖),
  바깥으로 306.1 -> 237.8 -> 191.5 -> 117.6 -> 92.3 -> 62.5 -> 손가락 0.0 감쇠 —
  `shoulder_roll` 자신의 포화된 300 Nm 가 체인을 타고 반작용하는 모양이다.

### 숫자로 닫은 후보 (전부 음성)

| 후보 | 측정 | 판정 |
|---|---|---|
| 베이스 이동 | ROOTDRIFT: f40~f430 전 구간 `drift 0.00 mm`, `jump 0.00 mm`, `vel 0.000 m/s` | 닫힘 |
| 감쇠 부족 | kd 20/120/600 -> 184.1/179.9/170.8 mrad, 구속력 1032.8/1114.3/1014.2 N | 닫힘 |
| 강성 부족 | kp 3000/12000/24000 -> 149.4/181.5/186.1 mrad | 닫힘(악화) |
| 손·바닥 접촉 | 오른손 전 링크 0.3~1.4 N, 최저 링크 `thumb_distal z +0.0194` | 닫힘 |
| 팔-다리 접촉 | SEG_GAP 전 구간 최소 301 mm; f180~f220 은 346~365 mm 로 오히려 벌어짐 | 닫힘 |
| 관절 한계 | 하드 [-2.2515 +1.5882], 소프트 [-2.0595 +1.3962], 명령 -0.867 | 닫힘(1.19 rad 안쪽) |
| 관절 마찰 | `joint_friction_coeff 0.00000`, `joint_friction 0.00000`, `joint_armature 0.00100` | 닫힘 |
| 자기충돌 | `enabled_self_collisions=False` | 닫힘 |
| 질량/관성 | f0 `shoulder_pitch F 34.7 N` = 3.54 kg x 9.81 정확히 | 정상 |
| 인덱스 오매핑 | `_ai = [names.index(n) for n in _an]`, `_aj = [body_ids[j] for j in _ai]` — cmd/got 모두 shoulder_roll | 닫힘 |
| 속도 추종 지연 | cmd 가 f180 -0.8506 -> f210 -0.8722 = 0.022 rad/s, kp 12000 은 ~4e-5 rad 만 뒤짐 | 닫힘 |

ROOTDRIFT 의 부수 결론 둘: `pelvis F 0.0` 은 하중이 없다는 증거가 아니라 **구조적**이다
(운동학 루트에는 들어오는 관절이 없다).  그리고 `jump 0.00` 은 리치 동안 root7 이 상수라는
뜻이므로 플래너의 `pelvis z 0.340..0.440` 은 참조값이고 테스터가 재생하는 값이 아니다.

### 남은 하나: effort 상한

```
eff  300 -> 181.5 mrad
eff 1000 ->  76.4 mrad
eff 4000 ->  75.5 mrad   (요구 -901 Nm, 클램프 없음 -> 1000 에서 효용이 끝난다)
```
105 mrad ≈ 47 mm, 80 mm 실행 오차의 절반이 넘는다.  **다만 이것은 물리적 해결이 아니다.**
`g1_29dof_rev_1_0.urdf` 의 정격은 `effort="25"` 이므로 기본값 300 도 이미 12배, 1000 은 40배다.
측정된 부작용: 상한을 올리면 `shoulder_pitch` 자신의 오차가 9.6 -> 21.3 mrad 로 나빠진다.
간극이 이것으로 닫히면 답은 40배를 쓰는 것이 아니라 **25 Nm 가 버티는 자세로 다시 계획**하는 것이다.

-> **v43 = `ARM_EFFORT=1000`**, 자세는 v42 그대로 재사용(재solve 없음).
렌더는 knob 하나 때문에 `play_in_cell_opus.py` 로 가지만 교란은 없다:
`SOLVER_IT/SOLVER_VIT` 기본값 12/4 는 Fable 의 `play_in_cell.py:164-167`, `:328-330`
하드코딩과 동일하고, 다른 `_opus` 추가분은 전부 env 게이트 + 기본 off 다.

### 오픈소스와의 차이는 차이로만 적는다 (원인으로 적지 않는다)

- 팔 감쇠: GraspGenX `end2end/robots/g1_inspire_arm.yaml` 은 arm_kp 2000 / arm_kd 100
  (kd/kp 0.05), 우리는 20/12000 (0.0017) — 30배 적다.  단 위 표대로 **오차의 원인은 아니다**
  (그리고 GraspGenX 쪽은 베이스 고정된 팔 단독이다).
- 손: 우리는 `CLOSE_MODE=position HAND_KP=40`, 오픈소스는 `gripper_control_mode: velocity`,
  `FINGER_KP_DEFAULT 2000`, `FINGER_KD_DEFAULT 200`, `finger_effort_limit 1000`.

## v43 의 판정: 상한을 올려도 파지는 생기지 않는다

`ARM_EFFORT=1000`, 자세는 v42 재사용. 8후보 전부 `+0.000 of 0.35 m` -> LOST.
렌더도 같다: `[eval] end pos [-0.3263 0.0486 0.1536] dxy 0.0264 m dz -0.0083 m -> LOST`,
`[obj ] idle 119 pos [-0.2808 0.0402 0.1499] vel [0. -0. 0.]`.
cuRobo 잔차는 v42 와 동일(4.9~18.8 mm) — 자세를 다시 풀지 않았으니 당연하다.
**관절에서 벌어준 47 mm 는 파지로 환산되지 않는다.**

플링은 계통적으로 없어지지도 않았다. f255 물체 기준:
c0 `-0.954` tilt 85.8deg -> `-0.291` tilt 8.1deg (좋아짐), c1 tilt 54.6 -> 8.1 (좋아짐),
c2~c4 거의 그대로, **c5 tilt 7.8 -> 165.4deg (나빠짐)**. 둘 줄이고 하나 만들었다.

### 내가 v43 에서 틀린 두 가지

1. **"손가락이 멈췄으니 손잡이가 사이에 있다"** — 성립하지 않는다. eff 300 인 v42 에서도
   후보 1/4/6/7 이 이미 1.09~1.36 에서 멈췄고, **후보 4 는 손가락이 망치보다 249 mm 위에
   있는 상태로 멈췄다**. 멈춤은 손가락끼리 또는 손바닥에 걸려도 생긴다. `ARM_EFFORT` 가 바꾼 것은
   *어느 후보가* 멈추느냐이지 멈춤의 유무가 아니다(c0 1.47->1.34, c2 1.47->1.32, c3 ->1.46, c5 ->1.43).
   후보 하나를 자기 자신과만 비교하고 나머지 일곱을 보지 않은 잘못이다.
2. **`box-relative` 를 닫는 순간의 값으로 읽은 것** — `test_grasps_in_isaac_opus.py:570` 의
   `_bp = box.data.root_pos_w[0]` 는 물체의 *현재* 위치이고, 출력 순서상 `after lift (frame 255)`
   뒤에 찍힌다. v42 의 `[0.669 ...]` 는 들어올리는 동안 망치가 쓸려 나간 뒤의 값이다.

### 닫힌 가설 (원본 숫자로)

| 가설 | 근거 | 판정 |
|---|---|---|
| 물체가 너무 가볍다 | 우리 `plan_scene.py:83` MassAttr(0.2) = GraspGenX `DEFAULT_OBJECT_MASS = 0.2` | 차이 없음 |
| 손가락 마찰이 낮다 | 우리 `plan_scene.py:28 FINGER_MU=3.0` + `combine_mode="max"` = 그쪽 `DEFAULT_FINGER_MU = 3.0` | 차이 없음 |
| 렌더가 떠 있는 물체를 채점한다 | `hammer_flat.obj` 원점이 메시 바닥보다 160 mm 위. 0.1499 - 0.1229*cos(6.8deg) = 0.027 vs 테스터 0.031 | 닫힘 (684fdf2) |

그래서 출처가 있는 남은 차이는 **솔버 하나**다. GraspGenX `end2end/dynamic_playback.py`:
`SOLVER_ITERATIONS = 100`, `SOLVER_LS_ITERATIONS = 50`, `SOLVER_IMPRATIO = 1000.0`,
`COLLIDE_SUBSTEPS = 4`, 주석 그대로 *"With only 10 iterations the constraint solver doesn't
fully converge and grasps slip during the lift segment."* 우리는 12/4 다.
정직하게 덧붙일 단서: PhysX 의 `solver_position_iteration_count`/`velocity_iteration_count` 는
Newton/MuJoCo 의 solver iteration 과 **가장 가까운 대응물이지 같은 양이 아니다**.

### 그 전에 필요한 것은 knob 이 아니라 계기다

손가락이 망치에 닿기는 하는지를 재는 것이 이 실행 어디에도 없다. 멈춤은 위와 같이 애매하고,
물체 위치는 밀린 *뒤*만 보여준다. `play_in_cell_opus.py:461-468` 이 IsaacLab 자신의
덱스터러스 과제(`dexsuite_kuka_allegro_env_cfg.py:43-56`, `mdp/rewards.py:50-71`, 임계값 `:111`)를
따른다: 손끝마다 ContactSensor 를 물체로 필터링하고, **엄지 > 1.0 N 이고 마주보는 손가락 하나 > 1.0 N**
일 때 파지로 친다(관측은 20 N 에서 클립, "contact force in finger tips is under 20N normally").
우리 파일의 주석이 말한다: *"We have never measured this number."*

### articulation 실패: 원인 미확인 (앞선 표 2개 철회)

**결론부터: 원인을 모른다.** 내가 앞서 이 절에 쓴 표
("다른 Isaac 있음/없음 × CONTACT_FORCE 있음/없음")는 **철회한다.**
`CONTACT_FORCE` 를 **끈** 대조군이 같은 자리에서 똑같이 죽었다
(`cf_ctrl.log`: `artic=2 mimic=12`). 따라서 `CONTACT_FORCE` 는 변수가 아니다.

측정된 사실만 적는다.

| 시각 | 실행 | mimic 오류 | 결과 |
|---|---|---|---|
| 08:21:47 | ae43c0 (v43 렌더) | 0 | 성공, `[eval]` 까지 |
| 08:22:09 | contact43 (CONTACT_FORCE=1, IsaacLab 헬퍼) | 12 | 실패 |
| 08:33:09 | cf_A (리포트 API만, CF_SENSOR=0) | 12 | 실패 |
| 08:39:37 | cf_ctrl (**CONTACT_FORCE 없음**) | 12 | 실패 |

`cf_ctrl` 은 08:21 에 성공한 그 실행과 **실제로 실행되는 코드가 같다.**
백업본과의 `diff` 가 보여주는 차이는 전부 `if os.environ.get("CONTACT_FORCE")` 안이나
`_force_line` 본문 안에 있고, 플래그를 끄면 그중 어느 줄도 실행되지 않는다.

폐기한 설명 네 개(전부 내 것이다):
`--no-video` 탓 / 동시 실행 탓(종료 단계에서 멈춘 pid 포함) /
IsaacLab 2.3.2 `schemas.py:551-554` 가 자식 대신 루트에 `sleepThreshold` 를 쓰는 버그 /
`CONTACT_FORCE` 자체. 세 번째는 `contact43d` 가 부정했다 — `R_` 12개 바디에만 API 를
붙였는데 12건의 mimic 오류에 **내가 건드리지 않은 `L_` 관절이 포함**돼 있었다.

**새로 찾은 단 하나의 구별자.** 실패한 실행에는 성공한 실행에 없는 경고가 먼저 뜬다:

```
[9,175ms] [Warning] PhysicsUSD: CreateJoint - found a joint with disjointed
          body transforms ... : /World/G1/root_joint
[9,175ms] [Error]   Usd Physics: failed to find internal joint object for
          PhysxMimicJointAPI at /World/G1/joints/L_index_intermediate_joint
```

12건의 mimic 오류는 이 경고와 **같은 밀리초**에 이어진다. `ae43c0.log` 에는
`disjointed body` 가 0건, 실패한 세 로그에는 1건씩이다. root_joint 가 먼저 어긋나고
mimic 관절이 소속 articulation 을 못 찾는 순서로 읽힌다 — 다만 root_joint 가 왜
어긋나는지는 아직 측정하지 못했다.

루트 프림에 찍힌 속성으로는 설명되지 않는다. `dump_physics` 출력이
`cf_ctrl` 은 `[phys] /World/G1 [Xform] schemas=[]:` (성공한 `ae43c0` 와 동일),
`contact43` 만 `sleepThreshold=0.0` 이다. 즉 루트가 깨끗한 실행도 죽는다.

배제한 환경 요인: IsaacLab `source/` 의 `.py` 중 12시간 내 수정된 파일 0개,
`assets/g1_inspire/g1_29dof_inspire_hand.usd` 는 09-28 11:48 그대로,
`~/.cache/ov`·`~/.nv/ComputeCache` 는 08:15 이후 변경 0건, GPU 5843/16303 MiB.

**원인을 찾았다: 내가 떨어뜨린 환경변수 `FIX_ROOT=1`.**

먼저 내 파일을 무혐의로 만들었다. ae43c0 를 만든 바로 그 백업 바이트
(`play_in_cell_opus.py.bak`)를 같은 명령으로 돌렸더니 **똑같이 죽었다**
(`bak_ab.log`: `mimic=12 disjoint=1`). 파일이 아니면 실행 환경이다.

성공한 렌더를 만든 체인 `ae43_chain.sh:45` 이 그 답을 갖고 있었다:

```
export FIX_ROOT=1 ARM_KP_SCALE=4 CLOSE_MODE=position PD_BODY=1 HAND_KP=40 \
       OBJECT_NO_SLEEP=1 SETTLE_IDLE=400 TEST_VERBOSE=1 TRACE_FRAME=195 ARM_EFFORT=1000
```

`FIX_ROOT` 는 `play_in_cell_opus.py:441` 에서
`ArticulationRootPropertiesCfg(..., fix_root_link=(os.environ.get("FIX_ROOT","0")=="1"))`
로 들어간다. 접촉 계측용으로 내가 새로 쓴 여섯 개 스크립트는 env 블록을 짧게 다시
쓰면서 이 export 를 전부 빠뜨렸다. 즉 `fix_root_link=True` -> `False` 가 됐고,
그래서 실패한 로그마다 `/World/G1/root_joint` 의 `disjointed body transforms` 경고가
먼저 뜨고 12건의 mimic 오류가 같은 밀리초에 따라붙는다.

교훈은 진단 기법 쪽이다. 여섯 번 동안 나는 **바뀐 코드**만 비교했고
**바뀐 env** 는 비교하지 않았다. 성공한 실행의 명령줄과 env 를 먼저 복원해
나란히 놓았어야 했다. 관련: [[diff-two-runs-that-should-agree]]

검증 중(`contact44`): `contact43c` 와 모든 것이 같고 `FIX_ROOT=1` 만 되돌린 실행.
예측은 `sim.reset()` 통과 + `[force]` 줄 출현이다.

### 아직 측정하지 못한 것: 손가락이 망치에 닿는가

`play_in_cell_opus.py:461-468` 이 인용하는 dexsuite 기준(엄지 > 1.0 N **및** 마주보는
손가락 하나 > 1.0 N, 관측 20 N 클립; `dexsuite_kuka_allegro_env_cfg.py:43-56`,
`mdp/rewards.py:50-71`, 임계값 `:111`)은 여섯 번 시도해 한 번도 얻지 못했다.
접촉 리포트가 계속 막히면 **접촉 센서가 전혀 필요 없는 대체 계측**으로 간다:
`robot.data.body_pos_w` 의 손끝 링크 위치와 물체 포즈로 손끝-망치 최소거리를 mm 로 찍는 것.
닿지도 않는다면 문제는 기하/배치이고 solver 반복수는 틀린 레버다.

## 측정했다: 손가락은 망치에 닿지 않는다 (contact44)

`FIX_ROOT=1` 을 되돌린 실행이 통과했다 — `exit 0`, `artic=0 mimic=0 disjoint=0`,
1054 줄, `[force]` 135 프레임. 여섯 번 실패한 계측이 열렸다.

**계기를 먼저 검증했다.** 접촉이 없는 모든 프레임이 `net 1.96 N / sum 0.00 N` 을
읽는다. 1.96 N = 0.2 kg x 9.81, 정지한 `/World/FloorSlab` 을 통해 전달되는 망치
자체의 무게다. 즉 0 이 찍힌 줄은 센서가 죽은 것이 아니라 진짜로 닿지 않은 것이다.

**힘을 받는 링크는 엄지뿐이다.** 헤더줄을 제외한 전 구간 히스토그램:

| 링크 | 프레임 |
|---|---|
| `R_thumb_proximal_base` | 20 |
| `R_thumb_proximal` | 19 |
| `R_thumb_intermediate` | 5 |
| index / middle / ring / pinky | **0** |

`best-opposing` 은 135 프레임 **전부** 0.00 N, `dexsuite_good=True` 는 0 회.
dexsuite 기준이 실패한 항은 크기가 아니라 **마주보는 항이 항등적으로 0** 이라는 것이다.

**그 접촉은 파지가 아니라 타격이다.** `frame 520 net 5390.46 N sum 7219.59 N
max 4269.82 N` — 0.2 kg 물체에 4270 N 이 `R_thumb_proximal_base` 하나에 걸린다.
망치는 밀린다: centroid `[-0.295 0.095 0.031]` (f<=460) -> `[-0.343 0.092 0.042]`
(620) -> `[-0.333 0.099 0.032]` (665). -x 로 약 4.8 cm 밀고 제자리에 눕는다.

**손가락은 허공에서 주먹을 쥔다.** `frame 660` 에서 오른손
`q [1.47 1.47 1.47 1.47 1.3 0.5 1.47 1.47 1.47 1.47 0.8 1.2]` 가 target 과
소수점까지 같다. 힘이 한 번도 걸리지 않은 완전한 주먹이다. 무언가가 사이에 있을
때(테스터 기록)는 index 가 target 1.47 에 대해 1.34/1.27 에서 멈춘다. 따라서
**제어 추종은 원인이 아니다.** v43 이 고친 effort 상한(181.5 -> 76.4 mrad)도
이 실패와는 무관하다.

**새 정보는 타이밍이다: 엄지가 착지보다 39 프레임 먼저 닿는다.**
클립 `ae43c0.pkl` = `384 walk+kneel + 255 reach + 30 hold = 669`, hands close 549,
lift 609. reach 구간 위상(`APPROACH_FRAMES=60, GRASP_FRAMES=60, n_slide=45,
n_close=30, n_hold=30, n_lift=30` = 255, +384):

| 구간 | 프레임 |
|---|---|
| pre | 384-444 |
| **descent** | **444-504** (504 착지) |
| slide | 504-549 |
| close | 549-579 |
| hold | 579-609 |
| lift | 609-639 |

접촉 개시는 **465** — 착지(504)보다 39 프레임(1.3 s) 이르다. 종료는 620, 즉
close 와 lift 구간에는 이미 힘이 없다.

**기하가 그 타이밍을 설명한다.** 테스터의 box-relative(=
`robot.data.body_pos_w - box.data.root_pos_w`, 월드축, 회전 없음:
`test_grasps_in_isaac_opus.py:571-580`) 로 v43 의 8 후보 전부에서 엄지끝이 손의
최하점이다 — 너클보다 아래로 `c0 15.0 / c1 46.0 / c2 14.0 / c3 25.0 / c4 16.0 /
c5 31.0 / c6 16.0 / c7 29.0` mm. 한 후보의 우연이 아니라 자세의 성질이다.

원인은 `reach_from_pose_opus.py:608` 한 줄이다:

```python
_hookq = _open.copy(); _hookq[[0, 1, 2, 3]] = hook; _hookq[[6, 7, 8, 9]] = 1.064 * hook - 0.045
```

엄지 슬롯은 4, 5, 10, 11 인데 `_hookq` 는 그 어느 것도 건드리지 않는다. 그래서
12 cm 하강 전 구간(617-623 줄)에서 네 손가락은 `POWER_HOOK=0.7` 로 말려 있고
엄지만 완전히 펴진 채 아래로 튀어나와 있다.

`py42_chain.sh` 헤더가 기하만으로 세웠던 "손잡이를 마주보는 것이 없다" 가설을
이 힘 기록이 처음으로 **직접 확인**했고, 타이밍을 덧붙였다. 동시에 그 가설을
따라 돌린 `POWER_Y` 0.04 -> 0.02,0.03 스윕이 이것을 고치지 못했음도 확인했다.

### 읽지 말아야 할 숫자

`[near]` 의 cm 는 물체 **메시 무게중심**까지의 거리다(`_cen = _op + _Rm @ _c_off`).
망치의 무게중심은 머리 쪽이고 손잡이가 아니다. 따라서 `[near] 12.9 cm` 같은 값은
파지 정렬을 측정하지 않는다. `[abs]` 는 강체 원점(z 0.15), `[near]` 는 무게중심
(z 0.031) — 12 cm 차이. 커밋 `684fdf2` 에서 이미 닫힌 구분이다.

`--walk-only` 도 함정이다. `play_in_cell_opus.py:1257` 이
`"walk-only: 669 frames, stopping before the pick"` 을 찍고 `os._exit(0)` 하지만,
`:949` 의 주석대로 `--clip-arms` 는 *"클립이 팔까지 담고 있다(cuRobo 리타게터가
푼 전신 리치)"* 는 뜻이다. reach/close/lift 는 클립의 관절각에 구워져 있고 손가락
스케줄은 `--hands ae43c0_hands.npy (669, 24)` 가 준다. `--walk-only` 가 건너뛰는
것은 *추가* 스크립트 픽 단계뿐이다. **contact44 는 파지를 실행했다.**

### 닫힌 레버 (출처 확인, 다시 손대지 말 것)

| 레버 | 근거 |
|---|---|
| solver iterations | GraspGenX `end2end/dynamic_playback.py:94-99` 의 목적은 *"10 회로는 수렴하지 않아 lift 구간에서 파지가 미끄러진다"*. 우리는 미끄러질 파지가 없다(opposing 0 N). 12/4 유지. |
| 마찰 | 그쪽 `:88-90` `DEFAULT_OBJECT_MASS 0.2 / OBJECT_MU 10.0 / FINGER_MU 3.0`. 우리 `plan_scene.py:27-28`, `:83` 과 동일. 이미 일치 = 바꿀 것이 아니다. |
| `POWER_X` | `reach_from_pose.py:516-523` 의 측정값(너클 x 0.127)에 이미 도달. 0.13 x9, 0.10/0.127 x4, 0.147/0.163 x1. |
| `POWER_Y` | 0.04 -> 0.02,0.03 스윕 후에도 opposing 0.00 N (위). |
| `POWER_CLOSE` | `thumb_first` 는 물리를 바꾸지 않았다(v41). |
| `ARM_EFFORT` | 184 -> 76 mrad 로 실행 오차는 줄였지만(v43) 이 실패와 무관하다(위). |

이 문서가 예고한 대로였다: *"닿지도 않는다면 문제는 기하/배치이고 solver
반복수는 틀린 레버다."* 닿지 않았고, 틀린 레버였다.

### v44 의 한 가지 변경

`reach_from_pose_opus.py` 에 `POWER_THUMB_HOOK` (기본 0.0 = 무변경)을 넣어 하강·슬라이드
구간에서 엄지 슬롯 5/10/11 을 `_closed` 의 그 비율만큼 말아 둔다(슬롯 4 는 외전,
`_open`/`_closed` 둘 다 1.308 이라 제외). v44 = 0.5. 나머지는 v42 해 + v43 의
`ARM_EFFORT=1000` 그대로, 렌더에 `CONTACT_FORCE=1` 유지.

판정 기준을 결과 전에 적어 두었다(`v44/note.txt`): 통과는 box-relative 에서
`thumb_distal z >= index_intermediate z` 이고 렌더 `[force]` 에 엄지가 아닌 링크가
등장하거나 `best-opposing > 0 N`. 엄지가 여전히 최하점이면 다음 변경은 더 큰
`POWER_THUMB_HOOK`. 돌출이 사라졌는데도 opposing 이 0 이면 원인은 엄지 높이가
아니므로 손잡이가 손바닥 평면에 들어오는지를 다시 봐야 한다.

## 260929 v45: the waist ceiling was real and it was not the cause; the hand is in the floor

Pre-registered two criteria, got a split verdict.

1. Torque ceiling: PASS. `WAIST_EFFORT=1000` -> tau samples 648, pinned at >=990 Nm:
   0 (0%), max |tau| 348.3 Nm. v44 sat at yaw -88.0 / roll +50.0 / pitch +50.0 on
   nearly every frame f120-f225 (`waisth.log`), which is IsaacLab's stock waist
   `effort_limit` (isaaclab_assets/robots/unitree.py:478-521) to the decimal. The
   ceiling was binding.
2. Position error following: FAIL. waist_roll 65.4 -> 65.0 mrad with 6.5x the torque.
   waist_yaw 92.6 -> 47.8. So the ceiling was not what held the position. `effort`
   is a closed lever now, alongside ARM_EFFORT, solver iterations, friction/mass,
   POWER_X, POWER_Y(plan-side cap) and POWER_CLOSE.

My stated ground for v45 was overstated: `reach_th44.npz` already caps the waist at
+-0.098 rad (the `_w10` retarget config), so the waist was barely being asked to reach.

### The candidate-selection bug: 12 renders of the worst candidate

`BEST=$(awk '/candidate/{c=$3} /-> HELD/{print c;exit}')` with `[ -z "$BEST" ] && BEST=0`.
Nothing has ever HELD, so v33-v45 rendered c0 every time. Measured at the close pose
(handle position in palm-frame x; design value 130 mm from POWER_X=0.13, thumb root
69.1 mm, index knuckle 136.5 mm, both measured from our own USD):

| cand | wrist err | palm-x | reading |
|---|---|---|---|
| c0 | 70.2 mm | 82.8 mm | shallow (what we rendered 12 times) |
| c1 | 29.2 mm | 178.3 mm | mirror wrap |
| c2 | 29.4 mm | 122.8 mm | inside the finger pocket |
| c3 | 26.0 mm | 180.1 mm | mirror wrap |
| c4 | 47.3 mm | 104.6 mm | inside the finger pocket |
| c5 | 28.2 mm | 178.0 mm | mirror wrap |

### Two settings silently off since v33

- `FINGER_COACD` unset -> `play_in_cell_opus.py:581` never ran, so the finger
  intermediate/distal colliders stayed convex hulls for all 12 renders. Its own print
  cites GraspGenX's `coacd_link_keywords`; v27-v32 had it on. Same failure class as the
  object collider that was a convex hull.
- The hand-gain block at `play_in_cell_opus.py:173` is gated on `HAND_KD` being set.
  No run since v33 set it, so the fingers ran at the default kp 10 / kd 0.2 / effort 30
  (build_reach_reference.py:102). Note the block replaces only `damping` and
  `effort_limit`, never `stiffness` -- so it is not a kp fix either.

### What the trace actually says: the hand closes 9 mm below the handle top

A print I misread first: the tester's `at grasp: palm z +0.188 ... palm - object
[.. +0.156]` is not the grasp pose. Pulling the wrist-z curve out of the render shows
0.189 is the post-lift value (f640-665), and `PALM_LINK["right"]` for inspire is
`right_wrist_yaw_link` (build_reach_reference.py:44).

The curve gives the real number, and it points the other way:

    f495 0.070 -> f505 0.054 (lands 504) -> f545 0.037 -> f550 0.029 (closes 549)
    -> 0.030 held to f605 -> lifts f610

The plan's target palm z is 0.063 (POWER_Y 0.035 above the handle). The handle centre
is at 0.031, its top at 0.038. So at the close the wrist is 34 mm below target and 9 mm
below the handle top: the 35 mm of clearance the plan reserved is entirely consumed by
vertical execution error. With a palm-down power grip that puts the curling fingers into
the floor, not around the handle, which is one cause for three symptoms:

- non-thumb links register force in 0 frames; best-opposing 0.00 N (v43, v44, v45)
- the four master finger joints sit 0.68 rad (39 deg) short of their 1.47 rad target
  through f565-580 with nothing between them
- 1787-4270 N continuous from f470 to f620 -- a five-second jam, not a strike

### Queued

- v46 (running): v27-v32's physics restored (FINGER_COACD=intermediate,distal,
  CLOSE_MODE=velocity, CLOSE_VEL=0.25, CLOSE_KD=40, HAND_VEL=5.0, MIMIC_URDF_RATIO=1,
  SOLVER_IT=100/50, OBJ_MAX_DEPEN_VEL=5) + WAIST_EFFORT=1000, rendering c2 not c0.
  Criterion: does a non-thumb link register force, or best-opposing leave 0.00 N.
- v47 (queued): POWER_Y 0.035 -> 0.069, pre-compensating the measured 34 mm.

## 260929 v46: the render stalled at frame 70; the only chain that ever raised solver iterations

The v46 render produced no verdict. It is worth writing down because the failure was
in my own chain, not in the grasp.

Measured:

    render start        10:02:52  (pid 1579885)
    last log write      10:04:38  = frame 70 of 669, 41191 bytes
    checked             10:22:01  -> 1043 s with no log growth
    process             ELAPSED 19:07, CPU TIME 24:28, 127% CPU, STAT Rl, no WCHAN
    GPU                 2808 MiB, 47% utilisation

Frames 0-70 came out in about 106 s -- normal speed -- and then the process kept
burning CPU without emitting a single frame. So it is neither deadlocked nor
uniformly slow; it stopped at one point while still computing. My first reading
("0.097 frames/s, 103 min for the remaining 599") assumed uniform slowness and was
wrong; the log mtime refutes that assumption.

What this render alone had:

    play_in_cell_opus.py:254-255   SOLVER_IT defaults to 12, SOLVER_VIT to 4
    mix46_chain.sh:71-75 exports at script scope, so the render at :102 inherited
    SOLVER_IT=100 SOLVER_VIT=50 -- 8.3x the position iterations, 12.5x the velocity
    iterations.

Every render that has ever finished -- 13 of them, v33 through v45 including th44 and
wf45 -- left SOLVER_IT unset. Grepping every chain still in the scratchpad, mix46 is
the only one that set it. Those 13 finished 669 frames in 7.5-8 min.

100/50 is not in itself fatal: the same setting ran the tester at 76 s per candidate
(10:00:18 -> 10:01:36 -> 10:02:52). All that is established is that it broke the
render, not why.

Action: killed by pid at the 25-min threshold, then re-ran the render with the plan,
the candidate, the pkl and every other physics setting untouched and only the solver
iterations back at the default 12/4 ($S/mix46b_chain.sh). The pre-registered
criterion requires this anyway -- it compares against "0 in all 12 previous renders",
so the solver condition has to match the renders it is compared with. The v47 chain
carried the same bug by construction (it was sed-derived from mix46); it was killed
in its wait loop and requeued with SOLVER_IT confined to the tester command line.

The tester half of v46 did complete and stands:

    c2  cuRobo 18.0 mm  slipped in the stand-up (-0.000 of 0.35 m)  -> LOST
    c4  cuRobo 18.8 mm  slipped in the stand-up (-0.000 of 0.35 m)  -> LOST

and FINGER_COACD fired for the first time since v32, on both candidates:
"finger colliders -> convexDecomposition on ('intermediate', 'distal'): 6 meshes
(GraspGenX coacd_link_keywords)".

## The wrap pose is inside the hammer; the weak solver was hiding it (09-29 11:02)

A single-variable A/B settles the 60 mm. Same plan file
(`results/fable40/reach_mix46.npz`, candidate 2), the mix46 tester environment
copied verbatim, `TRACE_FRAME=195`, and only `SOLVER_IT`/`SOLVER_VIT` changed:

    SOLVER_IT   right_wrist_yaw_link world   total |d|     object after the close
    12 / 4      [-0.3832  0.0092  0.0396]    5351.3 mrad   [-0.304 0.095]    7.9 deg
    24 / 12     [-0.3853  0.0028  0.0417]    5571.7 mrad   [-0.306 0.091]    6.9 deg
    48 / 24     [-0.3684 -0.0015  0.0528]    3524.0 mrad   [-0.312 0.096]   15.6 deg
    72 / 36     [-0.3845  0.0017  0.0536]    5407.2 mrad   [-0.310 0.091]   12.8 deg
    100 / 50    [-0.4469  0.0036  0.0700]    3396.5 mrad   [-0.526 0.081]  172.9 deg

The plan asks for wrist world x -0.441 (FK of `reach_th44.npz` candidate 2). At
12-72 iterations the wrist stops 64 mm short and the hammer does not move: the
contact stands the hand up and it reads as a tracking error. At 100/50 the wrist
reaches the commanded pose to 5.9 mm, and in the same 30 frames the close throws
the hammer from x -0.290 to -0.526 (236 mm) and flips it to 172.9 deg.

So the thirteen renders did not fail because the hand was 60 mm short. They
failed because the commanded wrap pose intersects the handle, and a solver at
12/4 does not converge hard enough to show it -- it lets the hand stall against
the object instead of ejecting it. GraspGenX says the same thing about its own
numbers (`end2end/dynamic_playback.py:97-98`, SOLVER_ITERATIONS=100,
SOLVER_LS_ITERATIONS=50: "With only 10 iterations the constraint solver doesn't
fully converge and grasps slip during the lift segment").

Corrections to what I wrote earlier today:

* The v46 note's "the remaining cause is vertical position" was wrong, and so is
  its successor "the cause is the solver iteration count". The iteration count is
  what *reveals* the cause; the cause is the pose.
* The renders ran at 12/4 because `$S/mix46b_chain.sh:47` dropped `SOLVER_IT`
  deliberately, to match the twelve renders the pre-registered criterion compared
  against. That was my decision, not an accident.
* The floor wrap is not something I invented against the open source:
  `grasp/reach_from_pose_opus.py:477` records it as Sehoon's instruction after
  GraspGen-X's fingertip pinches slipped. What it lacks is any check that the
  pose clears the object. GraspGenX's own filter
  (`graspgenx/utils/collision_filter.py`) would not have caught this either --
  it removes the target's points before testing, by design.

Raw: `results/fable40/ab_{lo12,it24,it48,it72,hi100}.txt`, `$S/ab_solver.sh`,
`$S/ladder.sh`.

### v47: the pinch off the floor, the wrap in the air

Two measurements already in the repository point at one path:

* `results/fable40/verify_138.txt` -- "grasp #138 conf 0.921 cuRobo 4.9 mm box at
  close +0.024 after lift dz +0.147 m -> HELD". GraspGen-X's own pinch lifts the
  hammer on an arm-only lift. `$S/rise_ab2.sh` records that the same pinch loses
  the whole-body stand-up (+0.000 of 0.36 m), which is why a wrap is needed.
* The floor wrap cannot be formed without penetrating the handle (above).

`grasp/reach_from_pose_opus.py:490 --regrasp-wrap` does exactly this: take the
verified pinch, lift, turn the palm up, let the handle settle into the curled
fingers, close everything. It was written on 09-28 (`$S/regrasp_chain.sh`) and
has never run -- no log, no pid. v47 runs it on the `_opus` scripts with the
mix46 physics plus the source's 100/50, tests both turn directions, picks the
candidate by object-followed rather than defaulting to c0, and renders whichever
wins regardless of the verdict. Criteria are pre-registered in
`.../09/260929/5지/hammer/v47/note.txt`: object followed >= +0.10 m of 0.35 m,
tilt at the close < 30 deg with < 50 mm of object travel, and a non-thumb link
carrying force in more than 0 frames.

## v47: the hammer left the floor for the first time; the loss is in the stand-up (09-29 11:20)

GraspGen-X's own pinch #138, lifted, then wrapped in the air (`--regrasp-wrap`,
a path written 09-28 and never run). Solver at the source's 100/50.

| phase | c0 (turn -150) | c1 (turn -120) |
|---|---|---|
| pinch f165 | z 0.032 | z 0.032 |
| lift f225 | **z 0.218** (+186 mm off the floor) | |
| turn f285 | z 0.302 tilt 116 | z 0.323 tilt 110 |
| cradle f305 | z 0.305 | z 0.321 |
| wrap f345 | z 0.289 tilt 126 | z 0.328 tilt 104 |
| hold2 f375 | z 0.296 | z 0.329 |
| stand-up | **-0.262 of 0.35** LOST | **-0.298 of 0.35** LOST |

Held airborne for 180 frames (6 s). Every earlier run reported `object
followed +0.000`. The failure recorded in `reach_from_pose_opus.py:499-501`
("fell the moment all fingers and the thumb opened -- hammer: at the cradle")
did not reproduce: `REGRASP_KEEP=pinch` carried both candidates through it.

**The stand-up is the only remaining loss, and it is driven differently from
the one segment that works.** The lift moves the arm joints and the object
follows 186 of 186 mm. The stand-up freezes the arm dofs and writes the root
alone, with the root's linear and angular velocity written as zero
(`test_grasps_in_isaac_opus.py:293`); the object follows -262 of +350 mm. The
rise speed is not the difference: 0.236 m/s equals the planner's own
0.35 m / 1.5 s. I am not asserting the root write is the cause -- the measured
fact is the asymmetry. `:629` already calls this vertical raise "a stand-in"
and offers `TEST_RISE_PKL` (the planner's own stand-up, legs/waist/torso
pitching) quoting Sehoon; that path had never been run. v48 runs it.

### Still open: the render does not reproduce the tester

v47's render: `[eval] dz -0.0075`, every hand-link contact force 0.00 N for all
789 frames, object net force 1.96-2.04 N (its own 0.2 kg weight). The object
pose is *not* the divergence -- both runs spawn at `[-0.3 0.05 0.162]` and the
render's own `[near] centroid [-0.295 0.096 0.032]` matches the tester to 1 mm.
The fingers are:

    tester  q 1.31 1.32 1.33 1.35  (target 1.47)  -> stopped by the hammer
    render  q 1.70 1.70 1.70 1.70  (target 1.47)  -> curled past target, nothing blocking

and the render's nearest hand link at the close is 6.9-8.1 cm from the object
centroid. Why one contacts and the other does not is not yet closed by
measurement, and is not guessed at here.

## v48 is void: one dropped export line ran the wrong hand (09-29 11:29)

`rg48_chain.sh` was written fresh instead of copied from `rg47_chain.sh`, and lost

```
export HAND=inspire TIDY_NO_FLOOR_CARTON=1 TIDY_CRATE_ON_DESK="-1.530,..." \
       OMP_NUM_THREADS=1 PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
```

so the tester fell back to the Dex3 3-finger default while replaying an Inspire
12-dof hand schedule:

| | v47 (Inspire) | v48 (what actually ran) |
|---|---|---|
| hand links kept | 15 | 4 |
| right finger q | 12 values | 7 values |
| closed reference | (Inspire) | `[1.57 1.75 1.57 1.75 0 -1.05 -1.5]` |
| fingertip names | `R_index_intermediate`, `R_thumb_distal` | `index_1`, `middle_1`, `thumb_2` |
| palm − object | `[0.143 0.054 0.012]` | `[0.122 -0.053 0.256]` |

The object sat at `[-0.295 0.097 0.032]` tilt 6.8° through every phase, including
the lift that moved it to z 0.218 in v47. So `object followed -0.000 of 0.33 m` is
not a stand-up measurement — the hand never reached the hammer. `TIDY_*` was lost
too, so the scene differed as well.

Two further defects in the same script: `build_reach_reference.py` was called with
the npz first, but `:156` reads `walk_pkl, reach_npz, name = sys.argv[1:4]`, so
joblib tried to unpickle an npz and died; the render then received a `--walk` path
that did not exist and exited after 8 s with no mp4. `render exit 0` was not a pass.

**v47's `-0.262 of 0.35 m` remains the only stand-up measurement taken.** v49 is
`rg47_chain.sh` copied, with exactly two edits: reuse `reach_rg47.npz`, and the
stand-up (`TEST_RISE_PKL` for the test, `rise_reference.py` so it is in the video).

## v49: the hammer is thrown, not dropped — and the render says the same (09-29 11:45)

**The stand-up hypothesis is refuted by measurement.** v48's intent (replace the
vertical root write with the planner's own articulated stand-up) ran correctly in
v49. It changes nothing:

| stand-up | rise | object followed |
|---|---|---|
| v47 c0  vertical root write            | 0.35 m | −0.262 |
| v49 c0  planner clip (legs/waist/torso) | 0.33 m | **−0.266** |
| v47 c1 / v49 c1                         |        | −0.298 / **−0.294** |

4 mm apart. Speed is eliminated too: v49's rise at `TEST_RISE_SLOW=2` is slower
than the lift (0.19 m/s) and lost more. The pick is frame-for-frame identical to
v47 (hold2 f375 `object [-0.28 -0.007 0.296] tilt 121.5 deg`), so the pipeline is
deterministic and v48's difference really was the wrong hand.

**What the rise probe measured** (`grasp/test_rise_probe_opus.py`, an instrumented
copy — the running file was not edited; 11 lines in
`results/fable40/rise_probe_c0.txt`):

```
f  0  palm [-0.124  0.079 0.304]  object [-0.269  0.041 0.281]  fing 1.34
f  9  palm [-0.196  0.027 0.441]  object [-0.443 -0.068 0.518]  fing 1.38
f 12  palm [-0.271 -0.024 0.540]  object [-0.687 -0.042 0.674]  fing 1.43
f 18  palm [-0.447 -0.135 0.650]  object [-1.224 -0.030 0.070]  fing 1.53
f 30  palm [-0.855 -0.352 0.657]  object [-1.260  0.066 0.031]  fing 1.70
```

- The object is **77 mm above the palm at f9 and 134 mm above at f12**. A dropped
  object does not go above the hand.
- It reaches x −1.30: over a metre from the hand, gaining 255 mm of z in ~0.1 s
  (≈2.5 m/s).
- The palm rotates less than 9° in roll and pitch through the whole rise, so hand
  rotation is measured and small.
- The fingers **never stop closing**: 1.34 → 1.70 (saturation), thumb 0.03 → 0.95.

**The render now agrees with the tester for the first time.** v49's render ends at
`[eval] dxy 1.3725 m, dz −0.0543` — the hammer 1.37 m from the target. Every
previous render ended with the object where it started (v47: dxy 0.0381). The
ejection reproduces in both.

**The number that selects the next change.** The fingers advance 0.36 rad in 2.0 s
= 0.18 rad/s against a commanded `CLOSE_VEL=0.25` — 72% of the free closing rate.
With `stiffness 0` the squeeze torque is `kd·(v_target − v)` = 40 × 0.07 =
**2.8 N·m**. The wrap never reached contact equilibrium; it was sweeping through
the object at 2.8 N·m.

**The open source documents this symptom at our exact values.**
`end2end/dynamic_playback.py:641-647`: "Closing happens by setting joint_target_vel
to a constant closing rate during the close phase; *contact equilibrium stops the
motion*. Match `newton_grasp_eval.py`'s `FINGER_KD=800`." — default 800.
`end2end/robots/g1_right_arm.yaml:63-72`: "The UR10e profiles override it down to
50, and *at 50 this hand's thumb was pushed back open by the object*"; and
`finger_effort_limit: 1000.0`, "1000, not surge_hand's 200 … 'finger_effort_limit
(default 200 N in dynamic_playback) was the previous bottleneck' — and *the object
was sliding out of a grip that closed but could not hold*."

We ran `CLOSE_KD=40` (below the 50 the source records as failing on this very hand)
and `HAND_EFFORT=200` (the value the source names as the bottleneck).

**v50 = one change, the source's velocity-close gain pair:** `CLOSE_KD 40 → 800`,
`HAND_EFFORT 200 → 1000`. Falsifier: if the probe still shows the fingers moving at
>70% of free rate, the gains are not the cause and `OBJ_MAX_DEPEN_VEL=5` is next.

**Two candidate mechanisms for the ejection remain open and neither is established:**
the still-velocity-driven fingers, and `OBJ_MAX_DEPEN_VEL=5` permitting a 5 m/s
de-penetration push. The probe is consistent with both.

### Process defect fixed alongside
v49's folder stood empty for 20 minutes because the chain shell exited after
launching the render, taking the mp4 copy step with it. A separate watcher now does
the copy. v33, v34 and v48 never built a motion file at all, so nothing could be
rendered for them; they were moved to `_무효_영상없음/` rather than left as empty
version folders. v35–v49 now all carry their three videos.

## v50: the gains are refuted, and the first contact measurement says the hand is *inside* the hammer (09-29 12:00)

**Verdict.** v50 (GraspGenX's own velocity-close pair, `CLOSE_KD 40→800`, `HAND_EFFORT 200→1000`,
both read from `end2end/dynamic_playback.py:641-661` and `end2end/robots/g1_right_arm.yaml:51-72`)
still loses the hammer.

| | render `[eval]` | tester stand-up |
|---|---|---|
| v50 c0 | end `[-0.8508 0.7513 -0.0877]`, dxy 0.8917 m, dz −0.2497 m → LOST | body up 0.33 m, object followed **−0.246 m** → LOST |

Video: `09/260929/5지/hammer/v50/hammer_rg50_c0{,_head,_wrist}.mp4`; room view verified
(`evidence/room_view_check.png`).

**The pre-registered falsifier was met.** `v50/note.txt` said, before the run: *"fingers still moving at
≥70 % of free rate ⇒ the gains are not the cause."* Measured: kd=40 → 72 %, kd=800 → **68 %**.
A 20× gain increase slowed the fingers by 4 points. The gains are not the cause.

### The measurement that had never been taken

`grep -n "ContactSensor\|net_forces_w\|CONTACT_FORCE" grasp/test_grasps_in_isaac_opus.py` → nothing.
The tester issues every HELD/LOST verdict and had **no contact sensing at all**; only the renderer
had it. Ported `play_in_cell_opus.py`'s `ContactSensor` into `grasp/test_rise_probe_opus.py` and
re-ran the identical v50 environment. Control: the 0.2 kg object's own weight, 1.96 N.

| phase | net | Σ over 12 right-hand links | largest link |
|---|---|---|---|
| approach f135 | 2.04 N | 0.00 N | — |
| pinch f165 | 1.96 N | 0.00 N | — (sensor alive: reads self-weight) |
| hold f195 | 3.18 N | 3.18 N | `R_ring_intermediate` 3.18 |
| lift f225 | 59.39 N | 255.31 N | `R_thumb_proximal` 100.90 |
| **turn f285** | **36 711.97 N** | 19 459.86 N | `R_thumb_proximal` 16 326.35 |
| cradle f305 | 6 962.92 N | 6 756.77 N | `R_thumb_proximal` 6 756.77 |
| settle f315 | 104.98 N | 135.26 N | `R_index_intermediate` 135.26 |
| wrap f345 | 334.09 N | 316.36 N | `R_thumb_proximal` 316.36 |
| hold2 f375 | 125.67 N | 337.35 N | `R_middle_intermediate` 174.86 |
| rise f0 | 3 048.45 N | 5 131.28 N | `R_thumb_proximal` 2 821.91 |
| rise f3 | 12 959.84 N | 16 186.23 N | `R_thumb_proximal` 14 194.23 |
| rise f6 | 29 562.26 N | 21 769.39 N | `R_pinky_proximal` 14 037.52 |
| rise f9 | 0.91 N | 0.00 N | (object gone) |
| rise f18 | **67 144.59 N** | **0.00 N** | (a link outside the 12-link filter) |

Measured facts, not interpretation:

1. 36 712 N on a 1.96 N object — **18 700× its weight**. That is a depenetration impulse, not a grip.
2. The blow-up starts in the **pick**, at `turn` (f285), not in the stand-up. `approach` and `pinch`
   read exactly 0.00 N, so the sensor was working and the hand genuinely was not touching yet.
3. At rise f18, net is 67 145 N while all twelve finger links read 0.00 N ⇒ a link that is **not a
   finger** (palm, wrist or forearm) struck the departing object. The current filter cannot name it.
4. The v50 gains changed the verdict not at all (−0.246 m).

### A second, separate defect: render and tester disagree

Same candidate, same scene. Over all 185 sampled frames of the 923-frame render
(`results/fable40/rg50c0.log`) the twelve-link sum never left 0.00 N and net stayed within
0.00–10.89 N. The tester reads kilonewtons over the same interval. These two must agree.

One confirmed difference: `OBJ_MAX_DEPEN_VEL` is referenced at `grasp/play_in_cell_opus.py:705` and
**nowhere else** (`grep -rn OBJ_MAX_DEPEN_VEL grasp/*.py` returns that one line). The tester never
applies it. So the second candidate recorded in `v50/note.txt` — *"the 5 m/s depenetration cap is the
ejector"* — **cannot** explain the tester's kilonewtons, because the tester does not set it.

### Also established by reading the source, not by guessing

- `grasp/reach_from_pose.py` carries `self_collision_check=True` and **no world collision model**
  (no `WorldConfig`, no obstacle, no `world_model`). That is not an oversight on its own: GraspGenX
  does the same and says so — `end2end/e2e_grasp_demo.py:2092-2094`, *"cuRobo never sees the target
  (it is deliberately left out of the collision world, scene_builder.py), so nothing else rejects
  those."*
- The official compensation is `graspgenx/utils/collision_filter.py: filter_colliding_grasps`, but its
  docstring is explicit that **the target object's points must already be removed** from `scene_pc`.
  It rejects grasps that hit the *rest* of the scene. It would not reject fingers inside the hammer,
  so it is not the missing piece here.

### Open, deliberately not asserted

- Which link carries the 67 kN at rise f18. Queued: the same probe with the filter widened from
  `/World/G1/R_.*` to every direct child of `/World/G1`.
- Why the renderer reads 0.00 N on the identical twelve link names.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---

## v51–v53 (2026-09-29): one hypothesis retracted, one defect found in our own env

### Retracted: "the hand closes in a plane that misses the handle"

I wrote this into `v51/note.txt` and `v52/note.txt` as if measured. It is wrong, and GraspGenX's
own gripper description refutes it three ways. Both notes now carry a `[정정]` section.

1. `gripper_descriptions/assets/x_grippers/inspire_hand/config.json` declares the grasp cavity:
   `sweep_volume` box1 extents `[0.080, 0.045, 0.040]` at `[0, 0, 0.135]`, box2 `[0.040, 0.045, 0.035]`
   at `[0, 0, 0.118]`, fingertip `[0, 0, 0.150]`. Transforming 40,000 sampled hammer surface points
   into each candidate's canonical gripper frame — `inv(plan_from_cell) @ inv(B) @ g`, one step before
   `grasps_in_cell()` applies `grasp_to_tool_transform` — puts the handle inside the cavity for
   **156 of 156** candidates (138: 2736 pts, 3: 2922, best 136: 3434).
2. The links Isaac measured at hold, in that same frame (mm): index_int `[-45.3, 22.5, 118.1]`,
   pinky_int `[-44.6, -37.8, 119.8]`, thumb_dist `[43.9, 4.9, 131.6]`, object `[0.4, -54.3, 151.1]`.
   Fingers at z 118–132 is exactly the declared cavity depth 118–135. The frame mapping is right.
3. The "17–30 mm miss" was a link-origin-to-surface distance, which does not measure contact. Its own
   refutation is in the data it came from: it called the pinky the furthest pad (34.6 mm) while the
   pinky carried the most force (56.46 N).

v52 (candidate 3, chosen by that retracted metric) lost the hammer in both testers — consistent.

### Also dead, for free: "the mimic followers are not tracking"

The hold-frame trace already answered it. Right hand `q [1.47 1.47 1.47 1.47 1.3 0.5 | 1.47 1.47 1.47
1.47 0.8 1.2]`, `target` identical. The four intermediates are at 1.47. The fingers are fully curled.
No render spent.

### The defect: our own `MIMIC_URDF_RATIO=1` contradicts our own target vector

`MIMIC_URDF_RATIO=1` (set in every chain v47–v52) rewrites the thumb PhysX gearings at
`grasp/play_in_cell_opus.py:519-565`, from the IsaacLab USD's **−1.6 / −2.4** to the manufacturer
URDF's **−1.334 / −0.667**:

```
inspire_hand/gripper.urdf
  thumb_intermediate  mimic thumb_proximal_pitch  multiplier="1.334"  limit 0..0.8
  thumb_distal        mimic thumb_proximal_pitch  multiplier="0.667"  limit 0..0.4
```

The target vector was not rewritten with them. `_closed_one`'s followers `0.8 / 1.2` were derived from
the old gearings — `0.8 = 1.6 × 0.5`, `1.2 = 2.4 × 0.5` — and Fable's own `stiffen_mimic()` docstring
in `grasp/build_reach_reference.py` records those native gearings as −1 / −1.6 / −2.4. Under the
gearing actually in force the correct targets are `1.334 × 0.5 = 0.667` and `0.667 × 0.5 = 0.334`.

So v47–v52 drove **thumb_distal to 1.2 where its four-bar allows 0.334** — 3.6×, and past the URDF's
own 0.4 limit. The position drive wins (measured `q` == `target` == 1.2), so the thumb was internally
jammed for the whole grasp, on the link pair carrying 33.42 N at hold.

v53 changes those two numbers and nothing else, on v51's candidate 138, so the comparison is clean.
Patched in `grasp/build_reach_reference_opus.py` and the three `_closed` sites of
`grasp/reach_from_pose_opus.py`, guarded by `MIMIC_URDF_RATIO == "1"`; the three opus testers and
`play_in_cell_opus.py` were rewired to import the opus reference module, without which the patch
would have been a no-op.

**Pre-registered falsifier.** If at hold the contact links and forces are unchanged (pinky ≈ 56 N,
thumb_intermediate ≈ 33 N, index 0 N) and the object still leaves with +27 mm, the thumb jam is not
the cause and I drop it.

### Still open, deliberately not asserted

- `thumb_proximal_pitch` is commanded 0.5 where `config.json`'s `close` says 0.6. Left unchanged in
  v53 so as not to confound the one change.
- The render/tester contact disagreement (0.00 N vs kilonewtons) is unexplained.

### Correction to the section above: the thumb-jam defect does not exist

I committed the section above before reading `soft_mimic()`. It is wrong and I am not
leaving it standing. `grasp/build_reach_reference.py`:

```python
def soft_mimic(robot, tgt):
    """... Call after robot.update() and before set_joint_position_target(), every substep."""
    for a, b, r in _mimic_ids:
        tgt[0, a] = r * float(q[b])
```

The follower entries of `_closed_one` / `HAND_CLOSED` / `hands.npy` are overwritten every
substep from the **measured** master angle. They are dead numbers. `SOFT_MIMIC` defaults on for
`HAND=inspire`, and both the v52 render and its tester print their respective activation lines.
So `0.8 / 1.2` were never driven "because `_closed_one` said so", and the patch I described above
was a no-op in every path. The v53 chain was killed at its solve, before it spent a render.

**What the same reading did establish, by measurement.** `MIMIC_URDF_RATIO` is implemented in
`grasp/play_in_cell_opus.py:545-564` and `grasp/thumb_free_opus.py:83` and **nowhere else**
(`grep -rn MIMIC_URDF_RATIO grasp/*.py`). The chain exports it to the tester too, and the tester
ignores it. Measured at hold, same candidate, same scene:

| | thumb_intermediate | thumb_distal | ratios in force |
|---|---|---|---|
| render (`rg52c0.log`) | 0.667 | 0.334 | URDF 1.334 / 0.667 |
| tester (`force_probe54_c0.txt`) | 0.8 | 1.2 | USD 1.6 / 2.4 |

Since v47 every tester verdict has been measured on a thumb driven 1.8× further than the render's,
past the URDF's own 0.4 limit. That is one concrete piece of the long-open "render and tester
disagree" item. The render's block is now ported verbatim into `test_grasps_in_isaac_opus.py` and
`test_rise_probe_opus.py`.

### v53, actually: `thumb_proximal_pitch` 0.5 → 0.6

Two sources, both already in the tree:

- `gripper_descriptions/assets/x_grippers/inspire_hand/config.json` `"close"` gives this hand's
  thumb pitch as **0.6**. We have driven 0.5.
- `grasp/play_in_cell_opus.py:532`, Fable's transcription of the URDF:
  `thumb_proximal_pitch_joint (master) limit 0 .. 0.6` — 0.6 is the mechanical stop.

Because `soft_mimic` derives both followers from the master, this one number moves the whole
four-bar: 0.5 gives `0.667 / 0.334`, which is 83 % of the URDF's `0.8 / 0.4` travel; 0.6 gives
exactly `0.8 / 0.4`. The thumb has been stopping a fifth short of its stroke. Consistent with the
measurement: at hold `thumb_distal` sat at cavity depth z 131.6 mm where `config.json` puts the
fingertip at 150 mm.

Same candidate 138 as v51, so the render comparison is clean. **Pre-registered falsifier** unchanged:
if the hold contacts and forces are unchanged (pinky ≈ 56 N, thumb_intermediate ≈ 33 N, index 0 N)
and the object still leaves with +27 mm, thumb stroke is not the cause and I drop it.

### v51 / v52 delivered

- v51 candidate 138: `[eval] dxy 0.8917 m, dz −0.2497 m -> LOST`
- v52 candidate 3: `[eval] dxy 0.4284 m, dz −0.0081 m -> LOST`

Three mp4s each, room-view frame checked.

## v53: 손실 지점은 일어서기가 아니라 손목 회전이고, 망치는 한 프레임에 발사된다

세 가지를 측정했다. 셋 다 이전 서술을 고친다.

### 1. 엄지 마스터 목표 0.6 은 자산이 자른다 — v53 의 변경은 무효였다

v53 c0 의 hold 프레임:

```
q      [1.47 1.47 1.47 1.47 1.3  0.5  1.47 1.47 1.47 1.47 0.67 0.33]
target [1.47 1.47 1.47 1.47 1.31 0.6  1.47 1.47 1.47 1.47 0.67 0.33]
```

`assets/g1_inspire/g1_29dof_inspire_hand.usd` 를 직접 덤프한 값:

```
R_thumb_proximal_pitch_joint   upper  28.648 deg = 0.5000 rad
R_thumb_intermediate_joint     upper  55.004 deg = 0.9600 rad
R_thumb_distal_joint           upper  82.506 deg = 1.4400 rad
```

제조사 쪽은 `config.json` `"close"` 가 thumb_proximal_pitch **0.6**, `gripper.urdf` 가
master **0..0.6** / intermediate 0..0.8 / distal 0..0.4 이다. USD 는 내부적으로도
어긋나 있다 — 추종자 한계 0.96 = 1.6 × 0.6, 1.44 = 2.4 × 0.6 으로 **0.6 마스터를
전제로 계산해 놓고 마스터 자신만 0.5 로 잠갔다**. 엄지는 제 행정의 83 % 에서 멈춘다.
명령값만 올리는 변경은 전부 무효다. 위치한계를 고쳐야 한다.

### 2. 손실은 일어서기가 아니다. turn 중이고, 그 뒤는 전부 빈 손이다

v53 c0 단계별 물체 자세:

| 단계 | 프레임 | 위치 | 기울기 |
|---|---|---|---|
| lift | 225 | `[-0.255  0.093  0.166]` | 17.8° |
| turn | 285 | `[-0.624  0.956  0.087]` | 157.4° |
| cradle / settle / wrap / hold2 | 305–375 | 전부 동일 | 157.4° |

turn 이후 네 단계가 소수점까지 같다 — 이미 바닥이고 아무도 안 건드린다. wrap 은 빈 손을
쥐고, 일어서기 시험은 4 초 전에 사라진 물체를 잰다. 판정문의
`slipped in the stand-up (-0.000 of 0.33 m)` 은 손실 지점을 가리키지 않는다.
**v47 이후 "일어서기에서 놓친다" 로 읽어 온 판정은 전부 이 오독일 수 있다.**

### 3. 프레임 단위로 재니 미끄러진 뒤 한 프레임에 발사된다

`TURN_TRACE` 계측 (테스터 사본에 추가):

```
f226 obj-palm [-117.0  47.6 -141.2] mm  |v| 0.083
f232 obj-palm [-125.7  77.6 -165.2] mm  |v| 0.078   <- 7 프레임에 손바닥 기준 30 mm 미끄러짐
f233 obj [-0.2895 0.1888 0.1908] tilt 46.9  |v| 2.763  v [-0.82  1.93  1.80]
f234                                        |v| 2.564  v [-0.82  1.93  1.48]
f235                                        |v| 2.392  v [-0.82  1.93  1.15]
...
f245                                        |v| 2.954  v [-0.82  1.93 -2.08]
```

f233 이후 x, y 성분이 `-0.82, 1.93` 으로 **고정**이고 z 만 프레임당 −0.327 m/s
(= g/30) 씩 떨어진다. 순수 포물선이다. 즉 f233 부터 아무것도 망치에 닿지 않는다.
그리고 f232 → f233 한 프레임(33 ms)에 0.078 → 2.763 m/s, Δv 2.69 m/s → 81 m/s² ≈ 8.3 g.

원심력은 아니다: turn 은 60 프레임에 150° = 1.31 rad/s, r ≈ 0.15 m → 0.26 m/s², g 의 3 %.
그리고 turn 은 `Tl @ Rx` 로 손목 **위치는 고정**하고 자세만 돌린다
(`reach_from_pose_opus.py:508-527`). 회전 내내 손가락 목표는 `_closed` 고정이다.

한 프레임에 완성된 속도가 생기는 것은 접촉력이 아니라 솔버가 관통을 밀어낸 서명이다.
**단 어느 링크와 관통했는지는 아직 측정하지 않았다 — 추정이다.**

### 4. 계측기 불일치가 또 있다: OBJ_MAX_DEPEN_VEL 은 렌더에만 있다

`grep -rn OBJ_MAX_DEPEN_VEL grasp/*.py` → `play_in_cell_opus.py:708` 한 곳뿐.
체인은 테스터에도 `OBJ_MAX_DEPEN_VEL=5` 를 넘기지만 테스터는 읽지 않는다.
`MIMIC_URDF_RATIO` 와 같은 패턴이다 (그건 v53 에서 이식했다).
다만 측정된 발사 속도 2.76 m/s 는 5 미만이라, 값 5 를 그대로 이식해도 이 사건은
바뀌지 않는다. **무효 변경을 하나 더 만들 뻔했다.**

### 이전 관측과의 충돌

`reach_from_pose_opus.py:500` 의 주석은 이렇게 적고 있다:

> `-150 deg: the tool rode through the turn, then fell the moment all fingers and the thumb opened (hammer: at the cradle)`

전에는 회전을 버티고 cradle 에서 떨어졌다. v53 은 회전 중에 발사된다. v53 에서
테스터에 새로 들어간 것은 `MIMIC_URDF_RATIO`(엄지 distal 목표 1.2 → 0.334 rad)뿐이므로,
과하게 말려 있던 엄지가 테스터에서 망치를 붙잡고 있었을 가능성이 있다.
**확정이 아니다** — 같은 씬을 옛 기어비로 한 번 더 돌려야 가른다.


## 260929 13:40 — 손실은 기립이 아니라 이른 들어올리기다 (v53/v54 렌더 계측)

두 후보의 납품 렌더에서 같은 프레임, 같은 값이 재현된다.

| | 닫힘 명령 | 들어올리기 시작 | 그때 검지 근위 | 목표 1.47 도달 | 물체 손실 |
|---|---|---|---|---|---|
| v53 c1 | f549 | f609 | 0.58 | f700 | f630 (z 0.163→0.347, 발사) |
| v54 c0 | f549 | f609 | 0.58 | f700 | f660 (\|v\| 1.34 m/s) |

손은 40 % 닫힌 채로 물체를 든다. 기립은 v54 기준 f909 부터이므로, 손실은
기립보다 249 프레임 앞선다. `[eval]` 의 "slipped in the stand-up" 은 사건을
잘못 가리킨다.

**원인은 값이 아니라 일정이다.** 우리 값은 GraspGenX 원본과 이미 같다:

- `CLOSE_VEL 0.25` = `gripper_close_velocity` 0.25 (g1_inspire_arm.yaml,
  g1_inspire_palm_arm.yaml, g1_right_arm.yaml 셋 다)
- `CLOSE_KD 800` = newton_grasp_eval `FINGER_KD` 800
- `HAND_EFFORT 1000` = `finger_effort_limit: 1000.0`
- 물체 마찰 10.0 = `DEFAULT_OBJECT_MU`, `FINGER_MU 3.0` = `DEFAULT_FINGER_MU`
  (게다가 `friction_combine_mode="max"` 라 유효 마찰은 10.0)

다른 것은 GraspGenX `end2end/tasks.py:267-278` 의 `hold_after_close` 구간이
우리 일정에 없다는 것뿐이다. 그 주석이 우리 증상을 그대로 적고 있다:
"a gripper that closes slowly (e.g. velocity-mode multi-finger hands) needs
LONGER here so the fingers fully settle on the object before the lift —
otherwise the object slips out (premature lift)."

1.47 rad 를 0.25 rad/s 로 닫으려면 176 프레임(실측 151)이 드는데 일정은 60 을
준다. 모자란 91 프레임이 이 실패의 크기다.

조치: `grasp/build_reach_reference_opus.py` 에 `HOLD_AFTER_CLOSE` 를 넣었다.
v55 는 120 프레임으로 돌고 있고, 로그가 간격 60 → 180 을 확인해 준다
(`hands close at frame 519 ... lift from 699`).

### 철회: "엄지가 닫히지 않는다"

테스터 `ab_rg54_c0.txt` 의 `thumb_proximal_pitch q 0.01 (목표 0.6)` 로 엄지
결함을 주장했으나, 같은 관절을 렌더 로그는 0.5 로, 네 손가락은 1.7 로 읽는다.
**렌더에서 손은 주먹까지 닫힌다.** 0.01 은 테스터 쪽 수치이며 납품 영상의
실패를 설명하지 않는다. 커밋 f41ad17 이 적어둔 "테스터와 렌더가 서로 다른
엄지를 구동한다"와 같은 결함이다. 엄지 관련 변경(THUMB_LIMIT_URDF, MIMIC_SPLIT)은
이 숫자로 정당화되지 않으므로 넣지 않았다.

### 남은 결함 (사실만)

- 체인의 `BEST` 선택이 `max(object followed)` 라, 물체가 이미 바닥에 있어
  움직이지 않은 후보를 고른다. hold2 에서의 물체–손바닥 거리로 판정해야 한다.
- 렌더는 모든 프레임에서 접촉력 `0.00 N` 을 보고하는데 테스터는 킬로뉴턴을
  보고한다. 둘 중 하나는 접촉을 보지 못하고 있다. 원인 미상.
- v48 폴더가 없다 (v47 다음이 v49).
