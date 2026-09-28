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
