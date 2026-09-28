# g1-factory-tidy

어질러진 공장을 정리해서 정해진 상태로 되돌리는 것이 최종 목표입니다. 깔끔한 상태를
이미지로 주면, 로봇이 지금 상태와 그 이미지를 비교해 어긋난 것을 치웁니다. 무엇을 어디에
둘지는 사람이 매번 지시하는 것이 아니라 그 이미지가 정합니다.

로봇에게 물체 좌표를 주지 않는 것이 이 저장소의 전제입니다. 물체 위치는 로봇에 달린
RGB-D camera에서 나오고, 참값은 추정이 얼마나 틀렸는지 채점할 때만 씁니다.

지금은 그 중 한 조각을 하고 있습니다. Unitree G1 한 기가 "책상 위 상자를 집어"라는
한 문장을 받아, 비전으로 그 물건을 찾고, 걸어가서, 집어 드는 것입니다.

### ▶ 시연 영상 (YouTube)

[![시연 영상 재생](docs/youtube_thumb.jpg)](https://youtu.be/0Tu-V0MvPYc)

위 이미지를 누르면 YouTube에서 재생됩니다 — https://youtu.be/0Tu-V0MvPYc

한 문장에서 파지까지의 전 과정을 영어 자막으로 단계마다 설명한 영상입니다.
`bash grasp/pick_by_language.sh box table` 한 번의 실행이고,
`grasp/compose_fable.sh`가 그 결과를 한 영상으로 엮습니다.

## 파이프라인

![pipeline](docs/pipeline.png)

1 보기(C-RADIO) → 2 이동·자세(GR00T-WholeBodyControl) → 3 파지 후보(GraspGen-X) → 4 전신 도달(cuRobo) →
5 물리 검증·렌더(Isaac Lab). 운반은 2와 4를 한 번 더 지납니다. 그림은 `scripts/make_pipeline_figure.py`가 그립니다.

세 개의 오픈소스가 순서대로 물립니다. 사람이 주는 것은 문장 하나뿐이고, 어디에 무엇이
있는지, 어떻게 잡을지, 어떻게 갈지는 이 셋이 정합니다.

**0. 언어 → 물체 (C-RADIO, cuRobo의 예제)**

머리 camera 한 장을 NVIDIA C-RADIO로 채점합니다. 모델과 텍스트 매칭은 cuRobo 자체
튜토리얼(`curobo/examples/getting_started/feature_mapping.py`)의 것을 그대로 씁니다.
"box"가 이긴 영역 중 "table"로 본 면의 윗면에 밑면이 닿아 있는 것만 답이 됩니다.
선반 위 크레이트(+0.36 m)와 바닥 상자(−0.69 m)는 이 기하로 걸러집니다. 아무것도
없으면 GR00T 플래너로 제자리 회전해 다시 봅니다. 답은 `obj_lang`이라는 이름으로
촬영본에 써 넣고, GraspGenX는 그 이름을 읽습니다 (`grasp/find_by_text.py`).

**1. GraspGen-X — 무엇을 어떻게 잡을 것인가**

![GraspGen-X](docs/graspgen.gif)

머리 RGB-D 한 장에서 물체의 점구름을 얻고, 거기서 파지 후보를 만들어 점수를 매깁니다.
물체의 mesh도, 좌표도 주지 않습니다. 모델이 보는 것은 camera가 실제로 본 점들뿐입니다.

**2. cuRobo — 거기까지 팔을 어떻게 가져갈 것인가**

![cuRobo](docs/curobo.gif)

파지 자세를 목표로 충돌 없는 관절 궤적을 풉니다. 닿지 않는 파지는 여기서 걸러지고,
후보 중 실제로 실행 가능한 것이 골라집니다.

**3. GR00T (GEAR-SONIC) — 거기까지 몸을 어떻게 가져갈 것인가**

![GR00T](docs/gr00t.gif)

팔이 닿는 자리가 지금 선 자리가 아니면 걸어가야 합니다. 걸어야 하는지는 cuRobo IK가
베이스를 지금 자리에 잠그고 먼저 묻고(0/80이면 걷습니다), 설 자리는 베이스를 풀고
푼 IK 해 1264개 중 물체를 정면에 두는 것을 고르며, 경로는 `plan_cspace`가 책상을
피해 냅니다 (`grasp/where_to_stand.py`). 보행 클립은 GR00T의 kinematic planner가
만듭니다 (`grasp/walk_clip.py`). 셀 안 재생은 아직 그 클립을 그대로 트는 것이고,
SONIC 추적 정책이 같은 클립을 따라 걷는 것은 공식 평가기로 따로 확인했습니다.

![G1 vision grasp](docs/g1_vision_grasp.gif)

머리 camera RGB-D 한 장에서 파지를 만들어 상자를 들어 올립니다. 왼쪽은 3인칭,
오른쪽은 손목 camera(D405)입니다. 물체 좌표는 주지 않았습니다.

## 지금 되는 것

- 셀 맵 생성과 viewer
- 집을 물건을 강체로 배치 (렉 선반 위, 바닥)
- 머리 camera에서 바닥 물체의 위치·크기 추정, 참값 대비 채점
- 머리 RGB-D 한 장에서 파지 생성 → 경로 계획 → 셀 안 재생 → 들어 올리기
- 머리와 손목 camera 두 장을 한 점구름으로 합쳐 파지 생성
- 전신 제어(SONIC) 위에서 골반 고정 없이 서기와 팔 뻗기
- 한 문장 → C-RADIO로 물체 찾기 → 못 찾으면 돌아서 다시 보기 → 걸어야 하는지 판단
  → 설 자리와 경로 → 걷기 → 다시 보고 파지 계획 → 집어 들기, 스크립트 한 번에
  (`grasp/pick_by_language.sh`)
- 도착 후 다리 지터링 없음. 원인은 IsaacLab `DCMotorCfg`의 토크 한계가 PhysX의
  잡음 섞인 관절 속도를 받는 것이었고, 골반을 고정한 파지에서는 같은 게인의
  implicit PD를 씁니다 (`docs/DIAGNOSIS.md`)

## 아직 안 되는 것

- 닫힌 루프. 시작 전에 한 번 보고 그 뒤에는 눈을 감습니다. 실행 중에 상자가
  움직여도 따라가지 않습니다
- 파지와 전신 제어가 한 실행 안에 있지 않습니다. 걷기는 플래너 클립을 재생하고,
  파지는 골반을 고정한 채 재생합니다. 둘의 이음매에 한 프레임의 도약이 남아 있습니다
- 팔 7관절만 씁니다. 그래서 파지 계획은 골반 높이 0.750~0.760 m에서만 풀리고,
  걷기 클립을 그 높이에서 잘라 맞춥니다. cuRobo의 전신 계획으로 옮기면 없어질 일입니다
- 진단 전체와 원본 저장소와 어긋난 곳은 `docs/DIAGNOSIS.md`에 있습니다

## 셀

바닥 6 x 4 m, 북쪽과 서쪽에만 벽이 있고 남/동쪽은 열려 있습니다. 서쪽 벽에 폭 1.2 m
출입구가 있습니다. G1 어깨가 약 0.45 m, 들고 가는 물건이 0.60 m라 그 둘을 함께 통과시키는
폭입니다. 문틀 위쪽은 막지 않았습니다.

렉은 북쪽 벽에 2대이며 `Environments/Hospital/Props/SM_MedShelf_01d`입니다.

정적인 것(바닥, 벽, 렉)은 `map/cell.usd` 하나로 구워 두고, 집을 물건은 굽지 않습니다.
강체를 정적 셸에 구우면 장식이 되거나 불러올 때 이미 떨어지는 중이기 때문입니다. 물건은
`map/props.py`가 실행 시마다 새로 놓습니다.

## 치수가 정해진 근거

바닥 상자는 0.30 x 0.30 x 0.32 m, 1.5 kg입니다. 로봇에서 재서 정했습니다
(`map/measure_g1_hands.py`) — 손바닥 간격 0.326 m, 몸 앞 0.21 m. 두 손으로 옆면을 눌러 드는
방식이라 상자 폭은 그 간격보다 약간 좁아야 손이 안쪽으로 눌립니다. 높이 0.32 m면 잡는
지점이 바닥에서 0.16 m 떠서 손등이 바닥에 닿지 않습니다. 이 치수는 선반 칸(간격 0.374 m,
앞널 깊이 0.369 m)에도 들어갑니다. 더 크면 집기는 쉬워도 넣을 자리가 없습니다.

## camera

실기 G1 머리에는 Intel RealSense D435i가 달립니다. 기하는 그대로 맞췄습니다 — 깊이 시야각
86도, 최소 거리 0.40 m. 해상도만 424 x 240으로 낮췄습니다. 실기에서도 정책에는 줄여서
넣으므로 사양 위반이 아니고, render 시간을 아낄 수 있는 유일한 자리입니다.

camera는 30도 아래를 봅니다. G1은 목 관절이 없어 head_link가 몸통에 고정돼 있고, 수평으로
달면 바닥이 1.5 m 밖에서야 화면에 들어옵니다. 0.9 m 앞의 상자는 화면 맨 아래에 걸려 앞면이
잘리고, 추정 중심이 76 mm 밀립니다. 30도 내리면 바닥이 0.49 m부터 보여 깊이 센서의 최소
거리와도 맞습니다.

깊이의 0.40 m 제한은 근평면(clipping)으로 구현하지 않았습니다. 그렇게 하면 색 영상까지
사라지는데, 실제 센서는 가까운 것도 보기는 하고 깊이만 반환하지 않기 때문입니다. 정상적으로
render한 뒤 0.40 m보다 가까운 깊이를 버립니다.

## 추정 정확도

로봇 자세 5가지에서 잰 값입니다. 상자 위치는 다섯 번 모두 같습니다.

| 로봇 위치 | 상자까지 | 중심 오차 | 높이 오차 |
|---|---|---|---|
| (-2.00, -0.40) 정면 | 0.80 m | 6 mm | +2 mm |
| (-2.00, +0.30) 정면 | 1.50 m | 8 mm | +3 mm |
| (-1.20, -1.20) 측면 | 0.80 m | 1 mm | +2 mm |
| (-2.60, -0.55) 대각 | 0.86 m | 2 mm | +2 mm |
| (-2.00, -0.20) 정면 | 1.00 m | 7 mm | +2 mm |

곧이곧대로 믿으면 안 되는 지점이 셋 있습니다. 시점만 바꾼 결과라 물체 위치가 달라질 때는
아직 모릅니다. 바닥이 z=0이라는 것을 코드에 넣었으므로 실기에서는 바닥 평면부터 추정해야
합니다. 벽과 렉은 셀 좌표로 잘라냈으므로, 물체 위치는 주지 않았지만 맵 구조는 준 셈입니다.

## Grasp 합성 (초기 조사)

grasp 자세는 규칙으로 정하지 않고 [Dexonomy](https://github.com/JYChen18/Dexonomy)로
합성합니다. hand와 grasp type마다 사람이 만든 template 하나에서 출발해, object를 hand
template에 맞춰 최적화한 뒤 simulator에서 hand를 object에 맞춰 다듬습니다. 지원 hand 목록에
`Unitree_G1`이 있고, 실제 asset은 `dex_3_1_r.xml` — G1이 실기에서 다는 Dex3-1 오른손입니다.
그래서 hand를 바꾸지 않고 쓸 수 있습니다.

Dex3-1로 annotate된 grasp type은 셋입니다: `1_Large_Diameter`, `3_Medium_Wrap`,
`6_Prismatic_4_Finger`.

![Dex3-1 grasp](docs/dex3_grasp.gif)

Dexonomy가 `1_Large_Diameter`로 합성한 grasp입니다. approach, grasp, squeeze 세 자세가
이어집니다. hand만 있고 팔과 몸은 없습니다.

## Pick

파지는 [GraspGenX](https://github.com/NVlabs/GraspGenX)가 만들고, 팔 경로는
[cuRobo](https://github.com/NVlabs/curobo)가 풉니다. 둘 다 GraspGenX 저장소의
`end2end` 파이프라인을 그대로 쓰고, 여기서는 결과 궤적만 받아 셀 안에서 재생합니다.

손은 G1이 실기에서 다는 Dex3-1입니다. GraspGenX가 같은 이름으로 손을 하나 갖고 있지만
다른 revision이라 닫힘 각도 넷이 이 손의 관절 한계를 넘습니다. 그 손으로 만든 파지는
손가락이 물체에 닿기 전에 한계에 걸려 밀어냅니다. 그래서 G1 URDF에서 오른손만 잘라내
GraspGenX에 새 gripper로 등록하고 씁니다.

| | GraspGenX `unitree_g1` | G1 실물 손 |
|---|---|---|
| index_0 닫힘 | 1.84 | 1.57 |
| index_1 닫힘 | 1.84 | 1.75 |
| middle_0 닫힘 | 1.84 | 1.57 |
| thumb_1 닫힘 | −1.20 | −1.05 |

### 파지가 안 잡히던 이유는 마찰이었습니다

같은 계획이 상자를 테이블 밖으로 튕겨내다가, 마찰만 맞추니 들립니다. GraspGenX는
물체 마찰 10.0, 손가락 패드 3.0에서 파지를 만들고 검증합니다
(`end2end/e2e_grasp_demo.py`의 `--object_mu`, `--finger_mu` 기본값). 셀에는 아무것도
걸려 있지 않아 PhysX 기본값 0.5로 재생하고 있었습니다. 20배 차이입니다.

| | 마찰 기본값 0.5 | 물체 10.0 / 손가락 3.0 |
|---|---|---|
| 손가락 닫는 순간 물체 이동 | 93.6 mm | 20.8 mm |
| 끝 높이 변화 | −43 mm (떨어짐) | +50.6 mm (들림) |
| 판정 | LOST | HELD |

접근 구간(0~360 프레임) 이동은 두 경우 모두 2.6 mm입니다. 깨지는 자리는 손가락을
닫는 20 프레임뿐이었습니다.

### camera 두 대를 합치면 파지 방향이 달라집니다

머리 camera는 눈높이가 0.80 m라 테이블 위 상자의 윗면을 거의 못 봅니다. 관측된 상자
점 1222개 중 윗면 15 mm 이내가 3.9%뿐이고, GraspGen은 관측된 점구름에 조건을 걸기
때문에 위에서 내려오는 파지를 제안하지 않습니다. 후보 36개 중 수직에 가까운 것이
2개였습니다.

손목 camera를 더하면 달라집니다.

| | 점 수 | 물체 z 범위 | 윗면 15 mm 이내 |
|---|---|---|---|
| 머리 (D435i) | 1436 | 0.766 ~ 0.888 | — |
| 손목 (D405) | 2792 | 0.885 ~ 0.887 | — |
| 합계 | 4228 | | 70.2% |

합친 구름의 중심은 참값과 4 mm 차이입니다. 합치는 자리는 GraspGenX 자신의 점구름
장면 형식(`scene_loaders.load_graspgenx_json_scene`)입니다.

## 팔이 닿는 범위

오른팔 7관절만 씁니다. 허리도 다리도 안 씁니다. 관절 한계 안에서 4만 자세를 뽑아
손바닥 위치를 모은 결과입니다. 높이는 torso 기준입니다.

| 손바닥 높이 | 앞으로 최대 |
|---|---|
| −0.10 | 0.238 |
| −0.05 | 0.294 |
| 0.00 | 0.317 |
| +0.05 | 0.368 |

손바닥은 torso −0.158 아래로 못 내려갑니다. 그래서 바닥 물건은 팔만으로 못 집습니다.

머리 camera는 torso +0.006에 있고 G1은 목 관절이 없습니다. 물건이 화면에 들어오려면
눈높이 아래여야 하는데, 그 높이에서 팔이 닿는 거리는 0.30 m 남짓입니다. 좌우도 같은
문제가 있어서, 물건을 카메라 정면으로 당기면(좌우 0.10 m) 계획은 되지만 물리가 발산하고,
0.16 m에서는 IK가 전부 실패합니다. 지금 동작하는 배치는 0.23 m이고 그때 물건은 화면
오른쪽 끝에 걸칩니다.

## 사용하는 것

| 무엇 | 어디 |
|---|---|
| Grasp 생성 | [GraspGenX](https://github.com/NVlabs/GraspGenX) ([arXiv:2606.00998](https://arxiv.org/abs/2606.00998)) |
| 경로 계획 | [cuRobo](https://github.com/NVlabs/curobo) |
| 전신 제어 | [GR00T-WholeBodyControl](https://github.com/NVlabs/GR00T-WholeBodyControl) (SONIC) |
| Grasp 합성 (초기 조사) | [Dexonomy](https://github.com/JYChen18/Dexonomy) (RSS 2025, [arXiv:2504.18829](https://arxiv.org/abs/2504.18829), [project page](https://pku-epic.github.io/Dexonomy/)) |
| Simulator | [Isaac Sim](https://developer.nvidia.com/isaac/sim) 5.1 / [IsaacLab](https://github.com/isaac-sim/IsaacLab) 2.3.2 |
| Robot | [Unitree G1](https://www.unitree.com/g1) — IsaacLab의 `G1_MINIMAL_CFG` / `G1_29DOF_CFG` |
| Rack / box / tray component | [humanoid-swarm-sim](https://github.com/hooneyskywalker0127/humanoid-swarm-sim) `common/` |
| Rack, carton asset | Isaac Sim `Environments/Hospital/Props`, `Environments/Simple_Warehouse/Props` |

## 파일

| 파일 | 하는 일 |
|---|---|
| `map/build_cell.py` | 정적 셸을 만들어 `map/cell.usd`로 저장 |
| `map/cell_layout.py` | 치수와 배치 표. 값의 근거가 주석에 있음 |
| `map/props.py` | 집을 물건을 강체로 배치 |
| `map/view_cell.py` | viewer |
| `map/probe_assets.py` | 후보 asset의 실제 크기 측정 |
| `map/measure_g1_hands.py` | G1 손·어깨 치수 측정 |
| `map/look_from_g1.py` | 머리 camera에서 한 장 |
| `map/find_box.py` | 깊이에서 바닥 물체 추정, 참값 대비 채점 |
| `map/vision.py` | 추정 부분만 떼어낸 모듈 |
| `grasp/traj_from_graspgen.py` | GraspGenX 궤적을 관절값 + 물체·받침 위치로 변환 |
| `grasp/plan_scene.py` | 계획이 가정한 테이블과 대상을 셀에 다시 세움 |
| `grasp/play_in_cell.py` | 궤적을 셀에서 재생, 3인칭과 머리 camera 두 영상 |
| `grasp/capture_rgbd.py` | 머리/손목 RGB-D 한 장을 GraspGenX가 읽는 형식으로 저장 |
| `grasp/merge_captures.py` | 여러 촬영본을 한 점구름 장면으로 합침 |
| `grasp/probe_wrist_view.py` | 어느 프레임에서 손목 camera가 물체를 보는지 측정 |
| `grasp/bake_props_usd.py` | 계획의 테이블·물체를 USD로 구움 |
| `grasp/reach_clip.py` | 플래너 클립의 오른팔을 파지점으로 굽힘 |
| `grasp/find_by_text.py` | 문장을 촬영본에 접지 (C-RADIO, cuRobo 예제), `obj_lang`으로 씀 |
| `grasp/where_to_stand.py` | 여기서 닿는가, 어디에 설까, 어떻게 갈까 — cuRobo IK와 `plan_cspace` |
| `grasp/walk_clip.py` | GR00T 플래너로 회전·보행 클립 생성 |
| `grasp/render_plan_views.py` | 로봇이 본 점구름·파지 후보·팔 경로를 그린 두 영상 |
| `grasp/check_render.py` | 렌더된 영상에서 도약·넘어짐 검사 |
| `grasp/pick_by_language.sh` | 문장에서 파지까지 한 번에. 단계별 출력은 `results/fable/` |
| `grasp/compose_fable.sh` | 위 결과를 영어 자막이 달린 한 영상으로 |
| `map/measure_reach.py` | 팔만 / 팔+허리의 도달 범위 측정 |
| `common/` | 렉·상자·트레이 부품. humanoid-swarm-sim에서 가져옴 |

## 실행

IsaacLab 2.3.2 / Isaac Sim 5.1 환경에서 실행합니다. 한 문장으로 시작하는 전체
파이프라인은 이렇게 돌립니다 (env_isaaclab과 graspgenx 두 환경을 번갈아 씁니다).

```
bash grasp/pick_by_language.sh box table     # 3뷰 영상과 단계별 산출물
bash grasp/compose_fable.sh                  # 자막 달린 한 영상
```

C-RADIO는 첫 실행 때 torch.hub로 받아옵니다. graspgenx 환경에는 `open_clip_torch`와
`einops`가 더 필요합니다 (이 환경의 transformers로는 `siglip2` 어댑터가 열리지 않아
`clip` 어댑터를 씁니다).

```
conda activate env_isaaclab
python map/build_cell.py
python map/view_cell.py
```

파지·경로는 GraspGenX 저장소에서 만들고 (`end2end/e2e_grasp_demo.py`), 그 `trajectory.json`을
여기로 가져옵니다.

```
python grasp/traj_from_graspgen.py <trajectory.json>
python grasp/play_in_cell.py results/g1_graspgen.npy --video results/pick.mp4
```

머리 camera 한 장을 GraspGenX 형식으로 저장할 때는 이렇게 씁니다.

```
python grasp/capture_rgbd.py <x> <y> <yaw> --plan results/g1_graspgen.json \
    --plan-stand <x> <y> <yaw> --out results/capture
```

## Troubleshooting

- 바닥 물체가 frame 안에 안 들어옵니다. G1은 neck joint가 없어 head_link가 torso에
  고정되고, camera 각도는 mounting으로만 정해집니다. 수평으로 달면 camera 높이 0.79 m에
  vertical FoV 55.7도라 floor가 1.5 m 밖에서야 들어옵니다. 0.9 m 앞 물체는 frame 하단에
  걸려 앞면이 잘리고, 그 상태로 추정하면 중심이 76 mm 밀립니다. 30도 아래로 달면 floor가
  0.49 m부터 보입니다.
- 손이 다가가면 depth가 사라집니다. D435i는 0.40 m보다 가까운 거리에서 depth를 반환하지
  않습니다. 즉 집기 직전 구간에는 depth가 없습니다. 실기도 같으므로 sim에서 지울 대상이
  아니라 다뤄야 할 조건입니다. 다만 이걸 near clipping plane으로 구현하면 RGB까지 같이
  사라집니다. 정상적으로 render한 뒤 0.40 m 미만을 버리는 쪽이 하드웨어와 같습니다.
- point cloud가 비어 나오는데 error는 안 납니다. `cam.data.pos_w`와 `quat_w_ros`가 CPU
  device에서 전부 0으로 나옵니다 — Fabric이 CPU에서 disable되고, 그 경로로 채워지는
  buffer이기 때문입니다. 0인 quaternion으로 depth를 unproject하면 모든 point가 NaN이 되고,
  유효 point만 남기는 단계에서 조용히 전부 걸러집니다. camera pose는 stage에서 직접
  읽습니다.
