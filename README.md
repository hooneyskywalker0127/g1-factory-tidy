# g1-factory-tidy

공장 한 구역에서 Unitree G1 한 기가 바닥의 물건을 집어 렉의 빈 칸에 넣는 작업을 다룹니다.

로봇에게 물체 좌표를 주지 않는 것이 이 저장소의 전제입니다. 물체 위치는 로봇 머리에 달린
RGB-D camera에서 나오고, 참값은 추정이 얼마나 틀렸는지 채점할 때만 씁니다.

## 지금 되는 것

- 셀 맵 생성과 viewer
- 집을 물건을 강체로 배치 (렉 선반 위, 바닥)
- 머리 camera에서 바닥 물체의 위치·크기 추정, 참값 대비 채점

## 아직 안 되는 것

- 팔을 뻗어 집는 동작. 상체 IK도 하체 균형 제어도 붙지 않았습니다
- 로봇은 현재 아무것도 집지 않습니다

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

## Grasp

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

## 사용하는 것

| 무엇 | 어디 |
|---|---|
| Grasp 합성 | [Dexonomy](https://github.com/JYChen18/Dexonomy) (RSS 2025, [arXiv:2504.18829](https://arxiv.org/abs/2504.18829), [project page](https://pku-epic.github.io/Dexonomy/)) |
| Simulator | [Isaac Sim](https://developer.nvidia.com/isaac/sim) 5.1 / [IsaacLab](https://github.com/isaac-sim/IsaacLab) 2.3.2 |
| Robot | [Unitree G1](https://www.unitree.com/g1) — IsaacLab의 `G1_MINIMAL_CFG` |
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
| `common/` | 렉·상자·트레이 부품. humanoid-swarm-sim에서 가져옴 |

## 실행

IsaacLab 2.3.2 / Isaac Sim 5.1 환경에서 실행합니다.

```
conda activate env_isaaclab
python map/build_cell.py
python map/view_cell.py
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
