# g1-factory-tidy

**Unitree G1이 한 문장을 받아 공장 안의 물건을 찾고, 걸어가서, 집어 정리한다.**
전 과정이 오픈소스 세 개(GR00T-WholeBodyControl · GraspGen-X · cuRobo)와 Isaac Lab 위에서 돈다.

📺 **프로젝트 재생목록 (YouTube):** https://www.youtube.com/playlist?list=PLOijdB1dOk8Y

[![시연 영상 재생](docs/youtube_thumb.jpg)](https://youtu.be/0Tu-V0MvPYc)

> 위 영상: "책상 위 상자를 집어" 한 문장에서 파지까지, 단계마다 영어 자막. `bash grasp/pick_by_language.sh box table` 한 번의 실행이고 `grasp/compose_fable.sh`가 결과를 한 영상으로 엮는다.

---

## 목표

어질러진 공장을 정해진 상태로 되돌리는 것이 최종 목표다. 깔끔한 상태를 이미지로 주면 로봇이
지금 상태와 그 이미지를 비교해 어긋난 것을 치운다. 무엇을 어디에 둘지는 사람이 매번 지시하지
않고 그 이미지가 정한다.

로봇에게 물체 좌표를 주지 않는 것이 이 저장소의 전제다. 물체 위치는 로봇에 달린 RGB-D camera에서
나오고, 참값은 추정이 얼마나 틀렸는지 채점할 때만 쓴다.

## 파이프라인

![pipeline](docs/pipeline.png)

1 보기(C-RADIO) → 2 이동·자세(GR00T-WholeBodyControl) → 3 파지 후보(GraspGen-X) → 4 전신 도달(cuRobo) →
5 물리 검증·렌더(Isaac Lab). 운반은 2와 4를 한 번 더 지난다. 그림은 `scripts/make_pipeline_figure.py`가 그린다.

**0. 언어 → 물체 (C-RADIO, cuRobo의 예제)**
머리 camera 한 장을 NVIDIA C-RADIO로 채점한다. 모델과 텍스트 매칭은 cuRobo 자체 튜토리얼
(`curobo/examples/getting_started/feature_mapping.py`)의 것을 그대로 쓴다. 아무것도 없으면 GR00T
플래너로 제자리 회전해 다시 본다. 답은 `obj_lang`이라는 이름으로 촬영본에 써 넣고 GraspGenX가
그 이름을 읽는다 (`grasp/find_by_text.py`).

**1. GraspGen-X — 무엇을 어떻게 잡을 것인가**
![GraspGen-X](docs/graspgen.gif)
머리 RGB-D 한 장에서 물체의 점구름을 얻고 파지 후보를 만들어 점수를 매긴다. 물체의 mesh도 좌표도
주지 않는다. 손잡이가 있는 도구는 `grasp/rank_handle.py`가 손잡이 위 후보를 앞세운다.

**2. cuRobo — 거기까지 팔을 어떻게 가져갈 것인가**
![cuRobo](docs/curobo.gif)
파지 자세를 목표로 전신(다리·허리·팔 + 가상 베이스 관절) 도달을 푼다. 닿지 않는 파지는 여기서
걸러진다. 계획은 **로봇이 실제로 도착해 취한 자세와 실제 물체 자세**에서 다시 푼다 — cuRobo 원본이
정지 후 측정 상태에서 다음 계획을 시작하는 것과 같은 방식이다.

**3. GR00T (GEAR-SONIC) — 거기까지 몸을 어떻게 가져갈 것인가**
![GR00T](docs/gr00t.gif)
설 자리와 보행 클립은 GR00T의 kinematic planner가 만들고(`grasp/walk_clip.py`), 셀 안에서는 SONIC
추적 정책이 부유 베이스 위에서 다리를 움직여 실제로 걷고 무릎을 꿇는다. 허리와 팔은 배포와 같이
목표 관절을 직접 받는다. 골반을 월드에 고정하거나 옮기는 일은 없다.

![G1 vision grasp](docs/g1_vision_grasp.gif)

## 현재 상태

**되는 것**
- 셀 맵 생성과 viewer, 집을 물건을 강체로 배치(선반 위, 바닥)
- 머리 camera에서 물체 위치·크기 추정과 참값 대비 채점(중심 오차 1~8 mm)
- 한 문장 → C-RADIO → 파지 후보 → 걸어야 하는지 판단 → 설 자리 → 걷기 → 다시 보고 계획 → 집어 들기
  (책상 위 상자, `grasp/pick_by_language.sh`)
- 머리·손목 camera 두 장을 한 점구름으로 합쳐 파지 생성
- 부유 베이스: SONIC이 걷기·무릎(양무릎)·기립을 물리로 수행, 루트 쓰기 없음
- 바닥 도구(망치): 무릎 자세를 측정해 cuRobo 전신 도달을 다시 풀고, 정지 도달 구간은 cuRobo 실행
  게인으로 계획을 그대로 실행 → 29관절 0.03 rad·골반 12 mm 로 계획과 일치, 조임 접촉 유지 확인
  (`docs/DIAGNOSIS_opus.md`)

**아직 안 되는 것**
- 바닥 도구를 든 채 일어나 크레이트까지 운반·투입 (진행 중, `fable/v4`)
- 닫힌 루프 인식: 도착 후 한 번 다시 보고, 그 뒤에는 눈을 감는다
- 실기 이식: 위치 측정을 시뮬 상태 덤프로 대신한다. 실기에서는 상태 추정과 camera가 맡을 자리

## 실행

Isaac Lab 2.3.2 / Isaac Sim 5.1. `env_isaaclab`(시뮬·렌더)과 `graspgenx`(파지·cuRobo) 두 conda 환경을
번갈아 쓴다. C-RADIO는 첫 실행 때 torch.hub로 받는다(`open_clip_torch`, `einops` 필요).

```bash
# 책상 위 상자: 한 문장에서 3뷰 영상까지
bash grasp/pick_by_language.sh box table
bash grasp/compose_fable.sh                  # 자막 달린 한 영상

# 바닥 도구(망치): 걷기 → 무릎 → 측정 → 재계획 → 렌더 (단계별 명령은 영상 폴더의 evidence/run.sh)
bash grasp/floor_object_chain.sh             # RUN/QUERY/MESH/OBJ 환경변수로 대상 지정

# 셀만
conda activate env_isaaclab
python map/build_cell.py && python map/view_cell.py
```

## 파일

| 파일 | 하는 일 |
|---|---|
| `map/build_cell.py`, `map/cell_layout.py`, `map/props.py`, `map/view_cell.py` | 셀 생성·치수 표·물건 배치·viewer |
| `map/look_from_g1.py`, `map/find_box.py`, `map/vision.py` | 머리 camera 촬영, 깊이에서 물체 추정, 채점 |
| `grasp/find_by_text.py` | 문장을 촬영본에 접지 (C-RADIO, cuRobo 예제) |
| `grasp/capture_rgbd.py`, `grasp/merge_captures.py` | 머리/손목 RGB-D를 GraspGenX 형식으로, 여러 장 합치기 |
| `grasp/where_to_stand.py`, `grasp/walk_clip.py` | 설 자리·경로(cuRobo IK, `plan_cspace`), GR00T 플래너 보행 클립 |
| `grasp/reach_from_pose_opus.py` | 측정 자세에서 cuRobo 전신 도달 (측정된 물체 자세로 파지 이동) |
| `grasp/kneel_from_state_opus.py`, `grasp/clip_tools_opus.py` | 측정→재계획용 클립 도구 |
| `grasp/build_reach_reference_opus.py`, `grasp/rise_reference_opus.py` | 걷기+도달+기립 레퍼런스와 손 일정 |
| `grasp/play_in_cell_opus.py` | 셀 안 재생·렌더: SONIC 부유 베이스, 상체 직접 PD, 상태 덤프 |
| `grasp/rank_handle.py`, `grasp/test_grasps_in_isaac.py` | 손잡이 순위, 파지 물리 검증 |
| `grasp/pick_by_language.sh`, `grasp/floor_object_chain.sh`, `grasp/compose_fable.sh` | 전체 체인과 영상 편집 |
| `docs/DIAGNOSIS.md`, `docs/DIAGNOSIS_opus.md` | 실패 원인 진단 기록 (측정값과 출처) |
| `common/` | 렉·상자·트레이 부품 (humanoid-swarm-sim) |

## 기술 노트

### 셀
바닥 6 x 4 m, 북·서쪽 벽, 서쪽 벽에 폭 1.2 m 출입구(어깨 0.45 m + 들고 가는 물건 0.60 m). 렉은 북쪽 벽에
2대(`Environments/Hospital/Props/SM_MedShelf_01d`). 정적인 것은 `map/cell.usd`로 굽고 집을 물건은
`map/props.py`가 실행 시마다 놓는다. 바닥 상자 0.30 x 0.30 x 0.32 m, 1.5 kg — 손바닥 간격 0.326 m에
맞춘 치수(`map/measure_g1_hands.py`).

### 손과 camera
- 손: Inspire RH56 5지(오른손). Isaac Lab `G1_INSPIRE_FTP_CFG` 게인(kp 10 / kd 0.2 / effort 30, 위치
  제어), 닫힘 목표는 GraspGenX Inspire 프로파일(엄지 0.6 / 손가락 1.47). GraspGenX의 kp 2000은
  Newton 소프트 접촉용이라 PhysX에 옮기면 관통 조임이 된다(`docs/DIAGNOSIS_opus.md` 260930).
- 머리: Intel RealSense D435i 기하 그대로 — 깊이 시야각 86도, 848 x 480, 최소 거리 0.3~0.4 m,
  30도 아래를 본다(목 관절이 없어 수평이면 바닥이 1.5 m 밖에서야 보인다). 깊이 최소 거리는
  근평면이 아니라 render 뒤 버리는 방식(실기와 같이 색은 남고 깊이만 없다).
- 손목: Intel RealSense D405 (영상 녹화용, 인식에는 아직 안 씀).

### 추정 정확도 (머리 camera, 바닥 상자, 참값 대비)
| 로봇 위치 | 상자까지 | 중심 오차 | 높이 오차 |
|---|---|---|---|
| (-2.00, -0.40) 정면 | 0.80 m | 6 mm | +2 mm |
| (-2.00, +0.30) 정면 | 1.50 m | 8 mm | +3 mm |
| (-1.20, -1.20) 측면 | 0.80 m | 1 mm | +2 mm |
| (-2.60, -0.55) 대각 | 0.86 m | 2 mm | +2 mm |
| (-2.00, -0.20) 정면 | 1.00 m | 7 mm | +2 mm |

바닥이 z=0이라는 것을 코드에 넣었으므로 실기에서는 바닥 평면부터 추정해야 한다.

### camera 두 대를 합치면 파지 방향이 달라진다
머리 camera(눈높이 0.80 m)는 테이블 위 상자의 윗면을 거의 못 본다(윗면 15 mm 이내 점 3.9%). 손목
camera를 더하면 70.2%가 되고 위에서 내려오는 파지가 나온다. 합치는 자리는 GraspGenX 자신의
점구름 장면 형식(`scene_loaders.load_graspgenx_json_scene`).

### 바닥 물체는 팔만으로 못 집는다
오른팔 7관절만으로는 손바닥이 torso −0.158 m 아래로 못 내려간다. 그래서 바닥 도구는 GR00T의 무릎
자세와 cuRobo 전신 도달을 쓴다. 측정 결과 SONIC의 한쪽 무릎(mode 6)은 플래너보다 골반이 6 cm
높아 모든 파지가 닿지 않고, 양무릎(mode 5)에서 닿는다.

## 사용하는 것

| 무엇 | 어디 |
|---|---|
| 파지 생성 | [GraspGenX](https://github.com/NVlabs/GraspGenX) ([arXiv:2606.00998](https://arxiv.org/abs/2606.00998)) |
| 전신 도달·경로 | [cuRobo](https://github.com/NVlabs/curobo) |
| 보행·자세·전신 제어 | [GR00T-WholeBodyControl](https://github.com/NVlabs/GR00T-WholeBodyControl) (GEAR-SONIC) |
| 언어 접지 | NVIDIA C-RADIO (cuRobo `feature_mapping` 예제) |
| 시뮬레이터 | [Isaac Sim](https://developer.nvidia.com/isaac/sim) 5.1 / [Isaac Lab](https://github.com/isaac-sim/IsaacLab) 2.3.2 |
| 로봇 | [Unitree G1](https://www.unitree.com/g1) 29 DoF + Inspire RH56 — Isaac Lab `G1_29DOF_CFG` / `G1_INSPIRE_FTP_CFG` |
| 부품·에셋 | [humanoid-swarm-sim](https://github.com/hooneyskywalker0127/humanoid-swarm-sim) `common/`, Isaac Sim `Environments/Hospital/Props`, `Simple_Warehouse/Props` |

## Troubleshooting

- 바닥 물체가 frame 안에 안 들어온다 → camera를 30도 아래로 단다(위 "손과 camera").
- 손이 다가가면 depth가 사라진다 → D435i의 최소 거리. 실기와 같으므로 지울 대상이 아니라 다룰 조건이다.
- point cloud가 비어 나오는데 error는 안 난다 → CPU device에서 `cam.data.pos_w`/`quat_w_ros`가 0으로
  나온다(Fabric 비활성). camera pose는 stage에서 직접 읽는다.
- articulation 생성 실패(`PhysxMimicJointAPI` 12건 → `root_joint`) → 부유 베이스는 `/World/G1/root_joint`를
  비활성화하고 articulation root를 pelvis에 둔다(`play_in_cell_opus.py`, SONIC 원본 `fix_base=False`와 같은 형태).
