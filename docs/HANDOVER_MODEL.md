# 모델이 바뀔 때: 무엇이 누구 것이고, 어디에 보존되어 있는가

세훈님 지시(2026-09-28 17:00): **이전 모델이 만든 코드와 문서는 건드리지 말고 보존한다. 사본을 만들어 거기서
작업한다. 그리고 이 사실과 보존 위치를 모델이 바뀌어도 알 수 있게 남긴다.** 이 파일이 그 기록이다. 새 세션이나 새
모델은 작업을 시작하기 전에 이 파일을 먼저 읽는다.

## 보존 지점

| 모델 | 보존 커밋 | 태그 | 시점 |
|---|---|---|---|
| Claude Fable 5.1 | `162c21d` | `fable-260928-preserved` | 2026-09-28 17:00 (모델이 Opus로 바뀌기 직전) |

`git show fable-260928-preserved:<경로>` 로 페이블 버전 그대로 꺼낼 수 있다.
그 시점의 커밋되지 않았던 작업(`grasp/reach_from_pose.py` +121줄, `grasp/test_grasps_in_isaac.py` +33줄)도
이 커밋에 들어가 있다.

## 누가 만든 파일인가 / 어디에 쓰는가

페이블이 만든 것(수정 금지, 읽기만):
- `grasp/reach_from_pose.py`, `grasp/test_grasps_in_isaac.py`, `grasp/build_place_reference.py`,
  `grasp/build_reach_reference.py`, `grasp/walk_clip.py`, `grasp/plan_scene.py`, `grasp/play_in_cell.py`,
  `grasp/rank_handle.py`, `grasp/relabel_by_footprint.py`
- `docs/DIAGNOSIS.md` (2026-09-24 ~ 09-28 15:10)
- `영상보관/g1-factory-tidy/09/260924~260928/` 의 모든 vN 폴더와 note.txt

Opus가 쓰는 사본(여기에만 쓴다):
- 코드: 원본 이름 + `_opus` (`grasp/test_grasps_in_isaac_opus.py`, `grasp/build_place_reference_opus.py`,
  `grasp/reach_from_pose_opus.py`). 사본은 `grasp/` 안에 두어야 형제 모듈 import가 그대로 동작한다.
- 진단: `docs/DIAGNOSIS_opus.md`
- 새 영상은 기존 규칙대로 `영상보관/.../<날짜>/5지/<부품>/vN/` 에 이어서 넣는다(폴더 규칙은 공용).

## 이어서 할 때 알아야 하는 상태 (2026-09-28 17:00 기준)

- 세훈님이 **영상으로 확인한 HELD 파지**: hammer #138 (클립 `fable40v2`, 영상 260928/5지/hammer/v3·v6),
  drill #57 (클립 `fable42p`, 영상 drill/v3·v5). 이 둘이 기준이다.
- 파워 그립(손바닥 감싸기) 시도는 전부 테스터 LOST이고 영상이 없다. 세훈님 지시로 **중단**.
- 지금 하는 일: 위 두 파지로 **크레이트까지 운반**(`place_any.sh`). 크레이트 자체 집기는 보류.
