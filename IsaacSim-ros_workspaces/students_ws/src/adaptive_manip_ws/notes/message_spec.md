# DetectedObject 메시지 스펙 (v3 — 팀원2 답변 반영, objects 배열 구조로 전환)

## 원본에서 이미 정해진 것 (팀 분담 실행계획서 "2. 두 팀 사이 인터페이스")
- 메시지 이름: DetectedObject (단수) — 다만 실제 토픽에는 이 문서 아래 `objects` 배열
  안에 담아 보낸다 (v3에서 변경, 아래 "v3" 항목 참고)
- 필드: object id / position(x,y,z) / size(width,depth,height) / orientation(필요 시 추가)
- 공통 약속: DetectedObject 메시지 형식 고정 / frame 이름 고정 / 단위 m,rad 통일
- (구현계획서) ROS_DOMAIN_ID=30, 토픽/frame 이름은 `ros2 topic list`로 실측 후 코드 작성
- (구현계획서) 첫날 Mock DetectedObject publisher부터 만든다 (역할 분담표상 팀원2 담당)

## v2 (2단계 확정 — 이후 v3에서 바뀐 부분 포함, 이력 보존용)
- 토픽: `/detected_object` (v3에서도 이름은 유지, 메시지 구조만 배열로 바뀜)
- size를 아직 모를 때 값: (0, 0, 0)
- frame_id 임시값: `camera_color_optical_frame`  ← **v3에서 `base_link`로 변경됨**, 아래 참고
- 여러 물체 동시 처리는 지금 범위 밖 (필요해지면 그때 배열로 확장) ← **v3에서 팀원2 확인을
  거쳐 정식으로 폐기됨**, 아래 참고

## v3 — 팀원2 확인 반영 (2026-09-12, objects 배열 구조로 전환)
확정 4개 항목만 반영. class(물체 종류)/orientation은 필드 자체를 아직 추가하지 않는다.

- **header.frame_id**: `"base_link"` 고정 (이전 `"camera_color_optical_frame"` 임시값에서 변경).
  camera → base_link 변환을 누가 수행하는지는 아직 완전히 정리 안 됨 — Phase 1(가짜 데이터,
  실제 TF 트리 없음) 동안은 Perception 코드 안의 고정 오프셋 스텁으로 대체하고, 서버 연결 후
  tf2_ros 기반 실제 변환으로 교체 예정 (`[TEST-ONLY · 교체 예정]`, `scripts/perception_pipeline.py`의
  `transform_to_base_link()` 참고).
- **detected**: bool — 이번 프레임에 검출된 물체가 하나라도 있으면 true, 없으면 false.
- **objects**: DetectedObject[] — 검출된 물체 목록. 지금은 원소가 1개뿐이어도 항상 배열 형태
  (팀원2: "카메라가 여러 개를 실제로 못 잡으면 배열 원소가 1개여도 된다").
  - `id` (int32): 프레임 내 순번(0, 1, 2, ...). **프레임 간 동일 물체 추적(tracking)은 아직
    하지 않는다** — 필요해지면 별도 논의 (열린 이슈).
  - `position` {x, y, z} (m, base_link 기준)
  - `size` {w, d, h} (m, 기존 스펙 유지 — 8단계 Point Cloud/AABB 계산 결과로 채워질 예정,
    그 전까지는 값 없음)
- **class_name / orientation**: 아직 필드 추가 안 함 (팀원2 요청으로 후순위 보류 — "위치
  좌표로 접근·동작하는 것부터 되면 된다").
- **publish 방식**: 연속 스트리밍 — 일정 주기로 계속 publish (한 번성 아님). 물체를 못 찾은
  프레임에도 topic을 생략하지 않고 `detected=false` + 빈 `objects` 배열로 채워서 publish한다.
  (실제 ROS2 publish 노드 자체는 9단계 정식 패키지 생성 시점에 만들어진다 — 지금
  `perception_pipeline.py`는 이 계산·검증만 미리 확인하는 테스트 노드다.)
- **메시지 필드 최종 리뷰**: 팀원2가 나중에 연구실에서 실제 로봇·토픽 구조를 보고 확인하기로
  함. 그 전까지는 "지금 개발 가능한 것부터" 먼저 만들고 나중에 맞춘다.

## DetectedObject.msg (초안 — 실제 .msg 파일은 9단계 정식 패키지 생성 시점에 작성)
```
string id                              # 참고: 이 초안은 v2(물체 1개 전제) 시점 것이라
                                        # 9단계에서 DetectedObjectArray 형태로 다시 정리 필요
geometry_msgs/Point position
geometry_msgs/Vector3 size
geometry_msgs/Quaternion orientation   # 선택, 당분간 미사용
```

## 토픽
`/detected_object` — 이름은 v2 그대로 유지 (내 제안, 원본에 명시 안 됨, Gate 0에서
팀원2와 재확인 필요). 다만 v3부터 이 topic에 담기는 건 "물체 1개"가 아니라
"`objects` 배열을 포함한 메시지 1개"로 바뀐다.

## 아직 확인 필요한 것 (열린 이슈)
- camera → base_link 변환을 Perception이 계속 코드로 직접 수행하는 게 맞는지, 서버 연결
  시점에 팀원2/시스템 담당과 재확인 필요.
- `objects[].id`를 프레임 간 추적 ID로 확장해야 하는지는 필요해지면 별도 논의.
- v3(동기화 버그 수정, `ApproximateTimeSynchronizer`) 코드가 실제로 5개 케이스 FAIL=0으로
  도는지 사용자 재실행 확인이 아직 안 된 채로, 이번에 배열 구조(v4)까지 얹은 상태다 —
  가능하면 먼저 v3부터 실제로 돌려서 확인하는 것을 권장한다.
