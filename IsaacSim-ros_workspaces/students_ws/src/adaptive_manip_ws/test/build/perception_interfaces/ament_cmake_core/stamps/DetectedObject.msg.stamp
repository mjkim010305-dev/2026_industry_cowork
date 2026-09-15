# 9단계: 물체 하나에 대한 정보.
# notes/message_spec.md v3 기준 — class_name/orientation은 아직 추가하지 않음(팀원2 요청, 후순위 보류).

int32 id                       # 프레임 내 순번(0,1,2,...). 프레임 간 추적 ID 아님 — 열린 이슈 참고.
geometry_msgs/Point position    # base_link 기준 (x, y, z), 단위 m
geometry_msgs/Vector3 size       # size.x=width(폭), size.y=depth(깊이), size.z=height(높이), 단위 m
                                  # (Vector3에는 w/d/h라는 이름이 없어서 x/y/z에 이 순서로 대응시킨다)
