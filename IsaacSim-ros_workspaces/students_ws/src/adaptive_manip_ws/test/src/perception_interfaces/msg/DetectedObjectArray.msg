# 9단계: 이 노드가 실제로 /detected_object 토픽에 publish하는 메시지.
# notes/message_spec.md v3 기준 — header.frame_id는 "base_link" 고정,
# detected=false일 때는 objects가 빈 배열이어야 한다(토픽 자체는 생략하지 않음).

std_msgs/Header header    # header.frame_id = "base_link"
bool detected              # 이번 프레임에 검출된 물체가 하나라도 있으면 true
DetectedObject[] objects   # 검출된 물체 목록. 원소가 1개뿐이어도 항상 배열 형태.
