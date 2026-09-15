#!/usr/bin/env python3
"""
5단계 재작업 v4: 물체 검출(object detection) 결과의 바운딩박스 "목록"을
ROI 목록으로 정리하는 함수.

[v4에서 바뀐 점 — 팀원2 답변 반영, 배열 구조로 재전환]
v1(과잉 확장 — 사용자 확인 없이 배열 처리로 임의 확장) -> v2/v3(정정 — 물체 1개로
되돌림)를 거쳐, 이번엔 팀원2가 실제로 "objects 배열 안에 id+position이 들어가는
형태"를 요청해서 다시 배열 처리 구조로 전환한다. v1과 다른 점은 "이번엔 실제로
확인을 거친 정당한 확장"이라는 것 — 팀원2도 "카메라가 여러 개를 실제로 못 잡으면
배열 원소가 1개여도 된다"는 여지를 남겼으므로, 지금 fake_input.json처럼 케이스당
물체가 1개뿐이어도 결과는 항상 "배열" 형태로 다룬다.

물체 1개를 처리하는 핵심 함수(bbox_to_roi, roi_center, detection_to_roi)는
v3에서 전혀 건드리지 않았다. 이번에 되살린 것은 그걸 여러 번 호출해서 리스트로
모으는 바깥쪽 함수(detections_to_rois)뿐이다.

[TEST-ONLY 안내] 지금은(Phase 1) 실제 객체 검출 모델이 없으므로, fake_detect()가
fake_input.json에 미리 적어둔 bbox를 "검출 모델이 방금 찾아낸 결과 목록인 척"
흉내낸다. 지금은 케이스당 물체가 1개라서 항상 원소 1개짜리 리스트를 돌려주지만,
형태는 처음부터 리스트다 — 나중에 fake_input.json에 물체를 여러 개 추가하면
코드 수정 없이 그대로 여러 개를 처리한다.

[코드 사용 범위 표시]
  [PROD]                 : 서버 접속 후 진짜 데이터로도 그대로 쓰는 부분
  [TEST-ONLY · 교체 예정] : 자리는 그대로 남고 값/내용만 나중에 실제 값으로 바뀌는 부분
  [TEST-ONLY · 삭제 예정] : 통째로 사라지고, 필요하면 완전히 새 코드로 다시 작성되는 부분
"""

import json
import pathlib


# [PROD] 검출 모델이 돌려준 바운딩박스(dict: x, y, w, h)를
# (x, y, w, h) 튜플로 정리한다. 검출 모델이 나중에 진짜로 바뀌어도,
# 그 모델의 출력을 이 형식(x, y, w, h)으로만 맞춰주면 이 함수는 그대로 쓴다.
def bbox_to_roi(bbox):
    """검출 결과의 바운딩박스를 (x, y, w, h) ROI로 정리한다."""
    return bbox["x"], bbox["y"], bbox["w"], bbox["h"]


# [PROD] ROI(바운딩박스) -> 중심 픽셀. pixel_to_3d()에 넘길 (u, v)를 만든다.
def roi_center(x, y, w, h):
    u = x + w // 2
    v = y + h // 2
    return u, v


# [PROD] 검출 모델이 찾아낸 물체 1개(detection 1개)를 ROI 정보로 정리한다.
# 물체가 여러 개일 때는 이 함수를 여러 번 호출한다 (아래 detections_to_rois 참고).
def detection_to_roi(detection):
    """
    detection: {"bbox": {"x","y","w","h"}, "label": str, "score": float}
    -> {"roi": (x,y,w,h), "pixel": (u,v), "label": str, "score": float}
    """
    x, y, w, h = bbox_to_roi(detection["bbox"])
    u, v = roi_center(x, y, w, h)
    return {
        "roi": (x, y, w, h),
        "pixel": (u, v),
        "label": detection.get("label", "unknown"),
        "score": detection.get("score", 1.0),
    }


# [PROD] v4에서 되살림 — 검출 결과 "목록"을 받아 ROI 목록으로 정리한다.
# (팀원2 요청으로 배열 구조를 다시 도입 — 물체가 1개뿐이어도 항상 리스트를 돌려준다.)
def detections_to_rois(detections):
    """detections: [detection, ...] -> [roi_info, ...] (순서 유지, 물체 개수만큼)"""
    return [detection_to_roi(d) for d in detections]


# [TEST-ONLY · 삭제 예정] fake_input.json에 미리 적어둔 bbox 1개를,
# "검출 모델이 이번 프레임에서 찾아낸 결과 목록인 척" 리스트로 감싸서 돌려준다.
# 지금은 항상 원소 1개짜리 리스트지만, 나중에 fake_input.json 케이스에 물체를
# 여러 개 추가하면 이 함수도 원소가 여러 개인 리스트를 돌려주도록 바뀌면 된다.
def fake_detect(case):
    return [{"bbox": case["bbox"], "label": "unknown", "score": 1.0}]


# [TEST-ONLY · 삭제 예정] fake_input.json을 읽는 테스트 전용 헬퍼.
def _load_test_cases():
    workspace_dir = pathlib.Path(__file__).resolve().parent.parent
    data_path = workspace_dir / "sample_data" / "fake_input.json"
    with open(data_path, "r") as f:
        data = json.load(f)
    return data["test_cases"]


# [TEST-ONLY · 삭제 예정] detections_to_rois()가 잘 동작하는지 확인하는 테스트 코드일 뿐이다.
# 실제 파이프라인에서는 이 main()이 아니라 perception_pipeline.py가 detections_to_rois()를 호출한다.
def main():
    test_cases = _load_test_cases()
    for case in test_cases:
        detections = fake_detect(case)          # 가짜 검출 결과 목록(지금은 항상 원소 1개)
        rois = detections_to_rois(detections)     # 이 파일이 실제로 담당하는 부분
        u, v = rois[0]["pixel"]
        ok = abs(u - case["u"]) <= 1 and abs(v - case["v"]) <= 1
        status = "OK" if ok else "FAIL"
        print(
            f"[{case['name']:6s}] 검출 개수={len(rois)}  찾은 중심(1번째) (u={u:>3}, v={v:>3})  "
            f"기대값 (u={case['u']:>3}, v={case['v']:>3})  -> {status}"
        )


if __name__ == "__main__":
    main()
