#!/usr/bin/env python3
"""
scripts/roi_detector.py(5단계 재작업 v4)를 정식 패키지 안으로 그대로 옮긴 것.
bbox_to_roi/roi_center/detection_to_roi/detections_to_rois는 [PROD] — 실제
검출 모델이 붙어도 그대로 재사용한다. fake_detect()는 여전히
[TEST-ONLY · 삭제 예정]이며, 이 패키지의 fake_d555_publisher.py가 테스트용으로
계속 사용한다(실제 검출 모델 노드가 생기면 fake_d555_publisher.py 자체가
사라지면서 이 함수도 같이 사라진다).
"""


def bbox_to_roi(bbox):
    """검출 결과의 바운딩박스를 (x, y, w, h) ROI로 정리한다."""
    return bbox["x"], bbox["y"], bbox["w"], bbox["h"]


def roi_center(x, y, w, h):
    u = x + w // 2
    v = y + h // 2
    return u, v


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


def detections_to_rois(detections):
    """detections: [detection, ...] -> [roi_info, ...] (순서 유지, 물체 개수만큼)"""
    return [detection_to_roi(d) for d in detections]


# [TEST-ONLY · 삭제 예정] fake_d555_publisher.py 전용 — 실제 검출 모델이 붙으면 사라진다.
def fake_detect(case):
    return [{"bbox": case["bbox"], "label": "unknown", "score": 1.0}]
