#!/usr/bin/env python3
"""
[TEST-ONLY · 삭제 예정] 이 파일 전체.

5단계 재작업 v4 반영판. 4·7단계에서 만든 가짜 D555 퍼블리셔에서 이미지(color/depth)
관련 코드는 전혀 바꾸지 않았고, "가짜 검출 결과" topic에 담는 내용의 형태만 바뀌었다.

[v4 변경점 — 팀원2 답변 반영, objects 배열 구조로 재전환]
- 기존(v3): roi_detector.fake_detect()가 검출 결과 1개(dict)를 돌려주고,
  그걸 그대로 JSON 객체로 담아 publish했다.
- 지금(v4): fake_detect()가 검출 결과 "목록"(list, 지금은 항상 원소 1개)을
  돌려주므로, 그 목록을 그대로 JSON 배열로 담아 publish한다. 실제 검출 모델이
  여러 물체를 찾아내도 이 코드는 수정할 필요가 없다 — 목록 원소 수만 늘어난다.

서버 접속 후에는 이 파일 자체가 진짜 D555 드라이버 + 진짜 검출 모델 노드로
완전히 대체되어 사라진다.
"""

import json
import pathlib

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from roi_detector import fake_detect

IMG_WIDTH = 640
IMG_HEIGHT = 480
BG_DEPTH = 2.0              # 배경 depth(m)
SWITCH_EVERY_N_FRAMES = 8   # 몇 번 publish마다 다음 위치로 바꿀지. 2Hz 기준 8프레임 = 4초.


class FakeD555Publisher(Node):
    def __init__(self):
        super().__init__('fake_d555_publisher')
        self.bridge = CvBridge()
        self.color_pub = self.create_publisher(Image, '/fake_d555/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/fake_d555/depth/image_raw', 10)
        # [TEST-ONLY · 삭제 예정] 실제 검출 모델 노드가 생기면 이 publisher는 사라지고,
        # 그 모델이 만드는 topic을 perception_pipeline.py가 대신 구독한다.
        self.detection_pub = self.create_publisher(String, '/fake_object_detector/detections', 10)

        self.test_cases = self._load_test_cases()
        self.case_index = 0
        self.frame_count = 0

        self.timer = self.create_timer(0.5, self.publish_frame)  # 2Hz
        self._log_current_case()

    def _load_test_cases(self):
        workspace_dir = pathlib.Path(__file__).resolve().parent.parent
        data_path = workspace_dir / 'sample_data' / 'fake_input.json'
        with open(data_path, 'r') as f:
            data = json.load(f)
        return data['test_cases']

    def _log_current_case(self):
        case = self.test_cases[self.case_index]
        bbox = case['bbox']
        self.get_logger().info(
            f"[{self.case_index + 1}/{len(self.test_cases)}] 케이스 '{case['name']}' 로 전환 "
            f"(bbox=({bbox['x']},{bbox['y']},{bbox['w']},{bbox['h']}), depth={case['depth']})"
        )

    def publish_frame(self):
        case = self.test_cases[self.case_index]
        bbox = case['bbox']
        depth = case['depth']
        x, y, w, h = bbox['x'], bbox['y'], bbox['w'], bbox['h']

        # 1) 가짜 RGB 이미지: 어두운 배경 + 빨간 사각형(사람이 보기 위한 시각화 용도.
        #    검출에는 쓰이지 않는다 — ROI는 아래 3)의 검출 결과 topic에서 온다.)
        color = np.zeros((IMG_HEIGHT, IMG_WIDTH, 3), dtype=np.uint8)
        color[:] = (40, 40, 40)
        cv2.rectangle(color, (x, y), (x + w, y + h), (0, 0, 255), -1)

        # 2) 가짜 depth 이미지: 배경은 멀리, 물체 영역만 그 케이스의 depth 값으로
        depth_img = np.full((IMG_HEIGHT, IMG_WIDTH), BG_DEPTH, dtype=np.float32)
        depth_img[y:y + h, x:x + w] = depth

        color_msg = self.bridge.cv2_to_imgmsg(color, encoding='bgr8')
        depth_msg = self.bridge.cv2_to_imgmsg(depth_img, encoding='32FC1')
        now = self.get_clock().now().to_msg()
        color_msg.header.stamp = now
        depth_msg.header.stamp = now
        color_msg.header.frame_id = 'fake_camera_frame'
        depth_msg.header.frame_id = 'fake_camera_frame'

        # 3) 가짜 검출 결과: v4부터는 "목록"(list)을 그대로 JSON 배열로 publish한다
        #    (팀원2 요청으로 objects 배열 구조를 재도입 — 지금은 항상 원소 1개).
        detections = fake_detect(case)
        detection_msg = String()
        detection_msg.data = json.dumps(detections)

        self.color_pub.publish(color_msg)
        self.depth_pub.publish(depth_msg)
        self.detection_pub.publish(detection_msg)

        self.frame_count += 1
        if self.frame_count % SWITCH_EVERY_N_FRAMES == 0:
            self.case_index = (self.case_index + 1) % len(self.test_cases)
            self._log_current_case()


def main():
    rclpy.init()
    node = FakeD555Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
