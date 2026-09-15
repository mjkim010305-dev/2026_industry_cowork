#!/usr/bin/env python3
"""
[TEST-ONLY · 삭제 예정] 이 파일 전체.

scripts/fake_d555_publisher.py(v4)를 9단계 정식 패키지 형태로 옮긴 것. 동작은
거의 같지만 두 가지가 다르다:

  1) fake_input.json을 소스 트리 상대경로가 아니라, colcon으로 설치된 뒤에도
     항상 찾을 수 있는 "패키지 공유 디렉터리"(ament_index_python)에서 읽는다 —
     정식 패키지는 설치되고 나면 실행 위치가 소스 코드 위치와 달라지기 때문에
     이렇게 해야 한다 (setup.py의 data_files 참고).
  2) 5개 케이스를 한 바퀴 돈 뒤 "물체 없음" 프레임을 하나 끼워 넣는다 —
     detected=false 경로가 실제로 한 번은 발생하도록 데모하기 위해서다
     (팀원2가 확인한 스펙: 미검출 시에도 topic을 생략하지 말고 detected=false로
     publish, notes/message_spec.md v3 참고).

서버 접속 후에는 이 노드 자체가 진짜 D555 드라이버 + 진짜 검출 모델 노드로
완전히 대체되어 사라진다.
"""

import json
import pathlib

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

IMG_WIDTH = 640
IMG_HEIGHT = 480
BG_DEPTH = 2.0
SWITCH_EVERY_N_FRAMES = 8

NO_OBJECT_CASE = None  # 이 값이 나오면 "이번엔 물체가 하나도 없는 프레임"이라는 뜻


class FakeD555Publisher(Node):
    def __init__(self):
        super().__init__('fake_d555_publisher')
        self.bridge = CvBridge()
        self.color_pub = self.create_publisher(Image, '/fake_d555/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/fake_d555/depth/image_raw', 10)
        self.detection_pub = self.create_publisher(String, '/fake_object_detector/detections', 10)

        test_cases = self._load_test_cases()
        # 5개 케이스 + "물체 없음" 프레임 1개를 순환한다.
        self.cases = test_cases + [NO_OBJECT_CASE]
        self.case_index = 0
        self.frame_count = 0

        self.timer = self.create_timer(0.5, self.publish_frame)  # 2Hz
        self._log_current_case()

    def _load_test_cases(self):
        share_dir = pathlib.Path(get_package_share_directory('perception_pkg'))
        data_path = share_dir / 'sample_data' / 'fake_input.json'
        with open(data_path, 'r') as f:
            data = json.load(f)
        return data['test_cases']

    def _log_current_case(self):
        case = self.cases[self.case_index]
        if case is None:
            self.get_logger().info(f"[{self.case_index + 1}/{len(self.cases)}] '물체 없음' 프레임으로 전환")
        else:
            bbox = case['bbox']
            self.get_logger().info(
                f"[{self.case_index + 1}/{len(self.cases)}] 케이스 '{case['name']}' 로 전환 "
                f"(bbox=({bbox['x']},{bbox['y']},{bbox['w']},{bbox['h']}), depth={case['depth']})"
            )

    def publish_frame(self):
        case = self.cases[self.case_index]

        color = np.zeros((IMG_HEIGHT, IMG_WIDTH, 3), dtype=np.uint8)
        color[:] = (40, 40, 40)
        depth_img = np.full((IMG_HEIGHT, IMG_WIDTH), BG_DEPTH, dtype=np.float32)

        if case is None:
            detections = []  # 이번 프레임엔 검출된 물체가 없다 (detected=false 데모용)
        else:
            bbox = case['bbox']
            depth = case['depth']
            x, y, w, h = bbox['x'], bbox['y'], bbox['w'], bbox['h']
            cv2.rectangle(color, (x, y), (x + w, y + h), (0, 0, 255), -1)
            depth_img[y:y + h, x:x + w] = depth
            detections = [{"bbox": bbox, "label": "unknown", "score": 1.0}]

        color_msg = self.bridge.cv2_to_imgmsg(color, encoding='bgr8')
        depth_msg = self.bridge.cv2_to_imgmsg(depth_img, encoding='32FC1')
        now = self.get_clock().now().to_msg()
        color_msg.header.stamp = now
        depth_msg.header.stamp = now
        color_msg.header.frame_id = 'fake_camera_frame'
        depth_msg.header.frame_id = 'fake_camera_frame'

        detection_msg = String()
        detection_msg.data = json.dumps(detections)

        self.color_pub.publish(color_msg)
        self.depth_pub.publish(depth_msg)
        self.detection_pub.publish(detection_msg)

        self.frame_count += 1
        if self.frame_count % SWITCH_EVERY_N_FRAMES == 0:
            self.case_index = (self.case_index + 1) % len(self.cases)
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
