#!/usr/bin/env python3
"""
4단계 확인용: fake_d555_publisher.py가 잘 동작하는지 확인하는 임시 구독자.

[TEST-ONLY] 이 파일 전체가 테스트 전용이다. 5단계 재작업 v4(objects 배열 구조 전환)와
무관하며 수정 대상이 아니다 — 이 파일은 color/depth 이미지 topic만 확인하고,
검출 결과(detections) topic은 보지 않는다.

color/depth 토픽에서 메시지를 하나씩 받아서, 이미지 크기와 물체 위치의
depth 값이 fake_d555_publisher.py가 만든 값과 일치하는지 출력한다.
"""

import json
import pathlib

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

EXPECTED_OBJ_DEPTH = 0.8


class FakeD555Checker(Node):
    def __init__(self):
        super().__init__('fake_d555_checker')
        self.bridge = CvBridge()
        self.color_received = False
        self.depth_received = False
        case = self._load_default_case()
        self.center_u = case['u']
        self.center_v = case['v']
        self.create_subscription(Image, '/fake_d555/color/image_raw', self.on_color, 10)
        self.create_subscription(Image, '/fake_d555/depth/image_raw', self.on_depth, 10)

    def _load_default_case(self):
        workspace_dir = pathlib.Path(__file__).resolve().parent.parent
        data_path = workspace_dir / 'sample_data' / 'fake_input.json'
        with open(data_path, 'r') as f:
            data = json.load(f)
        return data['test_cases'][0]

    def on_color(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        print(f'[color] 이미지 크기: {img.shape[1]}x{img.shape[0]} (가로x세로)')
        self.color_received = True

    def on_depth(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        value = depth[self.center_v, self.center_u]
        ok = abs(value - EXPECTED_OBJ_DEPTH) < 1e-3
        status = 'OK' if ok else 'FAIL'
        print(
            f'[depth] 물체 중심(u={self.center_u}, v={self.center_v}) depth = {value:.3f}  '
            f'(기대값 {EXPECTED_OBJ_DEPTH}) -> {status}'
        )
        self.depth_received = True

    def done(self):
        return self.color_received and self.depth_received


def main():
    rclpy.init()
    node = FakeD555Checker()
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()  # Ctrl+C를 누르면 rclpy가 이미 자동으로 shutdown을 호출해두므로,
                               # rclpy.shutdown() 대신 이미 종료됐으면 조용히 넘어가는 try_shutdown()을 쓴다.


if __name__ == '__main__':
    main()
