#!/usr/bin/env python3
# [TEST-ONLY · 교체 예정] — 지금은 "핑크 띠" 색으로 찾지만, 나중에 실제 검출 모델(YOLO 등)이
# 정해지면 이 함수만 통째로 교체됨. 나머지(3D 위치 계산, publish 등)는 안 바뀜.

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

COLOR_TOPIC = '/camera/camera/color/image_raw'
DEBUG_TOPIC = '/perception/debug/color_detect'

# 사용자가 준 사진 속 핑크 테이프에서 실측한 색 기준 (필요하면 조정 가능)
LOWER1 = np.array([155, 25, 120]); UPPER1 = np.array([179, 140, 255])
LOWER2 = np.array([0,   25, 120]); UPPER2 = np.array([8,   140, 255])


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.bridge = CvBridge()
        self.create_subscription(Image, COLOR_TOPIC, self.color_cb, qos_profile_sensor_data)
        self.debug_pub = self.create_publisher(Image, DEBUG_TOPIC, 10)
        self.get_logger().info(f'{COLOR_TOPIC} 기다리는 중...')

    def color_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(cv2.inRange(hsv, LOWER1, UPPER1),
                               cv2.inRange(hsv, LOWER2, UPPER2))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        debug = frame.copy()

        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 200:  # 너무 작은 노이즈는 무시
                x, y, w, h = cv2.boundingRect(c)
                cu, cv_ = x + w // 2, y + h // 2
                cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(debug, (cu, cv_), 5, (0, 0, 255), -1)
                self.get_logger().info(f'검출됨: bbox=({x},{y},{w},{h}) 중심=({cu},{cv_})')
            else:
                self.get_logger().info('핑크 영역이 너무 작음 (노이즈로 판단, 무시)')
        else:
            self.get_logger().info('검출 안 됨 - 핑크 띠가 화면에 안 보이나?')

        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)


def main():
    rclpy.init()
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
