#!/usr/bin/env python3
# [PROD] — 실제 D555 depth 토픽을 구독해서 화면 중앙 픽셀의 실측 depth와 3D 좌표를 출력.
# 원본 팀분담 실행계획서 W1 산출물("화면 중앙 픽셀 depth 출력, 대상 1개 중심 좌표 계산 -> (x,y,z) 출력") 확인용.

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
import numpy as np

DEPTH_TOPIC = '/camera/camera/depth/image_rect_raw'
DEPTH_INFO_TOPIC = '/camera/camera/depth/camera_info'


def pixel_to_3d(u, v, depth, fx, fy, cx, cy):
    x = (u - cx) * depth / fx
    y = (v - cy) * depth / fy
    z = depth
    return x, y, z


class CenterDepthChecker(Node):
    def __init__(self):
        super().__init__('center_depth_checker')
        self.intrinsics = None
        self.create_subscription(CameraInfo, DEPTH_INFO_TOPIC, self.info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, DEPTH_TOPIC, self.depth_cb, qos_profile_sensor_data)
        self.get_logger().info('depth/camera_info 기다리는 중...')

    def info_cb(self, msg):
        if self.intrinsics is None:
            k = msg.k
            self.intrinsics = dict(fx=k[0], fy=k[4], cx=k[2], cy=k[5])
            self.get_logger().info(f'depth intrinsics 확보: {self.intrinsics}')

    def depth_cb(self, msg):
        if self.intrinsics is None:
            return
        if msg.encoding != '16UC1':
            self.get_logger().warn(f'예상과 다른 encoding: {msg.encoding} (16UC1 예상)')
            return
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        cu, cv = msg.width // 2, msg.height // 2
        depth_mm = int(arr[cv, cu])
        if depth_mm == 0:
            self.get_logger().warn(f'중앙 픽셀({cu},{cv}) depth=0 - 너무 가깝거나(52cm 이내) 표면이 안 잡힘.')
            return
        depth_m = depth_mm / 1000.0
        fx, fy, cx, cy = self.intrinsics['fx'], self.intrinsics['fy'], self.intrinsics['cx'], self.intrinsics['cy']
        x, y, z = pixel_to_3d(cu, cv, depth_m, fx, fy, cx, cy)
        self.get_logger().info(
            f'중앙픽셀({cu},{cv}) depth={depth_m:.3f}m  ->  camera 기준 (x={x:.3f}, y={y:.3f}, z={z:.3f})'
        )


def main():
    rclpy.init()
    node = CenterDepthChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
