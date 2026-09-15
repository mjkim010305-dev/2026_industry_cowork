#!/usr/bin/env python3
"""
[PROD] 실제 D555 하드웨어에서 depth 이미지를 직접 구독해서
depth 기반 검출 -> 3D 위치/크기 계산 -> 안정화 -> /detected_object 로 publish하는
최종 노드. color를 쓰지 않으므로 depth 한 토픽만 구독하면 되고, 예전처럼 여러
토픽을 동기화할 필요가 없다 (fake_d555_publisher/roi_detector는 더 이상 안 씀).

추가로 /perception/debug/depth_detect 토픽에 depth를 색으로 보기 좋게 바꾼
이미지 + 검출된 물체 위에 초록 네모를 그린 디버그 화면을 같이 publish한다.
(web_video_server로 브라우저에서 확인 가능)
"""

import pathlib

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from perception_interfaces.msg import DetectedObject, DetectedObjectArray
from .depth_detector import DepthBlobDetector
from .pixel_to_3d import pixel_to_3d
from .point_cloud_utils import roi_to_point_cloud, compute_aabb
from .stabilizer import ObjectStabilizer
from .experiment_logger import ExperimentLogger

DEPTH_TOPIC = "/camera/camera/depth/image_rect_raw"      # 2026-09-14 실측 확정
DEPTH_INFO_TOPIC = "/camera/camera/depth/camera_info"
OUTPUT_TOPIC = "/detected_object"
DEBUG_TOPIC = "/perception/debug/depth_detect"
FRAME_ID = "camera_depth_optical_frame"                   # 2026-09-14 실측 확정
LOG_DIR = str(pathlib.Path.home() / "adaptive_manip_ws" / "test" / "logs")


class DetectedObjectNode(Node):
    def __init__(self):
        super().__init__('detected_object_node')
        self.bridge = CvBridge()
        self.detector = DepthBlobDetector(warmup_frames=30, background_frames=15)   
        self.stabilizer = ObjectStabilizer()
        self.logger = ExperimentLogger(log_dir=LOG_DIR)
        self.fx = self.fy = self.cx = self.cy = None
        self.frame_count = 0

        self.create_subscription(CameraInfo, DEPTH_INFO_TOPIC, self._on_camera_info, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self._on_depth, 10)
        self.publisher = self.create_publisher(DetectedObjectArray, OUTPUT_TOPIC, 10)
        self.debug_publisher = self.create_publisher(Image, DEBUG_TOPIC, 10)

        self.get_logger().info(
            "배경 학습 중입니다 - 카메라 앞에 물체를 놓지 말고 잠시 기다려주세요."
        )

    def _on_camera_info(self, msg: CameraInfo):
        self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]

    def _on_depth(self, msg: Image):
        if self.fx is None:
            return

        depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_m = depth_raw.astype(np.float32) / 1000.0  # D555는 보통 mm 단위 uint16

        if not self.detector.is_background_ready:
            self.detector.update_background(depth_m)
            progress = self.detector._warmup_count + len(self.detector._bg_accum)
            total = self.detector.warmup_frames + self.detector.background_frames
            if progress % 5 == 0 or progress >= total:
                self.get_logger().info(f"[배경 준비 중] {progress}/{total} 프레임")
            return

        detections = self.detector.detect(depth_m)
        self.frame_count += 1

        # [디버그용 - 원인 찾으면 지울 것]
        if self.frame_count % 15 == 0:
            valid = depth_m > 0
            diff = np.where(valid, self.detector.background - depth_m, 0)
            max_diff = float(np.max(diff))
            closer_count = int(np.sum(diff > self.detector.distance_threshold_m))
            self.get_logger().info(
                f"[디버그] 최대 차이={max_diff:.3f}m, threshold 넘는 픽셀 수={closer_count}, "
                f"찾은 덩어리 수={len(detections)}"
            )

        self._publish_debug_image(depth_m, detections)

        objects_msg = []
        objects_for_log = []
        seen_slots = set()

        for i, det in enumerate(detections):
            u_min, v_min, u_max, v_max = det["bbox"]
            roi = (u_min, v_min, u_max - u_min, v_max - v_min)  # (x, y, w, h)
            roi_depth = depth_m[v_min:v_max, u_min:u_max]
            valid_depth = roi_depth[roi_depth > 0]
            if valid_depth.size == 0:
                continue
            median_depth = float(np.median(valid_depth))

            position = pixel_to_3d(
                u=(u_min + u_max) / 2, v=(v_min + v_max) / 2, depth=median_depth,
                fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy,
            )
            points = roi_to_point_cloud(depth_m, roi, self.fx, self.fy, self.cx, self.cy)
            size = compute_aabb(points)
            if size is None:
                continue

            stable_position = self.stabilizer.update(slot_index=i, position=position)
            seen_slots.add(i)

            obj = DetectedObject()
            obj.id = i
            obj.position.x, obj.position.y, obj.position.z = stable_position
            obj.size.x, obj.size.y, obj.size.z = size["w"], size["d"], size["h"]
            objects_msg.append(obj)
            objects_for_log.append({
                "id": i,
                "position": {"x": stable_position[0], "y": stable_position[1], "z": stable_position[2]},
                "size": {"w": size["w"], "d": size["d"], "h": size["h"]},
            })

        self.stabilizer.reset_missing(seen_slots)

        array_msg = DetectedObjectArray()
        array_msg.header.stamp = self.get_clock().now().to_msg()
        array_msg.header.frame_id = FRAME_ID
        array_msg.detected = len(objects_msg) > 0
        array_msg.objects = objects_msg
        self.publisher.publish(array_msg)

        self.logger.log_frame(self.frame_count, array_msg.detected, objects_for_log)

    def _publish_debug_image(self, depth_m, detections):
        """depth 이미지를 색깔로 보기 좋게 바꾸고, 검출된 물체는 초록 네모로 표시."""
        clipped = np.clip(depth_m, 0, 3.0)  # 0~3m 범위만 보이게
        normalized = (clipped / 3.0 * 255).astype(np.uint8)
        color_img = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)

        for det in detections:
            u_min, v_min, u_max, v_max = det["bbox"]
            cv2.rectangle(color_img, (u_min, v_min), (u_max, v_max), (0, 255, 0), 2)

        debug_msg = self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8')
        debug_msg.header.stamp = self.get_clock().now().to_msg()
        debug_msg.header.frame_id = FRAME_ID
        self.debug_publisher.publish(debug_msg)

    def destroy_node(self):
        self.logger.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectedObjectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()