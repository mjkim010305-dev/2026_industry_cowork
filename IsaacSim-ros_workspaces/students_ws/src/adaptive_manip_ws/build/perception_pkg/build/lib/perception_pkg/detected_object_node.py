#!/usr/bin/env python3
"""
9단계: pose+size 통합 publish 노드 — 정식 ROS2 패키지(perception_pkg) 버전.

지금까지 scripts/perception_pipeline.py(v5, [TEST-ONLY] 검증 노드)가 화면에
출력만 하던 계산(픽셀+depth -> 3D, ROI -> AABB 크기, camera->base_link 변환)을
그대로 재사용하되, 이 노드는:

  - 진짜 DetectedObjectArray 메시지(perception_interfaces)로 /detected_object에
    실제 publish한다 (v5까지는 계산 결과를 화면에 찍기만 하고 topic publish는 안 했다).
  - 10단계 안정화(ObjectStabilizer)를 물체(슬롯)별로 적용해서 흔들림/이상치를 줄인다.
  - 11단계 실험 로그(ExperimentLogger)를 매 프레임 CSV로 남긴다
    (~/adaptive_manip_ws/logs/ 아래).
  - 검출된 물체가 없는 프레임도 detected=False + 빈 objects 배열로 계속
    publish한다 (팀원2 확인 사항 — notes/message_spec.md v3 참고). fake_d555_publisher가
    주기적으로 끼워 넣는 "물체 없음" 프레임에서 이 경로가 실제로 확인된다.

[코드 사용 범위 표시]
  [PROD]                 : 서버 접속 후 진짜 데이터로도 그대로 쓰는 부분
  [TEST-ONLY · 교체 예정] : 자리는 그대로 남고 값/내용만 나중에 실제 값으로 바뀌는 부분
  [TEST-ONLY · 삭제 예정] : 통째로 사라지고, 필요하면 완전히 새 코드로 다시 작성되는 부분
"""

import json
import pathlib

import message_filters
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String

from perception_interfaces.msg import DetectedObject, DetectedObjectArray

from perception_pkg.base_link_transform import transform_to_base_link
from perception_pkg.experiment_logger import ExperimentLogger
from perception_pkg.pixel_to_3d import pixel_to_3d
from perception_pkg.point_cloud_utils import compute_aabb, roi_to_point_cloud
from perception_pkg.roi_detector import detections_to_rois
from perception_pkg.stabilizer import ObjectStabilizer


# [TEST-ONLY · 삭제 예정] 실제 CameraInfo topic 구독으로 완전히 교체된다.
def load_camera_intrinsics():
    share_dir = pathlib.Path(get_package_share_directory('perception_pkg'))
    data_path = share_dir / 'sample_data' / 'fake_input.json'
    with open(data_path, 'r') as f:
        data = json.load(f)
    return data['camera_intrinsics']


class DetectedObjectNode(Node):
    def __init__(self):  # [PROD] 뼈대 구조
        super().__init__('detected_object_node')
        self.bridge = CvBridge()

        intr = load_camera_intrinsics()  # [TEST-ONLY · 삭제 예정]
        self.fx, self.fy, self.cx, self.cy = intr['fx'], intr['fy'], intr['cx'], intr['cy']

        # [PROD] 팀원2에게 넘기는 실제 토픽. 메시지 타입은 이제 String이 아니라
        # 정식 커스텀 메시지(perception_interfaces/DetectedObjectArray)다.
        self.publisher = self.create_publisher(DetectedObjectArray, '/detected_object', 10)

        # [TEST-ONLY · 교체 예정] 문자열 값만 실제 D555 topic 이름으로 바뀐다.
        color_sub = message_filters.Subscriber(self, Image, '/fake_d555/color/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/fake_d555/depth/image_raw')
        detection_sub = message_filters.Subscriber(self, String, '/fake_object_detector/detections')
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub, detection_sub], queue_size=10, slop=0.1, allow_headerless=True
        )
        self.sync.registerCallback(self.on_frame)

        self.stabilizer = ObjectStabilizer(window_size=5, outlier_threshold_m=0.05)  # 10단계
        log_dir = pathlib.Path.home() / 'adaptive_manip_ws' / 'logs'
        self.logger = ExperimentLogger(log_dir=log_dir)  # 11단계
        self.get_logger().info(f'실험 로그 저장 위치: {self.logger.path}')

        self.frame_count = 0
        self.get_logger().info(
            'detected_object_node 시작 (9단계 정식 패키지) - /detected_object 로 연속 publish 예정'
        )

    def on_frame(self, color_msg, depth_msg, detection_msg):  # [PROD] 기본 구조
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        detections = json.loads(detection_msg.data)

        self.frame_count += 1
        rois = detections_to_rois(detections)

        msg = DetectedObjectArray()
        msg.header = Header()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()

        objects_for_log = []
        seen_slots = set()

        for idx, roi_info in enumerate(rois):
            x, y, w, h = roi_info['roi']

            # 8단계: point cloud -> AABB 크기
            points = roi_to_point_cloud(depth_img, (x, y, w, h), self.fx, self.fy, self.cx, self.cy)
            aabb = compute_aabb(points)
            if aabb is None:
                continue  # ROI 안에 유효한 depth 픽셀이 없음 — 이 물체는 건너뜀

            median_depth = float(np.median(points[:, 2]))
            u, v = roi_info['pixel']
            X_cam, Y_cam, Z_cam = pixel_to_3d(u, v, median_depth, self.fx, self.fy, self.cx, self.cy)
            X_base, Y_base, Z_base = transform_to_base_link(X_cam, Y_cam, Z_cam)

            # 10단계: 슬롯(=현재는 배열 인덱스) 기준 안정화.
            # objects[].id가 아직 프레임 간 추적 ID가 아니라서 쓰는 임시 방식이다
            # (notes/message_spec.md 열린 이슈 참고).
            X_stable, Y_stable, Z_stable = self.stabilizer.update(idx, (X_base, Y_base, Z_base))
            seen_slots.add(idx)

            obj = DetectedObject()
            obj.id = idx
            obj.position = Point(x=X_stable, y=Y_stable, z=Z_stable)
            obj.size = Vector3(x=aabb['w'], y=aabb['d'], z=aabb['h'])
            msg.objects.append(obj)

            objects_for_log.append({
                "id": idx,
                "position": {"x": X_stable, "y": Y_stable, "z": Z_stable},
                "size": {"w": aabb['w'], "d": aabb['d'], "h": aabb['h']},
            })

        self.stabilizer.reset_missing(seen_slots)
        msg.detected = len(msg.objects) > 0  # 팀원2 확정 사항 — notes/message_spec.md v3

        self.publisher.publish(msg)  # [PROD] 물체가 없어도(빈 objects) 매 프레임 publish한다
        self.logger.log_frame(self.frame_count, msg.detected, objects_for_log)  # 11단계

        if msg.detected:
            print(f"[frame {self.frame_count:>3}] detected=True  objects={len(msg.objects)}개  -> /detected_object publish")
            for o in msg.objects:
                print(
                    f"    id={o.id}  base_link position=({o.position.x:+.3f}, {o.position.y:+.3f}, {o.position.z:+.3f})  "
                    f"size(w={o.size.x:.3f}, d={o.size.y:.3f}, h={o.size.z:.3f})"
                )
        else:
            print(f"[frame {self.frame_count:>3}] detected=False  objects=[]  -> /detected_object publish")

    def destroy_node(self):
        self.logger.close()
        super().destroy_node()


def main():  # [PROD]
    rclpy.init()
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
