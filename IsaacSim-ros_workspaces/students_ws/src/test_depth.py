import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Quaternion
from cv_bridge import CvBridge
import cv2
import numpy as np
import open3d as o3d
import tf_transformations # pip install tf-transformations 또는 ros-humble-tf-transformations

class DepthToNormalPose(Node):
    def __init__(self):
        super().__init__('depth_to_normal_pose')

        # 1. 파라미터 설정 (필요에 따라 수정)
        self.depth_topic = '/depth_camera'
        self.camera_info_topic = '/depth_camera_info'
        # 1_obs 환경에서, tb3의 초기값에서 정면을 보는 상자에 맞춘 값.
        self.roi_top_left = (660, 240)      # ROI 좌상단 (x, y) 픽셀
        self.roi_bottom_right = (1310, 625)  # ROI 우하단 (x, y) 픽셀
        self.approach_distance = 0.2          # 물체 표면에서 몇 m 앞에서 멈출 것인가

        # 2. ROS Subscriber & Publisher
        self.bridge = CvBridge()
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        
        # 카메라 내적 파라미터(Intrinsics)를 받기 위한 subscriber
        self.info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.info_callback, 10)
        self.camera_intrinsics = None

        # 결과 Pose 발행
        self.pose_pub = self.create_publisher(PoseStamped, '/target_approach_pose', 10)
        
        self.get_logger().info('Depth to Normal Pose Node started.')

    def info_callback(self, msg):
        """카메라 파라미터를 한 번만 받아서 저장"""
        if self.camera_intrinsics is None:
            # [[fx, 0, cx], [0, fy, cy], [0, 0, 1]] 형태의 K 행렬
            self.camera_intrinsics = msg.k.reshape((3, 3))
            self.get_logger().info('Camera intrinsics received.')

    def depth_callback(self, msg):
        if self.camera_intrinsics is None:
            self.get_logger().warn('Waiting for camera intrinsics...', once=True)
            return

        try:
            # 1. ROS Depth Image -> OpenCV/Numpy (Float32, Meter 단위 가정)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_array = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0)

            # 2. ROI(관심 영역) 자르기
            x1, y1 = self.roi_top_left
            x2, y2 = self.roi_bottom_right
            roi_depth = depth_array[y1:y2, x1:x2]

            # 3. ROI 영역의 3D Point Cloud 복원
            fx, fy = self.camera_intrinsics[0, 0], self.camera_intrinsics[1, 1]
            cx, cy = self.camera_intrinsics[0, 2], self.camera_intrinsics[1, 2]

            # ROI 내 유효한 픽셀 인덱스 추출
            h, w = roi_depth.shape
            grid_y, grid_x = np.mgrid[0:h, 0:w]
            
            # ROI 내의 절대 픽셀 좌표 계산
            abs_x = grid_x + x1
            abs_y = grid_y + y1
            
            Z = roi_depth
            valid_mask = Z > 0.05 # 5cm 이상 먼 곳만 처리
            
            X = (abs_x[valid_mask] - cx) * Z[valid_mask] / fx
            Y = (abs_y[valid_mask] - cy) * Z[valid_mask] / fy
            Z_valid = Z[valid_mask]

            if len(Z_valid) < 10:
                return

            points = np.vstack((X, Y, Z_valid)).T

            # 4. Open3D PointCloud 객체 생성
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)

            # 다운샘플링 (성능 및 노이즈)
            pcd = pcd.voxel_down_sample(voxel_size=0.005) # 5mm

            # 5. 법선(Normal) 추정 및 카메라 방향 정렬
            pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.03, max_nn=30))
            # 카메라 좌표계 원점 (0,0,0)을 바라보도록 정렬 (매우 중요)
            pcd.orient_normals_towards_camera_location(camera_location=np.array([0., 0., 0.]))

            # 6. ROI 영역의 평균 표면 정보 계산
            mean_points = np.mean(np.asarray(pcd.points), axis=0) # 물체 표면 중앙
            mean_normal = np.mean(np.asarray(pcd.normals), axis=0) # 평균 법선 벡터
            mean_normal /= np.linalg.norm(mean_normal) # 정규화

            self.get_logger().info(f'mean surface: {mean_points}')
            self.get_logger().info(f'mean vector: {mean_normal}')


            # 7. 접근 Pose (목표 Pose) 생성
            # 목표 위치: 표면 중앙점에서 법선 방향으로 approach_distance만큼 떨어진 곳
            target_position = mean_points + (mean_normal * self.approach_distance)

            # 목표 회전: 법선 벡터와 일치하는 회전 쿼터니언 계산
            # 'z-axis' 접근 방식 (가장 흔함)
            # 회전 행렬을 만들기 위해 z축을 法線벡터로 삼고, 이에 직교하는 x, y축 계산
            z_axis = -mean_normal # 로봇 그리퍼는 물체를 향해야 하므로 법선의 반대 방향
            
            # 임의의 orthogonal 벡터 계산
            if abs(z_axis[0]) > 0.9: 
                ref_axis = np.array([0.0, 1.0, 0.0])
            else:
                ref_axis = np.array([1.0, 0.0, 0.0])
                
            y_axis = np.cross(z_axis, ref_axis)
            y_axis /= np.linalg.norm(y_axis)
            x_axis = np.cross(y_axis, z_axis)

            # Rotation matrix -> Quaternion 변환
            rotation_matrix = np.vstack([x_axis, y_axis, z_axis]).T
            # tf_transformations 라이브러리 활용 (4x4 Homogeneous matrix 필요)
            homogeneous_matrix = np.eye(4)
            homogeneous_matrix[0:3, 0:3] = rotation_matrix
            
            target_quat = tf_transformations.quaternion_from_matrix(homogeneous_matrix)


            # 8. PoseStamped 메시지 발행
            pose_msg = PoseStamped()
            pose_msg.header = msg.header # 동일한 타임스탬프와 카메라 프레임 아이디 사용
            # frame_id는 보통 'camera_depth_optical_frame' 형태
            
            pose_msg.pose.position.x = target_position[0]
            pose_msg.pose.position.y = target_position[1]
            pose_msg.pose.position.x = target_position[2]
            
            pose_msg.pose.orientation.x = target_quat[0]
            pose_msg.pose.orientation.y = target_quat[1]
            pose_msg.pose.orientation.z = target_quat[2]
            pose_msg.pose.orientation.w = target_quat[3]

            self.pose_pub.publish(pose_msg)
            
            # [시각화 추가] ROI 영역 표시된 Image 띄우기
            depth_viz = (depth_array / 5.0 * 255).astype(np.uint8) # 5m 기준 스케일링
            cv2.rectangle(depth_viz, self.roi_top_left, self.roi_bottom_right, (255, 255, 255), 2)
            cv2.imshow("Depth ROI Check", depth_viz)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Processing Error: {e}')

def main():
    rclpy.init()
    node = DepthToNormalPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()