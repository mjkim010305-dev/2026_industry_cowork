#!/usr/bin/env python3
"""
5·6단계 통합 파이프라인의 재작업 v5 — v4(팀원2 답변 4가지 반영)에 8단계
(Point Cloud 분리 + AABB 크기 계산)까지 통합한다.

[v4 변경점 — 전부 팀원2 확인 반영, notes/message_spec.md의 "v3" 항목 참고]
1) objects 배열: process_frame()이 결과 1개가 아니라 "결과 목록"을 돌려주도록
   바뀌었다 (roi_detector.detections_to_rois() 사용). 지금은 fake_input.json이
   케이스당 물체 1개라서 목록 원소도 항상 1개지만, 형태 자체는 배열이다.
2) base_link 좌표계: 각 물체의 카메라 기준 3D 좌표(pixel_to_3d 결과)를
   transform_to_base_link()로 한 번 더 변환한다. 실제 TF 트리가 없는 Phase 1
   동안은 "고정 축 변환 + 고정 오프셋" 스텁이고, 서버 연결 후 tf2_ros 기반
   실제 변환으로 교체된다 ([TEST-ONLY · 교체 예정]).
3) detected 플래그: 이번 프레임에 검출된 물체가 하나도 없으면
   detected=False + objects=[] 로, 있으면 detected=True + objects 배열로 처리한다.
4) 연속 publish: 이 파이프라인은 (9단계 정식 publish 노드가 생기기 전까지는) topic
   publish 없이 계산 결과만 화면에 출력하는 테스트 노드다. 다만 color/depth/detections
   콜백 자체가 fake_d555_publisher의 타이머(연속 스트리밍)에 맞춰 매 프레임 계속
   호출되므로, "물체를 못 찾은 프레임도 매번 빠짐없이 처리한다"는 v3 스펙의 동작 자체는
   이미 여기서 확인된다.

[v5에서 추가된 것 — 8단계]
- 물체마다 point_cloud_utils.roi_to_point_cloud()로 ROI 안 depth 픽셀들을 점
  구름으로 만들고, compute_aabb()로 크기(w,d,h)를 계산해서 objects[].size를
  실제 값으로 채운다 (v4까지는 이 자리가 None이었다).
- Phase 1 가짜 depth 데이터는 물체 전체가 평평(depth 값 하나)해서 "깊이(d, 전후
  방향)"는 항상 0에 가깝게 나온다 — 이는 point_cloud_utils.py에 이미 설명된
  가짜 데이터의 한계이지 계산 버그가 아니다.

[열린 이슈 — notes/message_spec.md 참고]
- camera -> base_link 변환을 Perception이 계속 코드로 수행하는 게 맞는지는 서버 연결
  시점에 재확인 필요.
- objects[].id는 아직 "프레임 내 순번"일 뿐 프레임 간 추적 ID가 아니다.
- v3(동기화 버그 수정) 실행 확인 전에 v4(배열 구조)를 얹은 상태 — 가능하면 v3부터
  먼저 실제로 돌려서 FAIL=0을 확인하는 것을 권장한다. (코드 구조상 v3->v4 변경은
  "물체 1개 처리"를 "여러 개 처리"로 감싸는 정도라, v3 검증과 별개로 먼저 짜 두는
  것 자체는 무방하다.)

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
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from pixel_to_3d import pixel_to_3d
from roi_detector import detections_to_rois
from point_cloud_utils import roi_to_point_cloud, compute_aabb

# [TEST-ONLY · 교체 예정] 값(허용 오차)만 나중에 실제 노이즈 수준에 맞게 조정될 수 있다.
POSITION_TOLERANCE = 0.01     # m 단위
# [TEST-ONLY · 삭제 예정] 픽셀+depth로 케이스를 역추적하는 로직 자체가 7단계 검증 전용이라 사라진다.
PIXEL_MATCH_TOLERANCE = 3     # px 단위
DEPTH_MATCH_TOLERANCE = 0.05  # m 단위 — "center"와 "far"처럼 픽셀 위치(u,v)는 같고 depth만
                               # 다른 케이스를 구분하는 데 필요하다 (7단계에서 발견한 버그 참고).

# [TEST-ONLY · 교체 예정] 실제 카메라 장착 위치를 아직 모르기 때문에 넣어둔 placeholder
# 값(미터, base_link 기준 x=전방/y=좌측/z=상단 방향). 서버 접속 후 실측하거나,
# tf2_ros의 lookupTransform("base_link", "camera_color_optical_frame", ...) 결과로
# 완전히 대체한다 — 지금 값은 순전히 "자리를 잡아두기 위한" 임의의 숫자다.
CAMERA_OFFSET_IN_BASE_LINK = (0.15, 0.0, 0.30)


# [TEST-ONLY · 교체 예정] camera_color_optical_frame 기준 3D 좌표를 base_link 기준으로
# 바꾸는 임시 스텁. 실제 TF 트리가 없는 Phase 1 동안만 쓰고, 서버 접속 후 tf2_ros
# 기반 실제 변환으로 완전히 대체한다 (열린 이슈: 이 변환을 Perception이 계속 맡을지는
# 서버 연결 시점에 재확인 필요 — notes/message_spec.md 참고).
#
# 두 가지를 함께 처리한다:
#   1) 좌표축 방향 변경 — 카메라 광학 프레임(REP-103 기준 X:오른쪽, Y:아래, Z:전방)을
#      로봇 base_link 관례(X:전방, Y:왼쪽, Z:위)로 맞춘다. 카메라가 base_link에 대해
#      기울어짐 없이(회전 없이) 달려있다고 가정한 단순화이며, 실제로는 카메라가 살짝
#      기울어져 있어도 지금은 무시한다.
#   2) 고정 오프셋(CAMERA_OFFSET_IN_BASE_LINK)만큼 평행이동 — 카메라가 base_link
#      원점에서 얼마나 떨어져 달려있는지를 흉내낸 placeholder 값을 더한다.
def transform_to_base_link(x_cam, y_cam, z_cam):
    x_axis_fixed = z_cam
    y_axis_fixed = -x_cam
    z_axis_fixed = -y_cam

    off_x, off_y, off_z = CAMERA_OFFSET_IN_BASE_LINK
    return (
        x_axis_fixed + off_x,
        y_axis_fixed + off_y,
        z_axis_fixed + off_z,
    )


# [PROD] 검출 결과 "목록" + depth 이미지를 받아서, 물체마다 ROI depth 중앙값,
# camera 기준/base_link 기준 3D 좌표, 그리고 (8단계) point cloud 기반 AABB 크기를
# 계산한다. 물체 하나를 못 찾으면(ROI 안 depth가 전부 무효) 그 물체는 결과 목록에서
# 제외한다 — 즉 이 함수는 항상 "검출 개수 이하"인 결과 목록을 돌려준다.
def process_frame(detections, depth_img, fx, fy, cx, cy):
    rois = detections_to_rois(detections)
    results = []
    for roi_info in rois:
        x, y, w, h = roi_info["roi"]

        # 8단계: ROI 안 depth 픽셀 전체를 점 구름으로 만들어 AABB 크기를 계산한다.
        # (중심 픽셀 depth 중앙값과는 별개로, ROI 전체를 훑는다는 점이 다르다.)
        points = roi_to_point_cloud(depth_img, (x, y, w, h), fx, fy, cx, cy)
        aabb = compute_aabb(points)
        if aabb is None:
            continue  # ROI 안에 유효한 depth 픽셀이 하나도 없음

        median_depth = float(np.median(points[:, 2]))

        u, v = roi_info["pixel"]
        X_cam, Y_cam, Z_cam = pixel_to_3d(u, v, median_depth, fx, fy, cx, cy)
        X_base, Y_base, Z_base = transform_to_base_link(X_cam, Y_cam, Z_cam)

        results.append({
            "roi": (x, y, w, h),
            "pixel": (u, v),
            "depth": median_depth,
            "position_camera": (X_cam, Y_cam, Z_cam),
            "position": (X_base, Y_base, Z_base),
            "size": {"w": aabb["w"], "d": aabb["d"], "h": aabb["h"]},
            "label": roi_info["label"],
        })
    return results


# [TEST-ONLY · 삭제 예정] 실제 데이터에는 "정답 케이스 목록" 자체가 없으므로 이 함수는 사라진다.
# u,v(픽셀)만으로 매칭하면 "center"(u=320,v=240,depth=1.0)와 "far"(u=320,v=240,depth=2.5)처럼
# 같은 자리에서 depth만 다른 케이스를 구분하지 못한다 — 그래서 depth도 함께 비교한다.
# v4에서도 로직은 그대로다 — 검증은 결과 목록의 "첫 번째 물체" 기준으로 한다
# (fake_input.json이 케이스당 물체 1개이므로 이걸로 충분하다).
def find_matching_case(u, v, depth, test_cases, tol_px=PIXEL_MATCH_TOLERANCE, tol_depth=DEPTH_MATCH_TOLERANCE):
    for case in test_cases:
        if (
            abs(case["u"] - u) <= tol_px
            and abs(case["v"] - v) <= tol_px
            and abs(case["depth"] - depth) <= tol_depth
        ):
            return case
    return None


# [TEST-ONLY · 삭제 예정] CameraInfo topic 구독으로 완전히 교체된다 (9단계).
def load_fake_data():
    workspace_dir = pathlib.Path(__file__).resolve().parent.parent
    data_path = workspace_dir / "sample_data" / "fake_input.json"
    with open(data_path, "r") as f:
        return json.load(f)


class PerceptionPipelineNode(Node):
    def __init__(self):  # [PROD] 뼈대 구조
        super().__init__('perception_pipeline_test')
        self.bridge = CvBridge()

        data = load_fake_data()  # [TEST-ONLY · 삭제 예정]
        intr = data['camera_intrinsics']
        self.fx, self.fy, self.cx, self.cy = intr['fx'], intr['fy'], intr['cx'], intr['cy']
        self.test_cases = data['test_cases']  # [TEST-ONLY · 삭제 예정] 7단계 검증 전용

        # [TEST-ONLY · 삭제 예정] 케이스별 OK/FAIL 집계 — 7단계 검증 전용
        self.stats = {c['name']: {'ok': 0, 'fail': 0} for c in self.test_cases}

        # [TEST-ONLY · 교체 예정] 문자열 값만 실제 D555 topic 이름으로 바뀐다.
        color_sub = message_filters.Subscriber(self, Image, '/fake_d555/color/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/fake_d555/depth/image_raw')
        # [TEST-ONLY · 교체 예정] 지금은 가짜 검출기 topic. 실제 검출 모델 노드가 생기면
        # topic 이름과 메시지 타입(String -> 정식 검출 메시지)만 바뀐다.
        detection_sub = message_filters.Subscriber(self, String, '/fake_object_detector/detections')

        # [PROD] std_msgs/String(검출 결과 topic)에는 header가 없어서 정확 동기화
        # (TimeSynchronizer)는 쓸 수 없다 — v2->v3에서 실제로 이 버그로 죽었었다.
        # ApproximateTimeSynchronizer + allow_headerless=True로 header 없는 메시지도
        # "도착한 시각" 기준으로 맞춘다. slop=0.1(초)은 세 topic이 사실상 동시에
        # publish되는 지금 상황에는 넉넉한 허용 오차다.
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub, detection_sub], queue_size=10, slop=0.1, allow_headerless=True
        )
        self.sync.registerCallback(self.on_frame)  # [PROD]

        self.frame_count = 0
        self.get_logger().info(
            'perception_pipeline_test 시작 (v5: objects 배열 + base_link 변환 + detected 플래그 '
            '+ AABB 크기 계산) - color/depth/detections 동기화 대기 중'
        )

    def on_frame(self, color_msg, depth_msg, detection_msg):  # [PROD] 기본 구조 + v4 배열/플래그 처리
        # color_msg는 지금 계산에는 쓰이지 않는다(사람이 보기 위한 시각화 topic일 뿐).
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        detections = json.loads(detection_msg.data)  # v4: 이제 목록(list)

        self.frame_count += 1
        results = process_frame(detections, depth_img, self.fx, self.fy, self.cx, self.cy)

        # v4 확정 ④: 검출된 물체가 하나도 없으면 detected=False + 빈 objects 배열.
        detected = len(results) > 0
        objects = []
        for idx, r in enumerate(results):
            X, Y, Z = r["position"]
            objects.append({
                "id": idx,  # 프레임 내 순번 — 프레임 간 추적 ID 아님 (열린 이슈)
                "position": {"x": X, "y": Y, "z": Z},
                "size": r["size"],  # v5: 8단계 Point Cloud/AABB 계산 결과로 채워짐
            })

        # v4 확정 ①③: base_link 기준, 연속 스트리밍(매 프레임 계속 publish 대상이 될 메시지)
        frame_message = {
            "header": {"frame_id": "base_link"},
            "detected": detected,
            "objects": objects,
        }

        if not detected:
            self.get_logger().warn(f'[frame {self.frame_count}] 물체를 찾지 못함 (detected=false)')
            print(f"[frame {self.frame_count:>3}] detected=False  objects=[]  "
                  f"(publish될 메시지: {json.dumps(frame_message)})")
            return

        # [TEST-ONLY · 삭제 예정] 이 블록부터 끝까지 — 케이스 매칭 + 기대값 비교 + 출력.
        # 검증은 첫 번째 물체 기준(fake_input.json이 케이스당 물체 1개이므로 충분하다).
        first = results[0]
        x, y, w, h = first['roi']
        u, v = first['pixel']
        Xc, Yc, Zc = first['position_camera']
        Xb, Yb, Zb = first['position']

        case = find_matching_case(u, v, first['depth'], self.test_cases)
        if case is None:
            print(
                f"[frame {self.frame_count}] detected=True objects={len(objects)}개  "
                f"label={first['label']} pixel=(u={u}, v={v}) depth={first['depth']:.3f}  "
                f"base_link 기준(X={Xb:+.3f}, Y={Yb:+.3f}, Z={Zb:.3f})  "
                f"(경고: 알고 있는 테스트 케이스와 일치하지 않음)"
            )
            return

        # 정확도 검증 자체는 camera 기준 좌표로 한다 — pixel_to_3d 공식이 맞는지
        # 확인하는 게 목적이라, base_link 변환(placeholder 스텁)의 오차가 섞이면 안 된다.
        eX, eY, eZ = pixel_to_3d(case['u'], case['v'], case['depth'], self.fx, self.fy, self.cx, self.cy)
        ok = (
            abs(Xc - eX) < POSITION_TOLERANCE
            and abs(Yc - eY) < POSITION_TOLERANCE
            and abs(Zc - eZ) < POSITION_TOLERANCE
        )
        status = 'OK' if ok else 'FAIL'
        self.stats[case['name']]['ok' if ok else 'fail'] += 1

        sw, sd, sh = first['size']['w'], first['size']['d'], first['size']['h']
        print(
            f"[frame {self.frame_count:>3}] case={case['name']:6s}  detected=True objects={len(objects)}개  "
            f"ROI=({x},{y},{w},{h})  pixel=(u={u}, v={v})  depth={first['depth']:.3f}  "
            f"camera 기준(X={Xc:+.3f}, Y={Yc:+.3f}, Z={Zc:.3f})  "
            f"기대값(camera, X={eX:+.3f}, Y={eY:+.3f}, Z={eZ:.3f})  -> {status}  |  "
            f"base_link 기준(X={Xb:+.3f}, Y={Yb:+.3f}, Z={Zb:.3f})  |  "
            f"size(w={sw:.3f}, d={sd:.3f}, h={sh:.3f})"
        )

    def print_summary(self):  # [TEST-ONLY · 삭제 예정] 7단계 검증 전용 요약 출력
        print("\n=== 결과 요약 (케이스별 OK/FAIL 횟수, camera 기준 좌표로 검증) ===")
        for name, s in self.stats.items():
            print(f"  {name:6s}: OK={s['ok']:>4}  FAIL={s['fail']:>4}")
        print("====================================================================\n")


def main():  # [PROD]
    rclpy.init()
    node = PerceptionPipelineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_summary()  # [TEST-ONLY · 삭제 예정]
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
