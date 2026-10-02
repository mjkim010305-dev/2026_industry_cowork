import math
import time

import pytest

from conftest import Dummy

FX = FY = 500.0
CX = 320.0
CAM_X, CAM_Y = 0.1205, -0.0115


@pytest.fixture
def node(box_on_path):
    n = box_on_path.BoxOnPath()
    n.intrinsics = (FX, FY, CX, 240.0)
    return n


def bbox(x, y, w, h, clip=0, img_w=640, img_h=480):
    return Dummy(data=[x, y, w, h, img_w, img_h, clip, 1.0])


# ---------------------------------------------------------------- pure geometry
def test_project_straight(box_on_path):
    assert box_on_path.project((0.5, 0.1), [(0, 0), (2, 0)]) == pytest.approx((0.5, 0.1))


def test_project_beyond_end_clamps(box_on_path):
    along, lat = box_on_path.project((3.0, 0.0), [(0, 0), (2, 0)])
    assert along == pytest.approx(2.0) and lat == pytest.approx(1.0)


def test_project_second_segment_accumulates(box_on_path):
    # route (0,0)->(1,0)->(1,1); point beside the second segment
    along, lat = box_on_path.project((1.2, 0.5), [(0, 0), (1, 0), (1, 1)])
    assert along == pytest.approx(1.5) and lat == pytest.approx(0.2)


def test_project_degenerate_route_falls_back(box_on_path):
    assert box_on_path.project((3.0, 4.0), [(0, 0), (0, 0)]) == pytest.approx((0.0, 5.0))


def test_compose_inverse_roundtrip(box_on_path):
    a = (1.0, 2.0, 0.7)
    x, y, yaw = box_on_path.compose(a, box_on_path.inverse(a))
    assert (x, y, yaw) == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_compose_known_value(box_on_path):
    # pose at (1,0) facing +90 deg, local point (1,0) -> world (1,1)
    x, y, yaw = box_on_path.compose((1.0, 0.0, math.pi / 2), (1.0, 0.0, 0.0))
    assert (x, y, yaw) == pytest.approx((1.0, 1.0, math.pi / 2))


def test_yaw_of(box_on_path):
    q = Dummy(x=0.0, y=0.0, z=math.sin(0.4), w=math.cos(0.4))
    assert box_on_path.yaw_of(q) == pytest.approx(0.8)


# ---------------------------------------------------------------- box position from bbox
def test_bbox_centered_height_distance(node):
    # h=60 px, H=0.12 m, fy=500 -> d = 0.12*500/60 = 1.0 m
    node._on_bbox(bbox(290, 100, 60, 60))
    _, clip, (bx, by) = node.target
    assert clip == 0
    assert bx == pytest.approx(CAM_X + 1.0)
    assert by == pytest.approx(CAM_Y)


def test_bbox_ignored_without_intrinsics_or_short_or_zero_width(node):
    node.intrinsics = None
    node._on_bbox(bbox(290, 100, 60, 60))
    assert node.target is None
    node.intrinsics = (FX, FY, CX, 240.0)
    node._on_bbox(Dummy(data=[1, 2, 3]))
    node._on_bbox(bbox(290, 100, 0, 60))
    assert node.target is None


def test_bbox_touching_top_uses_width_distance(node):
    # y=0 touches the top border: d = 0.24*500/120 = 1.0 (height would give 0.5)
    node._on_bbox(bbox(260, 0, 120, 120))
    assert node.target[2][0] == pytest.approx(CAM_X + 1.0)


def test_bbox_touching_bottom_uses_width_distance(node):
    node._on_bbox(bbox(260, 360, 120, 120))  # y+h = 480 = img_h
    assert node.target[2][0] == pytest.approx(CAM_X + 1.0)


def test_bbox_small_offset_uses_centre(node):
    # centre u=350 -> y = cy - 30/500*1.0 = -0.0715; |offset| 0.06 < 0.12 -> centre rule
    node._on_bbox(bbox(320, 100, 60, 60))
    assert node.target[2][1] == pytest.approx(CAM_Y - 0.06)


def test_bbox_right_side_box_uses_outer_edge(node):
    # image right (u>cx) = -y. bbox 390..450: outer edge u=450 -> y_out = -0.0115-0.26 = -0.2715;
    # the box centre is half a face (0.0925) inward of the outer edge: -0.179
    # (the bbox centre alone would say -0.2115: the visible inner side face pulls it outward)
    node._on_bbox(bbox(390, 100, 60, 60))
    assert node.target[2][1] == pytest.approx(-0.2715 + 0.0925)


def test_bbox_left_side_box_uses_outer_edge(node):
    # bbox 190..250: outer (left) edge u=190 -> y_out = -0.0115+0.26 = 0.2485; centre inward 0.0925
    node._on_bbox(bbox(190, 100, 60, 60))
    assert node.target[2][1] == pytest.approx(0.2485 - 0.0925)


def test_bbox_clip1_left_border_adds_centre_offset(node):
    # clip 1: u = x+w = 40 -> y = -0.0115 + 280/500 = 0.5485, plus box_centre_off 0.11
    node._on_bbox(bbox(0, 100, 40, 60, clip=1))
    assert node.target[1] == 1
    assert node.target[2][1] == pytest.approx(0.5485 + 0.11)
    assert node.recent == []  # clipped frames do not feed the median


def test_bbox_clip2_right_border_subtracts_centre_offset(node):
    # clip 2: u = x = 600 -> y = -0.0115 - 280/500 = -0.5715, minus 0.11
    node._on_bbox(bbox(600, 100, 40, 60, clip=2))
    assert node.target[2][1] == pytest.approx(-0.5715 - 0.11)


def test_bbox_clip_touching_top_uses_clip_close_dist(node):
    node._on_bbox(bbox(0, 0, 40, 60, clip=1))
    assert node.target[2][0] == pytest.approx(CAM_X + 0.25)


def test_bbox_clip3_straight_ahead(node):
    node._on_bbox(bbox(0, 100, 640, 60, clip=3))
    assert node.target[1] == 3
    assert node.target[2] == pytest.approx((CAM_X + 0.25, CAM_Y))


def test_bbox_median_window_bounded(node):
    for i in range(8):
        node._on_bbox(bbox(290, 100, 60, 60))
    assert len(node.recent) == node.p['median_n']


# ---------------------------------------------------------------- in-path judgement
def make_ready(node, monkeypatch, plan=((0.0, 0.0), (3.0, 0.0))):
    """Route straight ahead along +x (or `plan`), robot at the map origin."""
    node.map_odom = (0.0, 0.0, 0.0)
    node.odom_hist = [(0.0, (0.0, 0.0, 0.0))]
    node.plan = (100.0, list(plan))
    monkeypatch.setattr(node, '_now', lambda: 100.5)


def see(node, box, clip=0):
    node.target = (time.monotonic(), clip, box)
    node.recent = []


def tick(node, box, clip=0):
    see(node, box, clip)
    node._tick()
    return node.on


@pytest.mark.parametrize('box,expected', [
    ((0.50, 0.0), True),
    ((0.10, 0.0), True),     # along == along_min
    ((1.10, 0.0), True),     # along == trigger_dist
    ((0.50, 0.20), True),    # lat == lateral_tol
    ((0.50, -0.20), True),
    ((1.11, 0.0), False),    # just past trigger_dist (fresh state: no hysteresis)
    ((0.50, 0.21), False),   # just past lateral_tol
    ((0.09, 0.0), False),    # before along_min
    ((2.00, 0.0), False),
])
def test_on_condition_from_off(node, monkeypatch, box, expected):
    make_ready(node, monkeypatch)
    assert tick(node, box) is expected


def test_hysteresis_stays_on_between_trigger_and_release_dist(node, monkeypatch):
    make_ready(node, monkeypatch)
    assert tick(node, (1.0, 0.0)) is True
    assert tick(node, (1.2, 0.0)) is True       # 1.1 < along <= 1.3: keeps on
    assert tick(node, (1.31, 0.0)) is False     # > release_dist
    assert tick(node, (1.2, 0.0)) is False      # and does not come back until <= trigger_dist


def test_hysteresis_stays_on_between_lateral_tol_and_release(node, monkeypatch):
    make_ready(node, monkeypatch)
    assert tick(node, (0.5, 0.0)) is True
    assert tick(node, (0.5, 0.25)) is True
    assert tick(node, (0.5, 0.30)) is True      # lat == release_lateral still on (> required)
    assert tick(node, (0.5, 0.31)) is False
    assert tick(node, (0.5, 0.25)) is False     # stays off in the band


def test_off_band_is_sticky_when_off(node, monkeypatch):
    make_ready(node, monkeypatch)
    assert tick(node, (0.5, 0.25)) is False


def test_box_passed_behind_releases(node, monkeypatch):
    make_ready(node, monkeypatch)
    assert tick(node, (0.5, 0.0)) is True
    assert tick(node, (0.12, 0.0)) is True      # >= along_min - 0.05 keeps on
    assert tick(node, (0.04, 0.0)) is False     # along < along_min - 0.05


def test_stale_image_turns_off(node, monkeypatch):
    make_ready(node, monkeypatch)
    assert tick(node, (0.5, 0.0)) is True
    node.target = (time.monotonic() - 2.0, 0, (0.5, 0.0))
    node._tick()
    assert node.on is False


def test_no_box_turns_off(node, monkeypatch):
    make_ready(node, monkeypatch)
    node.target = None
    node._tick()
    assert node.on is False
    node.pub.publish.assert_called()


def test_no_route_turns_off(node, monkeypatch):
    make_ready(node, monkeypatch)
    assert tick(node, (0.5, 0.0)) is True
    node.plan = None
    assert tick(node, (0.5, 0.0)) is False


def test_old_plan_is_no_route(node, monkeypatch):
    make_ready(node, monkeypatch)
    monkeypatch.setattr(node, '_now', lambda: 103.5)   # plan_timeout 3.0
    assert tick(node, (0.5, 0.0)) is False


def test_no_amcl_map_odom_is_no_route(node, monkeypatch):
    make_ready(node, monkeypatch)
    node.map_odom = None
    assert tick(node, (0.5, 0.0)) is False


def test_route_turning_away_holds_off(node, monkeypatch):
    # route goes sideways first (look point 0.4 m ahead is 90 deg off the nose)
    make_ready(node, monkeypatch, plan=((0.0, 0.0), (0.0, 0.5), (1.0, 0.5)))
    assert tick(node, (0.15, 0.2)) is False


def test_same_box_on_when_route_is_straight(node, monkeypatch):
    make_ready(node, monkeypatch, plan=((0.0, 0.0), (0.5, 0.2), (1.0, 0.4)))
    # bearing of the point 0.4 m along ~22 deg < 45 deg: no hold
    assert tick(node, (0.5, 0.2)) is True


def test_clip0_uses_median_of_recent(node, monkeypatch):
    make_ready(node, monkeypatch)
    now = time.monotonic()
    node.target = (now, 0, (5.0, 5.0))                     # latest outlier
    node.recent = [(now, 0.5, 0.0), (now, 0.5, 0.0), (now, 5.0, 5.0)]
    node._tick()
    assert node.on is True
    assert node.front == (0.5, 0.0)


def test_on_publishes_front_pose(node, monkeypatch):
    make_ready(node, monkeypatch)
    tick(node, (0.5, 0.05))
    pose = node.front_pub.publish.call_args[0][0]
    assert pose.pose.position.x == 0.5 and pose.pose.position.y == 0.05
    assert pose.header.frame_id == 'base_link'


def test_off_does_not_publish_front(node, monkeypatch):
    make_ready(node, monkeypatch)
    tick(node, (2.0, 0.0))
    node.front_pub.publish.assert_not_called()


def test_route_follows_robot_motion(node, monkeypatch):
    # robot has moved to (1,0) in odom (map_odom identity): plan point nearest it is (1,0),
    # a box 0.5 m ahead of the robot is at map x 1.5 -> along 0.5
    make_ready(node, monkeypatch, plan=[(0.5 * i, 0.0) for i in range(8)])
    node.odom_hist = [(0.0, (1.0, 0.0, 0.0))]
    assert tick(node, (0.5, 0.0)) is True
    assert tick(node, (1.5, 0.0)) is False   # 1.5 m ahead of the robot


def test_on_amcl_rejects_stale_odom_sample(node):
    node.odom_hist = [(10.0, (0.0, 0.0, 0.0))]
    msg = Dummy(header=Dummy(stamp=Dummy(sec=20, nanosec=0)))
    node._on_amcl(msg)
    assert node.map_odom is None


def test_on_amcl_composes_map_odom(node):
    node.odom_hist = [(10.0, (1.0, 0.0, 0.0))]
    msg = Dummy(header=Dummy(stamp=Dummy(sec=10, nanosec=0)),
                pose=Dummy(pose=Dummy(position=Dummy(x=3.0, y=0.0, z=0.0),
                                      orientation=Dummy(x=0.0, y=0.0, z=0.0, w=1.0))))
    node._on_amcl(msg)
    assert node.map_odom == pytest.approx((2.0, 0.0, 0.0))
