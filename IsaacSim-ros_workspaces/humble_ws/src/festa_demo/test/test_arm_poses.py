import math
import sys
from unittest import mock

import pytest

TOL_XZ = 0.003
TOL_PITCH = 0.01


def fk(j2, j3, j4):
    """OpenManipulator-X planar FK relative to joint2: (forward, up, pitch); pitch>0 = down."""
    def rot(x, z, q):
        return x * math.cos(q) + z * math.sin(q), -x * math.sin(q) + z * math.cos(q)
    x, z = rot(0.024, 0.128, j2)
    ax, az = rot(0.124, 0.0, j2 + j3)
    bx, bz = rot(0.126, 0.0, j2 + j3 + j4)
    return x + ax + bx, z + az + bz, j2 + j3 + j4


def test_fk_helper_known_values():
    # hand-checked: all joints 0 -> link2 (0.024, 0.128) + 0.124 + 0.126 forward
    assert fk(0, 0, 0) == pytest.approx((0.274, 0.128, 0.0))
    # joint2 = +90 deg tips link2 forward-down: (0.024,0.128)->(0.128,-0.024), link3 0.124 down
    x, z, pitch = fk(math.pi / 2, 0, 0)
    assert (x, z, pitch) == pytest.approx((0.128 + 0.0, -0.024 - 0.124 - 0.126, math.pi / 2))


@pytest.mark.parametrize('name,tip', [('P_PRE_PICK', (0.334, -0.011)), ('P_PICK', (0.336, -0.057))])
def test_rear_pick_front_poses(rear_pick, name, tip):
    _, j2, j3, j4 = getattr(rear_pick, name)
    x, z, pitch = fk(j2, j3, j4)
    assert abs(pitch) < TOL_PITCH
    assert x == pytest.approx(tip[0], abs=TOL_XZ)
    assert z == pytest.approx(tip[1], abs=TOL_XZ)


def test_obstacle_clear_rear_pick(obstacle_clear):
    _, j2, j3, j4 = obstacle_clear.P_REAR_PICK
    x, z, pitch = fk(j2, j3, j4)
    assert abs(pitch) < TOL_PITCH
    assert x == pytest.approx(0.305, abs=TOL_XZ)
    assert z == pytest.approx(-0.055, abs=TOL_XZ)


def test_rear_place_is_rear_pick(obstacle_clear):
    assert obstacle_clear.P_REAR_PLACE == obstacle_clear.P_REAR_PICK


def test_fold_poses_share_joints_2_to_4_with_rear_carry(rear_pick):
    carry = rear_pick.P_REAR_CARRY
    assert carry[0] == pytest.approx(-3.0234761329225988)
    for pose in (rear_pick.P_LIFT_FOLD, rear_pick.P_SIDE_CARRY):
        assert pose[1:] == carry[1:]
    assert rear_pick.P_LIFT_FOLD[0] == pytest.approx(-0.0030679615757712823)   # folded while facing forward
    assert rear_pick.P_SIDE_CARRY[0] == pytest.approx(-1.59073807703741)       # about half way round


def test_rear_carry_identical_in_both_scripts(rear_pick, obstacle_clear):
    assert rear_pick.P_REAR_CARRY == obstacle_clear.P_REAR_CARRY


def test_gripper_constants_agree(rear_pick, obstacle_clear):
    assert rear_pick.G_OPEN == obstacle_clear.G_OPEN
    assert rear_pick.G_GRASP == obstacle_clear.G_GRASP


def test_p_home_matches_send_goal_and_obstacle_fold(rear_pick, obstacle_clear):
    assert obstacle_clear.ARM_FOLD == rear_pick.P_HOME[1:]


def test_all_poses_have_four_finite_joints(rear_pick, obstacle_clear):
    for m in (rear_pick, obstacle_clear):
        for name in dir(m):
            if name.startswith('P_'):
                pose = getattr(m, name)
                assert len(pose) == 4 and all(math.isfinite(v) for v in pose), name


# ---------------------------------------------------------------- run_place / run_pick order
@pytest.fixture
def rp(rear_pick, monkeypatch):
    node = rear_pick.RearPick()
    node.calls = []

    def arm(pose, label):
        node.calls.append(('arm', label, pose))
        return node.fail_at != label

    def grip(pos, label):
        node.calls.append(('grip', label, pos))
        return node.fail_at != label
    node.fail_at = None
    node.move_arm, node.move_gripper = arm, grip
    node.check_start_pose = lambda: True
    monkeypatch.setattr(rear_pick.time, 'sleep', lambda s: None)
    return node


def test_run_place_step_order(rear_pick, rp):
    assert rp.run_place() is True
    assert [c[1] for c in rp.calls] == ['P_SIDE_CARRY', 'P_LIFT_FOLD', 'P_LIFT', 'P_PRE_PICK',
                                        'P_PICK', 'G_OPEN', 'P_PRE_PICK', 'P_HOME']
    kinds = [c[0] for c in rp.calls]
    assert kinds == ['arm'] * 5 + ['grip'] + ['arm'] * 2
    for kind, label, val in rp.calls:
        assert val == getattr(rear_pick, label) , label


@pytest.mark.parametrize('fail', ['P_SIDE_CARRY', 'P_LIFT', 'P_PICK', 'G_OPEN', 'P_PRE_PICK', 'P_HOME'])
def test_run_place_stops_at_first_failure(rp, fail):
    rp.fail_at = fail
    assert rp.run_place() is False
    labels = [c[1] for c in rp.calls]
    assert labels[-1] == fail
    assert labels.count(fail) == 1 or fail == 'P_PRE_PICK'


def test_run_place_never_closes_gripper(rear_pick, rp):
    rp.run_place()
    assert not [c for c in rp.calls if c[0] == 'grip' and c[2] == rear_pick.G_GRASP]


def test_run_pick_step_order(rp):
    assert rp.run_pick() is True
    assert [c[1] for c in rp.calls] == ['G_OPEN', 'P_PRE_PICK', 'P_PICK', 'G_GRASP', 'P_LIFT',
                                        'P_LIFT_FOLD', 'P_SIDE_CARRY', 'P_REAR_CARRY']


def test_run_pick_aborts_if_not_at_home(rp):
    rp.check_start_pose = lambda: False
    assert rp.run_pick() is False
    assert rp.calls == []


def test_check_start_pose_tolerance(rear_pick):
    n = rear_pick.RearPick()
    for off, ok in ((0.11, True), (0.13, False)):
        n.current_pose = None

        def spin_once(node, timeout_sec=0.1, off=off):
            n.current_pose = [v + off for v in rear_pick.P_HOME]
        with mock.patch.object(rear_pick.rclpy, 'spin_once', spin_once):
            assert n.check_start_pose() is ok


def test_check_start_pose_no_joint_states(rear_pick):
    n = rear_pick.RearPick()
    assert n.check_start_pose() is False


# ---------------------------------------------------------------- main() argument handling
class FakeRearPick:
    instances = []

    def __init__(self):
        self.ran = []
        self._log = mock.MagicMock()
        FakeRearPick.instances.append(self)

    def get_logger(self):
        return self._log

    def wait_servers(self):
        return True

    def run_place(self):
        self.ran.append('place')
        return True

    def run_pick(self):
        self.ran.append('pick')
        return True

    def destroy_node(self):
        pass


@pytest.mark.parametrize('argv', [['rear_pick.py'], ['rear_pick.py', 'foo'], ['rear_pick.py', 'all'],
                                  ['rear_pick.py', 'clear'], ['rear_pick.py', 'PICK'],
                                  ['rear_pick.py', 'pick', 'place'], ['rear_pick.py', '']])
def test_rear_pick_main_rejects_other_args(rear_pick, monkeypatch, argv):
    FakeRearPick.instances = []
    monkeypatch.setattr(sys, 'argv', argv)
    monkeypatch.setattr(rear_pick, 'RearPick', FakeRearPick)
    rear_pick.rclpy.init.reset_mock()
    rear_pick.main()
    rear_pick.rclpy.init.assert_not_called()
    assert FakeRearPick.instances == []


@pytest.mark.parametrize('arg', ['pick', 'place'])
def test_rear_pick_main_dispatch(rear_pick, monkeypatch, arg):
    FakeRearPick.instances = []
    monkeypatch.setattr(sys, 'argv', ['rear_pick.py', arg])
    monkeypatch.setattr(rear_pick, 'RearPick', FakeRearPick)
    rear_pick.main()
    assert FakeRearPick.instances[0].ran == [arg]


def test_obstacle_clear_main_rejects_pick_and_place(obstacle_clear, monkeypatch, capsys):
    monkeypatch.setattr(obstacle_clear.rclpy, 'init', mock.MagicMock())
    for bad in ('pick', 'place', '', 'foo'):
        monkeypatch.setattr(sys, 'argv', ['obstacle_clear_sequence.py', bad])
        obstacle_clear.main()
    monkeypatch.setattr(sys, 'argv', ['obstacle_clear_sequence.py'])
    obstacle_clear.main()
    monkeypatch.setattr(sys, 'argv', ['obstacle_clear_sequence.py', 'clear', 'all'])
    obstacle_clear.main()
    obstacle_clear.rclpy.init.assert_not_called()
