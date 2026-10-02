import math
import subprocess
from unittest import mock

import pytest

from conftest import Dummy

P_HOME_J2 = -1.0461748973380072
REAR_J1 = -3.0234761329225988


# ---------------------------------------------------------------- place_part / pick_part
@pytest.fixture
def node():
    return mock.MagicMock()


def run_ok(*a, **k):
    return subprocess.CompletedProcess(a, 0)


@pytest.mark.parametrize('j1,j2,expected', [
    (0.0, P_HOME_J2, True),
    (-0.0015, P_HOME_J2, True),
    (0.14, P_HOME_J2, True),
    (-0.14, P_HOME_J2, True),
    (0.16, P_HOME_J2, False),
    (-0.16, P_HOME_J2, False),
    (0.0, P_HOME_J2 + 0.14, True),
    (0.0, P_HOME_J2 - 0.14, True),
    (0.0, P_HOME_J2 + 0.16, False),
    (0.0, P_HOME_J2 - 0.16, False),
    (REAR_J1, -0.98, False),          # still at P_REAR_CARRY
])
def test_place_part_joint_check(send_goal, node, monkeypatch, j1, j2, expected):
    monkeypatch.setattr(send_goal.subprocess, 'run', run_ok)
    monkeypatch.setattr(send_goal, 'read_joints', lambda n: {'joint1': j1, 'joint2': j2})
    assert send_goal.place_part(node) is expected


def test_place_part_no_joint_states(send_goal, node, monkeypatch):
    monkeypatch.setattr(send_goal.subprocess, 'run', run_ok)
    monkeypatch.setattr(send_goal, 'read_joints', lambda n: {})
    assert send_goal.place_part(node) is False


def test_place_part_timeout_does_not_raise(send_goal, node, monkeypatch):
    def boom(*a, **k):
        raise subprocess.TimeoutExpired(cmd='x', timeout=150)
    monkeypatch.setattr(send_goal.subprocess, 'run', boom)
    monkeypatch.setattr(send_goal, 'read_joints', lambda n: {'joint1': 0.0, 'joint2': P_HOME_J2})
    assert send_goal.place_part(node) is True      # arm did reach P_HOME anyway
    node.get_logger().error.assert_called()


def test_place_part_timeout_and_arm_not_home(send_goal, node, monkeypatch):
    def boom(*a, **k):
        raise subprocess.TimeoutExpired(cmd='x', timeout=150)
    monkeypatch.setattr(send_goal.subprocess, 'run', boom)
    monkeypatch.setattr(send_goal, 'read_joints', lambda n: {'joint1': REAR_J1, 'joint2': -0.98})
    assert send_goal.place_part(node) is False


def test_place_part_runs_rear_pick_place(send_goal, node, monkeypatch):
    calls = []
    monkeypatch.setattr(send_goal.subprocess, 'run', lambda cmd, **k: calls.append((cmd, k)))
    monkeypatch.setattr(send_goal, 'read_joints', lambda n: {})
    send_goal.place_part(node)
    cmd, kw = calls[0]
    assert cmd[-2].endswith('rear_pick.py') and cmd[-1] == 'place'
    assert kw['timeout'] == 150


@pytest.mark.parametrize('j1,expected', [
    (REAR_J1, True), (REAR_J1 + 0.14, True), (REAR_J1 - 0.14, True),
    (REAR_J1 + 0.16, False), (REAR_J1 - 0.16, False),
    (0.0, False),                     # never left P_HOME
])
def test_pick_part_joint_check(send_goal, node, monkeypatch, j1, expected):
    monkeypatch.setattr(send_goal.subprocess, 'run', run_ok)
    monkeypatch.setattr(send_goal, 'read_joints', lambda n: {'joint1': j1})
    assert send_goal.pick_part(node) is expected


def test_pick_part_no_joint_states(send_goal, node, monkeypatch):
    monkeypatch.setattr(send_goal.subprocess, 'run', run_ok)
    monkeypatch.setattr(send_goal, 'read_joints', lambda n: {})
    assert send_goal.pick_part(node) is False


def test_pick_part_both_timeouts_do_not_raise(send_goal, node, monkeypatch):
    def boom(*a, **k):
        raise subprocess.TimeoutExpired(cmd='x', timeout=1)
    monkeypatch.setattr(send_goal.subprocess, 'run', boom)
    monkeypatch.setattr(send_goal, 'read_joints', lambda n: {'joint1': REAR_J1})
    assert send_goal.pick_part(node) is True


def test_pick_part_commands(send_goal, node, monkeypatch):
    calls = []
    monkeypatch.setattr(send_goal.subprocess, 'run', lambda cmd, **k: calls.append((cmd, k)))
    monkeypatch.setattr(send_goal, 'read_joints', lambda n: {})
    send_goal.pick_part(node)
    assert calls[0][0][:3] == ['ros2', 'action', 'send_goal']
    assert 'positions: [-0.0015339807878856412, -1.0461748973380072' in calls[0][0][-1]
    assert calls[1][0][-1] == 'pick' and calls[1][0][-2].endswith('rear_pick.py')


# ---------------------------------------------------------------- initial pose acceptance
@pytest.fixture
def sg(send_goal, monkeypatch):
    n = send_goal.SendGoal()
    n.p.update(initial_x=1.0, initial_y=2.0, initial_yaw=0.5)
    return n


def feed(n, monkeypatch, xy, yaw):
    def spin(seconds):
        n.amcl_xy, n.amcl_yaw = xy, yaw
    monkeypatch.setattr(n, 'spin_for', spin)


@pytest.mark.parametrize('xy,yaw,expected', [
    ((1.0, 2.0), 0.5, True),
    ((1.29, 2.0), 0.5, True),
    ((1.0, 2.29), 0.84, True),
    ((1.31, 2.0), 0.5, False),            # 0.31 m off
    ((1.0, 1.69), 0.5, False),
    ((1.0, 2.0), 0.86, False),            # 0.36 rad off
    ((1.0, 2.0), 0.14, False),
    ((0.0, 0.0), 0.0, False),             # AMCL's own default pose (R33 regression)
])
def test_initial_pose_acceptance(sg, monkeypatch, xy, yaw, expected):
    feed(sg, monkeypatch, xy, yaw)
    assert sg.set_initial_pose() is expected


def test_initial_pose_yaw_wraps_around_pi(sg, monkeypatch):
    sg.p['initial_yaw'] = 3.1
    feed(sg, monkeypatch, (1.0, 2.0), -3.1)       # 0.083 rad apart across +-pi
    assert sg.set_initial_pose() is True


def test_initial_pose_never_answered_returns_false_after_60_tries(sg, monkeypatch):
    spins = []
    monkeypatch.setattr(sg, 'spin_for', lambda s: spins.append(s))
    assert sg.set_initial_pose() is False
    assert len(spins) == 60


def test_initial_pose_published_message(sg, monkeypatch):
    feed(sg, monkeypatch, (1.0, 2.0), 0.5)
    sg.set_initial_pose()
    msg = sg.init_pub.publish.call_args[0][0]
    assert msg.pose.pose.position.x == 1.0 and msg.pose.pose.position.y == 2.0
    assert msg.pose.pose.orientation.z == pytest.approx(math.sin(0.25))
    assert msg.pose.pose.orientation.w == pytest.approx(math.cos(0.25))


# ---------------------------------------------------------------- main(): leg selection / shuttle
GOAL = dict(goal_x=1.9, goal_y=-1.7, goal_yaw=-1.5708,
            initial_x=-0.14, initial_y=0.5, initial_yaw=-1.536)


class FakeNode:
    sends = None

    def __init__(self, params, results):
        self.p = dict(set_initial_pose=False, return_to_start=True, pick_first=False, send_goal=True,
                      round_trips=1, shuttle=False, **GOAL)
        self.p.update(params)
        self._results = list(results)
        self.sent, self.waits, self.events = [], [], []
        self._log = mock.MagicMock()

    def get_logger(self):
        return self._log

    def wait_nav2_active(self):
        pass

    def set_initial_pose(self):
        self.events.append('init')

    def clear_costmaps(self):
        pass

    def send(self, x, y, yaw):
        self.sent.append((x, y, yaw))
        self.events.append('send')
        return self._results.pop(0) if self._results else True

    def spin_for(self, s):
        self.waits.append(s)

    def destroy_node(self):
        pass


@pytest.fixture
def run_main(send_goal, monkeypatch):
    def go(params, results=(), place=(True,), pick=(True,), cap=40):
        fake = FakeNode(params, results)
        place_q, pick_q = list(place), list(pick)
        counter = {'n': 0}

        def ok():
            counter['n'] += 1
            return counter['n'] < cap          # safety net against an endless loop
        monkeypatch.setattr(send_goal, 'SendGoal', lambda: fake)
        monkeypatch.setattr(send_goal.rclpy, 'ok', ok)

        def do_place(n):
            fake.events.append('place')
            return place_q.pop(0) if len(place_q) > 1 else place_q[0]

        def do_pick(n):
            fake.events.append('pick')
            return pick_q.pop(0) if len(pick_q) > 1 else pick_q[0]
        monkeypatch.setattr(send_goal, 'place_part', do_place)
        monkeypatch.setattr(send_goal, 'pick_part', do_pick)
        send_goal.main()
        return fake
    return go


def test_no_shuttle_legs(run_main):
    f = run_main({'shuttle': False})
    assert f.sent == [(1.9, -1.7, pytest.approx(-1.5708)),
                      (-0.14, 0.5, pytest.approx(-1.536 + math.pi))]
    assert 'place' not in f.events and 'pick' not in f.events


def test_shuttle_legs(run_main):
    f = run_main({'shuttle': True})
    assert f.sent == [(1.9, -1.7, pytest.approx(-1.5708 + math.pi)),
                      (-0.14, 0.5, pytest.approx(-1.536))]
    # place then pick after each successful leg
    assert f.events == ['send', 'place', 'pick', 'send', 'place', 'pick']


def test_return_to_start_false_sends_single_goal(run_main):
    f = run_main({'return_to_start': False, 'shuttle': True})
    assert f.sent == [(1.9, -1.7, -1.5708)]
    assert f.events == ['send']


def test_send_goal_false_sends_nothing(run_main):
    f = run_main({'send_goal': False})
    assert f.sent == []


def test_round_trips_count(run_main):
    f = run_main({'round_trips': 3})
    assert len(f.sent) == 6


def test_round_trips_zero_runs_until_rclpy_stops(run_main):
    f = run_main({'round_trips': 0}, cap=11)
    assert len(f.sent) >= 8


@pytest.mark.parametrize('place,pick', [((False,), (True,)), ((True,), (False,))])
def test_shuttle_stops_when_place_or_pick_fails(run_main, place, pick):
    f = run_main({'shuttle': True, 'round_trips': 0}, place=place, pick=pick)
    assert len(f.sent) == 1
    f._log.error.assert_called()


def test_shuttle_place_failure_skips_pick(run_main):
    f = run_main({'shuttle': True, 'round_trips': 0}, place=(False,))
    assert 'pick' not in f.events


def test_shuttle_stops_on_second_end_failure(run_main):
    f = run_main({'shuttle': True, 'round_trips': 0}, place=(True, False))
    assert len(f.sent) == 2


def test_failed_leg_is_retried_with_growing_wait(run_main):
    f = run_main({'round_trips': 1}, results=[False, False, False, False, True, True])
    assert f.waits == [10.0, 20.0, 30.0, 30.0]
    assert len(f.sent) == 6 and f.sent[0] == f.sent[1] == f.sent[4]


def test_failure_counter_resets_after_success(run_main):
    f = run_main({'round_trips': 1}, results=[False, True, False, True])
    assert f.waits == [10.0, 10.0]


def test_pick_first_failure_sends_no_goal(run_main):
    f = run_main({'pick_first': True}, pick=(False,))
    assert f.sent == [] and f.events == ['pick']


def test_pick_first_then_initial_pose_then_goal(run_main):
    f = run_main({'pick_first': True, 'set_initial_pose': True})
    assert f.events[:3] == ['pick', 'init', 'send']


# ---------------------------------------------------------------- send(): AMCL cross-check
def test_send_resends_when_robot_far_from_goal(send_goal, monkeypatch):
    n = send_goal.SendGoal()
    monkeypatch.setattr(n, 'spin_for', lambda s: None)
    n.amcl_xy = (5.0, 5.0)
    calls = []
    monkeypatch.setattr(n, '_send_once', lambda x, y, yaw: calls.append(1) or True)
    assert n.send(0.0, 0.0, 0.0) is False
    assert len(calls) == 4


def test_send_ok_when_close_or_no_amcl(send_goal, monkeypatch):
    n = send_goal.SendGoal()
    monkeypatch.setattr(n, 'spin_for', lambda s: None)
    monkeypatch.setattr(n, '_send_once', lambda x, y, yaw: True)
    assert n.send(0.0, 0.0, 0.0) is True            # no AMCL pose known
    n.amcl_xy = (0.5, 0.0)
    assert n.send(0.0, 0.0, 0.0) is True            # 0.5 <= 0.6
    n.amcl_xy = (0.61, 0.0)
    assert n.send(0.0, 0.0, 0.0) is False


def test_send_returns_false_when_goal_fails(send_goal, monkeypatch):
    n = send_goal.SendGoal()
    monkeypatch.setattr(n, '_send_once', lambda x, y, yaw: False)
    assert n.send(0.0, 0.0, 0.0) is False


def test_quat_z(send_goal):
    z, w = send_goal.quat_z(math.pi)
    assert z == pytest.approx(1.0) and w == pytest.approx(0.0, abs=1e-12)
