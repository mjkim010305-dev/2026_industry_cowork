"""Shared helpers: load festa_demo scripts with rclpy and the ROS message packages stubbed.

The stubs are installed only while a script is imported (sys.modules is restored afterwards),
so tests that need the real launch / launch_ros packages are not affected.
"""
import importlib.util
import os
import sys
import types
from unittest import mock

import pytest

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(PKG_DIR, 'scripts')

STUBBED = [
    'rclpy', 'rclpy.node', 'rclpy.action', 'rclpy.qos',
    'action_msgs', 'action_msgs.msg', 'builtin_interfaces', 'builtin_interfaces.msg',
    'control_msgs', 'control_msgs.action', 'control_msgs.msg',
    'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg',
    'nav2_msgs', 'nav2_msgs.action', 'nav2_msgs.srv',
    'sensor_msgs', 'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv',
    'trajectory_msgs', 'trajectory_msgs.msg',
]


class Dummy:
    """Message stand-in: keyword init, auto-vivifying attributes, list-like item access."""

    def __init__(self, *args, **kw):
        self.__dict__.update(kw)

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        child = Dummy()
        self.__dict__[name] = child
        return child

    def __setitem__(self, k, v):
        self.__dict__[('item', k)] = v

    def __getitem__(self, k):
        return self.__dict__.get(('item', k), 0.0)


class _Param:
    def __init__(self, value):
        self.value = value


class StubNode:
    def __init__(self, name='node', **kw):
        self.node_name = name
        self._log = mock.MagicMock()

    def declare_parameter(self, name, default=None):
        return _Param(default)

    def get_logger(self):
        return self._log

    def create_subscription(self, *a, **k):
        return mock.MagicMock()

    def create_publisher(self, *a, **k):
        return mock.MagicMock()

    def create_timer(self, *a, **k):
        return mock.MagicMock()

    def create_client(self, *a, **k):
        return mock.MagicMock()

    def get_clock(self):
        return mock.MagicMock()

    def destroy_node(self):
        pass


class _Meta(type):
    """Class-level attribute access (enum constants, Result.SUCCESSFUL, ...) yields 0."""

    def __getattr__(cls, name):
        if name.startswith('__'):
            raise AttributeError(name)
        return 0


class _StubModule(types.ModuleType):
    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        cls = _Meta(name, (Dummy,), {})
        setattr(self, name, cls)
        return cls


class _GoalStatus(Dummy):
    STATUS_SUCCEEDED = 4
    STATUS_CANCELED = 5
    STATUS_ABORTED = 6


def _make_stubs():
    mods = {n: _StubModule(n) for n in STUBBED}
    rclpy = mods['rclpy']
    rclpy.init = mock.MagicMock()
    rclpy.shutdown = mock.MagicMock()
    rclpy.ok = mock.MagicMock(return_value=True)
    rclpy.spin_once = mock.MagicMock()
    rclpy.spin = mock.MagicMock()
    rclpy.spin_until_future_complete = mock.MagicMock()
    mods['rclpy.node'].Node = StubNode
    mods['action_msgs.msg'].GoalStatus = _GoalStatus
    return mods


def load_script(filename):
    """Import scripts/<filename> as a fresh module with rclpy & message packages stubbed."""
    path = os.path.join(SCRIPTS, filename)
    name = 'festa_test_' + os.path.splitext(filename)[0]
    saved = {n: sys.modules.get(n) for n in STUBBED}
    sys.modules.update(_make_stubs())
    try:
        spec = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        for n, m in saved.items():
            if m is None:
                sys.modules.pop(n, None)
            else:
                sys.modules[n] = m
    return mod


@pytest.fixture(scope='session')
def box_on_path():
    return load_script('box_on_path.py')


@pytest.fixture(scope='session')
def send_goal():
    return load_script('send_goal.py')


@pytest.fixture(scope='session')
def rear_pick():
    return load_script('rear_pick.py')


@pytest.fixture(scope='session')
def obstacle_clear():
    return load_script('obstacle_clear_sequence.py')
