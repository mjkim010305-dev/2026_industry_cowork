#!/usr/bin/env python3
"""Unit tests for obstacle_marks' pure functions. Import only, no ROS graph."""
import json
import os
import sys

from pytest import approx

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from obstacle_marks import make_mark, split_expired, goal_ended, marks_json  # noqa: E402


def test_1_square_centre_and_side():
    # face at x = 2.0 facing the robot at -x, width 0.6 -> centre 0.3 behind the face
    m = make_mark((2.0, 1.0), (-1.0, 0.0), 0.6, 5.0)
    assert (m["x"], m["y"]) == approx((2.3, 1.0))
    assert m["side"] == approx(0.6)
    assert m["t"] == 5.0


def test_2_tiny_width_floor():
    m = make_mark((0.0, 0.0), (0.0, -1.0), 0.0, 0.0)
    assert m["side"] == approx(0.1)
    assert (m["x"], m["y"]) == approx((0.0, 0.05))


def test_3_ttl_expiry():
    a = make_mark((0, 0), (1, 0), 0.5, 0.0)
    b = make_mark((1, 1), (1, 0), 0.5, 100.0)
    alive, expired = split_expired([a, b], 120.0, 120.0)
    assert alive == [b] and expired == [a]
    alive, expired = split_expired([a, b], 119.9, 120.0)
    assert alive == [a, b] and expired == []


def test_4_goal_terminal_clears_once():
    seen = set()
    assert not goal_ended([("g1", 2)], seen)          # executing
    assert goal_ended([("g1", 4)], seen)              # succeeded -> clear
    # old terminal g1 still listed next to a running g2: no clear
    assert not goal_ended([("g1", 4), ("g2", 2)], seen)
    assert goal_ended([("g1", 4), ("g2", 6)], seen)   # g2 aborted -> clear
    assert not goal_ended([("g1", 4), ("g2", 6)], seen)
    assert goal_ended([("g3", 5)], seen)              # canceled


def test_5_json_shape_for_cpp_parser():
    marks = [make_mark((2.0, 1.0), (-1.0, 0.0), 0.6, 0.0), make_mark((0, 0), (1, 0), 0.0, 0.0)]
    text = marks_json(marks, "map")
    assert " " not in text
    d = json.loads(text)
    assert d["frame_id"] == "map"
    assert len(d["objects"]) == 2
    for o in d["objects"]:
        assert set(o) == {"x", "y", "width_m", "movable", "pushable"}
        assert o["movable"] is False and o["pushable"] is False
    assert d["objects"][0]["width_m"] == approx(0.6)
    assert json.loads(marks_json([], "map"))["objects"] == []
