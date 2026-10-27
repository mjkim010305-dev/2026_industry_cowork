#!/usr/bin/env python3
"""Unit tests for vlm_movability_node's on-demand JudgeObstacle SAM pipeline.

Pure functions are called directly; the per-frame pipeline runs on a bare node
(``__new__``, no rclpy init) with SAM and the VLM monkeypatched. No ROS graph.
"""
import base64
import json
import os
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import vlm_movability_node as vm  # noqa: E402
from vlm_movability_node import (  # noqa: E402
    VlmMovability, _JudgeFail, _apply_transform, _assemble_verdict, _build_prompts,
    _face_samples, _floor_from_face, _judge_marker_spec, _judge_verdict, _mask_bbox, _mask_highlight, _mask_size,
    _mask_valid, _project_to_pixel, _region_box, _select_mask, _valid_points,
)

# Depth-grid intrinsics shared by the tests: 160x120, f=100, centre (80, 60).
INTR = (100.0, 100.0, 80.0, 60.0)
W, H = 160, 120
# map -> optical for a level camera at the map origin looking along map +x,
# 0.6 m above the floor: opt = (-y, 0.6 - z, x). q is the inverse of the launch
# file's body->optical quaternion (checked by the first test).
Q = (0.5, -0.5, 0.5, 0.5)
T = (0.0, 0.6, 0.0)


def test_map_to_optical_convention():
    assert np.allclose(_apply_transform((2.0, 0.3, 0.0), T, Q), (-0.3, 0.6, 2.0))


# ---------------------------------------------------------------------------
# projection / prompts

def test_project_to_pixel_hand_computed():
    # u = 100*0.3/2 + 80 = 95, v = 100*0.1/2 + 60 = 65
    assert _project_to_pixel((0.3, 0.1, 2.0), INTR, 1.0, 1.0, W, H) == (95, 65)
    # RGB twice the depth grid: pixel scales with it
    assert _project_to_pixel((0.3, 0.1, 2.0), INTR, 2.0, 2.0, 2 * W, 2 * H) == (190, 130)
    assert _project_to_pixel((0.3, 0.1, -1.0), INTR, 1.0, 1.0, W, H) is None   # behind
    assert _project_to_pixel((5.0, 0.0, 2.0), INTR, 1.0, 1.0, W, H) is None    # off-image


def test_project_through_rotated_tf():
    # 90 deg about z maps (x, y, z) -> (-y, x, z): map point (0.1, -0.3, 2.0)
    # lands at (0.3, 0.1, 2.0) in the camera frame -> pixel (95, 65).
    q = (0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4))
    p = _apply_transform((0.1, -0.3, 2.0), (0.0, 0.0, 0.0), q)
    assert np.allclose(p, (0.3, 0.1, 2.0))
    assert _project_to_pixel(p, INTR, 1.0, 1.0, W, H) == (95, 65)


def test_build_prompts_two_heights_and_depth_validity():
    depth = np.full((H, W), 2.0, np.float32)
    # face at map x=2.0, y=+-0.2 and 0.0 -> opt x = -y; heights 0.2/0.3 ->
    # opt y = 0.4/0.3 -> v = 80/75; opt x -0.2/0/0.2 -> u = 70/80/90
    pts = _build_prompts([(2.0, 0.2), (2.0, 0.0), (2.0, -0.2)], [0.2, 0.3], T, Q, INTR,
                         depth, 1.0, 1.0, W, H, 0.25, 5.0)
    assert sorted(pts) == [[70, 75], [70, 80], [80, 75], [80, 80], [90, 75], [90, 80]]
    # a hole in the depth at one prompt pixel drops just that prompt
    depth[80, 70] = np.nan
    pts = _build_prompts([(2.0, 0.2)], [0.2, 0.3], T, Q, INTR, depth, 1.0, 1.0, W, H,
                         0.25, 5.0)
    assert pts == [[70, 75]]
    # a face behind the camera yields nothing
    assert _build_prompts([(-1.0, 0.0)], [0.2], T, Q, INTR, depth, 1.0, 1.0, W, H,
                          0.25, 5.0) == []


def test_face_samples():
    line = [(2.0, y / 10.0) for y in range(8)]
    assert _face_samples(line, (2.0, 0.0), (2.0, 0.7)) == [line[2], line[4], line[6]]
    assert _face_samples([(2.0, 0.0)], (2.0, 0.0), (2.0, 0.4)) == [(2.0, 0.2)]


# ---------------------------------------------------------------------------
# depth region box

def _box_scene():
    """Box (x -0.3..0.3, z 2.0..2.5, above floor) and a forklift 0.6 m to the
    side (x 0.5..1.0), both 0.1..0.5 m high."""
    rng = np.random.default_rng(1)

    def block(x0, x1):
        n = 400
        x = rng.uniform(x0, x1, n)
        z = rng.uniform(2.0, 2.5, n)
        h = rng.uniform(0.1, 0.5, n)
        return x, 0.6 - h, z, h
    bx, by, bz, bh = block(-0.3, 0.3)
    fx_, fy_, fz, fh = block(0.5, 1.0)
    pts = np.stack([np.r_[bx, fx_], np.r_[by, fy_], np.r_[bz, fz]], axis=1)
    us = pts[:, 0] * 100.0 / pts[:, 2] + 80.0
    vs = pts[:, 1] * 100.0 / pts[:, 2] + 60.0
    return pts, us, vs, np.r_[bh, fh]


def test_region_box_isolates_box_from_neighbour():
    pts, us, vs, h = _box_scene()
    face = [(-0.3, 0.6, 2.0), (0.3, 0.6, 2.0)]
    box = _region_box(pts, us, vs, h, face, 0.15, 1.0, 0.07, 2.5, 60, 1.0, 1.0)
    assert box is not None
    # the neighbour's pixels start at u = 0.5*100/2.5+80 = 100; box ends <= 80+0.3*100/2
    assert box[2] < 100
    assert box[0] >= 80 - 0.3 * 100 / 2.0 - 1
    # without the lateral limit (region wide) the neighbour is included
    wide = _region_box(pts, us, vs, h, [(-0.3, 0.6, 2.0), (1.0, 0.6, 2.0)],
                       0.15, 1.0, 0.07, 2.5, 60, 1.0, 1.0)
    assert wide[2] > 100
    # nothing in the region -> None
    assert _region_box(pts, us, vs, h, [(-3.0, 0.6, 2.0), (-2.5, 0.6, 2.0)],
                       0.15, 1.0, 0.07, 2.5, 60, 1.0, 1.0) is None


# ---------------------------------------------------------------------------
# masks

def _depth_with_box():
    depth = np.full((H, W), 4.0, np.float32)
    depth[60:85, 65:95] = 2.0                       # box front face at 2.0 m
    mask = np.zeros((H, W), bool)
    mask[60:85, 65:95] = True
    return depth, mask


def test_select_mask_rejects_depth_inconsistent_mask():
    depth, box_mask = _depth_with_box()
    wall = np.zeros((H, W), bool)
    wall[0:50, :] = True                            # background wall at 4.0 m
    masks, scores = [wall, box_mask], [0.99, 0.60]
    # the wall scores higher but is 2 m behind the face
    assert _select_mask(masks, scores, depth, 2.0, 0.3, 0.25, 5.0, 50) == 1
    # only inconsistent masks -> None
    assert _select_mask([wall], [0.99], depth, 2.0, 0.3, 0.25, 5.0, 50) is None
    # two consistent masks -> highest score wins
    assert _select_mask([box_mask, box_mask.copy()], [0.5, 0.8], depth, 2.0, 0.3,
                        0.25, 5.0, 50) == 1


def test_mask_highlight_dims_outside_keeps_inside():
    bgr = np.full((H, W, 3), 200, np.uint8)
    mask = np.zeros((H, W), bool)
    mask[40:80, 50:100] = True
    out = _mask_highlight(bgr, mask, 0.45)
    assert (out[50:70, 60:90] == 200).all()         # interior untouched
    assert (out[0:20, 0:20] == 90).all()            # int(200 * 0.45)
    assert (out[40, 50:100] == (255, 0, 255)).all(axis=1).any()   # contour drawn


def test_mask_size_synthetic_box_with_fitted_floor_and_face_floor():
    depth, mask = _depth_with_box()
    ys, xs, z = _mask_valid(mask, depth, 0.25, 5.0)
    pts = np.stack([(xs - 80.0) * z / 100.0, (ys - 60.0) * z / 100.0, z], axis=1)
    # camera 0.5 m above the floor: floor y = +0.5, height = -y + 0.5
    width, height = _mask_size(pts, (np.array([0.0, -1.0, 0.0]), 0.5))
    assert width == pytest.approx(0.6, abs=0.05)
    assert height == pytest.approx(0.5, abs=0.05)
    # same numbers from the face: face points on the floor, up = -y_opt
    plane = _floor_from_face([(-0.3, 0.5, 2.0), (0.3, 0.5, 2.0)], (0.0, -1.0, 0.0))
    assert _mask_size(pts, plane) == pytest.approx((width, height))


def test_mask_valid_resizes_mask_to_depth_grid():
    depth, mask = _depth_with_box()
    big = cv2.resize(mask.astype(np.uint8), (2 * W, 2 * H),
                     interpolation=cv2.INTER_NEAREST).astype(bool)
    assert _mask_valid(big, depth, 0.25, 5.0)[2].size == _mask_valid(mask, depth, 0.25, 5.0)[2].size


def test_valid_points_matches_pinhole():
    depth = np.full((H, W), 2.0, np.float32)
    pts, us, vs = _valid_points(depth, INTR, 4, 0.25, 5.0)
    i = int(np.argmax((us == 96) & (vs == 68)))
    assert pts[i] == pytest.approx([0.32, 0.16, 2.0])


# ---------------------------------------------------------------------------
# verdict assembly

def test_judge_verdict_vote_and_h1():
    assert _judge_verdict([True], 1, 0.6) == "MOVABLE"
    assert _judge_verdict([False], 1, 0.6) == "FIXED"
    assert _judge_verdict([True, True], 2, 0.6) == "MOVABLE"
    assert _judge_verdict([True, False], 2, 0.6) == "FIXED"
    assert _judge_verdict([False, True], 2, 0.6) == "FIXED"     # 0.5 < 0.6
    assert _judge_verdict([False, False], 2, 0.6) == "FIXED"
    # 2/3 passes the ratio but H1 needs the last call movable
    assert _judge_verdict([True, True, False], 3, 0.6) == "FIXED"


def test_assemble_verdict_pushable_is_independent_of_verdict():
    r = _assemble_verdict([False], ["forklift"], 0.5, 0.4, 1, 0.6, 0.7, 1.2)
    assert (r["verdict"], r["label"], r["pushable"], r["votes_n"], r["votes_movable"]) == \
        ("FIXED", "forklift", True, 1, 0)
    r = _assemble_verdict([True], ["box"], 0.5, 0.9, 1, 0.6, 0.7, 1.2)
    assert (r["verdict"], r["pushable"]) == ("MOVABLE", False)


# ---------------------------------------------------------------------------
# per-frame pipeline on a bare node, fake SAM / VLM

def _bare_node():
    n = object.__new__(VlmMovability)
    n.intrinsics = INTR
    n.min_range, n.max_range, n.depth_stride = 0.25, 10.0, 4
    n.floor_iters, n.floor_tol = 120, 0.035
    n.obstacle_min_h, n.obstacle_max_h = 0.07, 2.5
    n.cluster_min_pts = 30                      # test depth grid is small
    n.region_margin, n.region_depth = 0.15, 1.0
    n.prompt_heights = [0.2, 0.3]
    n.mask_depth_tol, n.mask_min_px = 0.3, 50
    n.highlight_dim, n.jpeg_quality = 0.45, 85
    n.push_max_height, n.push_max_width = 0.7, 1.2
    n.movable_vote_ratio, n.judge_k = 0.6, 1
    n.sam_url, n.sam_timeout = "http://sam.invalid:8091", 1.0
    n.judge_backend, n.judge_url = "ollama", "http://judge.invalid:8092"
    n.movable_threshold, n.request_timeout = 0.5, 7.0
    return n


def _scene():
    """Floor at y=0.6 (camera 0.6 m up), a box 0.6 wide x 0.5 high with its
    front face 2.0 m ahead; RGB same size as depth."""
    depth = np.full((H, W), np.nan, np.float32)
    for v in range(H):
        if v > 60:                                  # floor rows: y = 0.6
            depth[v, :] = 0.6 * 100.0 / (v - 60.0)
    depth[(depth > 10.0)] = np.nan
    depth[65:90, 65:95] = 2.0                       # v 65..89 = 0.5 m above the floor
    mask = np.zeros((H, W), bool)
    mask[65:90, 65:95] = True
    bgr = np.full((H, W, 3), 120, np.uint8)
    face = {"samples": [(2.0, 0.0)],
            "pts": [(2.0, 0.3, 0.0), (2.0, -0.3, 0.0)]}
    return bgr, depth, mask, face


def _frames(bgr, depth):
    count = {"n": 0}

    def next_frame(prev):
        count["n"] += 1
        header = SimpleNamespace(stamp=SimpleNamespace(sec=count["n"], nanosec=0))
        return header, bgr, depth, T, Q
    return next_frame


def _patch(monkeypatch, node, masks, movable=True, label="cardboard box", sam_exc=None,
           vlm_exc=None):
    seen = {}

    def fake_sam(bgr, points, box):
        seen["points"], seen["box"] = points, box
        if sam_exc:
            raise sam_exc
        return masks, [0.9] * len(masks)

    def fake_ask(image, cand):
        seen["image"] = image
        if vlm_exc:
            raise vlm_exc
        cand["label"], cand["raw_movable"] = label, movable
        return 0.1, "{}"
    monkeypatch.setattr(node, "_call_sam", fake_sam)
    monkeypatch.setattr(node, "_ask_image", fake_ask)
    return seen


def test_pipeline_movable_with_size(monkeypatch):
    bgr, depth, mask, face = _scene()
    node = _bare_node()
    seen = _patch(monkeypatch, node, [mask])
    result, debug = node._judge(face, _frames(bgr, depth))
    assert result["verdict"] == "MOVABLE" and result["label"] == "cardboard box"
    assert result["pushable"] is True and result["votes_n"] == 1
    assert len(seen["points"]) == 2 and seen["box"] is not None
    # the VLM got the mask-highlighted image: outside dimmed, inside original
    assert seen["image"][0, 0, 0] == int(120 * 0.45) and seen["image"][70, 80, 0] == 120
    w, h = debug["size"]
    assert w == pytest.approx(0.6, abs=0.06) and h == pytest.approx(0.5, abs=0.06)


def test_pipeline_fixed_and_not_pushable(monkeypatch):
    bgr, depth, mask, face = _scene()
    node = _bare_node()
    node.push_max_height = 0.3
    _patch(monkeypatch, node, [mask], movable=False, label="forklift")
    result, _ = node._judge(face, _frames(bgr, depth))
    assert (result["verdict"], result["label"], result["pushable"]) == \
        ("FIXED", "forklift", False)


def test_pipeline_two_votes_use_two_frames(monkeypatch):
    bgr, depth, mask, face = _scene()
    node = _bare_node()
    node.judge_k = 2
    _patch(monkeypatch, node, [mask])
    frames = _frames(bgr, depth)
    result, _ = node._judge(face, frames)
    assert result["verdict"] == "MOVABLE" and result["votes_n"] == 2
    assert frames(None)[0].stamp.sec == 3                 # two frames were consumed


def test_sam_failure_is_error(monkeypatch):
    bgr, depth, mask, face = _scene()
    node = _bare_node()
    _patch(monkeypatch, node, [mask], sam_exc=OSError("connection refused"))
    result, _ = node._judge(face, _frames(bgr, depth))
    assert result["verdict"] == "ERROR" and "sam" in result["detail"]
    assert "connection refused" in result["detail"]


def test_sam_zero_masks_and_inconsistent_mask_are_error(monkeypatch):
    bgr, depth, mask, face = _scene()
    node = _bare_node()
    _patch(monkeypatch, node, [])
    result, _ = node._judge(face, _frames(bgr, depth))
    assert result["verdict"] == "ERROR" and "no masks" in result["detail"]
    wall = np.zeros((H, W), bool)
    wall[100:120, :] = True                               # floor strip, not at 2.0 m
    _patch(monkeypatch, node, [wall])
    result, _ = node._judge(face, _frames(bgr, depth))
    assert result["verdict"] == "ERROR" and "consistent" in result["detail"]


def test_vlm_exception_is_error(monkeypatch):
    bgr, depth, mask, face = _scene()
    node = _bare_node()
    _patch(monkeypatch, node, [mask], vlm_exc=RuntimeError("no parsable verdict"))
    result, _ = node._judge(face, _frames(bgr, depth))
    assert result["verdict"] == "ERROR" and "vlm" in result["detail"]


def test_no_valid_prompt_is_no_candidate(monkeypatch):
    bgr, depth, mask, face = _scene()
    node = _bare_node()
    _patch(monkeypatch, node, [mask])
    result, _ = node._judge({"samples": [(-2.0, 0.0)], "pts": [(-2.0, 0.3, 0.0), (-2.0, -0.3, 0.0)]},
                            _frames(bgr, depth))
    assert result["verdict"] == "NO_CANDIDATE"


def test_cancel_stops_before_any_vote(monkeypatch):
    bgr, depth, mask, face = _scene()
    node = _bare_node()
    _patch(monkeypatch, node, [mask])
    result, _ = node._judge(face, _frames(bgr, depth), lambda: True)
    assert result["verdict"] == "ERROR" and result["detail"] == "canceled"


# ---------------------------------------------------------------------------
# SAM HTTP layer

def test_call_sam_request_and_response(monkeypatch):
    node = _bare_node()
    bgr = np.full((H, W, 3), 120, np.uint8)
    png = np.zeros((H, W), np.uint8)
    png[10:20, 10:20] = 255
    b64 = base64.b64encode(cv2.imencode(".png", png)[1].tobytes()).decode()
    captured = {}

    class Resp:
        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

        def read(self):
            return json.dumps({"masks_png_b64": [b64], "scores": [0.7],
                               "latency_s": 0.1, "model": "sam2.1_s"}).encode()

    def fake_urlopen(req, timeout):
        captured["url"], captured["timeout"] = req.full_url, timeout
        captured["body"] = json.loads(req.data)
        return Resp()
    monkeypatch.setattr(vm.urllib.request, "urlopen", fake_urlopen)
    masks, scores = node._call_sam(bgr, [[5, 6], [7, 8]], [1, 2, 3, 4])
    assert captured["url"] == "http://sam.invalid:8091/segment" and captured["timeout"] == 1.0
    body = captured["body"]
    assert body["points"] == [[5, 6], [7, 8]] and body["point_labels"] == [1, 1]
    assert body["box"] == [1, 2, 3, 4] and body["image_jpeg_b64"]
    assert scores == [0.7] and masks[0].dtype == bool and masks[0].sum() == 100


# ---------------------------------------------------------------------------
# judge_backend=logit

def test_mask_bbox_hand_computed():
    m = np.zeros((H, W), bool)
    m[10:20, 30:50] = True
    m[12, 60] = True
    assert _mask_bbox(m) == (30, 10, 61, 20)


def _fake_judge(monkeypatch, out=None, exc=None):
    captured = {}

    class Resp:
        def __enter__(self):
            return self

        def __exit__(self, *e):
            return False

        def read(self):
            return json.dumps(out).encode()

    def fake_urlopen(req, timeout):
        captured["url"], captured["timeout"] = req.full_url, timeout
        captured["body"] = json.loads(req.data)
        if exc:
            raise exc
        return Resp()
    monkeypatch.setattr(vm.urllib.request, "urlopen", fake_urlopen)
    return captured


def _score(p):
    return {"results": {"movable": {"p": p, "p_ab": p, "p_ba": p}},
            "latency_s": 0.4, "model": "Qwen3.5-4B"}


def _logit_node(monkeypatch):
    node = _bare_node()
    node.judge_backend = "logit"

    def no_ask(image, cand):
        raise AssertionError("ollama _ask_image must not be called")
    monkeypatch.setattr(node, "_ask_image", no_ask)
    return node


@pytest.mark.parametrize("p,verdict", [(0.7, "MOVABLE"), (0.3, "FIXED"), (0.5, "MOVABLE")])
def test_logit_threshold(monkeypatch, p, verdict):
    bgr, depth, mask, face = _scene()
    node = _logit_node(monkeypatch)
    monkeypatch.setattr(node, "_call_sam", lambda b, pts, box: ([mask], [0.9]))
    cap = _fake_judge(monkeypatch, _score(p))
    result, debug = node._judge(face, _frames(bgr, depth))
    assert result["verdict"] == verdict
    assert debug["p_movable"] == [p] and debug["model"] == "Qwen3.5-4B"
    assert "p_movable=%.2f" % p in result["detail"]
    assert cap["url"] == "http://judge.invalid:8092/score" and cap["timeout"] == 7.0


def test_logit_request_uses_mask_bbox_rectangle(monkeypatch):
    bgr, depth, mask, face = _scene()
    bgr = np.full((H, W, 3), 200, np.uint8)
    node = _logit_node(monkeypatch)
    # L-shaped mask: the bbox corner (65, 89) is outside the mask but inside its bbox
    mask = mask.copy()
    mask[65:90, 65:80] = False
    mask[65:90, 80:95] = True
    mask[65:70, 65:80] = True
    monkeypatch.setattr(node, "_call_sam", lambda b, pts, box: ([mask], [0.9]))
    cap = _fake_judge(monkeypatch, _score(0.9))
    node._judge(face, _frames(bgr, depth))
    body = cap["body"]
    assert body["context"] == vm.JUDGE_CONTEXT
    assert body["questions"] == [vm.MOVABLE_QUESTION]
    assert body["questions"][0]["options"] == ["movable", "fixed"]
    img = cv2.imdecode(np.frombuffer(base64.b64decode(body["image_jpeg_b64"]), np.uint8),
                       cv2.IMREAD_COLOR)
    assert abs(int(img[80, 75, 0]) - 200) < 25       # outside mask, inside bbox: original
    assert abs(int(img[2, 2, 0]) - 90) < 25          # outside bbox: dimmed (200*0.45)


def test_logit_failure_is_error_without_ollama_fallback(monkeypatch):
    bgr, depth, mask, face = _scene()
    node = _logit_node(monkeypatch)
    monkeypatch.setattr(node, "_call_sam", lambda b, pts, box: ([mask], [0.9]))
    _fake_judge(monkeypatch, exc=OSError("connection refused"))
    result, _ = node._judge(face, _frames(bgr, depth))
    assert result["verdict"] == "ERROR" and "judge" in result["detail"]
    assert "connection refused" in result["detail"]


def test_logit_response_without_question_id_is_error(monkeypatch):
    bgr, depth, mask, face = _scene()
    node = _logit_node(monkeypatch)
    monkeypatch.setattr(node, "_call_sam", lambda b, pts, box: ([mask], [0.9]))
    _fake_judge(monkeypatch, {"results": {}, "latency_s": 0.1, "model": "Qwen3.5-4B"})
    result, _ = node._judge(face, _frames(bgr, depth))
    assert result["verdict"] == "ERROR" and "judge" in result["detail"]


def test_logit_result_keys_match_ollama(monkeypatch):
    bgr, depth, mask, face = _scene()
    node = _bare_node()
    _patch(monkeypatch, node, [mask])
    ollama, ollama_debug = node._judge(face, _frames(bgr, depth))
    assert "p_movable" not in ollama_debug
    node = _logit_node(monkeypatch)
    monkeypatch.setattr(node, "_call_sam", lambda b, pts, box: ([mask], [0.9]))
    _fake_judge(monkeypatch, _score(0.8))
    logit, debug = node._judge(face, _frames(bgr, depth))
    assert set(logit) == set(ollama)                  # Result setattr stays safe
    assert debug["p_movable"] == [0.8]


# the S33-measured wording (mona-scenes judge/k2_vllm_logprob.py DECISION), written out
# literally so the test does not compare the node's constants with themselves
_PROMPT_HEAD_RECT = (
    "You are the perception module of a warehouse mobile robot with a small arm.\n"
    "The robot is driving down an aisle and something is in its way.\n"
    "\n"
    "Exactly one object is outlined by the magenta rectangle. Judge only that object. The\n"
    "rest of the image is context - do not describe it. Parked vehicles and machinery are\n"
    "objects too and must be judged, not dismissed as background.\n"
    "\n")
_DECISION = (
    "Decide whether the robot can PUSH the outlined object aside with its small arm:\n"
    "- movable : light, free-standing, small enough to shove\n"
    "            (cardboard box, empty crate, small carton, cone, bin)\n"
    "- fixed   : heavy machinery, vehicles, or fixed structure\n"
    "            (forklift, pallet jack, loaded pallet, rack, wall, pillar)\n"
    "\n"
    "Answer with exactly one word: movable or fixed.")


def test_logit_prompt_equals_measured_text_both_orders():
    text = vm.MOVABLE_QUESTION["text"]
    ab = vm.JUDGE_CONTEXT + text.replace("{A}", "movable").replace("{B}", "fixed")
    ba = vm.JUDGE_CONTEXT + text.replace("{A}", "fixed").replace("{B}", "movable")
    assert ab == _PROMPT_HEAD_RECT + _DECISION
    assert ba == _PROMPT_HEAD_RECT + _DECISION.replace("movable or fixed.", "fixed or movable.")
    assert ba.endswith("Answer with exactly one word: fixed or movable.")


@pytest.mark.parametrize("p", [float("nan"), float("inf"), -0.1, 1.1])
def test_call_judge_rejects_non_finite_or_out_of_range_p(monkeypatch, p):
    node = _bare_node()
    _fake_judge(monkeypatch, _score(p))
    with pytest.raises(ValueError):
        node._call_judge(np.zeros((8, 8, 3), np.uint8))


def test_nan_p_becomes_error_verdict(monkeypatch):
    bgr, depth, mask, face = _scene()
    node = _logit_node(monkeypatch)
    monkeypatch.setattr(node, "_call_sam", lambda b, pts, box: ([mask], [0.9]))
    _fake_judge(monkeypatch, _score(float("nan")))
    result, _ = node._judge(face, _frames(bgr, depth))
    assert result["verdict"] == "ERROR" and "judge" in result["detail"]


def _published_model(node, debug):
    sent = []
    node.judgments_pub = SimpleNamespace(publish=lambda m: sent.append(json.loads(m.data)))
    node.model = "gemma4:e4b"
    obstacle = SimpleNamespace(along_m=1.0, width_m=0.5, n_cells=3,
                               centroid=SimpleNamespace(x=1.0, y=0.0))
    node._publish_judge(obstacle, {"latency_s": 0.1, "verdict": "ERROR",
                                   "label": "", "detail": "x"}, debug)
    return sent[0]["model"]


def test_publish_judge_model_provenance():
    logit = _bare_node()
    logit.judge_backend = "logit"
    assert _published_model(logit, {}) is None                       # no judge call succeeded
    assert _published_model(logit, {"model": "Qwen3.5-4B"}) == "Qwen3.5-4B"
    assert _published_model(_bare_node(), {}) == "gemma4:e4b"        # ollama: self.model


# ---------------------------------------------------------------------------
# RViz judgment marker spec

def _geom(normal=(1.0, 0.0), start=(2.0, -0.3), end=(2.0, 0.3), width=0.65, present=True):
    return {"present": present, "face_start": start, "face_end": end,
            "normal": normal, "width_m": width}


def _res(verdict="MOVABLE", pushable=True, detail=""):
    return {"verdict": verdict, "pushable": pushable, "detail": detail}


def test_marker_geometry_normal_x():
    c = _judge_marker_spec(_geom(), _res(), {})["cube"]
    assert c["center"][:2] == pytest.approx((2.0 - 0.025, 0.0))
    assert c["yaw"] == pytest.approx(0.0)
    assert c["scale"][:2] == pytest.approx((0.05, 0.65))


def test_marker_yaw_from_normal_y():
    c = _judge_marker_spec(_geom(normal=(0.0, 1.0)), _res(), {})["cube"]
    assert c["yaw"] == pytest.approx(np.pi / 2)


def test_marker_single_cell_face_keeps_width():
    c = _judge_marker_spec(_geom(start=(2.0, 0.0), end=(2.0, 0.0), width=0.05),
                           _res(), {})["cube"]
    assert c["scale"][1] == pytest.approx(0.05)


def test_marker_height_from_size_or_fallback():
    c = _judge_marker_spec(_geom(), _res(), {"size": (0.53, 0.46)})["cube"]
    assert c["scale"][2] == pytest.approx(0.46) and c["center"][2] == pytest.approx(0.23)
    c = _judge_marker_spec(_geom(), _res("ERROR", detail="x"), {})["cube"]
    assert c["scale"][2] == pytest.approx(0.3) and c["center"][2] == pytest.approx(0.15)


def test_marker_colors():
    def rgb(r):
        return _judge_marker_spec(_geom(), r, {})["cube"]["rgba"]
    assert rgb(_res()) == (0.3, 0.85, 0.3, 0.5)
    assert rgb(_res(pushable=False)) == (1.0, 0.75, 0.15, 0.5)
    assert rgb(_res("FIXED")) == (0.9, 0.25, 0.25, 0.5)
    assert rgb(_res("ERROR")) == rgb(_res("NO_CANDIDATE")) == (0.6, 0.6, 0.6, 0.5)


def test_marker_text():
    def text(r, d):
        return _judge_marker_spec(_geom(), r, d)["text"]["text"]
    assert text(_res(), {"p_movable": [0.98], "size": (0.53, 0.46)}) == \
        "MOVABLE p=0.98 / 0.53x0.46 m"
    assert text(_res(), {"size": (0.53, 0.46)}) == "MOVABLE / 0.53x0.46 m"
    assert text(_res(pushable=False), {"size": (0.53, 0.46)}) == \
        "MOVABLE / 0.53x0.46 m (too large)"
    assert text(_res("ERROR", detail="e" * 60), {}) == "ERROR: " + "e" * 40


def test_marker_absent_obstacle_is_none():
    assert _judge_marker_spec(_geom(present=False), _res(), {}) is None


# ---------------------------------------------------------------------------
# _publish_judge_markers (ROS builder) on a bare node, fake publisher

class _FakePub:
    def __init__(self, exc=None):
        self.sent, self.exc = [], exc

    def publish(self, msg):
        if self.exc:
            raise self.exc
        self.sent.append(msg)


class _FakeLog:
    def __init__(self):
        self.warnings = []

    def warn(self, text):
        self.warnings.append(text)


def _marker_node(pub):
    n = _bare_node()
    n.judge_markers_pub, n._log = pub, _FakeLog()
    n.get_logger = lambda: n._log
    return n


def _obstacle(present=True, normal=(0.0, 1.0)):
    from std_msgs.msg import Header
    v = lambda x, y: SimpleNamespace(x=x, y=y)
    header = Header()
    header.frame_id = "map"
    return SimpleNamespace(
        present=present, face_start=v(1.0, 0.0), face_end=v(3.0, 0.0),
        normal=v(*normal), width_m=0.6,
        header=header)


def test_publish_markers_present_obstacle():
    from visualization_msgs.msg import Marker
    pub = _FakePub()
    _marker_node(pub)._publish_judge_markers(_obstacle(), _res(), {})
    assert len(pub.sent) == 1
    cube, text = pub.sent[0].markers
    assert [(m.ns, m.id) for m in (cube, text)] == [("vlm_judge", 0), ("vlm_judge", 1)]
    assert (cube.type, text.type) == (Marker.CUBE, Marker.TEXT_VIEW_FACING)
    assert cube.action == text.action == Marker.ADD
    # normal (0, 1): yaw = pi/2 -> z = sin(pi/4), w = cos(pi/4) = sqrt(2)/2
    assert cube.pose.orientation.x == 0.0 and cube.pose.orientation.y == 0.0
    assert cube.pose.orientation.z == pytest.approx(0.7071067811865476)
    assert cube.pose.orientation.w == pytest.approx(0.7071067811865476)
    assert cube.header.frame_id == text.header.frame_id == "map"


def test_publish_markers_absent_obstacle_deletes_all():
    from visualization_msgs.msg import Marker
    pub = _FakePub()
    _marker_node(pub)._publish_judge_markers(_obstacle(present=False), _res(), {})
    assert len(pub.sent[0].markers) == 1
    assert pub.sent[0].markers[0].action == Marker.DELETEALL


def test_publish_markers_publisher_error_is_swallowed():
    node = _marker_node(_FakePub(exc=RuntimeError("pub down")))
    node._publish_judge_markers(_obstacle(), _res(), {})          # must not raise
    assert len(node._log.warnings) == 1 and "pub down" in node._log.warnings[0]
