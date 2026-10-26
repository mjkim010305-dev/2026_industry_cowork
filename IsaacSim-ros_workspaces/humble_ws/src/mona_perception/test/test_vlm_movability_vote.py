#!/usr/bin/env python3
"""Unit tests for vlm_movability_node's independent-trial vote gate
(_vote, _match_votes) and the vote_frame (odom) transform (_apply_transform).

Import only - no rclpy node is constructed, no ROS graph is touched.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from vlm_movability_node import (  # noqa: E402
    _vote, _match_votes, _vote_round, _apply_transform,
    _pending, _is_judged, _majority_label,
)

K = 2
RATIO = 0.6
GATE = 0.5
SIZE_RATIO = 0.5


def _cand(distance, lateral, width_m, height_m, raw_movable):
    return {"distance": distance, "lateral": lateral, "width_m": width_m,
            "height_m": height_m, "raw_movable": raw_movable}


def _run_round(history, candidates):
    """Thin wrapper over the node's own ``_vote_round`` - kept so existing
    call sites below read the same as before, but the no-candidates rule
    (R3: a failed round is a no-op) now comes from the node's function
    itself rather than being re-derived here."""
    return _vote_round(candidates, history, GATE, SIZE_RATIO, K, RATIO)


# ---------------------------------------------------------------------
# _vote

def test_single_movable_call_does_not_confirm_movable():
    # Exactly the forklift-called-"Carton" scenario: one round, one true.
    assert _vote([True], K, RATIO) is False


def test_majority_of_movable_calls_confirms_movable():
    assert _vote([True, True], K, RATIO) is True
    assert _vote([True, True, False], K, RATIO) is True  # 2/3 >= 0.6


def test_fixed_mixed_in_below_ratio_stays_false():
    assert _vote([True, False], K, RATIO) is False        # 1/2 < 0.6
    assert _vote([True, False, False], K, RATIO) is False  # 1/3 < 0.6


# ---------------------------------------------------------------------
# _match_votes

def test_same_object_across_rounds_accumulates_votes():
    # Round 1: one movable call on the box.
    history, voted = _match_votes(
        [_cand(1.0, 0.0, 0.5, 0.4, True)], [], GATE, SIZE_RATIO, K, RATIO)
    assert voted == [False]  # only 1 trial, below k=2

    # Round 2: same box (geometry close, similar size), another movable call.
    history, voted = _match_votes(
        [_cand(1.02, 0.01, 0.51, 0.41, True)], history, GATE, SIZE_RATIO, K, RATIO)
    assert voted == [True]
    assert history[0]["votes"] == [True, True]


def test_lost_track_resets_history_for_the_new_occupant():
    # Round 1: box votes movable, becomes 2/2 majority.
    history, _ = _match_votes(
        [_cand(1.0, 0.0, 0.5, 0.4, True)], [], GATE, SIZE_RATIO, K, RATIO)
    history, voted = _match_votes(
        [_cand(1.0, 0.0, 0.5, 0.4, True)], history, GATE, SIZE_RATIO, K, RATIO)
    assert voted == [True]

    # Round 3: box is gone from the frame; a forklift 1.3 m farther away,
    # 0.9 m to the side, and a different size appears instead - it must not
    # inherit the box's votes, even once, no matter what this round's raw
    # call says.
    history, voted = _match_votes(
        [_cand(2.3, 0.9, 1.35, 0.58, True)], history, GATE, SIZE_RATIO, K, RATIO)
    assert voted == [False]
    assert history[0]["votes"] == [True]  # fresh history, not [True, True, True]


# ---------------------------------------------------------------------
# R4: box + forklift together, pending/judged, blank rounds, odom matching

def test_box_and_forklift_match_1to1_in_the_same_round():
    # Round 1: box near (movable call), forklift farther and bigger (fixed
    # call) - both present together, must match to their own history entry.
    box1 = _cand(1.0, 0.0, 0.5, 0.4, True)
    fork1 = _cand(2.5, 0.8, 1.4, 1.8, False)
    history, voted = _match_votes([box1, fork1], [], GATE, SIZE_RATIO, K, RATIO)
    assert voted == [False, False]          # 1 trial each, below k=2 -> pending

    # Round 2: same pair, box confirmed movable again; forklift misjudged
    # "movable" once - a single misfire must not flip a majority.
    box2 = _cand(1.02, 0.01, 0.51, 0.41, True)
    fork2 = _cand(2.48, 0.79, 1.41, 1.79, True)
    history, voted = _match_votes([box2, fork2], history, GATE, SIZE_RATIO, K, RATIO)
    assert voted == [True, False]           # box: 2/2 movable; forklift: 1/2 < 0.6
    assert history[0]["votes"] == [True, True]
    assert history[1]["votes"] == [False, True]


def test_below_k_votes_is_pending_not_judged():
    # Exactly the R1 contract: with movable_vote_k=2, a single round is not
    # enough to call it either way - pending, not FIXED. Uses the node's own
    # _pending/_is_judged rather than re-deriving "n < k" here (R11).
    history, voted = _match_votes(
        [_cand(1.4, 0.0, 0.22, 0.92, True)], [], GATE, SIZE_RATIO, K, RATIO)
    n = len(history[0]["votes"])
    pending = _pending(n, K)
    assert voted == [False]
    assert pending is True
    assert n == 1
    assert _is_judged("unknown", pending) is False   # never named -> not judged
    assert _is_judged("box", pending) is False        # named, but still short of k
    assert _is_judged("box", False) is True           # named and past the vote gate


def test_blank_round_does_not_erase_history():
    # Round 1: one movable call on the box.
    history, voted = _run_round([], [_cand(1.0, 0.0, 0.5, 0.4, True)])
    assert history[0]["votes"] == [True]

    # Round 2: segmentation/VLM failure or nothing detected - the node never
    # calls _match_votes for this round (see _run_round), so the box's
    # single vote must survive untouched.
    history, voted = _run_round(history, [])
    assert voted == []
    assert history[0]["votes"] == [True]

    # Round 3: same box seen again - the vote count picks up from 1, not 0.
    history, voted = _run_round(history, [_cand(1.0, 0.0, 0.5, 0.4, True)])
    assert voted == [True]
    assert history[0]["votes"] == [True, True]


# ---------------------------------------------------------------------
# R2: vote_frame (odom) transform survives robot motion between rounds

def test_apply_transform_matches_same_object_after_robot_moves_forward():
    # Standard optical-frame (x-right, y-down, z-forward) -> base/odom-frame
    # (x-forward, y-left, z-up) rotation (REP103). A stationary point 1.5 m
    # dead ahead of the camera transforms to (1.5, 0.0, 0.0) in odom.
    optical_to_base = (-0.5, 0.5, -0.5, 0.5)

    # Round 1: robot (and camera) at the odom origin, box 1.5 m ahead.
    odom1 = _apply_transform([0.0, 0.0, 1.5], (0.0, 0.0, 0.0), optical_to_base)
    assert odom1[0] == 1.5 and odom1[1] == 0.0

    # Round 2: robot has driven 0.8 m forward in odom; the same, stationary
    # box is now only 0.7 m from the camera - in the camera frame alone that
    # 0.8 m jump exceeds track_gate (0.5 m) and the object would wrongly
    # look lost. Transformed into odom it lands on the same spot.
    odom2 = _apply_transform([0.0, 0.0, 0.7], (0.8, 0.0, 0.0), optical_to_base)
    assert odom2[0] == 1.5 and odom2[1] == 0.0

    cam_dist_1, cam_dist_2 = 1.5, 0.7
    assert abs(cam_dist_1 - cam_dist_2) > GATE          # camera frame: looks lost
    assert abs(odom1[0] - odom2[0]) < GATE               # odom frame: still matches

    box1 = _cand(odom1[0], odom1[1], 0.5, 0.4, True)
    box2 = _cand(odom2[0], odom2[1], 0.51, 0.4, True)
    history, voted = _match_votes([box1], [], GATE, SIZE_RATIO, K, RATIO)
    history, voted = _match_votes([box2], history, GATE, SIZE_RATIO, K, RATIO)
    assert voted == [True]
    assert history[0]["votes"] == [True, True]


# ---------------------------------------------------------------------
# R11: greedy 1:1 competition for one history slot, and label majority

def test_greedy_match_gives_one_history_slot_to_the_nearer_candidate():
    # Two of this round's candidates both fall inside the gate of a single
    # history slot at once - only one may inherit its vote count. Greedy
    # nearest-neighbour (shared with _carry_verdicts via _greedy_match)
    # picks the nearer one; the other starts a fresh history entry rather
    # than being dropped or double-counted.
    history = [{"distance": 1.0, "lateral": 0.0, "width_m": 0.5, "height_m": 0.4,
                "votes": [True], "labels": ["box"]}]
    near = _cand(1.02, 0.0, 0.51, 0.41, True)    # 0.02 m from the history slot
    far = _cand(1.2, 0.0, 0.52, 0.41, True)      # 0.2 m away, still inside gate alone

    history2, voted = _match_votes([near, far], history, GATE, SIZE_RATIO, K, RATIO)
    assert history2[0]["votes"] == [True, True]   # near candidate inherited the slot
    assert history2[1]["votes"] == [True]          # far candidate starts fresh
    assert voted == [True, False]                  # near: 2/2 majority; far: 1 trial, pending


def test_majority_label_breaks_ties_with_the_latest_round():
    assert _majority_label(["forklift", "forklift", "carton"]) == "forklift"
    assert _majority_label(["forklift", "carton"]) == "carton"   # tie -> most recent round
    # A 1-vote latest label must not beat two 2-vote leaders; the leader seen
    # last ("pallet", round 4) wins.
    assert _majority_label(["forklift", "pallet", "forklift", "pallet", "machine"]) == "pallet"
    assert _majority_label([]) is None


# ---------------------------------------------------------------------
# Session 9: this round's own FIXED call vetoes a movable vote majority

def test_latest_fixed_call_vetoes_latch_despite_movable_majority():
    # t7: a forklift fragment inherited 23 movable votes from a merged
    # cluster; its own round said fixed. _select must not pick it.
    from vlm_movability_node import VlmMovability
    node = VlmMovability.__new__(VlmMovability)
    node.push_max_height, node.push_max_width = 0.7, 1.2
    fork = dict(_cand(1.3, -0.2, 0.76, 0.51, False), movable=True)
    assert node._select([fork]) is None
    box = dict(_cand(1.4, 0.1, 0.5, 0.4, True), movable=True)
    assert node._select([fork, box]) is box


# ---------------------------------------------------------------------
# Session 10: re-acquiring the pushed object after the robot backs off

from vlm_movability_node import (  # noqa: E402
    _advance_along, _carried_prediction, _pick_reacquire,
)


def test_carried_prediction_uses_furthest_advance_not_current():
    # Lost at box (30.4, -3.7), heading +x; robot pushed 1.5 m then reversed
    # 1.0 m. The box stayed where the push left it.
    start = (28.9, -3.45)
    assert abs(_advance_along(start, 0.0, (30.4, -3.45)) - 1.5) < 1e-9
    assert abs(_advance_along(start, 0.0, (29.4, -3.45)) - 0.5) < 1e-9
    px, py = _carried_prediction((30.4, -3.7), 0.0, 1.5)
    assert abs(px - 31.9) < 1e-9 and abs(py + 3.7) < 1e-9


def test_pick_reacquire_nearest_pushable_within_gate():
    predicted = (31.9, -3.7)
    swept_box = (31.8, -4.2)      # swung 0.5 m to the right by the sweep
    fork_part = None              # too large: never pickable
    far_box = (33.5, -3.7)        # 1.6 m off - outside the 1.0 m gate
    assert _pick_reacquire([fork_part, far_box, swept_box], predicted, 1.0) == 2
    assert _pick_reacquire([fork_part, far_box], predicted, 1.0) is None
