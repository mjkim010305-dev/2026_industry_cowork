#!/usr/bin/env python3
# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""VLM movability node.

Publishes the movable/fixed judgement that the rest of the stack has always
consumed but nobody produced - until now it was a human typing
``ros2 topic pub /obstacle/is_movable``.

The node watches the robot's forward RGB stream, asks a local vision-language
model whether each object blocking the aisle can be shoved aside by the small
arm, and turns that answer into the two topics the existing pipeline already
expects:

  * ``/obstacle/is_movable``  std_msgs/Bool              gate for
    ``perception_stabilizer`` / ``rgbd_obstacle_localizer`` / the BT condition
  * ``/vlm/roi``              sensor_msgs/RegionOfInterest  point
    ``perception_stabilizer``'s ``roi_topic`` at this and it stops falling back
    to a centred crop

plus two topics for the demo video and for provenance:

  * ``/vlm/annotated``        sensor_msgs/Image   RGB with boxes and labels
  * ``/vlm/judgments``        std_msgs/String     the raw JSON verdict, with
    each candidate carrying both this round's raw ``raw_movable`` call and
    the vote-gated ``movable``/``votes`` (n, n_movable) it takes k
    independent rounds to confirm (see ``movable_vote_k`` below) - a
    candidate short of k rounds is ``pending``, not yet judged either way

Scene assumption: a cardboard box and a forklift block the aisle together. The
VLM must label BOTH - the forklift as fixed, the box as movable - and the node
then selects the movable candidate closest to the robot and publishes only that
one downstream. The single-Bool contract is therefore preserved and no
behavior-tree or stabilizer change is needed.

Who answers what
----------------
The language model classifies reliably but localizes badly: asked for bounding
boxes on a warehouse frame it named the box and the forklift correctly every
time while placing both boxes tens of percent off, mostly onto empty floor. A
region that covers floor instead of the obstacle makes the downstream depth PCA
fit a plane to the floor, which points the approach pose at nothing. So the node
never asks it where anything is. Three sources each answer what they are good
at:

  * depth  - is something there, and how far. The floor is segmented out with
    RANSAC and the rest is clustered in a bird's-eye grid. Class-agnostic, so
    an obstacle nobody thought to name still gets found, which is the whole
    premise of working in an unstructured aisle. Distance is what decides which
    obstacle the robot meets first.
  * detector - what it is called, and where one cluster is really two objects
    touching. Optional, and **off by default**: the open-vocabulary detector
    tried here classified these scenes worse than the language model does, so
    the wiring stays and the service does not (see ``detector_url``). With no
    detector the node runs on depth alone and candidates are simply unnamed.
  * language model - can the arm push it. Each candidate is outlined on the
    full frame and judged on its own, with the detector's class passed in only
    as an overrulable hint. This is the judgement the project is actually
    about, and it is the one part that has to hold up on objects outside any
    fixed class list.

Every coordinate that leaves this node came from depth or the detector, never
from the language model.

Three traps this node works around, all found while bringing gemma4:e4b up:

  1. The model is a *thinking* model. With too small a token budget the whole
     budget goes into the hidden reasoning and ``response`` comes back as an
     empty string - no error, no warning. ``think`` stays on and
     ``num_predict`` stays large (see ``num_predict`` parameter).
  2. The prompt has to say that parked vehicles count as objects. Asking only
     about "objects blocking the path" makes the model report the box and treat
     the forklift as background, which destroys the contrast the demo is about.
  3. Numbering several candidates in one image and asking for a verdict per
     number is cheaper but wrong: a near object's tag is drawn inside a farther
     object's rectangle, and the model answers about the wrong one. It swapped
     the box and the forklift on one viewpoint and invented an undrawn region
     number on another. Hence one highlighted candidate per call.
"""

import base64
import json
import math
import re
import threading
import time
import urllib.error
import urllib.request

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_system_default
from rclpy.time import Time

from cv_bridge import CvBridge
import cv2

from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener, TransformException


# Split rather than formatted: the tail carries the JSON schema, and its braces
# make str.format choke on the very template it is meant to fill.
PROMPT_HEAD = """You are the perception module of a warehouse mobile robot with a small arm.
The robot is driving down an aisle and something is in its way.

Exactly one object is outlined by the magenta rectangle. Judge only that object. The
rest of the image is context - do not describe it. Parked vehicles and machinery are
objects too and must be judged, not dismissed as background.

"""

def _same_size(a, b, ratio):
    """Extent may not jump by more than ``ratio`` between the two readings.

    Shared by the tick-to-tick tracker (``VlmMovability._same_size``) and the
    round-to-round vote matcher below - same question, same threshold, two
    call sites that must not drift apart.
    """
    for key in ("width_m", "height_m"):
        big, small = max(a[key], b[key]), min(a[key], b[key])
        if big > 0.0 and (big - small) / big > ratio:
            return False
    return True


def _vote(history, k, ratio):
    """Majority vote over independent VLM verdicts for one tracked candidate.

    Each round is a fresh, independent Ollama call (see ``_query_tick``), so a
    single misfire - the forklift edge the model called "Carton" once at
    1.4-1.5 m - must not decide the candidate on its own. Below ``k`` trials
    the candidate is not yet trusted and reads as pending, the conservative
    side that keeps it off the semantic map without yet claiming FIXED.
    """
    if len(history) < k:
        return False
    return (sum(1 for v in history if v) / len(history)) >= ratio


def _pending(n_votes, k):
    """A candidate is pending - queried, but not yet trusted either way -
    until it has k independent vote rounds behind it. Same condition
    ``_vote`` uses internally, exposed so ``_update_votes`` and its tests
    share one place that decides it instead of each re-deriving ``n < k``.
    """
    return n_votes < k


def _is_judged(label, pending):
    """True once the vote gate has confirmed a verdict either way for a
    named candidate - what ``/vlm/candidates`` calls ``judged`` and what
    ``movable_obstacle_map_node`` gates on before writing to the semantic
    map. A candidate never queried (``label`` still "unknown") or short of
    k votes (``pending``) is not judged.
    """
    return label != "unknown" and not pending


def _majority_label(labels):
    """Most common label across a tracked candidate's vote rounds. A tie, or
    no rounds at all, goes to the tied label seen most recently - the same
    latest-wins rule ``movable_obstacle_map_node._merge_pair`` applies to its
    own verdict ties.
    """
    if not labels:
        return None
    counts = {}
    for lab in labels:
        counts[lab] = counts.get(lab, 0) + 1
    best = max(counts.values())
    return next(lab for lab in reversed(labels) if counts[lab] == best)


def _planar_distance(a, b):
    """Distance between two candidate-like dicts' ``distance`` (forward) and
    ``lateral`` (side) fields - the same "how far apart" question whether
    those fields hold camera-frame or vote_frame (odom) coordinates.
    """
    return ((a["distance"] - b["distance"]) ** 2 +
            (a["lateral"] - b["lateral"]) ** 2) ** 0.5


def _greedy_match(items_a, items_b, gate, size_ratio):
    """Greedy nearest-neighbour, one-to-one matching gated by planar distance
    and comparable size.

    Shared by ``_match_votes`` (vote history, one query round apart) and
    ``VlmMovability._carry_verdicts`` (tick-to-tick geometry, 0.2 s apart) -
    both are exactly this same matching problem at different cadences, so one
    function decides both instead of two loops that could drift apart.

    Pure function: no ROS, no node state. Returns ``{i: j}`` for each matched
    pair, index into ``items_a`` -> index into ``items_b``.
    """
    pairs = []
    for i, a in enumerate(items_a):
        for j, b in enumerate(items_b):
            d = _planar_distance(a, b)
            if d >= gate or not _same_size(a, b, size_ratio):
                continue
            pairs.append((d, i, j))
    pairs.sort()
    used_a, used_b = set(), set()
    matched = {}
    for _, i, j in pairs:
        if i in used_a or j in used_b:
            continue
        used_a.add(i)
        used_b.add(j)
        matched[i] = j
    return matched


def _match_votes(candidates, history, gate, size_ratio, k, ratio):
    """Match this round's candidates to the running vote history by
    position + size, accumulate one vote per matched candidate, and start a
    fresh (reset) history entry for anything that did not survive the gate -
    a lost or newly appeared candidate inherits no one else's votes.

    Pure function: no ROS, no node state, so it is unit-testable directly.
    Returns ``(new_history, voted_movable)`` aligned with ``candidates``.
    """
    matched = _greedy_match(candidates, history, gate, size_ratio)
    new_history = []
    voted = []
    for i, cand in enumerate(candidates):
        hist = history[matched[i]] if i in matched else {"votes": [], "labels": []}
        votes = hist["votes"] + [cand["raw_movable"]]
        labels = hist.get("labels", []) + [cand.get("label", "")]
        new_history.append({
            "distance": cand["distance"], "lateral": cand["lateral"],
            "width_m": cand["width_m"], "height_m": cand["height_m"],
            "votes": votes, "labels": labels,
        })
        voted.append(_vote(votes, k, ratio))
    return new_history, voted


def _vote_round(candidates, history, gate, size_ratio, k, ratio):
    """One query round's effect on the vote history, including the "no
    candidates this round" case ``_query`` itself short-circuits on (a
    segmentation miss, detector timeout, or VLM failure is a no-op: it must
    not erase votes a transient failure never got the chance to cast).
    Wraps ``_match_votes`` so the node and its tests share this one rule
    instead of each re-deriving it.
    """
    if not candidates:
        return history, []
    return _match_votes(candidates, history, gate, size_ratio, k, ratio)


def _apply_transform(xyz, translation, rotation):
    """Rotate then translate a point by a TransformStamped's translation and
    rotation quaternion ``(x, y, z, w)`` - the same math
    ``tf2_geometry_msgs.do_transform_point`` does, reimplemented without any
    ROS/TF import so round-to-round vote matching across robot motion is
    unit-testable without a live TF tree.
    """
    v = np.array(xyz, dtype=np.float64)
    qx, qy, qz, qw = rotation
    q_xyz = np.array([qx, qy, qz], dtype=np.float64)
    uv = np.cross(q_xyz, v)
    uuv = np.cross(q_xyz, uv)
    rotated = v + 2.0 * qw * uv + 2.0 * uuv
    return (rotated + np.array(translation, dtype=np.float64)).tolist()


def _advance_along(start_xy, heading, pos_xy):
    """Signed distance from ``start_xy`` to ``pos_xy`` along ``heading``."""
    return ((pos_xy[0] - start_xy[0]) * math.cos(heading)
            + (pos_xy[1] - start_xy[1]) * math.sin(heading))


def _carried_prediction(box_xy, heading, advance):
    """Where a pushed object should be once the push is over: it rode ahead
    of the robot for the whole forward run, so it moved by the robot's
    furthest advance along the heading it had when tracking was lost.
    Reversing afterwards leaves it where it is - hence the maximum, not the
    current advance."""
    return (box_xy[0] + advance * math.cos(heading),
            box_xy[1] + advance * math.sin(heading))


def _pick_reacquire(points, predicted, gate):
    """Index of the point nearest ``predicted`` within ``gate``, else None.
    ``points`` holds an (x, y) per candidate, or None for one that may not
    be picked (too large to be the pushed object)."""
    best, best_d = None, gate
    for i, p in enumerate(points):
        if p is None:
            continue
        d = math.hypot(p[0] - predicted[0], p[1] - predicted[1])
        if d <= best_d:
            best, best_d = i, d
    return best


PROMPT_TAIL = """Decide whether the robot can PUSH the outlined object aside with its small arm:
- movable = true  : light, free-standing, small enough to shove
                    (cardboard box, empty crate, small carton, cone, bin)
- movable = false : heavy machinery, vehicles, or fixed structure
                    (forklift, pallet jack, loaded pallet, rack, wall, pillar)

The label must name the outlined object itself, not whatever is most common in the
picture. If it is a vehicle or machine, say which one (for example "forklift").

Answer with ONLY a JSON object, no prose, no markdown fence:
{"label":"<short name>","movable":true|false,"reason":"<max 8 words>"}"""


class VlmMovability(Node):

    def __init__(self):
        super().__init__("vlm_movability")

        # --- topics -------------------------------------------------------
        self.rgb_topic = self.declare_parameter("rgb_topic", "/front_camera").value
        self.depth_topic = self.declare_parameter("depth_topic", "/depth_camera").value
        self.info_topic = self.declare_parameter(
            "camera_info_topic", "/depth_camera_info").value
        self.movable_topic = self.declare_parameter(
            "movable_topic", "/obstacle/is_movable").value
        self.roi_out_topic = self.declare_parameter("roi_topic", "/vlm/roi").value
        self.annotated_topic = self.declare_parameter(
            "annotated_topic", "/vlm/annotated").value
        self.judgments_topic = self.declare_parameter(
            "judgments_topic", "/vlm/judgments").value
        self.reset_topic = self.declare_parameter("reset_topic", "/vlm/reset").value

        # --- model --------------------------------------------------------
        self.ollama_url = self.declare_parameter(
            "ollama_url", "http://127.0.0.1:11434/api/generate").value
        self.model = self.declare_parameter("model", "gemma4:e4b").value
        # Trap 1: a thinking model silently returns an empty response when the
        # budget is too small. 2000 leaves room for the reasoning AND the JSON.
        self.num_predict = int(self.declare_parameter("num_predict", 2000).value)
        self.request_timeout = self.declare_parameter("request_timeout", 60.0).value
        self.max_retries = int(self.declare_parameter("max_retries", 2).value)

        # --- policy -------------------------------------------------------
        # "Call once on encounter" - the plan's latency policy. The node keeps
        # querying at query_period until it latches a movable target, then
        # holds that verdict so a push in progress is never contradicted.
        self.query_period = self.declare_parameter("query_period", 2.0).value
        self.publish_rate = self.declare_parameter("publish_rate", 5.0).value
        self.latch = self.declare_parameter("latch", True).value
        self.roi_dilate = self.declare_parameter("roi_dilate", 0.04).value
        self.min_roi_px = int(self.declare_parameter("min_roi_px", 24).value)
        self.jpeg_quality = int(self.declare_parameter("jpeg_quality", 85).value)
        self.highlight_dim = self.declare_parameter("highlight_dim", 0.45).value
        # Geometry is cheap, meaning is expensive. The verdict latches only
        # after the vote gate below confirms movable across k independent
        # rounds, but the box it is attached to is re-measured from depth
        # every tick - otherwise /vlm/roi keeps pointing at the pixels the
        # obstacle occupied when the robot was somewhere else.
        self.track_gate = self.declare_parameter("track_gate", 0.5).value
        self.track_max_miss = int(self.declare_parameter("track_max_miss", 10).value)
        # Position alone was not enough: when the robot turned and the box left
        # the frame, the forklift body sat within track_gate of the box's last
        # position and inherited its MOVABLE verdict. A carton does not become
        # a metre wide between two ticks - size is the second witness.
        self.track_size_ratio = self.declare_parameter("track_size_ratio", 0.5).value
        # Re-acquiring the pushed object (Session 10). While it is pressed
        # against the robot the camera cannot separate it from the floor, so
        # tracking drops for the whole push. Once the robot has backed off
        # reacquire_backoff metres, the pushable-sized cluster nearest to the
        # carried prediction (see _carried_prediction) within reacquire_gate
        # is asked about once more; only a MOVABLE answer re-attaches the
        # track, so a forklift fragment beside the pushed box cannot take it.
        # reacquire_retry: seconds before trying again after a FIXED answer.
        self.reacquire_gate = self.declare_parameter("reacquire_gate", 1.0).value
        self.reacquire_backoff = self.declare_parameter("reacquire_backoff", 0.5).value
        self.reacquire_retry = self.declare_parameter("reacquire_retry", 5.0).value
        # A candidate does not latch on one VLM call. It needs k independent
        # query rounds with a majority saying movable - the forklift-called-
        # "Carton" misfire was exactly one round, and one round is not a
        # majority of two. Below k trials the candidate reads pending, not
        # FIXED (see the PENDING/UNCONFIRMED overlay in _publish_annotated).
        self.movable_vote_k = int(self.declare_parameter("movable_vote_k", 2).value)
        self.movable_vote_ratio = self.declare_parameter(
            "movable_vote_ratio", 0.6).value
        # Vote rounds are matched in this frame, not the camera frame. The
        # camera frame moves with the robot, so between two query rounds
        # (query_period apart) a stationary object's cam_xyz drifts on its
        # own - at 0.26 m/s and a 2 s period that drift alone can exceed
        # track_gate. odom holds a stationary object still between rounds so
        # the position+size gate keeps meaning what it says.
        self.vote_frame = self.declare_parameter("vote_frame", "odom").value
        # Frame the candidate points are expressed in. The scene's depth header
        # names sim_camera, which is published as identity; the launch file
        # provides camera_optical with the measured mounting, so that is what
        # downstream TF lookups must use (same convention as the stabilizer).
        self.optical_frame = self.declare_parameter("optical_frame", "camera_optical").value

        # --- depth candidate extraction -----------------------------------
        self.min_range = self.declare_parameter("min_range", 0.25).value
        self.max_range = self.declare_parameter("max_range", 5.0).value
        self.depth_stride = int(self.declare_parameter("depth_stride", 4).value)
        # RANSAC floor fit: the floor is the dominant plane in a warehouse aisle.
        self.floor_iters = int(self.declare_parameter("floor_iters", 120).value)
        self.floor_tol = self.declare_parameter("floor_tol", 0.035).value
        self.obstacle_min_h = self.declare_parameter("obstacle_min_h", 0.07).value
        self.obstacle_max_h = self.declare_parameter("obstacle_max_h", 2.5).value
        # Bird's-eye clustering grid.
        # Bird's-eye cell size. This is what decides whether two obstacles
        # standing near each other survive as two candidates. At 0.10 m the
        # closing step below spans 0.30 m and welded a carton to the forklift
        # 0.29 m behind it into one blob labelled "large container, FIXED";
        # at 0.06 m the same pair separates cleanly.
        self.cluster_cell = self.declare_parameter("cluster_cell", 0.06).value
        self.cluster_min_pts = int(self.declare_parameter("cluster_min_pts", 60).value)
        self.max_candidates = int(self.declare_parameter("max_candidates", 4).value)
        # Merge only when one region essentially contains the other. That is
        # what a tall object split across bird's-eye cells looks like. Two
        # different obstacles standing near each other merely overlap, and
        # fusing them loses the contrast the whole decision rests on - a carton
        # in front of a forklift came back as one "large container, FIXED".
        self.merge_overlap = self.declare_parameter("merge_overlap", 0.75).value
        # What a small arm can actually shove. Bigger things are still detected
        # and still judged - they just cannot be chosen as the push target. A
        # palletised stack of cartons looks exactly like a loose carton to the
        # model and was selected as one; it is 1.5 m tall, and depth knows that.
        self.push_max_height = self.declare_parameter("push_max_height", 0.7).value
        self.push_max_width = self.declare_parameter("push_max_width", 1.2).value
        # Half-width of the corridor the robot actually drives through. A
        # TurtleBot3 Waffle is 0.28 m wide; anything further out to the side is
        # not in the way. Set too wide, the loaded shelving lining a warehouse
        # aisle becomes a push candidate - a rack 2 m off to the left was
        # selected as "cardboard box, MOVABLE" before this was tightened.
        self.lateral_limit = self.declare_parameter("lateral_limit", 0.9).value

        # --- open-vocabulary detector (separate HTTP service) -------------
        # Off by default. yolov8l-worldv2 was measured on these warehouse
        # renders and did not earn its place: it called both the carton and
        # the forklift "trash bin", scored 0.05-0.49, and found nothing at all
        # under some class vocabularies. Its boxes did agree closely with the
        # depth clusters, which is a useful cross-check but not new
        # information. Point detector_url at a service to switch it on once a
        # detector that classifies these scenes is available.
        self.det_url = self.declare_parameter("detector_url", "").value
        self.det_classes = self.declare_parameter("detector_classes", [
            "cardboard box", "forklift", "pallet", "wooden crate",
            "trash bin", "traffic cone", "shelving rack"]).value
        self.det_overlap = self.declare_parameter("detector_overlap", 0.35).value
        # A weak detection is worse than none: passing a wrong class name to
        # the model as a hint biases the judgement it is there to make.
        self.det_min_score = self.declare_parameter("detector_min_score", 0.35).value
        self.det_timeout = self.declare_parameter("detector_timeout", 15.0).value

        # --- state --------------------------------------------------------
        self.bridge = CvBridge()
        self.last_frame = None          # (header, bgr ndarray)
        self.last_depth = None          # metres, float32 ndarray
        self.intrinsics = None          # (fx, fy, cx, cy)
        self.frame_lock = threading.Lock()
        self.busy = False
        self.latched = None             # selected candidate dict once latched
        # Identifier for the current latch, handed to movable_obstacle_map_node
        # via /vlm/candidates so it can move that object's map entry instead of
        # merging it into a new one. time.time_ns() rather than a round counter
        # so a node restart cannot hand out a value the map node already saw.
        self.latch_track_id = None
        self.last_candidates = []
        self.last_latency = None
        self.last_query_t = 0.0
        self.track_miss = 0             # consecutive ticks with no geometric match
        # Odom anchor of the last good match of the latched target, for
        # re-acquiring it after the push: box xy, camera start xy, heading,
        # and the furthest advance since. None until the first match.
        self.carry = None
        self._reacquire_after = 0.0
        self.vote_history = []          # per-candidate independent-trial vote log
        # Bumped on every /vlm/reset. A query round started before a reset
        # can still be in flight (network call) when it lands - checked
        # against this before the round is allowed to write vote_history, so
        # a stale round cannot revive a history reset already threw away.
        self._generation = 0
        # Guards the generation check together with the vote_history/
        # last_candidates/latched writes it gates - a query round finishing
        # mid-reset must not partially apply (write votes for the old
        # generation after _on_reset already cleared them).
        self._vote_lock = threading.Lock()

        # 120 s: a query round's own TF lookup happens right after the frame
        # is split into candidates, well before the slow VLM round trip, so
        # the cache only needs to outlive query_period plus one round - the
        # long window is headroom, not a requirement.
        self.tf_buffer = Buffer(cache_time=Duration(seconds=120.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- ROS I/O ------------------------------------------------------
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Image, self.rgb_topic, self._on_rgb, sensor_qos)
        self.create_subscription(Image, self.depth_topic, self._on_depth, sensor_qos)
        self.create_subscription(CameraInfo, self.info_topic, self._on_info, sensor_qos)
        self.create_subscription(Bool, self.reset_topic, self._on_reset, 10)

        self.movable_pub = self.create_publisher(
            Bool, self.movable_topic, qos_profile_system_default)
        self.roi_pub = self.create_publisher(
            RegionOfInterest, self.roi_out_topic, qos_profile_system_default)
        self.annotated_pub = self.create_publisher(
            Image, self.annotated_topic, sensor_qos)
        self.judgments_pub = self.create_publisher(
            String, self.judgments_topic, qos_profile_system_default)

        # Every candidate with its verdict and a 3D point, every tick - the
        # feed for anything that wants obstacles on the map rather than in
        # the picture (movable_obstacle_map_node).
        self.candidates_pub = self.create_publisher(String, "/vlm/candidates", 10)
        self.create_timer(1.0 / max(1.0, self.publish_rate), self._publish_tick)
        self.create_timer(0.25, self._query_tick)

        self.get_logger().info(
            "vlm_movability: rgb='%s' model='%s' -> '%s' / '%s' "
            "(query %.1fs, publish %.1f Hz, latch=%s, vote k=%d ratio=%.2f frame=%s)" % (
                self.rgb_topic, self.model, self.movable_topic,
                self.roi_out_topic, self.query_period, self.publish_rate,
                self.latch, self.movable_vote_k, self.movable_vote_ratio,
                self.vote_frame))

    # ------------------------------------------------------------------
    # subscriptions

    def _on_rgb(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:                      # noqa: BLE001 - log and drop
            self.get_logger().warn("rgb convert failed: %s" % exc)
            return
        with self.frame_lock:
            self.last_frame = (msg.header, bgr)

    def _on_depth(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:                      # noqa: BLE001 - log and drop
            self.get_logger().warn("depth convert failed: %s" % exc)
            return
        depth = np.asarray(depth)
        if depth.dtype == np.uint16:                  # 16UC1 is millimetres
            depth = depth.astype(np.float32) * 0.001
        else:
            depth = depth.astype(np.float32)
        with self.frame_lock:
            self.last_depth = depth

    def _on_info(self, msg):
        if self.intrinsics is None:
            k = msg.k
            self.intrinsics = (k[0], k[4], k[2], k[5])
            self.get_logger().info(
                "vlm_movability: intrinsics fx=%.1f fy=%.1f cx=%.1f cy=%.1f" %
                self.intrinsics)

    def _on_reset(self, msg):
        if msg.data:
            with self._vote_lock:
                self.latched = None
                self.latch_track_id = None
                self.last_candidates = []
                self.track_miss = 0
                self.vote_history = []
                self._generation += 1
            self.get_logger().info("vlm_movability: verdict reset")

    # ------------------------------------------------------------------
    # querying

    def _query_tick(self):
        if self.busy:
            return
        if self.latched is not None and self.latch:
            return
        if (time.time() - self.last_query_t) < self.query_period:
            return
        with self.frame_lock:
            if self.last_frame is None or self.last_depth is None:
                return
            if self.intrinsics is None:
                return
            header, bgr = self.last_frame
            bgr = bgr.copy()
            depth = self.last_depth.copy()
        self.busy = True
        self.last_query_t = time.time()
        generation = self._generation
        threading.Thread(target=self._query, args=(header, bgr, depth, generation),
                         daemon=True).start()

    def _query(self, header, bgr, depth, generation):
        # Any exception escaping a round would leave busy=True and stop the
        # node querying for good, with nothing but a thread traceback to show.
        try:
            self._query_round(header, bgr, depth, generation)
        finally:
            self.busy = False

    def _query_round(self, header, bgr, depth, generation):
        try:
            candidates = self._depth_candidates(depth, bgr.shape[1], bgr.shape[0])
            candidates = self._fuse_detections(
                candidates, self._detect(bgr), depth, bgr.shape[1], bgr.shape[0])
        except Exception as exc:                      # noqa: BLE001 - keep node alive
            self.get_logger().warn("obstacle segmentation failed: %s" % exc)
            self.busy = False
            return

        if not candidates:
            # A failed round - no candidates here, a segmentation exception
            # above, or a VLM exception below - is a no-op: it replaces
            # neither last_candidates nor vote_history. Only a round that
            # produced a real judgment updates state, so a transient miss
            # never erases votes it never got the chance to cast. One rule
            # for all three failure kinds instead of one each.
            self.busy = False
            return

        # Snapshot the vote_frame <- optical_frame transform for this round's
        # stamp right here, before _ask_candidates - the slow VLM round trip
        # (measured 32 s wall clock, ~7 s sim time at RTF 0.22). Looking the
        # TF up afterwards risked the stamp falling out of the tf2 cache
        # before the round ever needed it, silently skipping every
        # candidate's vote that round (Review 2차 #1).
        tf = self._snapshot_vote_transform(header)

        try:
            latency, raw = self._ask_candidates(bgr, candidates)
        except Exception as exc:                      # noqa: BLE001 - keep node alive
            self.get_logger().warn("vlm query failed: %s" % exc)
            self.busy = False
            return

        with self._vote_lock:
            if generation != self._generation:
                # /vlm/reset fired while this round's network calls were in
                # flight. The round belongs to a vote history that no longer
                # exists - drop it rather than reviving old votes.
                self.get_logger().info(
                    "vlm_movability: round discarded (generation %d != current "
                    "%d) - /vlm/reset fired mid-query" % (generation, self._generation))
                self.busy = False
                return

            self._update_votes(candidates, tf)
            self.last_candidates = candidates
            self.last_latency = latency
            target = self._select(candidates)
            if target is not None:
                # Id first: _publish_tick reads both without the lock, and a
                # latched target must never go out paired with a stale id.
                self.latch_track_id = time.time_ns()
                self.latched = target

        if target is not None:
            self.get_logger().info(
                "vlm_movability: MOVABLE '%s' at %.2f m selected (%.1fs) - %s "
                "[votes %d/%d]" % (
                    target["label"], target["distance"], latency, target["reason"],
                    target["votes"]["n_movable"], target["votes"]["n"]))
        else:
            def _state(c):
                if c.get("pending"):
                    return "pending"
                return "movable" if c["movable"] else "fixed"
            seen = ["%s@%.1fm(raw=%s,votes=%d/%d,%s)" % (
                        c["label"], c["distance"],
                        "movable" if c["raw_movable"] else "fixed",
                        c["votes"]["n_movable"], c["votes"]["n"], _state(c))
                    for c in candidates]
            self.get_logger().info(
                "vlm_movability: no movable target (%.1fs); saw %s" % (
                    latency, ", ".join(seen)))

        self.judgments_pub.publish(String(data=json.dumps({
            "stamp": {"sec": header.stamp.sec, "nanosec": header.stamp.nanosec},
            "model": self.model,
            "latency_s": round(latency, 3),
            "candidates": [{k: v for k, v in c.items() if k != "mask"}
                           for c in candidates],
            "selected_id": target["id"] if target else None,
            "raw_len": len(raw),
        })))
        self._publish_annotated(header, bgr, candidates, self.latched)
        self.busy = False

    def _snapshot_vote_transform(self, header):
        """Look up vote_frame <- optical_frame at this round's own stamp.

        Called right after the frame is split into candidates, not after the
        VLM call - see the comment at the call site in ``_query``. A short
        timeout (0.2 s) is enough because the TF is normally already in the
        buffer by the time a query round starts; no fallback to the latest
        TF, ever - matching this round's candidates with a stale transform
        could silently mismatch objects, so a lookup failure skips the vote
        for this round rather than guessing.
        """
        if header.stamp.sec == 0 and header.stamp.nanosec == 0:
            self.get_logger().warn(
                "vlm_movability: zero header stamp on query frame; vote round skipped",
                throttle_duration_sec=5.0)
            return None
        cam_frame = self.optical_frame
        try:
            return self.tf_buffer.lookup_transform(
                self.vote_frame, cam_frame, Time.from_msg(header.stamp),
                # Wall-clock wait: 0.2 s was only ~0.04 s of sim time at RTF
                # 0.22. This runs on the query thread, so waiting costs nothing.
                timeout=Duration(seconds=2.0))
        except TransformException as exc:
            self.get_logger().warn(
                "vlm_movability: no TF %s <- %s at query stamp (%s); vote round skipped"
                % (self.vote_frame, cam_frame, exc), throttle_duration_sec=5.0)
            return None

    # ------------------------------------------------------------------
    # geometry: depth -> candidate obstacles

    def _depth_candidates(self, depth, rgb_w, rgb_h):
        """Segment the floor out and cluster what is left into obstacles.

        Returns candidates with pixel bboxes in RGB coordinates and a metric
        distance, so nothing downstream depends on the model guessing where
        things are.
        """
        fx, fy, cx, cy = self.intrinsics
        step = max(1, self.depth_stride)
        h, w = depth.shape[:2]
        vs, us = np.mgrid[0:h:step, 0:w:step]
        z = depth[0:h:step, 0:w:step]

        valid = np.isfinite(z) & (z > self.min_range) & (z < self.max_range)
        if valid.sum() < 500:
            return []
        z = z[valid].astype(np.float32)
        us = us[valid].astype(np.float32)
        vs = vs[valid].astype(np.float32)
        # optical frame: x right, y down, z forward
        x = (us - cx) * z / fx
        y = (vs - cy) * z / fy
        pts = np.stack([x, y, z], axis=1)

        plane = self._fit_floor(pts)
        if plane is None:
            return []
        normal, offset = plane
        height = pts @ normal + offset          # signed distance above floor

        obstacle = (height > self.obstacle_min_h) & (height < self.obstacle_max_h)
        obstacle &= np.abs(x) < self.lateral_limit
        if obstacle.sum() < self.cluster_min_pts:
            return []

        ox, oz = x[obstacle], z[obstacle]
        oy = y[obstacle]
        ou, ov = us[obstacle], vs[obstacle]
        oh = height[obstacle]

        cell = self.cluster_cell
        gx = np.floor((ox + self.lateral_limit) / cell).astype(np.int32)
        gz = np.floor((oz - self.min_range) / cell).astype(np.int32)
        gw = int(np.ceil(2 * self.lateral_limit / cell)) + 1
        gh = int(np.ceil((self.max_range - self.min_range) / cell)) + 1
        gx = np.clip(gx, 0, gw - 1)
        gz = np.clip(gz, 0, gh - 1)

        grid = np.zeros((gh, gw), dtype=np.uint8)
        grid[gz, gx] = 255
        # close 1-cell gaps so one object does not split into two candidates
        grid = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        count, labels = cv2.connectedComponents(grid, connectivity=8)

        point_label = labels[gz, gx]
        sx = rgb_w / float(w)
        sy = rgb_h / float(h)

        candidates = []
        for lab in range(1, count):
            sel = point_label == lab
            n = int(sel.sum())
            if n < self.cluster_min_pts:
                continue
            u0, u1 = float(ou[sel].min()), float(ou[sel].max())
            v0, v1 = float(ov[sel].min()), float(ov[sel].max())
            bbox = [int(u0 * sx), int(v0 * sy),
                    int(np.ceil(u1 * sx)), int(np.ceil(v1 * sy))]
            if (bbox[2] - bbox[0]) < self.min_roi_px or \
               (bbox[3] - bbox[1]) < self.min_roi_px:
                continue
            candidates.append({
                "id": 0,                       # assigned after sorting
                "bbox_px": bbox,
                "distance": float(np.percentile(oz[sel], 10)),
                "lateral": float(np.median(ox[sel])),
                # Optical-frame point for the object's footprint centre: front
                # face at distance, half the measured width further along the
                # ray (depth of the object is not observable from one view),
                # on the floor plane. Consumers transform this with TF.
                "cam_xyz": [float(np.median(ox[sel])),
                            float(np.median(oy[sel])),
                            float(np.percentile(oz[sel], 10))],
                # Measured extent of the visible part. Size is most of what
                # decides whether a small arm can shove it, so _pushable gates
                # on it. It is not given to the model: a partial view (the
                # forklift's 0.22 m front) pulled its verdict wrong.
                "height_m": float(np.percentile(oh[sel], 98)),
                "width_m": float(ox[sel].max() - ox[sel].min()),
                "points": n,
                "label": "unknown",
                "movable": False,
                "raw_movable": False,   # this round's unvoted VLM verdict
                "votes": {"n": 0, "n_movable": 0},
                "pending": False,       # True once queried but short of k votes
                "reason": "",
                "det_label": None,
                "det_score": 0.0,
            })

        candidates = self._merge_overlapping(candidates)
        candidates.sort(key=lambda c: c["distance"])
        return candidates

    def _fuse_detections(self, candidates, detections, depth, rgb_w, rgb_h):
        """Let the detector name and, where needed, split the depth clusters.

        Depth knows how far away a thing is and that it is there at all, which
        is what picking a push target needs. It cannot tell a forklift from a
        crate, and it merges two obstacles that touch into one cluster. An
        open-vocabulary detector covers exactly those two gaps, so its boxes
        refine the clusters rather than replace them: every candidate that
        survives still carries a distance measured from depth.
        """
        out = []
        for cand in candidates:
            hits = [d for d in detections
                    if self._overlap_ratio(cand["bbox_px"], d["bbox"]) >= self.det_overlap]
            if len(hits) >= 2:
                for det in hits:
                    box = self._intersect(cand["bbox_px"], det["bbox"])
                    if box is None:
                        continue
                    part = dict(cand)
                    part["bbox_px"] = box
                    part["det_label"] = det["label"]
                    part["det_score"] = det["score"]
                    dist = self._box_distance(depth, box, rgb_w, rgb_h)
                    part["distance"] = dist if dist is not None else cand["distance"]
                    out.append(part)
            else:
                if hits:
                    cand["det_label"] = hits[0]["label"]
                    cand["det_score"] = hits[0]["score"]
                out.append(cand)

        out.sort(key=lambda c: c["distance"])
        out = out[:self.max_candidates]
        for i, cand in enumerate(out, start=1):
            cand["id"] = i
        return out

    @staticmethod
    def _intersect(a, b):
        x0, y0 = max(a[0], b[0]), max(a[1], b[1])
        x1, y1 = min(a[2], b[2]), min(a[3], b[3])
        if x1 <= x0 or y1 <= y0:
            return None
        return [x0, y0, x1, y1]

    def _box_distance(self, depth, bbox, rgb_w, rgb_h):
        """Nearest surface inside a pixel box, in metres."""
        dh, dw = depth.shape[:2]
        x0 = int(bbox[0] * dw / rgb_w)
        x1 = int(bbox[2] * dw / rgb_w)
        y0 = int(bbox[1] * dh / rgb_h)
        y1 = int(bbox[3] * dh / rgb_h)
        patch = depth[max(0, y0):max(1, y1), max(0, x0):max(1, x1)]
        good = patch[np.isfinite(patch) & (patch > self.min_range) &
                     (patch < self.max_range)]
        if good.size < 20:
            return None
        return float(np.percentile(good, 10))

    def _detect(self, bgr):
        """Ask the detector service. A failure here is not fatal - depth alone
        still yields candidates, just unnamed."""
        if not self.det_url:
            return []
        ok, buf = cv2.imencode(".jpg", bgr,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return []
        body = json.dumps({
            "image": base64.b64encode(buf.tobytes()).decode(),
            "classes": list(self.det_classes),
        }).encode()
        req = urllib.request.Request(
            self.det_url, data=body, headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=self.det_timeout) as resp:
                found = json.loads(resp.read()).get("detections", [])
            return [d for d in found if d.get("score", 0.0) >= self.det_min_score]
        except Exception as exc:                  # noqa: BLE001 - degrade, stay up
            self.get_logger().warn("detector unavailable (%s); depth only" % exc)
            return []

    def _merge_overlapping(self, candidates):
        """Fold clusters that are one object seen at several depths.

        A forklift is tall: its body, mast and seat land in different
        bird's-eye cells and come back as three candidates stacked on the same
        patch of image. Three boxes over one vehicle clutter the overlay and
        waste the model's attention, so candidates that cover the same pixels
        are merged into their union.
        """
        merged = []
        for cand in sorted(candidates, key=lambda c: -self._area(c["bbox_px"])):
            for other in merged:
                if self._overlap_ratio(cand["bbox_px"], other["bbox_px"]) < \
                        self.merge_overlap:
                    continue
                ax0, ay0, ax1, ay1 = other["bbox_px"]
                bx0, by0, bx1, by1 = cand["bbox_px"]
                other["bbox_px"] = [min(ax0, bx0), min(ay0, by0),
                                    max(ax1, bx1), max(ay1, by1)]
                other["distance"] = min(other["distance"], cand["distance"])
                other["height_m"] = max(other["height_m"], cand["height_m"])
                other["width_m"] = max(other["width_m"], cand["width_m"])
                other["points"] += cand["points"]
                break
            else:
                merged.append(cand)
        return merged

    @staticmethod
    def _area(bbox):
        return max(0, bbox[2] - bbox[0]) * max(0, bbox[3] - bbox[1])

    @classmethod
    def _overlap_ratio(cls, a, b):
        """Intersection over the area of the smaller box."""
        ix0, iy0 = max(a[0], b[0]), max(a[1], b[1])
        ix1, iy1 = min(a[2], b[2]), min(a[3], b[3])
        inter = max(0, ix1 - ix0) * max(0, iy1 - iy0)
        smaller = min(cls._area(a), cls._area(b))
        return inter / smaller if smaller > 0 else 0.0

    def _fit_floor(self, pts):
        """RANSAC the dominant plane. In an aisle that is the floor."""
        rng = np.random.default_rng(0)
        n = pts.shape[0]
        best_inliers, best = -1, None
        for _ in range(self.floor_iters):
            idx = rng.choice(n, 3, replace=False)
            p0, p1, p2 = pts[idx]
            normal = np.cross(p1 - p0, p2 - p0)
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                continue
            normal = normal / norm
            # floor normal points up, i.e. against +y in the optical frame
            if normal[1] > 0:
                normal = -normal
            if normal[1] > -0.85:            # reject walls and box faces
                continue
            offset = -float(normal @ p0)
            inliers = int((np.abs(pts @ normal + offset) < self.floor_tol).sum())
            if inliers > best_inliers:
                best_inliers, best = inliers, (normal, offset)
        if best is None or best_inliers < 0.15 * n:
            return None
        return best

    def _select(self, candidates):
        """Pick the movable candidate the robot meets first.

        Both a box and a forklift block the aisle, so "is anything movable" is
        not enough - the node has to say WHICH one to push. Distance comes from
        depth, so nearest is a real measurement rather than an image heuristic.
        """
        # raw_movable too: accumulated votes must not overrule this round's
        # own FIXED call - t7 latched a forklift that way (the fragment
        # inherited 23 movable votes while the model said "large, heavy
        # vehicle"). Refusing to push is the safe side of a disagreement.
        movable = [c for c in candidates
                   if c["movable"] and c.get("raw_movable") and self._pushable(c)]
        if not movable:
            return None
        return min(movable, key=lambda c: (c["distance"], abs(c["lateral"])))

    def _pushable(self, cand):
        """Small enough for the arm, by measurement rather than appearance."""
        return (cand["height_m"] <= self.push_max_height
                and cand["width_m"] <= self.push_max_width)

    def _update_votes(self, candidates, tf):
        """Turn this round's raw verdicts into the vote-gated ``movable``
        every downstream consumer reads - ``_select`` (so only a candidate
        with a real majority can latch) and ``/vlm/candidates`` (so a single
        misfire never reaches the semantic map, whether or not it was ever
        the selected target). Also fills ``votes`` (n, n_movable), ``label``
        (the majority label across vote rounds) and ``pending`` (queried but
        short of k votes) on every candidate.

        ``tf`` is the vote_frame <- optical_frame transform for this round's
        own stamp, already looked up in ``_snapshot_vote_transform`` before
        the VLM call - ``None`` means that lookup failed or was skipped
        (already logged there), so this round casts no votes.

        Matching happens in ``vote_frame`` (odom by default), not the camera
        frame the candidates were measured in: the robot moves between query
        rounds, so a stationary object's camera-frame position drifts on its
        own and can exceed track_gate with nothing having actually moved.
        odom holds it still.
        """
        if tf is None:
            for cand in candidates:
                cand["movable"] = False
                cand["pending"] = True
                cand["votes"] = {"n": 0, "n_movable": 0}
                # Distinguishes "no TF this round" from a brand-new object.
                cand["vote_skipped"] = True
            return

        t, r = tf.transform.translation, tf.transform.rotation
        odom_candidates = []
        for cand in candidates:
            ox, oy, _ = _apply_transform(
                cand["cam_xyz"], (t.x, t.y, t.z), (r.x, r.y, r.z, r.w))
            odom_candidates.append({**cand, "distance": ox, "lateral": oy})

        self.vote_history, voted = _match_votes(
            odom_candidates, self.vote_history, self.track_gate,
            self.track_size_ratio, self.movable_vote_k, self.movable_vote_ratio)
        for cand, hist, movable in zip(candidates, self.vote_history, voted):
            cand["movable"] = movable
            cand["label"] = _majority_label(hist["labels"]) or cand["label"]
            cand["votes"] = {"n": len(hist["votes"]), "n_movable": sum(hist["votes"])}
            cand["pending"] = _pending(len(hist["votes"]), self.movable_vote_k)

    # ------------------------------------------------------------------
    # meaning: one marked-up query to the model

    def _ask_candidates(self, bgr, candidates):
        """Ask about one outlined object at a time.

        Numbering every candidate in a single image and asking for a verdict
        per number saves calls but is not reliable here: the tag of a nearer
        object lands inside a farther object's rectangle, and the model then
        answers about the wrong one - in testing it swapped the box and the
        forklift outright, and once invented a region number that was never
        drawn. One highlighted object per call removes the association step
        entirely, and at well under a second per call the extra round trips
        cost nothing that matters.
        """
        total = 0.0
        raw_all = []
        for cand in candidates:
            latency, raw = self._ask_one(bgr, cand)
            total += latency
            raw_all.append(raw)
        return total, "".join(raw_all)

    def _ask_one(self, bgr, cand):
        # The detector's class name goes in as a hint, not as the answer: the
        # movable/fixed call stays with the model, which is the part of this
        # pipeline that has to generalise beyond a fixed class list.
        #
        # No measured-size hint. Depth measures only the visible slice of an
        # object - the forklift reads 0.22 m wide - and the model followed the
        # numbers over the image ("that size is a carton"). Offline, same 10
        # frames: with the hint box 8/10, forklift 7/10 correct; without it
        # 10/10 and 10/10. Size still decides pushability, in _pushable.
        hint = ""
        det_label = cand.get("det_label")
        if det_label:
            hint += ("A separate detector labelled it \"%s\". Treat that as a hint you may\n"
                     "overrule if the image disagrees.\n\n" % det_label)
        prompt = PROMPT_HEAD + hint + PROMPT_TAIL
        highlighted = self._draw_highlight(bgr, cand["bbox_px"])
        ok, buf = cv2.imencode(".jpg", highlighted,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            raise RuntimeError("jpeg encode failed")
        b64 = base64.b64encode(buf.tobytes()).decode()

        budget = self.num_predict
        last_raw = ""
        for attempt in range(self.max_retries + 1):
            body = json.dumps({
                "model": self.model,
                "prompt": prompt,
                "images": [b64],
                "stream": False,
                # Trap 1: keep thinking ON and the budget large.
                "think": True,
                "options": {"temperature": 0.0, "num_predict": budget},
            }).encode()
            req = urllib.request.Request(
                self.ollama_url, data=body,
                headers={"Content-Type": "application/json"})
            t0 = time.time()
            with urllib.request.urlopen(req, timeout=self.request_timeout) as resp:
                out = json.loads(resp.read())
            latency = time.time() - t0
            last_raw = out.get("response") or ""

            verdict = self._parse(last_raw)
            if verdict is not None:
                cand["label"] = verdict["label"]
                # This round's raw call only - the vote gate in _query turns
                # it (and prior rounds for the same tracked object) into the
                # "movable" the rest of the node reads.
                cand["raw_movable"] = verdict["movable"]
                cand["reason"] = verdict["reason"]
                return latency, last_raw

            # Empty response or truncated JSON: both mean the budget ran out.
            budget = int(budget * 1.5)
            self.get_logger().warn(
                "vlm_movability: unparsable response (attempt %d, %d chars); "
                "retrying with num_predict=%d" % (attempt + 1, len(last_raw), budget))

        raise RuntimeError("no parsable verdict after %d attempts" % (self.max_retries + 1))

    def _draw_highlight(self, bgr, bbox):
        """Outline one object in magenta and dim everything around it.

        Magenta because nothing in the aisle wears it - a yellow outline was
        the first attempt and it vanished into the yellow forklift. The dimming
        matters just as much: when two obstacles stand close together their
        rectangles overlap, and with the surroundings at full brightness the
        model answered about the neighbour that happened to poke into the
        rectangle. Context stays readable, the subject is unambiguous.
        """
        x0, y0, x1, y1 = bbox
        img = (bgr * self.highlight_dim).astype(np.uint8)
        img[y0:y1, x0:x1] = bgr[y0:y1, x0:x1]
        cv2.rectangle(img, (x0, y0), (x1, y1), (255, 0, 255), 5)
        return img

    @staticmethod
    def _parse(text):
        if not text:
            return None
        match = re.search(r"\{.*\}", text, re.S)
        if not match:
            return None
        try:
            data = json.loads(match.group(0))
        except json.JSONDecodeError:
            return None
        if "movable" not in data:
            return None
        return {
            "label": str(data.get("label", "object")),
            "movable": bool(data.get("movable")),
            "reason": str(data.get("reason", "")),
        }

    # ------------------------------------------------------------------
    # publishing

    def _carry_verdicts(self, fresh, prior):
        """Move the model's verdicts onto freshly measured clusters.

        Matching is nearest-neighbour in the camera frame and one-to-one (via
        the shared ``_greedy_match``): at 5 Hz an obstacle shifts centimetres
        between ticks, so a gate of track_gate metres is loose enough to
        follow it and tight enough that a MOVABLE verdict cannot slide onto
        the forklift standing behind it - which is the one mistake here that
        would drive the robot into a vehicle. A cluster with no partner keeps
        no verdict.
        """
        matched = _greedy_match(fresh, prior, self.track_gate, self.track_size_ratio)
        for i, j in matched.items():
            for key in ("id", "label", "movable", "reason", "det_label", "det_score",
                       "votes", "pending"):
                fresh[i][key] = prior[j][key]
        for i, cand in enumerate(fresh):
            if i not in matched:
                cand["id"] = -1 - i          # never collides with a fused id
        return fresh

    def _same_size(self, a, b):
        """Extent may not jump by more than track_size_ratio between ticks.

        Thin delegate to the module-level gate - kept as a method for any
        external caller that reaches for VlmMovability()._same_size.
        """
        return _same_size(a, b, self.track_size_ratio)

    def _publish_tick(self):
        target = self.latched
        self.movable_pub.publish(Bool(data=target is not None))
        if target is None:
            return
        with self.frame_lock:
            if self.last_frame is None or self.last_depth is None \
                    or self.intrinsics is None:
                return
            header, bgr = self.last_frame
            depth = self.last_depth.copy()
        height, width = bgr.shape[:2]

        # Re-measure the scene. The verdict is latched because asking the model
        # again is expensive; the geometry is not, and freezing it meant
        # /vlm/roi kept naming the pixels the obstacle filled when the robot
        # was standing somewhere else - the stabilizer then estimated the
        # obstacle pose from whatever had since moved into that rectangle.
        try:
            fresh = self._carry_verdicts(
                self._depth_candidates(depth, width, height),
                self.last_candidates)
        except Exception as exc:                  # noqa: BLE001 - keep node alive
            self.get_logger().warn("retrack failed: %s" % exc)
            fresh = []

        matched = next((c for c in fresh if c["id"] == target["id"]), None)
        # The push target must still be pushable. If the matched cluster has
        # grown past the arm's limits it is not the carton any more, whatever
        # the matcher says - keep the last trustworthy geometry and count a miss.
        if matched is not None and not self._pushable(matched):
            matched = None
        if matched is not None:
            self.latched = target = matched
            self.last_candidates = fresh
            self.track_miss = 0
            self._anchor_carry(matched)
        else:
            self.track_miss += 1
            if self.track_miss == self.track_max_miss:
                self.get_logger().warn(
                    "vlm_movability: lost '%s' for %d ticks - ROI held back"
                    % (target["label"], self.track_miss))
            self._try_reacquire(bgr, fresh, target)

        # Keep drawing the verdict on the live frame. Publishing it only at the
        # end of a query meant exactly one overlay image existed for the whole
        # run, because latching stops the queries - fine for a log, useless for
        # watching or recording what the robot decided.
        if self.last_candidates:
            self._publish_annotated(header, bgr, self.last_candidates, target,
                                    self.track_miss > 0)
        self._publish_candidates(header, self.last_candidates, target)
        # Past the miss budget the rectangle is a guess, and a guess is worse
        # than silence for anything steering off it.
        if self.track_miss >= self.track_max_miss:
            return
        roi = self._to_roi(target["bbox_px"], width, height)
        if roi is not None:
            self.roi_pub.publish(roi)

    def _cam_pose(self):
        """(translation, rotation, x, y, heading) of the camera in vote_frame,
        latest TF; None if unavailable. Heading is the optical z axis."""
        try:
            tf = self.tf_buffer.lookup_transform(self.vote_frame, self.optical_frame, Time())
        except TransformException:
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        t, r = (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)
        ahead = _apply_transform([0.0, 0.0, 1.0], t, r)
        return t, r, t[0], t[1], math.atan2(ahead[1] - t[1], ahead[0] - t[0])

    def _anchor_carry(self, matched):
        pose = self._cam_pose()
        if pose is None:
            return
        t, r, cx, cy, heading = pose
        bx, by, _ = _apply_transform(matched["cam_xyz"], t, r)
        self.carry = {"box": (bx, by), "start": (cx, cy), "heading": heading, "advance": 0.0}

    def _try_reacquire(self, bgr, fresh, target):
        """See reacquire_* in __init__. Runs every lost tick to keep the
        furthest advance current; fires one VLM check at most at a time."""
        if self.carry is None:
            return
        pose = self._cam_pose()
        if pose is None:
            return
        t, r, cx, cy, _ = pose
        c = self.carry
        adv = _advance_along(c["start"], c["heading"], (cx, cy))
        c["advance"] = max(c["advance"], adv)
        if (self.track_miss < self.track_max_miss or self.busy
                or time.time() < self._reacquire_after
                or c["advance"] - adv < self.reacquire_backoff):
            return
        predicted = _carried_prediction(c["box"], c["heading"], c["advance"])
        points = [tuple(_apply_transform(f["cam_xyz"], t, r)[:2]) if "cam_xyz" in f and self._pushable(f) else None
                  for f in fresh]
        i = _pick_reacquire(points, predicted, self.reacquire_gate)
        if i is None:
            return
        self.busy = True
        dist = math.hypot(points[i][0] - predicted[0], points[i][1] - predicted[1])
        threading.Thread(target=self._reacquire, args=(bgr.copy(), fresh, i, target, dist),
                         daemon=True).start()

    def _reacquire(self, bgr, fresh, i, target, dist):
        try:
            cand = fresh[i]
            try:
                self._ask_one(bgr, cand)
            except Exception as exc:              # noqa: BLE001 - keep node alive
                self.get_logger().warn("vlm_movability: reacquire query failed: %s" % exc)
                self._reacquire_after = time.time() + self.reacquire_retry
                return
            if not cand.get("raw_movable"):
                self.get_logger().info(
                    "vlm_movability: reacquire candidate %.2f m from prediction judged "
                    "fixed ('%s') - ignored" % (dist, cand.get("label")))
                self._reacquire_after = time.time() + self.reacquire_retry
                return
            with self._vote_lock:
                if self.latched is not target:
                    return                        # reset or re-matched meanwhile
                for key in ("id", "movable", "votes", "pending"):
                    cand[key] = target[key]
                self.latched = cand
                self.last_candidates = fresh
                self.track_miss = 0
            self.get_logger().info(
                "vlm_movability: reacquired '%s' %.2f m from the carried prediction "
                "(pushed %.2f m) - %s" % (cand["label"], dist, self.carry["advance"],
                                          cand.get("reason", "")))
        finally:
            self.busy = False

    def _publish_candidates(self, header, candidates, target):
        """Everything measured and judged this tick, as one JSON message."""
        out = []
        for c in candidates:
            if "cam_xyz" not in c:
                continue
            pending = bool(c.get("pending", False))
            selected = target is not None and c["id"] == target["id"]
            out.append({
                "id": c["id"], "label": c["label"], "movable": bool(c["movable"]),
                "pushable": bool(c["movable"] and self._pushable(c)),
                # judged only once the vote gate has confirmed a verdict
                # either way (votes.n >= movable_vote_k); a candidate short
                # of k votes is pending, not judged - keeps a single
                # misfire off the semantic map (movable_obstacle_map_node
                # skips unjudged candidates by default).
                "judged": _is_judged(c["label"], pending),
                "pending": pending,
                "votes": c.get("votes", {"n": 0, "n_movable": 0}),
                "selected": selected,
                # Set only on the latched candidate, so movable_obstacle_map_node
                # can move that map entry instead of averaging it into a new one.
                # None while nothing is latched or for every other candidate.
                "track_id": self.latch_track_id if selected else None,
                "cam_xyz": c["cam_xyz"], "distance": c["distance"],
                "width_m": c["width_m"], "height_m": c["height_m"],
                "points": c["points"], "reason": c.get("reason", ""),
            })
        # While the target is lost this re-sends the last measured snapshot
        # under the current stamp: cam_xyz is old, the stamp is new, so a
        # consumer projecting it through current TF drags ghosts along with
        # the robot. stale_ticks > 0 marks exactly that case, and
        # movable_obstacle_map_node drops such messages whole.
        self.candidates_pub.publish(String(data=json.dumps({
            "stamp": header.stamp.sec + header.stamp.nanosec * 1e-9,
            "frame_id": self.optical_frame,
            "stale_ticks": self.track_miss,
            "candidates": out,
        })))

    def _to_roi(self, bbox_px, width, height):
        x0, y0, x1, y1 = bbox_px
        pad_x = self.roi_dilate * (x1 - x0)
        pad_y = self.roi_dilate * (y1 - y0)
        x0 = int(round(max(0, x0 - pad_x)))
        x1 = int(round(min(width, x1 + pad_x)))
        y0 = int(round(max(0, y0 - pad_y)))
        y1 = int(round(min(height, y1 + pad_y)))
        if (x1 - x0) < self.min_roi_px or (y1 - y0) < self.min_roi_px:
            return None
        roi = RegionOfInterest()
        roi.x_offset = x0
        roi.y_offset = y0
        roi.width = x1 - x0
        roi.height = y1 - y0
        roi.do_rectify = False
        return roi

    def _publish_annotated(self, header, bgr, candidates, selected, stale=False):
        img = bgr.copy()
        height = img.shape[0]
        for cand in candidates:
            x0, y0, x1, y1 = cand["bbox_px"]
            chosen = selected is not None and cand["id"] == selected["id"]
            # Four states, not two. "The model called it movable" and "the
            # robot will push it" are different claims, and drawing both in
            # green reads as the robot planning to shove a loaded pallet.
            # "Queried once" and "confirmed by the vote gate" are different
            # claims too - a single misfire must not paint FIXED or MOVABLE.
            if cand["label"] == "unknown":
                # Never asked the model about this one. Drawing it red would
                # claim a FIXED verdict that was never given.
                colour, verdict = (150, 150, 150), "unjudged"
            elif cand.get("pending"):
                votes = cand.get("votes", {"n": 0})
                # Once latched, querying stops (see _query_tick) - a
                # still-pending candidate other than the latched target will
                # never get another vote round, so it is stuck short of k
                # forever rather than merely waiting for the next round.
                # "PENDING" implies motion; say so with a different word.
                label = "UNCONFIRMED" if self.latched is not None else "PENDING"
                colour, verdict = (0, 191, 255), "%s %d/%d" % (
                    label, votes["n"], self.movable_vote_k)
            elif not cand["movable"]:
                colour, verdict = (60, 60, 230), "FIXED"
            elif self._pushable(cand):
                colour, verdict = (80, 220, 80), "MOVABLE"
            else:
                colour, verdict = (40, 190, 250), "TOO LARGE"
            cv2.rectangle(img, (x0, y0), (x1, y1), colour, 4 if chosen else 2)
            tag = "%s: %s  %.1fm  %.2fx%.2fm" % (
                cand["label"], verdict, cand["distance"],
                cand["width_m"], cand["height_m"])
            if chosen:
                tag += "  -> PUSH"
            (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            ty = max(th + 8, y0)
            cv2.rectangle(img, (x0, ty - th - 8), (x0 + tw + 10, ty), colour, -1)
            cv2.putText(img, tag, (x0 + 5, ty - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 20, 20), 2)
        if self.last_latency is not None:
            note = "%s  %.1fs" % (self.model, self.last_latency)
            if stale:
                # Say so on the picture. An overlay that silently keeps drawing
                # the last known box looks exactly like one that is tracking.
                note += "   [STALE %d]" % self.track_miss
            cv2.putText(img, note,
                        (12, height - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (60, 200, 255) if stale else (240, 240, 240), 2)
        out = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        out.header = header
        self.annotated_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VlmMovability()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
