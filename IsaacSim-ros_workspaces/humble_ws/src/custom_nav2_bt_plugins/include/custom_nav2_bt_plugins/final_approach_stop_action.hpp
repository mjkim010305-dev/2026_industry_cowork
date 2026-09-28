// Copyright (c) 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CUSTOM_NAV2_BT_PLUGINS__FINAL_APPROACH_STOP_ACTION_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__FINAL_APPROACH_STOP_ACTION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "behaviortree_cpp_v3/action_node.h"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Final leg of the green-box approach: turn to the box and creep toward it,
 * until the LIDAR says the box is right in front, then stop. No pushing.
 *
 * This deliberately bypasses Nav2's planner/controller/costmap for the last
 * stretch, same reasoning as OdomLimitedDriveAction in this package: costmap
 * inflation around the box would make "stop just in front of it" an
 * unreachable goal for ComputePathToPose/FollowPath (the inflated cost keeps
 * the goal itself inside the "too close to an obstacle" region). By the time
 * this leaf runs, ComputeGreenBoxApproachGoal + FollowPath have already put
 * the robot roughly square-on at `standoff_distance`, so a short blind
 * forward creep with a LIDAR range gate is enough and safer than trying to
 * coerce the costmap.
 *
 * Limitation: the only collision check is the forward lidar cone. When the
 * bearing error exceeds align_threshold the robot turns in place with no
 * side or rear sensing, so a large correction next to a wall is unguarded.
 *
 * onRunning: take the freshest /scan, look at ranges whose angle falls inside
 * [-front_half_angle, +front_half_angle] (LaserScan frame, 0 = forward), take
 * the minimum range r among readings that carry any distance evidence:
 * finite r < range_min is closer than the sensor can report and is clamped
 * to 0 (it still drives the stop, it is not discarded); inf/NaN readings are
 * "no return" and are skipped (absence of a return is not proof the box is
 * gone, so it must not count as clear space either). min_range <=
 * stop_distance -> publish zero Twist and SUCCEED, in either phase below.
 * This creep is open-loop and blind between scans, so a missing/stale scan
 * is a fail-safe, not a shrug: if no scan has ever arrived, or the latest
 * one is older than `scan_loss_timeout`, publish zero Twist and FAIL
 * immediately rather than let the robot keep creeping on stale data. One
 * transiently late scan inside that window is tolerated (falls through to
 * the same command as last tick); only a loss that outlasts the window
 * fails. `time_allowance` remains the separate, much larger bound on total
 * leaf runtime when scans are healthy but the box is never reached.
 *
 * g6 (Session 11 live run): ComputeGreenBoxApproachGoal/FollowPath can land
 * the robot facing anywhere close to the goal (e.g. FollowPath reports
 * immediate arrival when the recomputed goal sits near the current pose),
 * so this leaf must not assume it is already square-on to the box - it was
 * driving straight ahead on whatever heading it happened to have, missed
 * the box outside its front cone entirely, and pushed it sideways for the
 * full `time_allowance`. Steering fix: read `pose_topic` (same detector
 * topic as ComputeGreenBoxApproachGoal) plus the robot pose from TF
 * (`global_frame` <- `robot_base_frame`, same `nav2_util::getCurrentPose`
 * pattern as ComputeGreenBoxApproachGoalAction) and compute the bearing
 * error from robot heading to the box every tick. The box is static, so
 * once a pose has arrived it stays valid for the rest of the leaf's run -
 * no per-tick staleness check like ComputeGreenBoxApproachGoal's `max_age`.
 * But the very first tick(s) need something to steer by, so the same
 * grace-then-fail pattern as the first-scan handling above applies: no
 * `pose_topic` message received yet since onStart is "not yet" for up to
 * `scan_loss_timeout` seconds (publish zero, stay RUNNING), and still
 * nothing after that is FAILURE with a log - reusing that timeout rather
 * than adding a near-duplicate port, since both are "how long to tolerate
 * having nothing to act on before this leaf gives up" grace windows. A TF
 * lookup failure is likewise FAILURE with a log: without a robot heading
 * there is nothing to steer by, and creeping blind on the last known
 * heading is exactly the g6 failure mode.
 * If |bearing error| > align_threshold, rotate in place (linear 0, angular
 * = clamp(heading_gain * error, +-max_angular_speed)); otherwise creep
 * forward at approach_speed with the same clamped angular term layered on
 * so the heading keeps correcting while closing distance. The stop check
 * above runs before this decision every tick, so it fires in both phases.
 * max_angular_speed defaults well above a token/tiny value (see .cpp) -
 * the sim base is known to turn poorly (near dead-band, doesn't visibly
 * rotate) at low commanded angular speeds.
 * onHalted always publishes zero, so a preempted approach never leaves the
 * robot creeping.
 */
class FinalApproachStopAction : public BT::StatefulActionNode
{
public:
  FinalApproachStopAction(const std::string & name, const BT::NodeConfiguration & conf);
  FinalApproachStopAction() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("scan_topic", std::string("/scan"), "LaserScan to gate on"),
      BT::InputPort<std::string>(
        "cmd_vel_topic", std::string("/cmd_vel"), "Twist topic to publish"),
      BT::InputPort<double>("stop_distance", 0.4, "Stop once min forward range <= this [m]"),
      BT::InputPort<double>("approach_speed", 0.15, "Forward creep speed [m/s]"),
      BT::InputPort<double>(
        "front_half_angle", 0.26, "Half-angle [rad] of the forward cone (0=full width)"),
      BT::InputPort<double>(
        "scan_loss_timeout", 0.5,
        "FAILURE if no scan has ever arrived or the latest one is older than this [s] "
        "(must stay well below time_allowance - this is the blind-creep fail-safe, "
        "not the overall runtime budget); also reused as the grace period for the "
        "first box pose to arrive on pose_topic"),
      BT::InputPort<double>(
        "time_allowance", 30.0, "FAILURE if stop_distance is not reached within this many seconds"),
      BT::InputPort<std::string>(
        "pose_topic", std::string("green_box/pose"),
        "geometry_msgs/PoseStamped published by the green-box detector (static box, "
        "last received pose stays valid)"),
      BT::InputPort<std::string>(
        "global_frame", std::string("map"), "Frame the robot pose (TF) is looked up in"),
      BT::InputPort<std::string>(
        "robot_base_frame", std::string("base_link"), "Robot base frame"),
      BT::InputPort<double>(
        "align_threshold", 0.26,
        "Rotate in place instead of creeping while |bearing error| exceeds this [rad] "
        "(matches front_half_angle by default: no point creeping toward a heading the "
        "stop-check cone can't even see)"),
      BT::InputPort<double>(
        "max_angular_speed", 0.8, "Angular speed clamp [rad/s], both phases"),
      BT::InputPort<double>(
        "heading_gain", 1.5, "Proportional gain, bearing error [rad] -> angular speed [rad/s]"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void scanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg);
  void poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void publishZero();
  void publishTwist(double linear, double angular);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  std::string scan_topic_;
  std::string cmd_vel_topic_;
  std::string pose_topic_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double stop_distance_ {0.4};
  double approach_speed_ {0.15};
  double front_half_angle_ {0.26};
  double scan_loss_timeout_ {0.5};
  double time_allowance_ {30.0};
  double align_threshold_ {0.26};
  double max_angular_speed_ {0.8};
  double heading_gain_ {1.5};

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  rclcpp::Time latest_scan_time_;
  rclcpp::Time start_time_;

  std::mutex pose_mutex_;
  geometry_msgs::msg::PoseStamped last_box_pose_;
  bool pose_received_since_start_ {false};
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__FINAL_APPROACH_STOP_ACTION_HPP_
