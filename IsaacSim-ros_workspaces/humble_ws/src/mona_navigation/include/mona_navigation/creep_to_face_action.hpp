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

#ifndef MONA_NAVIGATION__CREEP_TO_FACE_ACTION_HPP_
#define MONA_NAVIGATION__CREEP_TO_FACE_ACTION_HPP_

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mona_interfaces/msg/path_obstacle.hpp"
#include "mona_navigation/costmap_geometry.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mona_navigation/push_geometry.hpp"
#include "tf2_ros/buffer.h"

namespace mona_navigation
{
namespace detail
{

// Gains / limits (named constants; tuned on the real robot, not here).
constexpr double kHeadingGain = 1.5;     // rad/s per rad of heading error
constexpr double kLateralGain = 1.0;     // rad of heading offset per m of lateral error
constexpr double kMaxLateralHeading = 0.35;  // rad, cap on the lateral-correction offset
constexpr double kMaxAngular = 0.5;      // rad/s
constexpr double kMinRotateAngular = 0.1;  // rad/s, so in-place rotation does not stall
constexpr double kStallMinCmd = 0.03;    // m/s, below this a stall is not judged

inline double wrap_angle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

inline double yaw_from_quat(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/// Distance still to go along the target heading (<= 0: target point passed).
inline double along_remaining(double x, double y, double tx, double ty, double tyaw)
{
  return std::cos(tyaw) * (tx - x) + std::sin(tyaw) * (ty - y);
}

/// Signed offset of the robot from the line through (tx,ty) along tyaw; + = left of the line.
inline double lateral_error(double x, double y, double tx, double ty, double tyaw)
{
  return -std::sin(tyaw) * (x - tx) + std::cos(tyaw) * (y - ty);
}

/// Angular velocity to rotate in place toward tyaw (clamped, with a minimum magnitude).
inline double rotate_cmd(double yaw, double tyaw)
{
  const double err = wrap_angle(tyaw - yaw);
  double w = std::clamp(kHeadingGain * err, -kMaxAngular, kMaxAngular);
  if (std::fabs(w) < kMinRotateAngular) {
    w = std::copysign(kMinRotateAngular, err);
  }
  return w;
}

/// Angular velocity while driving: heading P-control toward a heading biased by lateral error.
inline double drive_cmd(double yaw, double tyaw, double lateral)
{
  const double bias = std::clamp(-kLateralGain * lateral, -kMaxLateralHeading, kMaxLateralHeading);
  const double err = wrap_angle(tyaw + bias - yaw);
  return std::clamp(kHeadingGain * err, -kMaxAngular, kMaxAngular);
}

/// Blocked detector: commanded >= kStallMinCmd but measured < ratio*cmd for stall_time seconds.
struct StallTracker
{
  bool active {false};
  double since {0.0};

  void reset() {active = false;}

  bool update(double t, double cmd, double measured, double ratio, double stall_time)
  {
    if (cmd < kStallMinCmd || measured >= ratio * cmd) {
      active = false;
      return false;
    }
    if (!active) {
      active = true;
      since = t;
    }
    return (t - since) >= stall_time;
  }
};

}  // namespace detail

/**
 * @brief Rotate to the target heading, then drive straight at low speed until the target
 * point is passed (along-heading remaining <= 0). Used to touch the object face.
 * FAILURE on timeout or stall (blocked). No costmap check by design: the object is lethal in
 * the costmap and the lidar is blind below ~1 m. Always publishes a zero Twist on stop.
 * Needs blackboard "node" (rclcpp::Node::SharedPtr) and "tf_buffer" (tf2_ros::Buffer).
 */
class CreepToFaceAction : public BT::StatefulActionNode
{
public:
  CreepToFaceAction(const std::string & name, const BT::NodeConfiguration & conf);
  ~CreepToFaceAction() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("target", "Face pose (from ComputeSweepPoses)"),
      BT::InputPort<double>("speed", 0.05, "Forward speed [m/s]"),
      BT::InputPort<double>("timeout", 30.0, "Total time limit [s]"),
      BT::InputPort<double>("stall_time", 2.0, "Blocked for this long -> FAILURE [s]"),
      BT::InputPort<double>("stall_ratio", 0.3, "Stall if odom speed < ratio * commanded"),
      BT::InputPort<double>("yaw_tol", 0.05, "Heading tolerance to start driving [rad]"),
      BT::InputPort<std::string>("cmd_vel_topic", "/cmd_vel", "Twist topic"),
      BT::InputPort<std::string>("odom_topic", "/odom", "Odometry topic"),
      BT::InputPort<std::string>("base_frame", "base_footprint", "Robot base frame"),
      BT::InputPort<std::string>("global_frame", "map", "Frame of the target"),
      BT::InputPort<bool>("rotate_only", false, "SUCCESS after the rotate phase (only the target heading is used)"),
      BT::InputPort<double>("max_travel", 0.0, "DRIVE phase: SUCCESS (REACHED) after this odom distance [m]; 0 = off"),
      BT::InputPort<bool>("side_clear_stop", false, "DRIVE: SUCCESS (SIDE_CLEAR) once a free window opens beside the robot"),
      BT::InputPort<mona_interfaces::msg::PathObstacle>("obstacle", "Object being pushed (width_m -> half width); optional"),
      BT::InputPort<double>("side_width", 0.5, "Side window width [m] (robot 0.44 + margin)"),
      BT::InputPort<double>("side_margin", 0.05, "Gap between the object edge and the side window [m]"),
      BT::InputPort<double>("side_along_min", 0.2, "Side window start ahead of the base [m]"),
      BT::InputPort<double>("side_along_len", 1.3, "Side window length along x [m]"),
      BT::InputPort<std::string>("costmap_topic", "/local_costmap/costmap_raw", "nav2_msgs/Costmap for side_clear_stop"),
      BT::InputPort<int>("lethal", 254, "Cell value counted as lethal"),
      BT::InputPort<double>("min_travel", 0.0, "side_clear_stop is checked only after this odom distance in DRIVE [m]"),
      BT::OutputPort<std::string>("result", "REACHED / SIDE_CLEAR / STALLED / TIMEOUT / TF_ERROR"),
      BT::OutputPort<std::string>("clear_side", "left / right when result is SIDE_CLEAR"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase { ROTATE, DRIVE };

  void stop();
  BT::NodeStatus fail(const char * reason, const char * result);
  BT::NodeStatus succeed(const char * msg);
  bool sideClear(const char *& side);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::string cmd_vel_topic_, odom_topic_;

  double odom_speed_ {0.0};
  double target_x_ {0.0}, target_y_ {0.0}, target_yaw_ {0.0};
  double speed_ {0.05}, timeout_ {30.0}, stall_time_ {2.0}, stall_ratio_ {0.3}, yaw_tol_ {0.05};
  std::string base_frame_, global_frame_;
  bool rotate_only_ {false};
  double max_travel_ {0.0};
  bool tf_failing_ {false};
  // side_clear_stop
  bool side_clear_stop_ {false};
  double side_half_ {0.35}, side_width_ {0.5}, side_margin_ {0.05};
  double side_along_min_ {0.2}, side_along_len_ {1.3}, min_travel_ {0.0};
  int lethal_ {254};
  std::string costmap_topic_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  detail::CostGrid grid_;
  std::string grid_frame_;
  detail::TravelTracker travel_;
  Phase phase_ {Phase::ROTATE};
  double start_s_ {0.0};
  double last_cmd_v_ {0.0};
  detail::StallTracker stall_;
};

}  // namespace mona_navigation

#endif  // MONA_NAVIGATION__CREEP_TO_FACE_ACTION_HPP_
