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

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "custom_nav2_bt_plugins/approach_obstacle_normal_action.hpp"

#include "nav2_util/robot_utils.hpp"

namespace custom_nav2_bt_plugins
{

namespace
{
geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

// Yaw (rotation about +z) of a quaternion. Returns false when the quaternion is
// unusable: near-zero norm, or an identity rotation - which is what the
// perception node publishes when it has no orientation estimate yet.
bool quaternionYaw(const geometry_msgs::msg::Quaternion & q, double & yaw)
{
  const double n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (n2 < 1e-6) {
    return false;
  }
  if (std::abs(q.w) >= 0.99995 &&
    std::abs(q.x) + std::abs(q.y) + std::abs(q.z) < 1e-3)
  {
    return false;
  }
  yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  return true;
}
}  // namespace

ApproachObstacleNormalAction::ApproachObstacleNormalAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf),
  last_pose_time_(0, 0, RCL_ROS_TIME),
  pose_received_(false),
  pose_timeout_(2.0)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  getInput("pose_topic", pose_topic_);
  getInput("pose_timeout", pose_timeout_);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&ApproachObstacleNormalAction::poseCallback, this, std::placeholders::_1),
    sub_option);

  RCLCPP_INFO(
    node_->get_logger(),
    "ApproachObstacleNormal: subscribed to \"%s\"", pose_topic_.c_str());
}

void ApproachObstacleNormalAction::poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_pose_ = *msg;
  last_pose_time_ = node_->now();
  pose_received_ = true;
}

BT::NodeStatus ApproachObstacleNormalAction::tick()
{
  callback_group_executor_.spin_some();

  nav_msgs::msg::Path path;
  if (!getInput("path", path) || path.poses.size() < 2) {
    return BT::NodeStatus::FAILURE;
  }

  double standoff = 0.4;
  double on_path_tol = 0.35;
  double recompute_thr = 0.2;
  double interp = 0.1;
  double transform_tolerance = 0.1;
  bool use_surface_normal = true;
  bool face_obstacle = true;
  std::string global_frame = "map";
  std::string robot_base_frame = "base_link";
  getInput("standoff_distance", standoff);
  getInput("on_path_tolerance", on_path_tol);
  getInput("recompute_threshold", recompute_thr);
  getInput("interp_spacing", interp);
  getInput("use_surface_normal", use_surface_normal);
  getInput("face_obstacle", face_obstacle);
  getInput("global_frame", global_frame);
  getInput("robot_base_frame", robot_base_frame);
  getInput("transform_tolerance", transform_tolerance);

  geometry_msgs::msg::PoseStamped obstacle;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pose_received_ || (node_->now() - last_pose_time_).seconds() > pose_timeout_) {
      have_cache_ = false;
      return BT::NodeStatus::FAILURE;
    }
    obstacle = last_pose_;
  }
  const double ox = obstacle.pose.position.x;
  const double oy = obstacle.pose.position.y;

  // Project the obstacle onto the (initial) path: closest segment + its tangent.
  const auto & poses = path.poses;
  double best_lateral = std::numeric_limits<double>::infinity();
  double tangent_yaw = 0.0;
  for (std::size_t i = 0; i + 1 < poses.size(); ++i) {
    const auto & A = poses[i].pose.position;
    const auto & B = poses[i + 1].pose.position;
    const double abx = B.x - A.x;
    const double aby = B.y - A.y;
    const double seg2 = abx * abx + aby * aby;
    if (seg2 < 1e-12) {continue;}
    double t = ((ox - A.x) * abx + (oy - A.y) * aby) / seg2;
    t = std::clamp(t, 0.0, 1.0);
    const double px = A.x + t * abx;
    const double py = A.y + t * aby;
    const double lateral = std::hypot(ox - px, oy - py);
    if (lateral < best_lateral) {
      best_lateral = lateral;
      tangent_yaw = std::atan2(aby, abx);
    }
  }

  if (best_lateral > on_path_tol) {
    have_cache_ = false;
    return BT::NodeStatus::FAILURE;   // not the on-path blocker
  }

  // Pick the approach geometry. Surface-normal mode when the obstacle pose
  // carries a usable orientation; initial-path tangent otherwise (legacy).
  double normal_yaw = 0.0;
  const bool have_normal =
    use_surface_normal && quaternionYaw(obstacle.pose.orientation, normal_yaw);

  double goal_dir_yaw;   // obstacle -> standoff goal
  double face_yaw;       // heading stamped on every approach pose
  const char * mode;
  if (have_normal) {
    goal_dir_yaw = normal_yaw;                                  // free-space side
    face_yaw = face_obstacle ? (normal_yaw + M_PI) : normal_yaw;
    mode = "surface-normal";
  } else {
    goal_dir_yaw = tangent_yaw + M_PI;   // stop back along -tangent (== ox - tx*standoff)
    face_yaw = tangent_yaw;              // face along the tangent, toward the obstacle
    mode = "path-tangent";
  }

  const double gx = ox + std::cos(goal_dir_yaw) * standoff;
  const double gy = oy + std::sin(goal_dir_yaw) * standoff;

  // Latch: reuse the cached approach path unless the obstacle moved or the
  // geometry mode flipped.
  if (have_cache_ && cached_have_normal_ == have_normal &&
    std::hypot(ox - cached_obs_x_, oy - cached_obs_y_) < recompute_thr)
  {
    setOutput("approach_path", cached_approach_);
    setOutput("approach_goal", cached_goal_);
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "ApproachObstacleNormal: waiting/standoff target (%.2f, %.2f) in \"%s\" "
      "[obstacle @ (%.2f, %.2f), standoff=%.2f m]",
      cached_goal_.pose.position.x, cached_goal_.pose.position.y,
      cached_goal_.header.frame_id.c_str(), ox, oy, standoff);
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf_buffer_, global_frame, robot_base_frame, transform_tolerance))
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "ApproachObstacleNormal: could not get robot pose in \"%s\"", global_frame.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;

  nav_msgs::msg::Path approach;
  approach.header.frame_id = global_frame;
  approach.header.stamp = node_->now();

  const double span = std::hypot(gx - rx, gy - ry);

  auto make_pose = [&](double x, double y) {
      geometry_msgs::msg::PoseStamped p;
      p.header = approach.header;
      p.pose.position.x = x;
      p.pose.position.y = y;
      p.pose.orientation = yawToQuaternion(face_yaw);
      return p;
    };

  bool hold;
  if (have_normal) {
    hold = (span < std::max(0.05, 0.5 * interp));
  } else {
    // Legacy check: do not drive backwards past the stop point along the tangent.
    const double tx = std::cos(tangent_yaw);
    const double ty = std::sin(tangent_yaw);
    const double forward = (gx - rx) * tx + (gy - ry) * ty;
    hold = (forward <= 0.05 || span < 1e-3);
  }

  if (hold) {
    // Already at / past the stop point: just hold, squared up to the obstacle.
    approach.poses.push_back(make_pose(rx, ry));
  } else {
    const int n = std::max(1, static_cast<int>(std::ceil(span / std::max(0.02, interp))));
    for (int k = 0; k <= n; ++k) {
      const double a = static_cast<double>(k) / static_cast<double>(n);
      approach.poses.push_back(make_pose(rx + a * (gx - rx), ry + a * (gy - ry)));
    }
  }

  cached_approach_ = approach;
  cached_goal_ = approach.poses.back();
  cached_obs_x_ = ox;
  cached_obs_y_ = oy;
  cached_have_normal_ = have_normal;
  have_cache_ = true;

  setOutput("approach_path", approach);
  setOutput("approach_goal", cached_goal_);

  RCLCPP_INFO(
    node_->get_logger(),
    "ApproachObstacleNormal[%s]: obstacle on initial path (lateral=%.2f m), "
    "standoff goal (%.2f, %.2f) heading=%.2f, %zu poses",
    mode, best_lateral, gx, gy, face_yaw, approach.poses.size());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace custom_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<custom_nav2_bt_plugins::ApproachObstacleNormalAction>(
    "ApproachObstacleNormal");
}
