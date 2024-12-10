#pragma once 

#include <chrono>
#include <functional>
#include <memory>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/bool.hpp"

#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"

#include "nav_msgs/msg/path.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using Empty = std_srvs::srv::Empty;
using Point = geometry_msgs::msg::Point;
using Pose = geometry_msgs::msg::Pose;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using PoseArray = geometry_msgs::msg::PoseArray;
using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using FollowPath = nav2_msgs::action::FollowPath;
using Path = nav_msgs::msg::Path;
using ClientComputePathToPoseGoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
using ClientFollowPathGoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

static const rmw_qos_profile_t rmw_qos_profile_custom =
{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  100,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

double euclidean_distance(Point p1, Point p2)
{
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

double get_yaw_from_q (geometry_msgs::msg::Quaternion q)
{
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

class  Evader : public rclcpp::Node
{
private:
  int difficulty, n_pursuers = 1;
  bool roadmap_ready = false;

  Pose gate_pose;
  bool gate_pose_ready = false;

  Pose evader_pose;
  Pose pursuer_pose;

  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr evader_pose_sub_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr pursuer_pose_sub_;

  rclcpp::Subscription<PoseArray>::SharedPtr gates_pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr evader_ready_pub_;
  rclcpp::Service<Empty>::SharedPtr start_service_;

  rclcpp_action::Client<ComputePathToPose>::SharedPtr compute_path_to_pose_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;


public:
  Evader(); 

private:
  void handle_evader_pose(const PoseWithCovarianceStamped::SharedPtr msg);
  void handle_pursuer_pose(const PoseWithCovarianceStamped::SharedPtr msg);

  void handle_gate_pose(const PoseArray::SharedPtr msg);

  void start_callback(const std::shared_ptr<Empty::Request> request,
             std::shared_ptr<Empty::Response> response);

  void goal_response_path_planning_callback(
    const ClientComputePathToPoseGoalHandle::SharedPtr & goal_handle);
  
  void feedback_path_planning_callback(ClientComputePathToPoseGoalHandle::SharedPtr,
    const std::shared_ptr<const ComputePathToPose::Feedback> feedback);

  void result_path_planning_callback(
    const ClientComputePathToPoseGoalHandle::WrappedResult & result);


  void move(const Path& path);
  void choose_gates_wrapper_(const std::vector<Pose>& gates);  

  void goal_response_path_following_callback(
    const ClientFollowPathGoalHandle::SharedPtr & goal_handle);
  
  void feedback_path_following_callback(ClientFollowPathGoalHandle::SharedPtr,
    const std::shared_ptr<const FollowPath::Feedback> feedback);
    
  void result_path_following_callback(
    const ClientFollowPathGoalHandle::WrappedResult & result);
  
};

