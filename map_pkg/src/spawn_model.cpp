#include "map_pkg/spawn_model.hpp"

void spawn_model(
      std::string xml, geometry_msgs::msg::Pose pose, 
      rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr& spawner_, 
      std::string node_namespace)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for service spawn_entity...");
  while(!spawner_->wait_for_service(std::chrono::milliseconds(100))){
    if (!rclcpp::ok()){
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown(nullptr, "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  // Configure request
  auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
  request->name = "obstacle"+std::to_string(pose.position.x)+"_"+std::to_string(pose.position.y);
  request->initial_pose = pose;
  request->robot_namespace = node_namespace;
  request->xml = xml;

  // Send request
  auto result = spawner_->async_send_request(request);
}