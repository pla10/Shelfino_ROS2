#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>

/**
 * @brief Spawn model of the gate in gazebo
 * 
 * @param pose The position in which the model should be spawned
 */
void spawn_model(
      std::string xml, geometry_msgs::msg::Pose pose, 
      rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr& spawner_, 
      std::string node_namespace = "");