#pragma once

#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>

#include "map_pkg/utilities.hpp"

/**
 * @brief Spawn model of the gate in gazebo
 * 
 * @param pose The position in which the model should be spawned
 * @param spawner_ The client to the gazebo service
 * @param xml The xml file to spawn the model
 * @param prefix The prefix of the model name
 * @param wait If the function should wait for the service to spawn the model
 * @return true If the model was spawned successfully. If wait is true, it will return true if the service was called successfully
 */
bool spawn_model(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_interface,
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr& spawner_,
  std::string xml, geometry_msgs::msg::Pose pose, 
  std::string prefix = "obstacle"
);