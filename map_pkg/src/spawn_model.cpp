#include "map_pkg/spawn_model.hpp"
#include <atomic>
bool spawn_model(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_interface,
      rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr& spawner_,
      std::string xml, geometry_msgs::msg::Pose pose, std::string prefix)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for service spawn_entity...");
  while(!spawner_->wait_for_service(std::chrono::milliseconds(100))){
    if (!rclcpp::ok()){
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  // Configure request
  auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
  request->name = prefix+std::to_string(pose.position.x)+"_"+std::to_string(pose.position.y);
  request->initial_pose = pose;
  request->xml = xml;

  using ServiceResponseFuture = rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedFuture;

  RCLCPP_INFO(rclcpp::get_logger("send_gates"), "Sending service request to spawn entity...");

  std::promise<int8_t> promise;
  // std::future<int8_t> future = promise.get_future();
  auto response_received_callback = [&promise](ServiceResponseFuture future) {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Object spawned successfully");
      promise.set_value(1);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to spawn object");
      promise.set_value(-1);
    }
  };

  // Send request
  // auto result = spawner_->async_send_request(request);
  auto result = spawner_->async_send_request(request, response_received_callback);

  // Wait for the response
  // rclcpp::spin_until_future_complete(node_interface, result);

  // future.wait();

  return true; // future.get() == 1;// succ.load(std::memory_order_relaxed) == 1;
}