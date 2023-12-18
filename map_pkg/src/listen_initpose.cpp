#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

int main(){
  rclcpp::init(0, nullptr);
  // Create listener node of type PoseWithCovarianceStamped on /shelfino1/initialpose
  auto node = rclcpp::Node::make_shared("listen_initPose");
  auto sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/shelfino1/initialpose", 10, 
    [](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I heard: [%f, %f]", msg->pose.pose.position.x, msg->pose.pose.position.y);
    }
  );

  rclcpp::spin(node);
  rclcpp::shutdown(); 

  return 0;
}