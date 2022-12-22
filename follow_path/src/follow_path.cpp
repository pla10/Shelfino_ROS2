#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "nav2_msgs/action/follow_path.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"



class PathPublisher : public rclcpp::Node
{
  public:
    PathPublisher()
    : Node("follow_path")
    {
        sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "cmd_path", 10, std::bind(&PathPublisher::received_path, this, std::placeholders::_1));
    }

  
  private:

    void received_path(const std::shared_ptr<nav_msgs::msg::Path> path_msg)
    {
        using FollowPath = nav2_msgs::action::FollowPath;
        // using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

        rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;

        client_ptr_ = rclcpp_action::create_client<FollowPath>(this,"follow_path");

        if (!client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = FollowPath::Goal();
        goal_msg.path = *path_msg;
        goal_msg.controller_id = "FollowPath";

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        client_ptr_->async_send_goal(goal_msg);

      return;
    }
    
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPublisher>());
  rclcpp::shutdown();
  return 0;
}