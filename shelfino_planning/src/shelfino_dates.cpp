#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <iostream>
#include <vector>
#include <random>
#include <math.h>
#include <tuple>
#include <algorithm>
#include <list>
#include <limits>
#include <set>
#include <tuple>
#include <map>


using std::placeholders::_1;

using namespace std::chrono_literals;

static const rmw_qos_profile_t rmw_qos_profile_custom =
    {
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        10,
        RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        RMW_QOS_DEADLINE_DEFAULT,
        RMW_QOS_LIFESPAN_DEFAULT,
        RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
        RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
        false};


class ShelfinoDateNode : public rclcpp::Node 
{
public:
    ShelfinoDateNode() : Node("shelfino_date_node") 
    {   
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
       
        shelfino0_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "shelfino1/odom",  
            qos,   
           std::bind(&ShelfinoDateNode::get_shelfino_odom_callback, this, std::placeholders::_1));
    }

private:
    void get_shelfino_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
          RCLCPP_INFO(this->get_logger(), "Odometry received: position (x: %.2f, y: %.2f), velocity (x: %.2f, y: %.2f)",
                msg->pose.pose.position.x, msg->pose.pose.position.y,
                msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr shelfino0_odom_subscription_;


    
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ShelfinoDateNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
