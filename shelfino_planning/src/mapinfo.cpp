#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon.hpp"

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



class MapInformationNode : public rclcpp::Node 
{
public:
    const int qos = 10;
    MapInformationNode() : Node("map_information_node") 
    {   
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
        
        map_subscription_ = this->create_subscription<geometry_msgs::msg::Polygon>(
            "map_borders",  
            qos,   
            std::bind(&MapInformationNode::get_map_boarder_callback, this, std::placeholders::_1));
    }

private:
    void get_map_boarder_callback(const geometry_msgs::msg::Polygon::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "point of border recived:");
        for (size_t i = 0; i < msg->points.size(); ++i)
        {
            const auto &point = msg->points[i];
            RCLCPP_INFO(this->get_logger(), "  point %zu: x=%.2f, y=%.2f, z=%.2f", i + 1, point.x, point.y, point.z);
        }
    }
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr map_subscription_;

    
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapInformationNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
