#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"

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
        
        gate_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "gates",  
            qos,   
            std::bind(&MapInformationNode::get_gate_callback, this, std::placeholders::_1));
        
        obstacle_subscription_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            "obstacles",  
            qos,   
            std::bind(&MapInformationNode::get_obstacle_callback, this, std::placeholders::_1));
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
    void get_gate_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "Received PoseArray with frame_id: %s, stamp: %d.%09d",
                msg->header.frame_id.c_str(),
                msg->header.stamp.sec,
                msg->header.stamp.nanosec);

        for (size_t i = 0; i < msg->poses.size(); ++i) {
            const auto &pose = msg->poses[i];
            RCLCPP_INFO(this->get_logger(), "Pose %zu: Position(x=%.f, y=%.f, z=%.f), Orientation(x=%.f, y=%.f, z=%.f, w=%.f)",
                        i + 1,
                        pose.position.x, pose.position.y, pose.position.z,
                        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        }
    }
    void get_obstacle_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "Obstecles recived with with frame_id: %s, stamp: %d.%09d",
                msg->header.frame_id.c_str(),
                msg->header.stamp.sec,
                msg->header.stamp.nanosec);
        
        for (size_t i = 0; i < msg->obstacles.size(); ++i) {
            const auto &obstacle = msg->obstacles[i];

            RCLCPP_INFO(this->get_logger(), "Obstacle %zu: frame_id: %s, stamp: %d.%09d",
                        i + 1,
                        obstacle.header.frame_id.c_str(),
                        obstacle.header.stamp.sec,
                        obstacle.header.stamp.nanosec);

            RCLCPP_INFO(this->get_logger(), "  Polygon:");
            for (size_t j = 0; j < obstacle.polygon.points.size(); ++j) {
                const auto &point = obstacle.polygon.points[j];
                RCLCPP_INFO(this->get_logger(), "    Point %zu: x=%.2f, y=%.f, z=%.f",
                            j + 1, point.x, point.y, point.z);
            }

            RCLCPP_INFO(this->get_logger(), "  Radius: %.f", obstacle.radius);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr map_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gate_subscription_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_subscription_;
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapInformationNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
