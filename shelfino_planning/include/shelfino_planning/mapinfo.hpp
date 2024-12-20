
// map info file header
#ifndef MAP_INFORMATION_NODE_HPP
#define MAP_INFORMATION_NODE_HPP

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"

class MapInformationNode : public rclcpp::Node 
{
    public:
        MapInformationNode();
        virtual ~MapInformationNode(); 
    private:
        void get_map_boarder_callback(const geometry_msgs::msg::Polygon::SharedPtr msg);
        void get_gate_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
        void get_obstacle_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg);

        rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr map_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gate_subscription_;
        rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_subscription_;

};

#endif